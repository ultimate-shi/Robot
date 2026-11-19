/* 作用：实现0.4反馈解析、免标定PWM、角度PID及从板看门狗；用法：由main调用，主机测试分别指定ARM_MASTER=1/0，J不再支持。 */
#include "arm_core.h"
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static float clip(float x, float lo, float hi) { return fminf(hi,fmaxf(lo,x)); }
void arm_defaults(arm_settings_t *c) {
    *c=(arm_settings_t){.encoder_dir=1,.motor_dir=1,.neutral_us=ARM_PWM_NEUTRAL,.neutral_valid=true,
        .vmax=ARM_DEFAULT_VMAX,.amax=ARM_DEFAULT_AMAX,.kp=ARM_DEFAULT_KP};
}
/* schema=2采用旧结构前40字节，schema=3追加hold；只在校验成功后覆盖配置。 */
bool arm_settings_decode(arm_settings_t *c,uint32_t schema,const void *blob,size_t size) {
    arm_settings_t stored; arm_defaults(&stored);
    _Static_assert(offsetof(arm_settings_t,hold)==40,"legacy settings layout changed");
    if (!((schema==2 && size==offsetof(arm_settings_t,hold)) ||
          (schema==3 && size==sizeof(stored)))) return false;
    /* 对标志逐字节检查，避免损坏Flash生成非法bool表示。 */
    const unsigned char *b=blob;
    for (size_t i=offsetof(arm_settings_t,zero_valid);i<=offsetof(arm_settings_t,neutral_valid);i++)
        if (b[i]>1) return false;
    memcpy(&stored,blob,size);
    if (stored.zero_raw<0 || stored.zero_raw>4095 ||
        (stored.encoder_dir!=1 && stored.encoder_dir!=-1) || (stored.motor_dir!=1 && stored.motor_dir!=-1) ||
        stored.neutral_us<500 || stored.neutral_us>2500 ||
        !isfinite(stored.vmax) || stored.vmax<0.01f || stored.vmax>ARM_MAX_VMAX ||
        !isfinite(stored.amax) || stored.amax<0.01f || stored.amax>ARM_MAX_AMAX ||
        !isfinite(stored.kp) || stored.kp<0 || stored.kp>100 ||
        !isfinite(stored.ki) || stored.ki<0 || stored.ki>100 ||
        !isfinite(stored.kd) || stored.kd<0 || stored.kd>10 || (stored.hold!=0 && stored.hold!=1)) return false;
    *c=stored; return true;
}
void arm_bus_seen(arm_joint_t *j,int64_t now) {
#if ARM_MASTER
    /* 主轴由USB独立控制，不因从板离线而停转，也不由从板回包续期。 */
    (void)now; j->bus_deadline_us=0;
#else
    j->bus_deadline_us=now+ARM_BUS_WATCHDOG_US;
#endif
}
void arm_init(arm_joint_t *j,const arm_settings_t *c) {
    memset(j,0,sizeof(*j)); j->cfg=*c; j->pwm_us=0;
    /* 旧配置未设置中立时也使用默认1500us，不覆盖已设置的逐轴中立值。 */
    if (!j->cfg.neutral_valid) {
        j->cfg.neutral_us=ARM_PWM_NEUTRAL; j->cfg.neutral_valid=true;
    }
}
void arm_stop(arm_joint_t *j) {
    j->state=j->fault?FAULT:STOP; j->mode=NONE; j->deadline_us=0;
    j->target=j->ref=j->angle; j->ref_speed=0; j->integral=0; j->prev_error=0; j->d_filtered=0;
    j->settled_us=0;
    j->pwm_us=j->cfg.neutral_valid?j->cfg.neutral_us:0;
}
void arm_fault(arm_joint_t *j,int fault) { j->fault=fault; arm_stop(j); }
void arm_sample(arm_joint_t *j,bool valid,int raw,int64_t now) {
    if (!valid || raw<0 || raw>4095) { j->sensor_valid=false; arm_fault(j,1); return; }
    float angle=remainderf((raw-j->cfg.zero_raw)*360.0f/4096.0f,360.0f)*j->cfg.encoder_dir;
    if (j->sensor_valid && j->sample_us && now>j->sample_us) {
        float dt=(now-j->sample_us)*1e-6f;
        float v=remainderf(angle-j->angle,360.0f)/dt;
        if (j->state==RUN && fabsf(v)>360.0f) { arm_fault(j,4); }
        /* 控制/制动保留原快速速度估计；到位/反馈另滤波，避免削弱限位响应。 */
        j->speed+=0.25f*(v-j->speed);
        j->arrival_speed+=dt/(ARM_SPEED_FILTER_US*1e-6f+dt)*(v-j->arrival_speed);
    } else j->speed=j->arrival_speed=0;
    j->raw=raw; j->angle=angle; j->sample_us=now; j->sensor_valid=true;
    if (j->cfg.zero_valid && fabsf(angle)>ARM_LIMIT_DEG+0.5f) arm_fault(j,3);
}
const char *arm_move_check(const arm_joint_t *j,char op,float value) {
    if (!isfinite(value) || (op=='R' && (value < -90 || value>90)) ||
        (op=='U' && (value<ARM_PWM_MIN || value>ARM_PWM_MAX || truncf(value)!=value))) return "RANGE";
    /* U直接输出限时脉宽，不依赖零位/方向/中立标定；真实故障和传感器检查保留。 */
    if (j->fault || !j->sensor_valid) return "FAULT";
    if (op=='R' && (!j->cfg.zero_valid || !j->cfg.direction_valid ||
                    !j->cfg.neutral_valid)) return "FAULT";
    return NULL;
}
void arm_move(arm_joint_t *j,char op,float value,int64_t deadline) {
    arm_mode_t mode=op=='R'?POS:PWM;
    if (mode!=j->mode) { j->integral=0; j->d_filtered=0; j->prev_error=0;
        j->ref=j->angle; j->ref_speed=j->speed; }
    if (mode!=j->mode || (mode==POS && j->target!=value)) j->settled_us=0;
    j->mode=mode; j->state=RUN; j->deadline_us=mode==PWM?deadline:0;
    if (mode==POS) j->target=value; else j->pwm_us=(int)value;
}
void arm_tick(arm_joint_t *j,int64_t now) {
    float dt=j->tick_us?(now-j->tick_us)*1e-6f:0.005f;
    j->tick_us=now;
    if (j->state!=RUN) return;
    if ((j->mode==PWM && now>=j->deadline_us) ||
        (j->bus_deadline_us && now>=j->bus_deadline_us)) { arm_stop(j); return; }
    if (dt>0.020f || dt<=0) { arm_fault(j,4); return; }
    if (!j->sensor_valid || now-j->sample_us>ARM_ENCODER_STALE_US) { arm_fault(j,1); return; }
    if (j->mode==POS) {
        bool arrived=fabsf(j->target-j->angle)<=ARM_ARRIVAL_DEG && fabsf(j->arrival_speed)<=ARM_ARRIVAL_DPS;
        if (!arrived) j->settled_us=0;
        else if (!j->settled_us) j->settled_us=now;
        if (!j->cfg.hold && j->settled_us && now-j->settled_us>=ARM_ARRIVAL_US) {
            arm_stop(j); return;
        }
        float distance=j->target-j->ref;
        float desired=copysignf(fminf(j->cfg.vmax,sqrtf(2*j->cfg.amax*fabsf(distance))),distance);
        j->ref_speed+=clip(desired-j->ref_speed,-j->cfg.amax*dt,j->cfg.amax*dt);
        float step=j->ref_speed*dt;
        if (fabsf(step)>fabsf(distance) && step*distance>=0) { j->ref=j->target; j->ref_speed=0; }
        else j->ref=clip(j->ref+step,-ARM_LIMIT_DEG,ARM_LIMIT_DEG);
        float e=j->ref-j->angle;
        /* 微分取参考速度减实测速度，避免离散目标更新造成微分冲击。 */
        j->d_filtered+=0.2f*((j->ref_speed-j->speed)-j->d_filtered);
        float next_i=clip(j->integral+e*dt,-90,90);
        float u=j->ref_speed+j->cfg.kp*e+j->cfg.ki*next_i+j->cfg.kd*j->d_filtered;
        float lo=(ARM_PWM_MIN-j->cfg.neutral_us)/(ARM_PWM_US_PER_DPS*j->cfg.motor_dir);
        float hi=(ARM_PWM_MAX-j->cfg.neutral_us)/(ARM_PWM_US_PER_DPS*j->cfg.motor_dir);
        float limited=clip(u,fmaxf(-j->cfg.vmax,fminf(lo,hi)),fminf(j->cfg.vmax,fmaxf(lo,hi)));
        if (u==limited || e*u<0) j->integral=next_i;
        j->pwm_us=(int)lroundf(j->cfg.neutral_us+j->cfg.motor_dir*limited*ARM_PWM_US_PER_DPS);
    }
    j->pwm_us=(int)clip(j->pwm_us,ARM_PWM_MIN,ARM_PWM_MAX);
    float drive=(j->pwm_us-j->cfg.neutral_us)*j->cfg.motor_dir;
    float stopping=j->speed*j->speed/(2*fmaxf(j->cfg.amax,1))+ARM_LIMIT_MARGIN_DEG;
    /* 方向及中立未知时，不能用默认映射将直接PWM误判为朝限位运动。 */
    if (j->cfg.zero_valid && j->cfg.direction_valid && j->cfg.neutral_valid &&
        ((drive>0 && j->angle+stopping>=ARM_LIMIT_DEG) ||
         (drive<0 && j->angle-stopping<=-ARM_LIMIT_DEG))) {
        j->pwm_us=j->cfg.neutral_us; j->integral=0;
    }
}
static bool number(const char *s,int *v) {
    if (!s || !*s) return false;
    const char *p=s; if (*p=='-' || *p=='+') ++p;
    if (!*p) return false;
    for (;*p;++p) if (!isdigit((unsigned char)*p)) return false;
    errno=0; char *end; long n=strtol(s,&end,10);
    if (errno || *end || n<INT_MIN || n>INT_MAX) return false;
    *v=(int)n; return true;
}
/* 仅接受十进制浮点（含科学计数），拒绝十六进制、非有限值和未消费字符。 */
static bool decimal(const char *s,float *v) {
    const char *p=s; if (*p=='-' || *p=='+') p++;
    bool digit=false;
    while (isdigit((unsigned char)*p)) { digit=true; p++; }
    if (*p=='.') { p++; while (isdigit((unsigned char)*p)) { digit=true; p++; } }
    if (!digit) return false;
    if (*p=='e' || *p=='E') {
        p++; if (*p=='-' || *p=='+') p++;
        if (!isdigit((unsigned char)*p)) return false;
        while (isdigit((unsigned char)*p)) p++;
    }
    if (*p) return false;
    errno=0; char *end; float n=strtof(s,&end);
    if (errno || *end || !isfinite(n)) return false;
    *v=n; return true;
}
const char *arm_parse(const char *line,bool bus,arm_command_t *c) {
    memset(c,0,sizeof(*c)); c->ttl_ms=100;
    if (strlen(line)>ARM_USB_LINE) return "ARG";
    char buf[ARM_BUS_LINE+1]; strcpy(buf,line); char *save=NULL;
    char *word=strtok_r(buf," \t",&save); if (!word) return "ARG";
    if (strlen(word)>=sizeof(c->name)) return "CMD";
    strcpy(c->name,word);
    if (word[0]=='$') {
        if (!strcmp(word,"$info")) c->op='i';
        else if (!strcmp(word,"$version")) c->op='v';
        else if (!strcmp(word,"$zero")) c->op='z';
        else if (!strcmp(word,"$save")) c->op='s';
        else if (!strcmp(word,"$clear")) c->op='c';
        else if (!strcmp(word,"$raw")) c->op='d';
        else if (!strcmp(word,"$bus")) c->op='b';
        else if (!strcmp(word,"$busraw")) c->op='h';
        else return "CMD";
    } else {
        c->op=word[0]; if (!strchr("RUXAPH",c->op)) return "CMD";
        if (word[1] && (!number(word+1,&c->id) || c->id<1 || c->id>ARM_COUNT)) return "ID";
    }
    char *args[4];
    while ((word=strtok_r(NULL," \t",&save))) {
        if (c->argc==4) return "ARG";
        args[c->argc++]=word;
    }
    for (int i=0;i<c->argc;i++) {
        bool real=c->op=='R' || c->op=='P';
        if (real) { if (!decimal(args[i],&c->val[i])) return "ARG"; }
        else { int n; if (!number(args[i],&n)) return "ARG"; c->val[i]=(float)n; }
    }
    if (strchr("zscd",c->op)) {
        if (c->argc!=1) return "ARG";
        if (!number(args[0],&c->id)) return "ARG";
        c->argc=0;
        return c->id>=1 && c->id<=ARM_COUNT?NULL:"ID";
    }
    if (c->op=='i' || c->op=='v' || c->op=='b' || c->op=='h' || c->op=='X') return c->argc?"ARG":NULL;
    if (c->op=='R' || c->op=='U') {
        if (bus && c->op=='U' && c->argc) {
            if (c->argc<2) return "ARG";
            int ttl; if (!number(args[--c->argc],&ttl)) return "ARG";
            c->ttl_ms=ttl; if (c->ttl_ms<1 || c->ttl_ms>100) return "RANGE";
        }
        if (c->op=='R' && !c->argc) return NULL;
        int count=c->id?1:ARM_COUNT;
        if (c->argc!=count) return "ARG";
        for (int i=0;i<count;i++) {
            float n=c->val[i]; if (c->op=='R'?(n < -90 || n>90):(n<500 || n>2500)) return "RANGE";
        }
        return NULL;
    }
    if (!c->id) return "ID";
    if (c->op=='H') return c->argc==0 || (c->argc==1 && (c->val[0]==0 || c->val[0]==1))?NULL:"ARG";
    return c->argc==0 || c->argc==3?NULL:"ARG";
}
const char *arm_setting(arm_joint_t *j,const arm_command_t *c) {
    if (j->state==RUN || (c->op=='H' && j->state!=STOP)) return "BUSY";
    if (c->op=='A') {
        if (c->val[0]<0 || c->val[0]>4095 || (c->val[1]!=1 && c->val[1]!=-1) || (c->val[2]!=1 && c->val[2]!=-1)) return "RANGE";
        j->cfg.zero_raw=c->val[0]; j->cfg.encoder_dir=c->val[1]; j->cfg.motor_dir=c->val[2];
        j->cfg.zero_valid=true; j->cfg.direction_valid=true;
        j->sensor_valid=false; j->sample_us=0;
    } else if (c->op=='P') {
        if (c->val[0]<0 || c->val[0]>100 || c->val[1]<0 || c->val[1]>100 ||
            c->val[2]<0 || c->val[2]>10) return "RANGE";
        j->cfg.kp=c->val[0]; j->cfg.ki=c->val[1]; j->cfg.kd=c->val[2];
    } else if (c->op=='H') {
        if (c->argc!=1 || (c->val[0]!=0 && c->val[0]!=1)) return "RANGE";
        j->cfg.hold=(int)c->val[0];
    } else return "CMD";
    arm_stop(j); return NULL;
}
bool arm_feedback_parse(const char *line,int expected_id,int *state,int *fault,int *raw) {
    char a[24],v[24],cur[24],pwm[24],age[24]; int id,remain,mode,n=0;
    if (sscanf(line,"R%d %23s %23s %23s %23s %d %d %d %23s %d %d%n",
        &id,a,v,cur,pwm,state,fault,&remain,age,&mode,raw,&n)!=11) return false;
    return line[n]==0 && id==expected_id && *state>=0 && *state<=2 &&
        *fault>=0 && *fault<=7 && mode>=0 && mode<=2 && remain>=0 && remain<=100 &&
        *raw>=-1 && *raw<=4095;
}
uint16_t arm_crc(const char *data,size_t n) {
    uint16_t crc=0xffff;
    for (size_t i=0;i<n;i++) { crc^=(uint16_t)(uint8_t)data[i]<<8;
        for (int b=0;b<8;b++) crc=(uint16_t)((crc&0x8000)?(crc<<1)^0x1021:crc<<1); }
    return crc;
}
bool arm_bus_encode(uint16_t seq,const char *body,char *out,size_t cap) {
    int n=snprintf(out,cap,"@%u %s",seq,body);
    if (n<0 || (size_t)n+8>cap) return false;
    uint16_t crc=arm_crc(out,(size_t)n);
    snprintf(out+n,cap-(size_t)n," *%04X\n",crc); return strlen(out)-1<=ARM_BUS_LINE;
}
bool arm_bus_decode(char *line,uint16_t *seq,char **body) {
    char *star=strrchr(line,'*');
    if (!star || star==line || star[-1]!=' ' || strlen(star+1)!=4 || line[0]!='@') return false;
    for (int i=1;i<=4;i++) if (!isxdigit((unsigned char)star[i])) return false;
    unsigned long crc=strtoul(star+1,NULL,16);
    if (crc!=arm_crc(line,(size_t)(star-line-1))) return false;
    char *space=strchr(line,' '); if (!space || space>star-1) return false;
    *space=0; int n; if (!number(line+1,&n) || n<0 || n>65535) return false;
    star[-1]=0; *seq=(uint16_t)n; *body=space+1; return **body!=0;
}
int arm_line_feed(arm_line_t *s,unsigned char byte,int64_t now,size_t max,int64_t timeout) {
    if (s->used && now-s->started_us>timeout) { s->discard=true; s->used=0; }
    if (byte=='\n') {
        if (s->discard) { s->discard=false; s->used=0; return 0; }
        if (s->used && s->data[s->used-1]=='\r') s->used--;
        s->data[s->used]=0; bool nonempty=s->used>0; s->used=0;
        return nonempty?1:0;
    }
    if (s->discard) return 0;
    if (byte!='\r' && byte!='\t' && (byte<32 || byte>126)) { s->discard=true; s->used=0; return -1; }
    if (!s->used) s->started_us=now;
    if (s->used>=max || s->used>=ARM_BUS_LINE) { s->discard=true; s->used=0; return -1; }
    s->data[s->used++]=(char)byte; return 0;
}
int arm_bus_line_feed(arm_line_t *s,unsigned char byte,int64_t now) {
    /* 总线实测存在帧间NUL；不能让帧外字节污染下一条有效应答。 */
    if (byte=='@') { s->used=0; s->discard=false; }
    else if (!s->used) return 0;
    return arm_line_feed(s,byte,now,ARM_BUS_LINE,ARM_BUS_FRAME_US);
}
