/* 作用：验证免标定PWM、主轴独立运行和从板看门狗；用法：scripts/test.sh或主机编译时分别指定ARM_MASTER=1/0。 */
#include "arm_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static arm_joint_t calibrated(void) {
    arm_settings_t cfg; arm_defaults(&cfg); cfg.zero_raw=2048;
    cfg.zero_valid=cfg.direction_valid=cfg.neutral_valid=true;
    arm_joint_t j; arm_init(&j,&cfg); arm_sample(&j,true,2048,1000); arm_stop(&j); return j;
}
static void parser_test(void) {
    arm_command_t c;
    assert(!arm_parse("R 30.5 -20.25",false,&c) && c.argc==2 && !c.id);
    assert(!arm_parse("U2 1510",false,&c) && c.id==2 && c.op=='U');
    assert(!arm_parse("R2 -20.25",true,&c) && c.ttl_ms==100 && c.argc==1);
    assert(!arm_parse("R2",true,&c) && !c.argc);
    assert(!arm_parse("$save 2",false,&c) && c.op=='s' && c.id==2);
    assert(!arm_parse("$bus",false,&c) && c.op=='b' && !c.argc);
    assert(arm_parse("$bus 2",false,&c));
    assert(!arm_parse("$busraw",false,&c) && c.op=='h' && !c.argc);
    assert(arm_parse("$busraw 2",false,&c));
    const char *bad[]={"R 0","R1 90.01","R1 2147483648","U1 499","U1 2501", "P1 1 2","A0","R3 1","$save 1 2","X1 1","R1 nan","R1 0x1","R1 1junk"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) assert(arm_parse(bad[i],false,&c));
    assert(arm_parse("U2 1500 0",true,&c));
    assert(arm_parse("U2 1500 101",true,&c));
    arm_joint_t j=calibrated();
    assert(!arm_parse("A1 0 -2147483648 1",false,&c));
    assert(arm_setting(&j,&c)); /* INT_MIN不得造成abs溢出。 */
}
static void timeout_modes_test(void) {
    arm_joint_t j=calibrated();
    assert(!arm_move_check(&j,'U',1510)); arm_move(&j,'U',1510,101000);
    arm_sample(&j,true,2048,6000); arm_tick(&j,6000);
    assert(j.mode==PWM && j.pwm_us==1510 && j.state==RUN);
    arm_command_t query; assert(!arm_parse("R1",false,&query));
    assert(j.deadline_us==101000); /* 查询解析不改变期限或模式。 */
    j.integral=3; arm_move(&j,'R',10,151000);
    assert(j.mode==POS && j.integral==0);
    j.integral=2; arm_move(&j,'R',10,151000);
    assert(j.integral==2); /* 相同模式刷新不清积分。 */
    arm_move(&j,'U',1520,151000); assert(j.integral==0 && j.pwm_us==1520);
    arm_tick(&j,151000); assert(j.state==STOP && j.mode==NONE && j.pwm_us==1500);
    arm_move(&j,'R',1,251000); assert(j.state==RUN);
    arm_fault(&j,1); assert(j.state==FAULT && j.mode==NONE && j.pwm_us==1500);
    assert(arm_move_check(&j,'R',100));
}
static void limits_sensor_test(void) {
    arm_joint_t j=calibrated();
    assert(arm_move_check(&j,'R',90.01f));
    arm_sample(&j,true,3061,6000); /* 约89.03度，向外保护区。 */
    j.speed=0; arm_move(&j,'U',1600,106000); arm_tick(&j,6000);
    assert(j.pwm_us==1500);
    arm_move(&j,'U',1400,111000); arm_sample(&j,true,3061,11000); arm_tick(&j,11000);
    assert(j.pwm_us==1400); /* 可向限位以内退出。 */
    arm_sample(&j,false,0,16000); assert(j.state==FAULT && j.fault==1);
    j=calibrated(); arm_move(&j,'U',1550,101000); arm_tick(&j,22000);
    assert(j.state==FAULT); /* 传感器过期/控制循环超时不得继续输出。 */
    arm_settings_t blank; arm_defaults(&blank); arm_init(&j,&blank);
    assert(j.pwm_us==0 && arm_move_check(&j,'R',0));
    assert(arm_move_check(&j,'U',1500)); /* 没有可信编码器时不可候选中立试动。 */
}
static void trajectory_test(void) {
    arm_joint_t j=calibrated(); float angle=0;
    for (int i=1;i<=200;i++) {
        int64_t now=1000+i*5000;
        if (i==1 || i%10==0) arm_move(&j,'R',20,now+100000);
        int raw=2048+(int)lroundf(angle*4096/360);
        arm_sample(&j,true,raw,now); arm_tick(&j,now);
        assert(j.state==RUN && j.mode==POS);
        assert(fabsf(j.ref_speed)<=j.cfg.vmax+0.001f && isfinite(j.integral));
        angle+=(j.pwm_us-1500)/ARM_PWM_US_PER_DPS*0.005f;
    }
    assert(angle>1 && angle<20);
}
static void uncalibrated_pwm_test(void) {
    /* 覆盖三个标定标志的全部组合，不能只放开“全新板”这一种情况。 */
    for (int flags=0;flags<8;flags++) {
        arm_settings_t cfg; arm_defaults(&cfg); cfg.zero_raw=2048;
        cfg.zero_valid=(flags&1)!=0; cfg.direction_valid=(flags&2)!=0;
        cfg.neutral_valid=(flags&4)!=0;
        arm_joint_t j; arm_init(&j,&cfg); arm_sample(&j,true,2048,1000); arm_stop(&j);
        assert(!arm_move_check(&j,'U',1600));
        assert(j.cfg.neutral_valid && j.cfg.neutral_us==1500);
        assert((arm_move_check(&j,'R',0)==NULL)==((flags&3)==3));
        assert(arm_move_check(&j,'U',499) && arm_move_check(&j,'U',2501));
        assert(arm_move_check(&j,'U',1500.5f) && arm_move_check(&j,'U',NAN));
        arm_move(&j,'U',1600,101000);
        for (int i=1;i<=20;i++) {
            int64_t now=1000+i*5000LL;
            arm_sample(&j,true,2048,now); arm_tick(&j,now);
            if(i<20) assert(j.state==RUN && j.mode==PWM && j.pwm_us==1600);
        }
        assert(j.state==STOP && j.mode==NONE);
        assert(j.pwm_us==1500);
        arm_move(&j,'U',1400,201000); arm_stop(&j);
        assert(j.mode==NONE && j.pwm_us==1500);
        arm_fault(&j,1); assert(arm_move_check(&j,'U',1600));
    }
    arm_settings_t cfg; arm_defaults(&cfg);
    arm_joint_t j; arm_init(&j,&cfg); arm_sample(&j,true,2048,1000);
    /* 未标定的默认角度约-180度，不能误触发依赖方向/中立的制动。 */
    assert(!arm_move_check(&j,'U',1400)); arm_move(&j,'U',1400,101000);
    arm_sample(&j,true,2048,6000); arm_tick(&j,6000); assert(j.pwm_us==1400);
    arm_sample(&j,false,0,11000); assert(j.state==FAULT && j.pwm_us==1500);
    arm_init(&j,&cfg); arm_sample(&j,true,2048,1000); arm_bus_seen(&j,1000);
    arm_move(&j,'U',1600,201000);
    for(int i=1;i<=20;i++) {
        int64_t now=1000+i*5000LL;
        arm_sample(&j,true,2048,now); arm_tick(&j,now);
    }
#if ARM_MASTER
    assert(j.state==RUN && j.pwm_us==1600 && j.bus_deadline_us==0);
#else
    assert(j.state==STOP && j.pwm_us==1500); /* 从板失去总线仍停止。 */
#endif
    /* 旧NVS中未启用的中立值回退1500，已保存的自定义中立值保留。 */
    cfg.zero_raw=1962; cfg.zero_valid=true; cfg.neutral_us=1700; cfg.neutral_valid=false;
    arm_settings_t loaded; arm_defaults(&loaded);
    assert(arm_settings_decode(&loaded,3,&cfg,sizeof(cfg)));
    arm_init(&j,&loaded); arm_stop(&j);
    assert(j.cfg.zero_raw==1962 && j.cfg.zero_valid && j.cfg.neutral_valid && j.pwm_us==1500);
    cfg.neutral_us=1499; cfg.neutral_valid=true;
    assert(arm_settings_decode(&loaded,3,&cfg,sizeof(cfg)));
    arm_init(&j,&loaded); arm_stop(&j); assert(j.pwm_us==1499);
}
static void framing_crc_test(void) {
    assert(arm_crc("123456789",9)==0x29b1);
    char packet[194]; assert(arm_bus_encode(42,"R 3000 -2000 96",packet,sizeof(packet)));
    assert(!strcmp(packet,"@42 R 3000 -2000 96 *58CA\n"));
    packet[strlen(packet)-1]=0; uint16_t seq; char *body;
    assert(arm_bus_decode(packet,&seq,&body) && seq==42 && !strcmp(body,"R 3000 -2000 96"));
    arm_bus_encode(42,"R2",packet,sizeof(packet)); packet[2]='9'; packet[strlen(packet)-1]=0;
    assert(!arm_bus_decode(packet,&seq,&body));
    arm_line_t line={0}; int n=0; const char *s="R1 100\r\nR2\n";
    for (size_t i=0;i<strlen(s);i++) if (arm_line_feed(&line,(unsigned char)s[i],i*100,160,100000)==1) n++;
    assert(n==2);
    memset(&line,0,sizeof(line));
    for (int i=0;i<161;i++) arm_line_feed(&line,'R',i,160,100000);
    assert(!arm_line_feed(&line,'\n',200,160,100000));
    assert(!arm_line_feed(&line,'R',300,160,100000));
    assert(!arm_line_feed(&line,'1',100301,160,100000));
    assert(!arm_line_feed(&line,'\n',100302,160,100000));
    assert(arm_line_feed(&line,0,200000,160,100000)==-1); /* 丢弃状态下无命令执行。 */
}
static void bus_query_test(void) {
    /* 模拟真实查询经过编码、逐字节组帧、CRC解码和从板命令解析。 */
    const char *queries[]={"R2","$raw 2"};
    for (unsigned q=0;q<sizeof(queries)/sizeof(*queries);q++) {
        char packet[194]; arm_line_t line={0};
        assert(arm_bus_encode(73,queries[q],packet,sizeof(packet)));
        int complete=0;
        for (size_t i=0;i<strlen(packet);i++)
            complete=arm_bus_line_feed(&line,(unsigned char)packet[i],i*87);
        assert(complete==1);
        uint16_t seq; char *body; arm_command_t c;
        assert(arm_bus_decode(line.data,&seq,&body) && seq==73);
        assert(!arm_parse(body,true,&c) && c.id==2 && !c.argc);
        assert((q==0 && c.op=='R') || (q==1 && c.op=='d'));
        const char *response=q?"$raw 2 2119 3708 4095 0 0":"R2 NA NA NA 0 2 2 0 2 0";
        assert(arm_bus_encode(seq,response,packet,sizeof(packet)));
        packet[strlen(packet)-1]=0;
        assert(arm_bus_decode(packet,&seq,&body) && seq==73 && !strcmp(body,response));
    }
}
static void bus_resync_test(void) {
    /* 回放用户抓取的真实数据：四帧均CRC正确，但旧实现被帧前NUL带入丢弃状态。 */
    const char *traces[]={
        "4E41204E41203020322032203020302030202A344530390A0040323232205232204E41204E41204E41203020322032203020322030202A454542350A0040323233205232204E41204E41204E41203020322032203020342030202A313941310A",
        "3020312030202A454435330A0040323935205232204E41204E41204E41203020322032203020332030202A433638370A0040323936205232204E41204E41204E41203020322032203020302030202A353130420A0040323937205232204E4120"
    };
    const uint16_t expected[]={222,223,295,296}; unsigned valid=0,old_valid=0;
    for (unsigned t=0;t<2;t++) {
        arm_line_t line={0},old={0};
        for (size_t i=0;i<strlen(traces[t]);i+=2) {
            unsigned b; assert(sscanf(traces[t]+i,"%2x",&b)==1);
            uint16_t seq; char *body;
            if (arm_line_feed(&old,(unsigned char)b,(int64_t)i*44,ARM_BUS_LINE,ARM_BUS_FRAME_US)==1 &&
                arm_bus_decode(old.data,&seq,&body)) old_valid++;
            if (arm_bus_line_feed(&line,(unsigned char)b,(int64_t)i*44)==1) {
                assert(arm_bus_decode(line.data,&seq,&body));
                assert(valid<4 && seq==expected[valid++] && !strncmp(body,"R2 ",3));
            }
        }
    }
    assert(valid==4 && old_valid==0);
    char clean[194]; assert(arm_bus_encode(42,"R2",clean,sizeof(clean)));
    arm_line_t damaged={0}; int accepted=0;
    for (size_t i=0;i<strlen(clean);i++) {
        if (i==5) assert(arm_bus_line_feed(&damaged,0,(int64_t)i*100)==-1);
        if (arm_bus_line_feed(&damaged,(unsigned char)clean[i],(int64_t)i*100+1)==1) accepted++;
    }
    assert(accepted==0); /* 即使删除NUL后CRC会正确，也不得接受遭到帧内污染的数据。 */
    /* 帧内NUL不能被简单删除后接受；缺换行、超时、CRC错误后可恢复接收下一帧。 */
    const char *bad[]={"@1 R\0002 *FFFF\n", "@1 truncated", "@1 R2 *0000\n"};
    const size_t sizes[]={sizeof("@1 R\0002 *FFFF\n")-1,strlen(bad[1]),strlen(bad[2])};
    for (unsigned t=0;t<3;t++) {
        arm_line_t line={0}; uint16_t seq; char *body; int64_t now=0;
        for (size_t i=0;i<sizes[t];i++,now+=87)
            if (arm_bus_line_feed(&line,(unsigned char)bad[t][i],now)==1)
                assert(!arm_bus_decode(line.data,&seq,&body));
        char packet[194]; assert(arm_bus_encode(42,"R2",packet,sizeof(packet)));
        now+=ARM_BUS_FRAME_US+1;
        int result=0;
        for (size_t i=0;i<strlen(packet);i++,now+=87)
            result=arm_bus_line_feed(&line,(unsigned char)packet[i],now);
        assert(result==1 && arm_bus_decode(line.data,&seq,&body) && seq==42 && !strcmp(body,"R2"));
    }
}

static void float_settings_test(void) {
    arm_command_t c; arm_joint_t j=calibrated();
    const char *good[]={"R1 30.5","R -90 90","R1 -2.025e1","R1 +.5","P1 1.25 .01 0","J1 10.5 30.25 1500","H1 0","H2 1","H1"};
    for (unsigned i=0;i<sizeof(good)/sizeof(*good);i++) assert(!arm_parse(good[i],false,&c));
    const char *bad[]={"R1 nan","R1 NaN","R1 inf","R1 -inf","R1 1e99","R1 1e-99","R1 0x1p0","R1 .","R1 1e","R1 1.0x","R1 90.01","R1 -90.01","P1 nan 0 0","J1 10 30 1500.5","U1 1500.0","H1 1.0","H1 2","H 1","H1 0 1","$save 2147483647","$save -2147483648"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) assert(arm_parse(bad[i],false,&c));
    assert(!arm_parse("P1 1.25 0.01 0.5",false,&c)); assert(!arm_setting(&j,&c));
    assert(j.cfg.kp==1.25f && j.cfg.ki==.01f && j.cfg.kd==.5f);
    assert(!arm_parse("J1 10.5 30.25 1499",false,&c)); assert(!arm_setting(&j,&c));
    assert(j.cfg.vmax==10.5f && j.cfg.amax==30.25f && j.cfg.neutral_us==1499);
    const char *range[]={"P1 -0.1 0 0","P1 0 100.1 0","P1 0 0 10.1","J1 .001 30 1500","J1 90.01 30 1500","J1 10 360.1 1500"};
    for (unsigned i=0;i<sizeof(range)/sizeof(*range);i++) {
        arm_settings_t before=j.cfg;
        assert(!arm_parse(range[i],false,&c) && arm_setting(&j,&c));
        assert(!memcmp(&before,&j.cfg,sizeof(before)));
    }
    assert(!arm_parse("H1 1",false,&c)); assert(!arm_setting(&j,&c) && j.cfg.hold==1);
    arm_move(&j,'R',10,0); assert(arm_setting(&j,&c));
    arm_stop(&j); assert(!arm_setting(&j,&c));
    arm_fault(&j,1); assert(arm_setting(&j,&c));
}
/* 用真实AS5600量化及理想脉宽/速度映射驱动真实控制核心；不是硬件精度证明。 */
static void simulate(arm_joint_t *j,float *angle,int64_t now) {
    int raw=2048+(int)lroundf(*angle*4096/360);
    arm_sample(j,true,raw,now); arm_tick(j,now);
    *angle+=(j->pwm_us-j->cfg.neutral_us)/(ARM_PWM_US_PER_DPS*j->cfg.motor_dir)*.005f;
}
static void position_modes_test(void) {
    arm_joint_t j=calibrated();
    arm_move(&j,'R',0,0);
    for (int i=1;i<=40;i++) {arm_sample(&j,true,2048,1000+i*5000LL);arm_tick(&j,1000+i*5000LL);}
    assert(j.state==RUN); /* 195ms还不能停。 */
    int64_t first=j.settled_us;
    arm_move(&j,'R',0,0); assert(j.settled_us==first);
    arm_sample(&j,true,2048,206000);arm_tick(&j,206000);
    assert(j.state==STOP && j.mode==NONE && j.pwm_us==1500);
    arm_sample(&j,true,2060,211000);arm_tick(&j,211000);
    assert(j.state==STOP && j.pwm_us==1500); /* 推偏不重新纠正。 */
    j=calibrated(); arm_move(&j,'R',0,0);
    for (int i=1;i<=30;i++) {arm_sample(&j,true,2048,1000+i*5000LL);arm_tick(&j,1000+i*5000LL);}
    j.arrival_speed=2; arm_tick(&j,156000); assert(j.settled_us==0);
    j.arrival_speed=0;arm_tick(&j,161000);assert(j.settled_us==161000);
    arm_move(&j,'R',.25f,0); assert(j.settled_us==0 && j.target==.25f);
    j=calibrated(); j.cfg.hold=1; float a=0;arm_move(&j,'R',2.5f,0);
    for (int i=1;i<=1600;i++) simulate(&j,&a,1000+i*5000LL);
    assert(j.state==RUN && j.mode==POS && fabsf(a-2.5f)<.2f && j.deadline_us==0);
    /* 缓慢施加外力，避免不可信编码器跳变触发保护。 */
    for (int i=1601;i<=1700;i++) {a-=.02f; simulate(&j,&a,1000+i*5000LL);}
    assert(j.state==RUN && a<2.0f);
    for (int i=1701;i<=3000;i++) simulate(&j,&a,1000+i*5000LL);
    assert(fabsf(a-2.5f)<.2f && j.state==RUN);
    arm_move(&j,'U',1510,15101000);assert(j.mode==PWM && j.integral==0);
    arm_move(&j,'R',-2.5f,0);assert(j.mode==POS && j.deadline_us==0);
    for (int i=3001;i<=4600;i++) simulate(&j,&a,1000+i*5000LL);
    assert(fabsf(a+2.5f)<.2f && j.state==RUN);
    arm_stop(&j);assert(j.state==STOP && j.pwm_us==1500);
}
static void independent_test(void) {
    arm_joint_t j[]={calibrated(),calibrated()}; float a[]={0,0}; int stopped[]={0,0};
    arm_command_t c;assert(!arm_parse("R 30.5 -10.25",false,&c));
    for (int k=0;k<2;k++) arm_move(&j[k],'R',c.val[k],0);
    for (int i=1;i<=2400;i++) for (int k=0;k<2;k++) {
        simulate(&j[k],&a[k],1000+i*5000LL);
        assert(j[k].state!=FAULT);
        if (!stopped[k] && j[k].state==STOP) stopped[k]=i;
    }
    assert(stopped[0]>stopped[1] && stopped[1]>0);
    assert(fabsf(a[0]-30.5f)<.5f && fabsf(a[1]+10.25f)<.5f);
    j[1].cfg.hold=1; arm_move(&j[0],'R',-15.25f,0);arm_move(&j[1],'R',-10.25f,0);
    for (int i=2401;i<=4800;i++) for (int k=0;k<2;k++) simulate(&j[k],&a[k],1000+i*5000LL);
    assert(j[0].state==STOP && j[1].state==RUN);
    assert(fabsf(a[0]+15.25f)<.5f && fabsf(a[1]+10.25f)<.2f);
}
static void watchdog_test(void) {
    for (int hold=0;hold<=1;hold++) {
        arm_joint_t j=calibrated();j.cfg.hold=hold;arm_bus_seen(&j,1000);arm_move(&j,'R',30,0);
        for(int i=1;i<=20;i++){arm_sample(&j,true,2048,1000+i*5000LL);arm_tick(&j,1000+i*5000LL);}
#if ARM_MASTER
        assert(j.state==RUN && j.mode==POS && j.bus_deadline_us==0);
#else
        assert(j.state==STOP && j.mode==NONE && j.pwm_us==1500);
#endif
        arm_bus_seen(&j,106000);arm_sample(&j,true,2048,106000);arm_tick(&j,106000);
#if ARM_MASTER
        assert(j.state==RUN); /* 主轴在从板离线及重连期间保持原目标。 */
#else
        assert(j.state==STOP); /* 从板通信恢复不能恢复旧动作。 */
#endif
        arm_move(&j,'R',30,0);
        for(int i=22;i<=80;i++) {
            int64_t now=1000+i*5000LL;
            if(i%10==0) arm_bus_seen(&j,now);
            arm_sample(&j,true,2048,now);arm_tick(&j,now);assert(j.state==RUN);
        }
        arm_sample(&j,false,0,406000);assert(j.state==FAULT && j.pwm_us==1500);
    }
    arm_joint_t j=calibrated();arm_move(&j,'U',1510,101000);
    for(int i=1;i<=30;i++) {
        int64_t now=1000+i*5000LL;
        arm_bus_seen(&j,now);arm_sample(&j,true,2048,now);arm_tick(&j,now);
    }
    assert(j.state==STOP); /* 总线查询不能延长PWM期限。 */
    j=calibrated();arm_move(&j,'U',1510,101000);
    for(int i=1;i<=29;i++) {
        int64_t now=1000+i*5000LL;
        if(i==10) arm_move(&j,'U',1520,now+100000);
        arm_sample(&j,true,2048,now);arm_tick(&j,now);assert(j.state==RUN);
    }
    arm_sample(&j,true,2048,151000);arm_tick(&j,151000);assert(j.state==STOP);
}
static void migration_test(void) {
    arm_settings_t old=calibrated().cfg,loaded;old.kp=1.25f;old.ki=.01f;old.kd=.5f;
    old.hold=1;arm_defaults(&loaded);
    assert(arm_settings_decode(&loaded,2,&old,40));
    assert(loaded.hold==0 && loaded.zero_raw==2048 && loaded.zero_valid && loaded.direction_valid && loaded.neutral_valid);
    assert(loaded.kp==1.25f && loaded.ki==.01f && loaded.kd==.5f);
    assert(arm_settings_decode(&loaded,3,&old,sizeof(old)) && loaded.hold==1);
    old.hold=2;assert(!arm_settings_decode(&loaded,3,&old,sizeof(old)) && loaded.hold==1);
    old.hold=1;old.kp=NAN;assert(!arm_settings_decode(&loaded,3,&old,sizeof(old)));
    assert(!arm_settings_decode(&loaded,2,&old,39));assert(!arm_settings_decode(&loaded,4,&old,sizeof(old)));
    unsigned char blob[sizeof(old)];old.kp=2;memcpy(blob,&old,sizeof(old));blob[offsetof(arm_settings_t,zero_valid)]=255;
    assert(!arm_settings_decode(&loaded,3,blob,sizeof(blob)));
}
static void bus_motion_test(void) {
    const char *commands[]={"R 30.5 -20.25","R2 -20.25","U 1510 1490 93","U2 1490 93","H2 1","P2 1.25 .01 0","J2 10.5 30.25 1500"};
    for(unsigned k=0;k<sizeof(commands)/sizeof(*commands);k++) {
        char frame[194];arm_line_t line={0};uint16_t seq;char *body;arm_command_t c;
        assert(arm_bus_encode(12,commands[k],frame,sizeof(frame)));
        int result=0;for(size_t i=0;i<strlen(frame);i++) result=arm_bus_line_feed(&line,(unsigned char)frame[i],i*87);
        assert(result==1 && arm_bus_decode(line.data,&seq,&body));assert(!arm_parse(body,true,&c));
        if(c.op=='R') assert(c.val[c.id?0:1]==-20.25f);
        if(c.op=='U') assert(c.ttl_ms==93);
    }
    arm_command_t c;assert(arm_parse("R2 -20.25 96",true,&c));
    assert(arm_parse("U2 1490",true,&c));assert(arm_parse("U2 1490 93.0",true,&c));
}
int main(void) {
    uncalibrated_pwm_test(); float_settings_test(); position_modes_test(); independent_test(); watchdog_test(); migration_test(); bus_motion_test(); parser_test(); timeout_modes_test(); limits_sensor_test(); trajectory_test(); framing_crc_test(); bus_query_test(); bus_resync_test();
    puts("PASS: float API, arrival/hold, independent axes, bus watchdog, settings migration, parser, expiry, modes, limits, sensor loss, trajectory, framing, CRC, bus queries, captured UART replay"); return 0;
}
