/* 作用：验证真实控制核心的超时/限位/模式与协议边界；用法：scripts/test.sh，使用主机clang和运行时检查。 */
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
    assert(!arm_parse("R 3000 -2000",false,&c) && c.argc==2 && !c.id);
    assert(!arm_parse("U2 1510",false,&c) && c.id==2 && c.op=='U');
    assert(!arm_parse("R2 -2000 96",true,&c) && c.ttl_ms==96 && c.argc==1);
    assert(!arm_parse("R2",true,&c) && !c.argc);
    assert(!arm_parse("$save 2",false,&c) && c.op=='s' && c.id==2);
    assert(!arm_parse("$bus",false,&c) && c.op=='b' && !c.argc);
    assert(arm_parse("$bus 2",false,&c));
    assert(!arm_parse("$busraw",false,&c) && c.op=='h' && !c.argc);
    assert(arm_parse("$busraw 2",false,&c));
    const char *bad[]={"R 0","R1 9001","R1 12.5","R1 2147483648","U1 499","U1 2501", "P1 1 2","A0","R3 1","$save 1 2","X1 1","R1 nan","R1 0x1","R1 1junk"};
    for (unsigned i=0;i<sizeof(bad)/sizeof(*bad);i++) assert(arm_parse(bad[i],false,&c));
    assert(arm_parse("R2 100 0",true,&c));
    assert(arm_parse("R2 100 101",true,&c));
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
    j.integral=3; arm_move(&j,'R',1000,151000);
    assert(j.mode==POS && j.integral==0);
    j.integral=2; arm_move(&j,'R',1000,151000);
    assert(j.integral==2); /* 相同模式刷新不清积分。 */
    arm_move(&j,'U',1520,151000); assert(j.integral==0 && j.pwm_us==1520);
    arm_tick(&j,151000); assert(j.state==STOP && j.mode==NONE && j.pwm_us==1500);
    arm_move(&j,'R',100,251000); assert(j.state==RUN);
    arm_fault(&j,1); assert(j.state==FAULT && j.mode==NONE && j.pwm_us==1500);
    assert(arm_move_check(&j,'R',100));
}
static void limits_sensor_test(void) {
    arm_joint_t j=calibrated();
    assert(arm_move_check(&j,'R',9001));
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
        if (i==1 || i%10==0) arm_move(&j,'R',2000,now+100000);
        int raw=2048+(int)lroundf(angle*4096/360);
        arm_sample(&j,true,raw,now); arm_tick(&j,now);
        assert(j.state==RUN && j.mode==POS);
        assert(fabsf(j.ref_speed)<=j.cfg.vmax+0.001f && isfinite(j.integral));
        angle+=(j.pwm_us-1500)/ARM_PWM_US_PER_DPS*0.005f;
    }
    assert(angle>1 && angle<20);
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
int main(void) {
    parser_test(); timeout_modes_test(); limits_sensor_test(); trajectory_test(); framing_crc_test(); bus_query_test(); bus_resync_test();
    puts("PASS: parser, expiry, modes, limits, sensor loss, trajectory, framing, CRC, bus queries, captured UART replay"); return 0;
}
