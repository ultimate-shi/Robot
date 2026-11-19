/* 作用：ESP32-C3独立主轴、0.4原始编码器反馈及主从总线；用法：master/slave配套构建，R状态末项为raw，J命令已移除。 */
#include "arm_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#define OUT_CAP 256
static arm_joint_t joint;
static SemaphoreHandle_t joint_lock,bus_lock;
static adc_oneshot_unit_handle_t adc;
static int current_raw=-1,temp_raw=-1;
static bool flash_busy;
static uint16_t bus_seq;
static int64_t bus_quiet_until;
static struct { char line[OUT_CAP]; int64_t received; bool online; } peer;
/* 仅统计本板UART，不计USB；上电清零，USB只读查询不影响总线或电机。 */
static struct { uint32_t rx,frame,bad,hit,tx,txerr,mismatch; uint16_t wait,seen; } bus_stats;
static unsigned char bus_trace[ARM_BUS_TRACE_BYTES];
static size_t bus_trace_next,bus_trace_used;
_Static_assert(ARM_BUS_TRACE_BYTES*2+16<=OUT_CAP,"UART trace exceeds USB response capacity");
static void lock(void) { xSemaphoreTake(joint_lock,portMAX_DELAY); }
static void unlock(void) { xSemaphoreGive(joint_lock); }
static int64_t now_us(void) { return esp_timer_get_time(); }
static void bus_record_rx(unsigned char byte) {
    lock();
    bus_stats.rx++;
    bus_trace[bus_trace_next]=byte;
    bus_trace_next=(bus_trace_next+1)%ARM_BUS_TRACE_BYTES;
    if (bus_trace_used<ARM_BUS_TRACE_BYTES) bus_trace_used++;
    unlock();
}
static void bus_raw_diagnostics(char *out) {
    unsigned char snapshot[ARM_BUS_TRACE_BYTES];
    lock(); size_t used=bus_trace_used;
    size_t start=(bus_trace_next+ARM_BUS_TRACE_BYTES-used)%ARM_BUS_TRACE_BYTES;
    for (size_t i=0;i<used;i++) snapshot[i]=bus_trace[(start+i)%ARM_BUS_TRACE_BYTES];
    unlock();
    size_t n=(size_t)snprintf(out,OUT_CAP,"$busraw ");
    if (!used) snprintf(out+n,OUT_CAP-n,"EMPTY");
    else for (size_t i=0;i<used;i++) snprintf(out+n+i*2,OUT_CAP-n-i*2,"%02X",snapshot[i]);
}
static void bus_diagnostics(char *out) {
    lock();
    snprintf(out,OUT_CAP,"$bus rx=%lu frame=%lu bad=%lu hit=%lu tx=%lu txerr=%lu wait=%u seen=%u mismatch=%lu",
        (unsigned long)bus_stats.rx,(unsigned long)bus_stats.frame,(unsigned long)bus_stats.bad,
        (unsigned long)bus_stats.hit,(unsigned long)bus_stats.tx,(unsigned long)bus_stats.txerr,
        (unsigned)bus_stats.wait,(unsigned)bus_stats.seen,(unsigned long)bus_stats.mismatch);
    unlock();
}
static bool bus_decode_counted(char *line,uint16_t *seq,char **body) {
    bool good=arm_bus_decode(line,seq,body);
    lock();
    if (good) { bus_stats.frame++; bus_stats.seen=*seq; } else bus_stats.bad++;
    unlock();
    return good;
}
static bool bus_write_packet(const char *packet) {
    size_t size=strlen(packet);
    int written=uart_write_bytes(UART_NUM_1,packet,size);
    esp_err_t done=uart_wait_tx_done(UART_NUM_1,pdMS_TO_TICKS(25));
    bool good=written==(int)size && done==ESP_OK;
    lock(); if (good) bus_stats.tx++; else bus_stats.txerr++; unlock();
    return good;
}
static void fail_reply(char *out,const char *name,const char *err) {
    snprintf(out,OUT_CAP,"!%s %s",*name?name:"?",err);
}
static void ok_reply(char *out,const arm_command_t *c) {
    if (strchr("zsc",c->op)) snprintf(out,OUT_CAP,"%s %d OK",c->name,c->id);
    else snprintf(out,OUT_CAP,"%s OK",c->name);
}
static void load_settings(arm_settings_t *c) {
    arm_defaults(c); nvs_handle_t h;
    if (nvs_open_from_partition("arm_cfg","arm_joint",NVS_READONLY,&h)!=ESP_OK) return;
    unsigned char stored[sizeof(arm_settings_t)]; size_t n=sizeof(stored); uint32_t schema=0;
    if (nvs_get_u32(h,"schema",&schema)==ESP_OK &&
        nvs_get_blob(h,"settings",stored,&n)==ESP_OK) arm_settings_decode(c,schema,stored,n);
    nvs_close(h);
}
static bool save_settings(const arm_settings_t *c) {
    nvs_handle_t h; if (nvs_open_from_partition("arm_cfg","arm_joint",NVS_READWRITE,&h)!=ESP_OK) return false;
    esp_err_t e=nvs_set_blob(h,"settings",c,sizeof(*c));
    if (e==ESP_OK) e=nvs_set_u32(h,"schema",3);
    if (e==ESP_OK) e=nvs_commit(h);
    nvs_close(h); return e==ESP_OK;
}
static void local_state(char *out) {
    lock(); arm_joint_t j=joint; unlock();
    int64_t now=now_us(); int age=j.sample_us?(int)((now-j.sample_us)/1000):-1;
    int remaining=j.state==RUN && j.mode==PWM?(int)fmax(0,(j.deadline_us-now)/1000):0;
    char a[24]="NA",v[24]="NA",ag[24]="NA";
    if (j.sensor_valid && j.cfg.zero_valid && j.cfg.direction_valid && age>=0 && age<=20) {
        snprintf(a,sizeof(a),"%.4f",(double)j.angle);
        snprintf(v,sizeof(v),"%.4f",(double)j.arrival_speed);
    }
    if (age>=0) snprintf(ag,sizeof(ag),"%d",age);
    int fault=j.fault;
    /* 未标定仍可运行限时PWM；此时状态应如实报告RUN/PWM而非合成未标定故障。 */
    if (!fault && !(j.state==RUN && j.mode==PWM) &&
        (!j.cfg.zero_valid || !j.cfg.direction_valid || !j.cfg.neutral_valid)) fault=2;
    int raw=j.sensor_valid && age>=0 && age<=20?j.raw:-1;
    snprintf(out,OUT_CAP,"R%d %s %s NA %d %d %d %d %s %d %d",ARM_NODE_ID,a,v,j.pwm_us,
        fault?FAULT:j.state,fault,remaining,ag,j.mode,raw);
}
static void local_command(const arm_command_t *c,int64_t deadline,char *out) {
    out[0]=0;
    if (c->op=='R' && !c->argc) { local_state(out); return; }
    lock();
    const char *err=NULL;
    if (c->op=='X') { arm_stop(&joint); }
    else if ((c->op=='R' || c->op=='U') && c->argc) {
        float value=c->val[c->id?0:ARM_NODE_ID-1];
        err=flash_busy?"BUSY":arm_move_check(&joint,c->op,value);
        if (!err && c->op=='U' && deadline<=now_us()) err="TIMEOUT";
        if (!err) arm_move(&joint,c->op,value,deadline);
    } else if (strchr("APH",c->op)) {
        if (c->argc) err=arm_setting(&joint,c);
        else if (c->op=='A') snprintf(out,OUT_CAP,"%s %d %d %d",c->name,joint.cfg.zero_raw,joint.cfg.encoder_dir,joint.cfg.motor_dir);
        else if (c->op=='H') snprintf(out,OUT_CAP,"%s %d",c->name,joint.cfg.hold);
        else snprintf(out,OUT_CAP,"%s %.6f %.6f %.6f",c->name,(double)joint.cfg.kp,
                (double)joint.cfg.ki,(double)joint.cfg.kd);
    } else if (c->op=='z') {
        if (joint.state==RUN) err="BUSY";
        else if (!joint.sensor_valid) err="FAULT";
        else { joint.cfg.zero_raw=joint.raw; joint.cfg.zero_valid=true;
            joint.angle=joint.speed=joint.arrival_speed=0; arm_stop(&joint); }
    } else if (c->op=='c') {
        if (joint.state==RUN) err="BUSY";
        else if (!joint.sensor_valid || !joint.cfg.zero_valid || !joint.cfg.direction_valid ||
                 !joint.cfg.neutral_valid || fabsf(joint.angle)>ARM_LIMIT_DEG) err="FAULT";
        else { joint.fault=0; arm_stop(&joint); }
    } else if (c->op=='d') {
        snprintf(out,OUT_CAP,"$raw %d %d %d %d %d %d",ARM_NODE_ID,joint.sensor_valid?joint.raw:-1,
            current_raw,temp_raw,joint.cfg.zero_valid,joint.cfg.neutral_valid);
    } else if (c->op=='s') {
        if (joint.state==RUN) err="BUSY";
        else { arm_settings_t cfg=joint.cfg; flash_busy=true; unlock();
            bool saved=save_settings(&cfg); lock(); flash_busy=false; if (!saved) err="FAULT"; }
    } else err="CMD";
    unlock();
    if (err) fail_reply(out,c->name,err);
    else if (!out[0]) ok_reply(out,c);
}
static void pwm_write(int pulse) {
    static int last=-1;
    if (pulse==last) return;
    if (!pulse) ledc_stop(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
    else { uint32_t duty=(uint32_t)((uint64_t)pulse*16384/20000);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0); }
    last=pulse;
}
static bool encoder_read(int *raw) {
    uint8_t reg=0x0b,b[3];
    esp_err_t e=i2c_master_write_read_device(I2C_NUM_0,ARM_AS5600_ADDR,&reg,1,b,3,pdMS_TO_TICKS(2));
    if (e!=ESP_OK || !(b[0]&0x20) || (b[0]&0x18)) return false;
    *raw=((b[1]&0x0f)<<8)|b[2]; return true;
}
static void control_task(void *arg) {
    (void)arg; TickType_t wake=xTaskGetTickCount(); unsigned count=0;
    while (true) {
        int raw=0; bool valid=encoder_read(&raw); int64_t now=now_us();
        lock(); arm_sample(&joint,valid,raw,now); arm_tick(&joint,now); int pwm=joint.pwm_us; unlock();
        pwm_write(pwm);
        if (++count%4==0) {
            int a=-1,b=-1; adc_oneshot_read(adc,ADC_CHANNEL_0,&a); adc_oneshot_read(adc,ADC_CHANNEL_1,&b);
            lock(); current_raw=a; temp_raw=b; unlock();
        }
        vTaskDelayUntil(&wake,pdMS_TO_TICKS(ARM_CONTROL_MS));
    }
}
static uint16_t send_bus(const char *body) {
    char packet[ARM_BUS_LINE+2]; uint16_t seq=++bus_seq;
    if (!seq) seq=++bus_seq;
    if (!arm_bus_encode(seq,body,packet,sizeof(packet))) return 0;
    return bus_write_packet(packet)?seq:0;
}
static bool receive_bus(uint16_t expected,char *out) {
    arm_line_t line={0}; int64_t end=now_us()+ARM_REPLY_US;
    lock(); bus_stats.wait=expected; unlock();
    while (now_us()<end) {
        unsigned char b;
        if (uart_read_bytes(UART_NUM_1,&b,1,pdMS_TO_TICKS(1))!=1) continue;
        bus_record_rx(b);
        if (arm_bus_line_feed(&line,b,now_us())==1) {
            uint16_t seq; char *body;
            if (bus_decode_counted(line.data,&seq,&body)) {
                lock();
                if (seq==expected) bus_stats.hit++; else bus_stats.mismatch++;
                unlock();
                if (seq==expected) { snprintf(out,OUT_CAP,"%s",body); return true; }
            }
        }
    }
    bus_quiet_until=now_us()+ARM_QUIET_US; return false;
}
static bool transaction(const char *body,char *out) {
    /* 调用者持有bus_lock；迟到应答由seq过滤，不清空整个串口。 */
    while (now_us()<bus_quiet_until) vTaskDelay(pdMS_TO_TICKS(1));
    uint16_t seq=send_bus(body); return seq && receive_bus(seq,out);
}
static bool peer_parse(const char *s,int *state,int *fault) {
    int raw;
    return arm_feedback_parse(s,2,state,fault,&raw);
}
static void peer_state(char *out) {
    lock(); char cached[OUT_CAP]; snprintf(cached,sizeof(cached),"%s",peer.line);
    int64_t received=peer.received; unlock();
    int age=received?(int)((now_us()-received)/1000):-1;
    if (age<0) { snprintf(out,OUT_CAP,"R2 NA NA NA NA 2 5 0 NA 0 -1"); return; }
    if (age>100) { snprintf(out,OUT_CAP,"R2 NA NA NA NA 2 5 0 %d 0 -1",age); return; }
    char a[24],v[24],cur[24],pwm[24],node_age[24]; int state,fault,remaining,mode,raw;
    if (sscanf(cached,"R2 %23s %23s %23s %23s %d %d %d %23s %d %d",a,v,cur,pwm,&state,&fault,&remaining,node_age,&mode,&raw)!=10) {
        snprintf(out,OUT_CAP,"R2 NA NA NA NA 2 5 0 %d 0 -1",age); return;
    }
    /* 把从板采样龄期合并到网关缓存龄期，主机收到的是保守总龄期。 */
    int total_age=age+atoi(node_age);
    snprintf(out,OUT_CAP,"R2 %s %s %s %s %d %d %d %d %d %d",a,v,cur,pwm,state,fault,
        remaining>age?remaining-age:0,total_age,mode,total_age<=100?raw:-1);
}
static void poll_task(void *arg) {
    (void)arg;
    while (true) {
        char reply[OUT_CAP]; int state=0,fault=0;
        if (xSemaphoreTake(bus_lock,pdMS_TO_TICKS(2))==pdTRUE) {
            bool good=transaction("R2",reply) && peer_parse(reply,&state,&fault);
            lock();
            if (good) { snprintf(peer.line,sizeof(peer.line),"%s",reply); peer.received=now_us(); peer.online=true; }
            else if (peer.received && now_us()-peer.received>100000) peer.online=false;
            bool running=joint.state==RUN;
            bool blocks=fault && !(fault==2 && joint.mode==PWM);
            /* 离线只更新从轴状态；收到从轴真实故障时保留现有整机故障联锁。 */
            if (good && blocks && running) arm_stop(&joint);
            unlock();
            if (good && blocks && running) send_bus("X");
            xSemaphoreGive(bus_lock);
        }
        vTaskDelay(pdMS_TO_TICKS(ARM_POLL_US/1000));
    }
}
static void usb_line(const char *line) {
    usb_serial_jtag_write_bytes(line,strlen(line),pdMS_TO_TICKS(20));
    usb_serial_jtag_write_bytes("\n",1,pdMS_TO_TICKS(20));
}
static void master_request(const char *input) {
    arm_command_t c; const char *err=arm_parse(input,false,&c); char out[OUT_CAP];
    int64_t deadline=now_us()+ARM_COMMAND_US;
    if (err) { fail_reply(out,c.name,err); usb_line(out); return; }
    if (c.op=='i') { usb_line("$info ARM 2 1 2"); return; }
    if (c.op=='v') { usb_line("$version 0.4"); return; }
    if (c.op=='b') { bus_diagnostics(out); usb_line(out); return; }
    if (c.op=='h') { bus_raw_diagnostics(out); usb_line(out); return; }
    if (c.op=='R' && !c.argc) {
        if (!c.id || c.id==1) { local_state(out); usb_line(out); }
        if (!c.id || c.id==2) { peer_state(out); usb_line(out); }
        if (!c.id) usb_line("R END");
        return;
    }
    bool motion=(c.op=='R' || c.op=='U') && c.argc;
    bool local=c.id==0 || c.id==1, remote=c.id==0 || c.id==2;
    if (c.op=='X' && local) { lock(); arm_stop(&joint); unlock(); }
    if (motion) {
        lock();
        if (local) err=flash_busy?"BUSY":arm_move_check(&joint,c.op,c.val[0]);
        bool online=peer.online && now_us()-peer.received<=100000;
        int ps=0,pf=0;
        if (remote && (!online || !peer_parse(peer.line,&ps,&pf))) err="TIMEOUT";
        else if (remote && pf && !(c.op=='U' && pf==2)) err="FAULT";
        unlock();
        if (err) { fail_reply(out,c.name,err); usb_line(out); return; }
    }
    if (!remote) { local_command(&c,deadline,out); usb_line(out); return; }
    xSemaphoreTake(bus_lock,portMAX_DELAY);
    if (remote) {
        char body[ARM_USB_LINE+20];
        if (motion && c.op=='R') { if (!send_bus(input)) err="TIMEOUT"; }
        else if (motion) {
            /* 先估计最坏帧长，扣除排队和串口在线时间，防止下游重新获得完整100ms。 */
            int wire_us=(int)((strlen(input)+24)*10*1000000/ARM_BUS_BAUD)+1000;
            int ttl=(int)((deadline-now_us()-wire_us)/1000);
            if (ttl<1) err="TIMEOUT";
            else { snprintf(body,sizeof(body),"%s %d",input,ttl); if (!send_bus(body)) err="TIMEOUT"; }
        } else if (c.op=='X') { if (!send_bus(input)) err="TIMEOUT"; }
        else {
            bool good=transaction(input,out);
            /* 错误也必须回显本请求命令，配置命令还需匹配ID。 */
            char expected[32];
            if (strchr("zscd",c.op)) snprintf(expected,sizeof(expected),"%s %d",c.name,c.id);
            else snprintf(expected,sizeof(expected),"%s",c.name);
            size_t n=strlen(expected);
            bool match=good && ((strncmp(out,expected,n)==0 && (out[n]==0 || out[n]==' ')) ||
                (out[0]=='!' && strncmp(out+1,c.name,strlen(c.name))==0 && out[1+strlen(c.name)]==' '));
            if (!match) err="TIMEOUT";
            xSemaphoreGive(bus_lock);
            if (err) fail_reply(out,c.name,err);
            usb_line(out); return;
        }
    }
    xSemaphoreGive(bus_lock);
    if (err) { fail_reply(out,c.name,err); usb_line(out); return; }
    if (local) local_command(&c,deadline,out); else ok_reply(out,&c);
    usb_line(out);
}
static void usb_task(void *arg) {
    (void)arg; arm_line_t line={0};
    while (true) {
        unsigned char b;
        if (usb_serial_jtag_read_bytes(&b,1,pdMS_TO_TICKS(10))!=1) continue;
        int got=arm_line_feed(&line,b,now_us(),ARM_USB_LINE,100000);
        if (got<0) usb_line("!? ARG");
        else if (got==1) {
#if ARM_MASTER
            master_request(line.data);
#else
            /* 从板USB只开放只读检查；运动与设置仅由总线master控制。 */
            arm_command_t c; const char *e=arm_parse(line.data,false,&c); char out[OUT_CAP];
            if (!e && c.op=='i') snprintf(out,sizeof(out),"$info ARM_NODE %d",ARM_NODE_ID);
            else if (!e && c.op=='v') snprintf(out,sizeof(out),"$version 0.4");
            else if (!e && c.op=='b') bus_diagnostics(out);
            else if (!e && c.op=='h') bus_raw_diagnostics(out);
            else if (!e && c.id==ARM_NODE_ID && (!c.argc || c.op=='d') && strchr("RAPHd",c.op)) local_command(&c,0,out);
            else fail_reply(out,c.name,e?e:"CMD");
            usb_line(out);
#endif
        }
    }
}
static void slave_task(void *arg) {
    (void)arg; arm_line_t line={0};
    while (true) {
        unsigned char b;
        if (uart_read_bytes(UART_NUM_1,&b,1,pdMS_TO_TICKS(5))!=1) continue;
        bus_record_rx(b);
        if (arm_bus_line_feed(&line,b,now_us())!=1) continue;
        uint16_t seq; char *body;
        if (!bus_decode_counted(line.data,&seq,&body)) continue;
        arm_command_t c; const char *err=arm_parse(body,true,&c);
        /* 无ID的信息请求不能触发所有从板同时应答。 */
        if (c.id!=ARM_NODE_ID && !(c.id==0 && strchr("RUX",c.op))) continue;
        lock(); bus_stats.hit++; if (!err) arm_bus_seen(&joint,now_us()); unlock();
        bool motion=(c.op=='R' || c.op=='U') && c.argc;
        bool reply=c.id==ARM_NODE_ID && !motion && c.op!='X';
        char out[OUT_CAP];
        if (err) { if (reply) fail_reply(out,c.name,err); else continue; }
        else local_command(&c,c.op=='U'?now_us()+c.ttl_ms*1000LL:0,out);
        if (reply) {
            char packet[ARM_BUS_LINE+2];
            if (!arm_bus_encode(seq,out,packet,sizeof(packet))) continue;
            /* 先接通Q2再发；等待最后一个停止位发送完毕才释放，错误同样释放。 */
            esp_rom_delay_us(100); gpio_set_level(ARM_TX_EN_PIN,1); esp_rom_delay_us(10);
            bus_write_packet(packet);
            gpio_set_level(ARM_TX_EN_PIN,0);
        }
    }
}
void app_main(void) {
    /* 不自动擦除NVS，避免升级或分区错误静默丢失标定。 */
    ESP_ERROR_CHECK(nvs_flash_init_partition("arm_cfg"));
    joint_lock=xSemaphoreCreateMutex(); bus_lock=xSemaphoreCreateMutex();
    arm_settings_t cfg; load_settings(&cfg); arm_init(&joint,&cfg);
    /* 仅从板启用总线看门狗；master不依赖从板在线，U仍受自身100ms期限约束。 */
    arm_bus_seen(&joint,now_us());
    gpio_config_t gate={.pin_bit_mask=1ULL<<ARM_TX_EN_PIN,.mode=GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&gate)); gpio_set_level(ARM_TX_EN_PIN,ARM_MASTER?1:0);
    ledc_timer_config_t timer={.speed_mode=LEDC_LOW_SPEED_MODE,.duty_resolution=LEDC_TIMER_14_BIT,
        .timer_num=LEDC_TIMER_0,.freq_hz=ARM_PWM_HZ,.clk_cfg=LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t channel={.gpio_num=ARM_PWM_PIN,.speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_0,.timer_sel=LEDC_TIMER_0,.duty=0};
    ESP_ERROR_CHECK(ledc_channel_config(&channel)); arm_stop(&joint); pwm_write(joint.pwm_us);
    i2c_config_t i2c={.mode=I2C_MODE_MASTER,.sda_io_num=ARM_SDA_PIN,.scl_io_num=ARM_SCL_PIN,
        .sda_pullup_en=GPIO_PULLUP_ENABLE,.scl_pullup_en=GPIO_PULLUP_ENABLE,.master.clk_speed=ARM_I2C_HZ};
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&i2c));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0));
    adc_oneshot_unit_init_cfg_t ai={.unit_id=ADC_UNIT_1}; ESP_ERROR_CHECK(adc_oneshot_new_unit(&ai,&adc));
    adc_oneshot_chan_cfg_t ac={.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_DEFAULT};
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc,ADC_CHANNEL_0,&ac));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc,ADC_CHANNEL_1,&ac));
    uart_config_t uc={.baud_rate=ARM_BUS_BAUD,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1,&uc));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1,ARM_MASTER?ARM_UART_TX_MASTER:ARM_UART_TX_SLAVE,
        ARM_MASTER?ARM_UART_RX_MASTER:ARM_UART_RX_SLAVE,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1,512,512,0,NULL,0));
    /* 短回包逐字节及时上交，不依赖可能受门控线路影响的RX空闲检测或默认120字节阈值。 */
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM_1,ARM_UART_RX_FULL_THRESHOLD));
    usb_serial_jtag_driver_config_t usb={.tx_buffer_size=512,.rx_buffer_size=512};
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb));
    xTaskCreate(control_task,"joint",4096,NULL,10,NULL);
    xTaskCreate(usb_task,"usb_api",6144,NULL,4,NULL);
#if ARM_MASTER
    xTaskCreate(poll_task,"bus_master",4096,NULL,3,NULL);
#else
    xTaskCreate(slave_task,"bus_slave",4096,NULL,4,NULL);
#endif
}
