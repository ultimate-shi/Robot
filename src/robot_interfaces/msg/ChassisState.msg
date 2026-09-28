# 使用方法：订阅 /hardware/chassis_state 获取三块控制板与整车输出使能状态。
std_msgs/Header header
bool dmc0_connected
bool dmc1_connected
bool se2_connected
bool outputs_enabled
bool watchdog_ok
bool manual_recovery_required
string fault
float64[4] wheel_current_ma
float64[4] leg_current_ma
