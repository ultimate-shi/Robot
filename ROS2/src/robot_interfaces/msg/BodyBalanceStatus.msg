# 使用方法：订阅 /body_balance/status 观察姿态平衡、档位和过流降级状态。
std_msgs/Header header
bool enabled
string level
float64 target_roll_deg
float64 target_pitch_deg
float64 actual_roll_deg
float64 actual_pitch_deg
bool overload_active
string state
float64[4] leg_target_rad
