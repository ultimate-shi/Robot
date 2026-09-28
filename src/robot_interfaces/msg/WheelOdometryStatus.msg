# 使用方法：订阅 /wheel/odometry_status 诊断各轮一致性和里程计可信度。
std_msgs/Header header
string state
float64[4] residual_mps
float64[4] weight
bool[4] suspected_fault
uint8 effective_wheels
float64 covariance_scale
float64 data_age
float64 sync_skew
