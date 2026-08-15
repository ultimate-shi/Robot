# 使用方法：任务规划节点发布共享状态；motion_enabled=false 时任何阶段都不得驱动底盘。
uint8 TYPE_NONE=0
uint8 TYPE_GOTO_OBJECT=1
uint8 TYPE_FOLLOW_PERSON=2
uint8 TYPE_EXPLORE=3
std_msgs/Header header
string mission_id
uint8 mission_type
string phase
string target_id
string message
bool motion_enabled
