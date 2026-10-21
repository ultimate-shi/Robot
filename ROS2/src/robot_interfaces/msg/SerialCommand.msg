# 文件作用：记录 robot_hardware 对控制板实际写入的串口指令及事务结果。
# 使用方法：订阅 /hardware/serial_tx；command 保留发送时的原始换行符。
std_msgs/Header header
string board
string device
string command
uint32 bytes_written
string result
