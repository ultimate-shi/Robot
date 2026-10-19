# 使用方法：robot_perception 发布单个语义目标，消费者应先检查 has_depth 和 has_map_position。
string id
string class_name
string label_zh
float32 confidence
int32 bbox_x_min
int32 bbox_y_min
int32 bbox_x_max
int32 bbox_y_max
bool has_depth
float32 distance
bool has_map_position
geometry_msgs/PointStamped map_position
