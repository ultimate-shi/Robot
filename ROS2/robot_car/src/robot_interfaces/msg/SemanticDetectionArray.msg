# 使用方法：发布与一帧图像对应的全部语义目标，header 时间戳必须来自相机帧。
std_msgs/Header header
uint32 image_width
uint32 image_height
string model
float32 latency_ms
SemanticDetection[] detections
