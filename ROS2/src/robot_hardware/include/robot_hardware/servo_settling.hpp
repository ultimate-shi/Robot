// 作用：判断舵机到位后是否继续输出中性 PWM，避免反馈抖动触发反复微调。
// 使用方法：每次计算 PWM 时传入角度误差和标定阈值；目标变化时将 settled 设为 false。
#pragma once

namespace robot_hardware
{

inline bool servo_should_brake(
  bool & settled, double absolute_error, double stop_threshold, double resume_threshold)
{
  if (absolute_error <= (settled ? resume_threshold : stop_threshold)) {
    settled = true;
    return true;
  }
  settled = false;
  return false;
}

}  // namespace robot_hardware
