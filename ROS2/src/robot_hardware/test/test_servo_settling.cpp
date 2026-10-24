// 作用：验证舵机到位后保持中性输出，并在目标变化或明显偏离时恢复调整。
// 使用方法：构建 robot_hardware 后由 colcon test 自动运行，无需连接控制板。
#include "gtest/gtest.h"
#include "robot_hardware/servo_settling.hpp"

TEST(ServoSettling, HoldsNeutralAcrossFeedbackJitter)
{
  bool settled = false;
  EXPECT_FALSE(robot_hardware::servo_should_brake(settled, 45.0, 2.5, 4.0));
  EXPECT_TRUE(robot_hardware::servo_should_brake(settled, 2.5, 2.5, 4.0));
  EXPECT_TRUE(robot_hardware::servo_should_brake(settled, 3.9, 2.5, 4.0));
  EXPECT_TRUE(robot_hardware::servo_should_brake(settled, 4.0, 2.5, 4.0));
  EXPECT_FALSE(robot_hardware::servo_should_brake(settled, 4.1, 2.5, 4.0));
  EXPECT_FALSE(robot_hardware::servo_should_brake(settled, 3.0, 2.5, 4.0));
  EXPECT_TRUE(robot_hardware::servo_should_brake(settled, 2.0, 2.5, 4.0));
  settled = false;  // 新目标到来时必须解除到位状态。
  EXPECT_FALSE(robot_hardware::servo_should_brake(settled, 3.0, 2.5, 4.0));
}
