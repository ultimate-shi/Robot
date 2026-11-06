// 作用：用伪终端执行真实串口驱动的并行、反馈、停车和故障测试；用法：colcon test --packages-select robot_hardware。
#include <gtest/gtest.h>
#include <pty.h>
#include <poll.h>
#include <atomic>
#include <future>
#include "../src/robot_serial_system.cpp"

using namespace std::chrono_literals;

namespace robot_hardware
{
class FakeBoard
{
public:
  explicit FakeBoard(std::string identity, int delay = 1) : identity_(identity), delay_ms(delay)
  {
    char path[128];
    if (openpty(&master_, &slave_, path, nullptr, nullptr) != 0) throw std::runtime_error("伪终端失败");
    path_ = path;
    thread_ = std::thread([this] {
      std::string line;
      while (running_) {
        pollfd fd{master_, POLLIN, 0};
        if (::poll(&fd, 1, 10) <= 0) continue;
        char value;
        if (::read(master_, &value, 1) != 1) continue;
        line += value;
        if (value != '\n') continue;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          commands_.push_back(line);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms.load()));
        if (!drop_response && !(drop_t_response && line.rfind("T ", 0) == 0)) {
          std::string response;
          if (line == "R \n") response = "R 0 0 10 0 0 10 0 0 10 0 0 10\n";
          else if (line == "D \n") response = "D 0 0 10 0 0 10 0 0 10 0 0 10\n";
          else if (line.rfind("f", 0) == 0) response = "f 0 0 0 0 0 0 300 300 300 300 300 300 300 300\n";
          else if (line.size() > 1 && line[0] == 'D' && std::isdigit(line[1])) {
            response = line.substr(0, line.find(' '))+" 0 0 10\n";
          } else response = line.substr(0, line.find(' '))+" OK\n";
          (void)::write(master_, response.data(), response.size());
        }
        line.clear();
      }
    });
  }
  ~FakeBoard() {running_ = false; thread_.join(); ::close(slave_); ::close(master_);}
  std::vector<std::string> commands()
  {std::lock_guard<std::mutex> lock(mutex_); return commands_;}
  std::string path_;
  std::atomic<int> delay_ms;
  std::atomic<bool> drop_response{false};
  std::atomic<bool> drop_t_response{false};
private:
  std::string identity_;
  int master_, slave_;
  std::thread thread_;
  std::atomic<bool> running_{true};
  std::mutex mutex_;
  std::vector<std::string> commands_;
};

struct SerialSystemTest
{
  static void prepare(RobotSerialSystem & s, FakeBoard & legs, FakeBoard & wheels, FakeBoard & servos, int timeout_ms = 50)
  {
    size_t i = 0;
    for (const auto & names : {kLegJoints, kWheelJoints, kSteerJoints}) {
      for (const auto & name : names) s.joint_index_[name] = i++;
    }
    for (const auto & name : kHeadJoints) s.joint_index_[name] = i++;
    s.state_position_.assign(i, 0); s.state_velocity_.assign(i, 0);
    s.command_position_.assign(i, 0); s.command_velocity_.assign(i, 0);
    s.io_state_position_.assign(i, 0); s.io_state_velocity_.assign(i, 0);
    s.io_command_position_.assign(i, 0); s.io_command_velocity_.assign(i, 0);
    s.cached_position_.assign(i, 0); s.cached_velocity_.assign(i, 0);
    s.latest_position_.assign(i, 0); s.latest_velocity_.assign(i, 0);
    s.encoder_cpr_.fill(1000);
    s.axis_to_wheel_ = {0, 1, 2, 3};
    for (auto & wheel : s.wheels_) {
      wheel.pwm = {100, 200}; wheel.speed = {1, 2};
    }
    s.dmc0_ = std::make_unique<SerialPort>(legs.path_, timeout_ms, 0);
    s.dmc1_ = std::make_unique<SerialPort>(wheels.path_, timeout_ms, 0);
    s.se2_ = std::make_unique<SerialPort>(servos.path_, timeout_ms, 0);
    ASSERT_TRUE(s.dmc0_->open_port()); ASSERT_TRUE(s.dmc1_->open_port()); ASSERT_TRUE(s.se2_->open_port());
    s.attach_trace(*s.dmc0_); s.attach_trace(*s.dmc1_); s.attach_trace(*s.se2_);
    s.dmc0_->set_board("DMC0"); s.dmc1_->set_board("DMC1"); s.se2_->set_board("SE2");
    s.outputs_enabled_ = true;
    s.heartbeat_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
    s.start_board_workers();
  }

  static void independence()
  {
    FakeBoard legs("DMC0"), wheels("DMC1", 25), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    std::this_thread::sleep_for(450ms);
    {
      std::lock_guard<std::mutex> lock(s.cache_mutex_);
      EXPECT_GE(s.feedback_sequence_[0], 4u);
      EXPECT_LT(s.feedback_sequence_[1], s.feedback_sequence_[0]);
    }
    const auto began = std::chrono::steady_clock::now();
    // 写缓存不等待正在执行的 DMC1 四轴事务。
    EXPECT_EQ(s.write(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
    EXPECT_LT(std::chrono::steady_clock::now()-began, 20ms);
    const auto read_at = std::chrono::steady_clock::now();
    EXPECT_EQ(s.read(rclcpp::Time(0), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
    EXPECT_LT(std::chrono::steady_clock::now()-read_at, 20ms);
    s.stop_board_workers();
    EXPECT_FALSE(s.async_active_);
    EXPECT_FALSE(s.async_fault_) << s.fault_;
    EXPECT_EQ(servos.commands().back(), "U 1500 1500 1500 1500 1500 1500 \n");
    EXPECT_EQ(wheels.commands().back(), "D7 0 500\n");
  }

  static void newest_and_recovery()
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    s.write(rclcpp::Time(0), rclcpp::Duration(0, 0));  // 恢复屏障先保持实测位置。
    for (int i = 1; i <= 100; ++i) {
      s.command_position_[s.joint(kLegJoints[0])] = i * kDegToRad / 10;
      s.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
    }
    std::this_thread::sleep_for(250ms);
    const auto commands = legs.commands();
    EXPECT_TRUE(std::any_of(commands.begin(), commands.end(), [](const auto & command) {
      return command.rfind("R -1000 ", 0) == 0;
    }));
    EXPECT_LT(commands.size(), 10u);  // 没有把 100 个旧目标排队执行。
    s.stop_board_workers();
    s.outputs_enabled_ = true;
    s.manual_recovery_required_ = false;
    s.start_board_workers();
    std::this_thread::sleep_for(150ms);
    s.stop_board_workers();
    EXPECT_EQ(legs.commands().back(), "R 0 0 0 0 \n");
  }

  static void leg_targets_are_one_shot()
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    s.stop_board_workers();
    s.outputs_enabled_ = true;
    s.async_fault_ = false;
    s.last_leg_target_valid_ = false;
    bool sent = false;
    ASSERT_TRUE(s.write_legs(false, &sent));
    ASSERT_TRUE(sent);
    const auto count = legs.commands().size();
    for (int i = 0; i < 20; ++i) {
      ASSERT_TRUE(s.write_legs(false, &sent));
      EXPECT_FALSE(sent);
    }
    EXPECT_EQ(legs.commands().size(), count);
    // 只有左前腿变化时，其余腿不重启板端定位；协议已有 32767 跳过语义。
    s.io_command_position_[s.joint(kLegJoints[0])] = 5*kDegToRad;
    ASSERT_TRUE(s.write_legs(false, &sent));
    EXPECT_TRUE(sent);
    EXPECT_EQ(legs.commands().back(), "R -500 32767 32767 32767 \n");
    // 发送失败不能当作已执行；下一次相同目标仍须尝试下发。
    s.io_command_position_[s.joint(kLegJoints[0])] = 6*kDegToRad;
    legs.drop_response = true;
    EXPECT_FALSE(s.write_legs(false, &sent));
    EXPECT_FALSE(sent);
    EXPECT_EQ(s.last_leg_target_[0], -500);
    legs.drop_response = false;
    ASSERT_TRUE(s.write_legs(false, &sent));
    EXPECT_TRUE(sent);
    EXPECT_EQ(legs.commands().back(), "R -600 32767 32767 32767 \n");
    // 停止保持即使数值相同也必须完整发送，不受去重影响。
    ASSERT_TRUE(s.write_legs(true, &sent));
    EXPECT_TRUE(sent);
    EXPECT_EQ(legs.commands().back(), "R -600 0 0 0 \n");
    // 查询反馈正常推进，重复缓存既不下发目标，也不刷新目标下发年龄。
    s.latest_position_ = s.io_command_position_;
    s.latest_command_at_ = std::chrono::steady_clock::now();
    s.sent_at_[0] = std::chrono::steady_clock::now()-1s;
    const auto sent_at = s.sent_at_[0];
    const auto feedback = s.feedback_sequence_[0];
    ASSERT_TRUE(s.board_cycle(0));
    EXPECT_EQ(s.sent_at_[0], sent_at);
    EXPECT_GT(s.feedback_sequence_[0], feedback);
    // 上游超时只保持实测位置一次，不能每次查询又以新反馈重启定位。
    s.latest_command_at_ -= 10s;
    ASSERT_TRUE(s.board_cycle(0));
    const auto held_count = legs.commands().size();
    ASSERT_TRUE(s.board_cycle(0));
    EXPECT_EQ(legs.commands().size(), held_count+1);  // 仅一次 R 查询。
  }

  static void fault_and_join()
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    legs.drop_response = true;
    std::this_thread::sleep_for(350ms);
    EXPECT_TRUE(s.async_fault_);
    EXPECT_TRUE(s.manual_recovery_required_);
    EXPECT_FALSE(s.outputs_enabled_);
    auto joined = std::async(std::launch::async, [&] {s.stop_board_workers();});
    EXPECT_EQ(joined.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(servos.commands().back(), "U 1500 1500 1500 1500 1500 1500 \n");
  }

  static void arm(RobotSerialSystem & s)
  {
    std::lock_guard<std::mutex> lock(s.encoder_motion_mutex_);
    s.wheel_motion_lease_seen_ = s.wheel_motion_lease_active_ = true;
    s.encoder_motion_cancelled_ = false;
    s.last_wheel_motion_lease_ = std::chrono::steady_clock::now();
    s.heartbeat_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
  }

  static void distance_and_cancel(bool lose_response)
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    arm(s);
    wheels.drop_t_response = lose_response;
    std::string message;
    auto accepted = s.board_workers_[1].submit([&] {
      return s.execute_encoder_target({0.1, 0.1, 0.1, 0.1}, message);
    });
    ASSERT_EQ(accepted.wait_for(2s), std::future_status::ready);
    EXPECT_EQ(accepted.get(), !lose_response);
    if (!lose_response) {
      auto commands = wheels.commands();
      const size_t after_t = commands.size();
      std::this_thread::sleep_for(200ms);
      commands = wheels.commands();
      for (size_t i = after_t; i < commands.size(); ++i) {
        EXPECT_EQ(commands[i], "D \n");  // T 活动期间只查询，不发逐轴 D 覆盖。
      }
      EXPECT_TRUE(s.stop_encoder_motion());
      EXPECT_EQ(wheels.commands().back(), "D7 0 500\n");
      std::lock_guard<std::mutex> lock(s.encoder_motion_mutex_);
      EXPECT_FALSE(s.encoder_motion_active_);
    } else {
      EXPECT_TRUE(s.manual_recovery_required_);
      EXPECT_FALSE(s.outputs_enabled_);
    }
    s.stop_board_workers();
  }

  static void expired_lease_stops()
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2");
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos);
    arm(s);
    std::string message;
    auto task = s.board_workers_[1].submit([&] {
      return s.execute_encoder_target({0.1, 0.1, 0.1, 0.1}, message);
    });
    ASSERT_TRUE(task.get());
    for (int i = 0; i < 20; ++i) {
      s.heartbeat_ns_ = std::chrono::steady_clock::now().time_since_epoch().count();
      s.write(rclcpp::Time(0), rclcpp::Duration(0, 0));
      std::this_thread::sleep_for(40ms);
    }
    {
      std::lock_guard<std::mutex> lock(s.encoder_motion_mutex_);
      EXPECT_FALSE(s.encoder_motion_active_);
      EXPECT_TRUE(s.encoder_motion_cancelled_);
    }
    EXPECT_FALSE(s.async_fault_) << s.fault_;
    EXPECT_GT(s.servo_gap_timing_.summary().count, 10u);
    EXPECT_LT(s.servo_gap_timing_.summary().maximum, s.servo_watchdog_ms_);
    s.stop_board_workers();
  }

  static void slow_servo_query_fails_closed()
  {
    FakeBoard legs("DMC0"), wheels("DMC1"), servos("SE2", 55);
    RobotSerialSystem s;
    prepare(s, legs, wheels, servos, 200);
    std::this_thread::sleep_for(450ms);
    EXPECT_TRUE(s.async_fault_);
    EXPECT_FALSE(s.outputs_enabled_);
    EXPECT_GE(s.servo_watchdog_overruns_.load(), 1u);
    s.stop_board_workers();
  }

  static void delayed_ack_does_not_fake_send_gap()
  {
    FakeBoard board("SE2");
    SerialPort port(board.path_, 250, 0);
    ASSERT_TRUE(port.open_port());
    port.set_board("SE2");
    RobotSerialSystem system;
    system.async_active_ = true;
    system.outputs_enabled_ = true;
    system.attach_trace(port);
    std::string response;
    ASSERT_TRUE(port.transact("U 1500 1500 1500 1500 1500 1500 \n", response));
    std::this_thread::sleep_for(15ms);
    board.delay_ms = 120;
    ASSERT_TRUE(port.transact("U 1500 1500 1500 1500 1500 1500 \n", response));
    // 第二条指令早已写出，仅 ACK 晚到；不能把 120ms ACK 延迟算成发送间隔。
    EXPECT_FALSE(system.async_fault_);
    EXPECT_EQ(system.servo_watchdog_overruns_.load(), 0u);
    EXPECT_LT(system.servo_gap_timing_.summary().maximum, 100.0);
    EXPECT_EQ(system.trace_queue_.size(), 2u);  // 串口线程只入队，不直接发布 ROS。
  }

  static void genuine_send_gap_still_latches()
  {
    RobotSerialSystem system;
    system.async_active_ = true;
    system.outputs_enabled_ = true;
    auto first = std::chrono::steady_clock::now();
    system.record_servo_send(first);
    system.record_servo_send(first+120ms);
    EXPECT_TRUE(system.async_fault_);
    EXPECT_EQ(system.servo_watchdog_overruns_.load(), 1u);
    EXPECT_FALSE(system.outputs_enabled_);
  }
};

TEST(AsyncSerial, IndependentBoardsAndNonblockingWrite) {SerialSystemTest::independence();}
TEST(AsyncSerial, LatestTargetAndRecoveryClearOldMotion) {SerialSystemTest::newest_and_recovery();}
TEST(AsyncSerial, LegTargetsAreOneShotWithIndependentChannelsAndForcedStop) {SerialSystemTest::leg_targets_are_one_shot();}
TEST(AsyncSerial, TimeoutLatchesAndStopsWithoutSelfJoin) {SerialSystemTest::fault_and_join();}
TEST(AsyncSerial, DistanceMotionIsNotOverwrittenAndCancellationStops) {SerialSystemTest::distance_and_cancel(false);}
TEST(AsyncSerial, LostDistanceResponseStopsAndLatches) {SerialSystemTest::distance_and_cancel(true);}
TEST(AsyncSerial, ExpiredLeaseStopsWhileServoRefreshContinues) {SerialSystemTest::expired_lease_stops();}
TEST(AsyncSerial, SlowServoQueryDoesNotSilentlyBreakWatchdog) {SerialSystemTest::slow_servo_query_fails_closed();}
TEST(AsyncSerial, DelayedAckDoesNotFakeSendGap) {SerialSystemTest::delayed_ack_does_not_fake_send_gap();}
TEST(AsyncSerial, GenuineSendGapStillLatches) {SerialSystemTest::genuine_send_gap_still_latches();}

TEST(BoardWorker, UrgentStopCancelsPendingNormalService)
{
  BoardWorker worker;
  std::promise<void> entered, release;
  auto gate = release.get_future().share();
  std::once_flag once;
  worker.start(10, [&] {std::call_once(once, [&] {entered.set_value();}); gate.wait(); return true;}, [] {});
  entered.get_future().wait();
  int ordinary = 0, stopped = 0;
  auto normal = worker.submit([&] {++ordinary; return true;});
  auto stop = worker.submit([&] {++stopped; return true;}, true);
  EXPECT_FALSE(normal.get());
  release.set_value();
  EXPECT_TRUE(stop.get());
  worker.stop();
  EXPECT_EQ(ordinary, 0); EXPECT_EQ(stopped, 1);
}

TEST(TimingWindow, PercentilesAndLifetimeFailures)
{
  TimingWindow timing;
  for (int i = 1; i <= 100; ++i) timing.add(i, i == 100);
  auto summary = timing.summary();
  EXPECT_EQ(summary.p50, 50); EXPECT_EQ(summary.p95, 95);
  EXPECT_EQ(summary.p99, 99); EXPECT_EQ(summary.maximum, 100);
  EXPECT_EQ(summary.failures, 1u);
}
}  // namespace robot_hardware
