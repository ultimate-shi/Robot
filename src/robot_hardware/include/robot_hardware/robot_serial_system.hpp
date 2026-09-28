// 使用方法：仅由 robot_serial_system.cpp 和 pluginlib 使用，业务节点不直接包含本头文件。
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/service.hpp"
#include "robot_interfaces/msg/chassis_state.hpp"
#include "robot_interfaces/srv/execute_encoder_motion.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_hardware
{

class SerialPort;

struct ServoConfig
{
  int zero_adc{0};
  int direction{1};
  double minimum{-90.0};
  double maximum{90.0};
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  int pwm_deadzone{0};
  double brake_deadzone{0.0};
  double target{0.0};
  double current{0.0};
  double integral{0.0};
  double last_error{0.0};
  double watch_position{0.0};
  std::chrono::steady_clock::time_point watch_since{};
  std::chrono::steady_clock::time_point cooldown_since{};
  int stuck_attempts{0};
  int stuck_state{0};
};

class RobotSerialSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool discover_controllers();
  bool load_calibration(const std::string & path);
  bool query_encoder_parameters();
  bool read_wheels();
  bool read_legs();
  bool read_servos_and_sonar(double dt);
  bool write_wheels();
  bool write_legs();
  bool write_servos(double dt);
  bool stop_outputs();
  bool stop_encoder_motion();
  void latch_fault(const std::string & reason);
  void publish_telemetry(const rclcpp::Time & stamp);
  int servo_pwm(std::size_t index, double dt);
  double adc_to_radians(std::size_t index, int adc);
  std::size_t joint(const std::string & name) const;

  std::vector<double> state_position_;
  std::vector<double> state_velocity_;
  std::vector<double> command_position_;
  std::vector<double> command_velocity_;
  std::unordered_map<std::string, std::size_t> joint_index_;
  std::vector<std::string> device_candidates_;
  std::unique_ptr<SerialPort> dmc0_;
  std::unique_ptr<SerialPort> dmc1_;
  std::unique_ptr<SerialPort> se2_;
  std::array<int, 4> encoder_cpr_{{0, 0, 0, 0}};
  std::array<int, 4> encoder_direction_{{0, 0, 0, 0}};
  std::array<std::int64_t, 4> encoder_count_{{0, 0, 0, 0}};
  std::array<std::uint32_t, 4> encoder_motion_target_{{0, 0, 0, 0}};
  bool encoder_motion_active_{false};
  bool encoder_motion_cancelled_{false};
  bool wheel_motion_lease_active_{false};
  bool wheel_motion_lease_seen_{false};
  bool wheel_motion_rearm_ready_{false};
  std::chrono::steady_clock::time_point last_wheel_motion_lease_{};
  std::mutex encoder_motion_mutex_;
  std::array<double, 4> wheel_current_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> leg_current_{{0.0, 0.0, 0.0, 0.0}};
  std::array<double, 8> sonar_m_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<bool, 8> sonar_initialized_{{false, false, false, false, false, false, false, false}};
  std::array<bool, 8> sonar_valid_{{false, false, false, false, false, false, false, false}};
  std::array<ServoConfig, 6> servos_{};
  std::array<bool, 6> servo_initialized_{{false, false, false, false, false, false}};
  std::array<bool, 4> encoder_initialized_{{false, false, false, false}};
  int serial_timeout_ms_{500};
  int inter_command_delay_ms_{3};
  int discovery_retry_count_{20};
  int discovery_retry_delay_ms_{700};
  int encoder_query_delay_ms_{20};
  int command_timeout_ms_{500};
  double wheel_diameter_{0.110};
  double sonar_alpha_{0.2};
  int sonar_min_valid_mm_{30};
  int sonar_max_valid_mm_{2500};
  bool outputs_enabled_{false};
  bool manual_recovery_required_{false};
  std::string fault_;
  std::chrono::steady_clock::time_point last_heartbeat_{};
  std::chrono::steady_clock::time_point last_write_{};

  rclcpp::Publisher<robot_interfaces::msg::ChassisState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::array<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr, 8> sonar_pubs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wheel_motion_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_encoder_motion_service_;
  rclcpp::Service<robot_interfaces::srv::ExecuteEncoderMotion>::SharedPtr
    encoder_motion_service_;
};

}  // namespace robot_hardware
