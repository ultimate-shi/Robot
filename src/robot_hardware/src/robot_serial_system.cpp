// 使用方法：由 controller_manager 通过 robot_hardware_plugins.xml 加载，不单独运行。
#include "robot_hardware/robot_serial_system.hpp"

#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "yaml-cpp/yaml.h"

namespace robot_hardware
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr int kServoNeutral = 1500;
constexpr int kServoMin = 500;
constexpr int kServoMax = 2500;
constexpr double kPidDeadzoneDeg = 0.5;
constexpr double kIntegralMaximum = 300.0;
constexpr double kIntegralSeparationDeg = 5.0;
constexpr double kStuckErrorDeg = 2.0;
constexpr double kStuckMovementDeg = 0.5;
constexpr auto kStuckDetect = std::chrono::milliseconds(800);
constexpr auto kStuckCooldown = std::chrono::milliseconds(1000);
constexpr int kStuckMaxAttempts = 3;

const std::array<std::string, 4> kLegJoints = {
  "lap_body_joint_fl", "lap_body_joint_fr", "lap_body_joint_rl", "lap_body_joint_rr"};
const std::array<std::string, 4> kSteerJoints = {
  "front_left_steer_joint", "front_right_steer_joint",
  "rear_left_steer_joint", "rear_right_steer_joint"};
const std::array<std::string, 4> kWheelJoints = {
  "front_left_wheel_joint", "front_right_wheel_joint",
  "rear_left_wheel_joint", "rear_right_wheel_joint"};
const std::array<std::string, 2> kHeadJoints = {"head_yaw_joint", "head_pitch_joint"};
const std::array<std::string, 8> kSonarTopics = {
  "/ultrasonic/front_left", "/ultrasonic/front_center", "/ultrasonic/front_right",
  "/ultrasonic/right", "/ultrasonic/rear_right", "/ultrasonic/rear_center",
  "/ultrasonic/rear_left", "/ultrasonic/left"};
const std::array<std::string, 8> kSonarFrames = {
  "sonar_fl", "sonar_fc", "sonar_fr", "sonar_right",
  "sonar_rr", "sonar_rc", "sonar_rl", "sonar_left"};

std::vector<std::string> split(const std::string & value, char delimiter)
{
  std::vector<std::string> result;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, delimiter)) {
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  return result;
}

double shortest_angle_deg(double value)
{
  while (value > 180.0) value -= 360.0;
  while (value < -180.0) value += 360.0;
  return value;
}
}  // namespace

class SerialPort
{
public:
  SerialPort(std::string path, int timeout_ms, int inter_command_delay_ms)
  : path_(std::move(path)), timeout_ms_(timeout_ms), delay_ms_(inter_command_delay_ms) {}

  ~SerialPort() {close_port();}

  bool open_port()
  {
    close_port();
    fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;
    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      close_port();
      return false;
    }
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = static_cast<cc_t>(
      std::clamp((timeout_ms_ + 99) / 100, 1, 255));
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      close_port();
      return false;
    }
    // 与已验收 hw_ctrl/setup_serial() 一致，显式拉高控制板要求的 DTR/RTS。
    int status = 0;
    if (ioctl(fd_, TIOCMGET, &status) == 0) {
      status |= TIOCM_DTR | TIOCM_RTS;
      (void)ioctl(fd_, TIOCMSET, &status);
    }
    tcflush(fd_, TCIOFLUSH);
    const int flags = fcntl(fd_, F_GETFL, 0);
    (void)fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    last_transaction_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(delay_ms_);
    return true;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool transact(const std::string & command, std::string & response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return false;
    const auto earliest = last_transaction_ + std::chrono::milliseconds(delay_ms_);
    if (std::chrono::steady_clock::now() < earliest) {
      std::this_thread::sleep_until(earliest);
    }
    // 与已验收 hw_ctrl/uart_command() 一致，每条命令前丢弃上一次事务的残帧。
    tcflush(fd_, TCIFLUSH);
    const char * cursor = command.data();
    std::size_t remaining = command.size();
    while (remaining > 0) {
      const ssize_t written = ::write(fd_, cursor, remaining);
      if (written <= 0) return false;
      cursor += written;
      remaining -= static_cast<std::size_t>(written);
    }
    tcdrain(fd_);
    response.clear();
    std::string line;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      char byte = 0;
      const ssize_t count = ::read(fd_, &byte, 1);
      if (count < 0) {
        if (errno == EAGAIN) continue;
        return false;
      }
      if (count == 0) continue;
      line.push_back(byte);
      if (byte == '\n') {
        if (line.size() > 1 &&
          (std::isalpha(static_cast<unsigned char>(line.front())) || line.front() == '$'))
        {
          response = std::move(line);
          last_transaction_ = std::chrono::steady_clock::now();
          return response.front() != '!';
        }
        line.clear();
      } else if (line.size() >= 1023) {
        line.clear();
      }
    }
    last_transaction_ = std::chrono::steady_clock::now();
    return false;
  }

  const std::string & path() const {return path_;}

private:
  std::string path_;
  int timeout_ms_{500};
  int delay_ms_{3};
  int fd_{-1};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_transaction_{};
};

namespace
{
// 使用方法：DMC1 单轴 S 命令和停车共用此校验，必须核对 ID、完整字段及板端错误码。
bool command_wheel_axis(SerialPort * port, std::size_t index, long ticks, int duration_ms)
{
  if (!port || index >= 4) return false;
  const int id = static_cast<int>(index + 4);
  std::ostringstream command;
  command << 'S' << id << ' ' << ticks << ' ' << duration_ms << '\n';
  std::string response;
  if (!port->transact(command.str(), response)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "DMC1 S%d 事务失败", id);
    return false;
  }
  std::stringstream parser(response);
  std::string response_command;
  int velocity = 0;
  std::string encoder_hex;
  int current = 0;
  int error = -1;
  parser >> response_command >> velocity >> encoder_hex >> current >> error;
  if (!parser || response_command != "S" + std::to_string(id) || error != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "DMC1 S%d 响应无效: %s", id,
      response.c_str());
    return false;
  }
  try {
    std::size_t parsed = 0;
    const auto encoder = std::stoul(encoder_hex, &parsed, 16);
    if (parsed != encoder_hex.size() ||
      encoder > std::numeric_limits<std::uint32_t>::max())
    {
      return false;
    }
  } catch (const std::exception &) {
    return false;
  }
  std::string unexpected;
  if (parser >> unexpected) return false;
  return true;
}

// 使用方法：任何故障或取消路径均尝试停车四轴，单轴失败也继续向其余轴发零速。
bool stop_wheel_axes(SerialPort * port, int duration_ms)
{
  bool all_stopped = true;
  for (std::size_t index = 0; index < 4; ++index) {
    const bool stopped = command_wheel_axis(port, index, 0, duration_ms);
    all_stopped = stopped && all_stopped;
  }
  return all_stopped;
}
}  // namespace

hardware_interface::CallbackReturn RobotSerialSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  state_position_.assign(info_.joints.size(), 0.0);
  state_velocity_.assign(info_.joints.size(), 0.0);
  command_position_.assign(info_.joints.size(), 0.0);
  command_velocity_.assign(info_.joints.size(), 0.0);
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    joint_index_[info_.joints[index].name] = index;
  }
  try {
    for (const auto & name : kLegJoints) (void)joint(name);
    for (const auto & name : kSteerJoints) (void)joint(name);
    for (const auto & name : kWheelJoints) (void)joint(name);
    for (const auto & name : kHeadJoints) (void)joint(name);
    const auto & parameters = info_.hardware_parameters;
    device_candidates_ = split(parameters.at("device_candidates"), ',');
    serial_timeout_ms_ = std::stoi(parameters.at("serial_timeout_ms"));
    inter_command_delay_ms_ = std::stoi(parameters.at("inter_command_delay_ms"));
    discovery_retry_count_ = std::stoi(parameters.at("discovery_retry_count"));
    discovery_retry_delay_ms_ = std::stoi(parameters.at("discovery_retry_delay_ms"));
    encoder_query_delay_ms_ = std::stoi(parameters.at("encoder_query_delay_ms"));
    command_timeout_ms_ = std::stoi(parameters.at("command_timeout_ms"));
    wheel_diameter_ = std::stod(parameters.at("wheel_diameter"));
    sonar_alpha_ = std::stod(parameters.at("sonar_lpf_alpha"));
    sonar_min_valid_mm_ = std::stoi(parameters.at("sonar_min_valid_mm"));
    sonar_max_valid_mm_ = std::stoi(parameters.at("sonar_max_valid_mm"));
    if (command_timeout_ms_ < 1 || command_timeout_ms_ > 1000 || wheel_diameter_ <= 0.0 ||
      !std::isfinite(wheel_diameter_) || discovery_retry_count_ < 1 ||
      discovery_retry_delay_ms_ < 0 ||
      encoder_query_delay_ms_ < 0 || sonar_min_valid_mm_ < 1 ||
      sonar_max_valid_mm_ <= sonar_min_valid_mm_)
    {
      throw std::invalid_argument("控制板发现重试参数超出范围");
    }
    if (!load_calibration(parameters.at("servo_calibration_file"))) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "硬件参数无效: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSerialSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  state_pub_ = node->create_publisher<robot_interfaces::msg::ChassisState>(
    "/hardware/chassis_state", rclcpp::SystemDefaultsQoS());
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  for (std::size_t index = 0; index < sonar_pubs_.size(); ++index) {
    sonar_pubs_[index] = node->create_publisher<sensor_msgs::msg::Range>(
      kSonarTopics[index], rclcpp::SensorDataQoS());
  }
  heartbeat_sub_ = node->create_subscription<std_msgs::msg::Empty>(
    "/hardware/command_heartbeat", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Empty::SharedPtr) {
      std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
      last_heartbeat_ = std::chrono::steady_clock::now();
    });
  wheel_motion_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "/wheel_motion/active", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Bool::SharedPtr message) {
      bool stop = false;
      {
        std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
        // 停止后仅允许下一次 false -> true 动作重新提交 T，拒绝晚到的旧请求。
        if (message->data && wheel_motion_rearm_ready_) {
          encoder_motion_cancelled_ = false;
          wheel_motion_rearm_ready_ = false;
        }
        if (!message->data) wheel_motion_rearm_ready_ = true;
        wheel_motion_lease_active_ = message->data;
        wheel_motion_lease_seen_ = true;
        last_wheel_motion_lease_ = std::chrono::steady_clock::now();
        stop = !message->data && encoder_motion_active_;
      }
      if (stop && !stop_encoder_motion()) {
        latch_fault("定距动作结束后 DMC1 停车失败");
      }
    });
  stop_encoder_motion_service_ = node->create_service<std_srvs::srv::Trigger>(
    "/hardware/stop_encoder_motion",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      response->success = stop_encoder_motion();
      response->message = response->success ? "DMC1 定距动作已停止" : "DMC1 停车失败，硬件已锁定";
      if (!response->success) latch_fault("DMC1 定距动作取消停车失败");
    });
  recovery_service_ = node->create_service<std_srvs::srv::Trigger>(
    "/hardware/recover",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      if (!manual_recovery_required_) {
        response->success = true;
        response->message = "硬件未锁定";
        return;
      }
      if (!discover_controllers()) {
        response->success = false;
        response->message = "控制板仍未全部连接，保持锁定";
        return;
      }
      manual_recovery_required_ = false;
      outputs_enabled_ = true;
      fault_.clear();
      last_heartbeat_ = std::chrono::steady_clock::now();
      response->success = true;
      response->message = "三块控制板已重新识别，输出已恢复";
    });
  encoder_motion_service_ =
    node->create_service<robot_interfaces::srv::ExecuteEncoderMotion>(
    "/hardware/execute_encoder_motion",
    [this](
      const std::shared_ptr<robot_interfaces::srv::ExecuteEncoderMotion::Request> request,
      std::shared_ptr<robot_interfaces::srv::ExecuteEncoderMotion::Response> response)
    {
      if (!outputs_enabled_ || manual_recovery_required_ || !dmc1_) {
        response->success = false;
        response->message = "硬件未使能";
        return;
      }
      std::unique_lock<std::mutex> lock(encoder_motion_mutex_);
      const auto now = std::chrono::steady_clock::now();
      if (encoder_motion_active_ || encoder_motion_cancelled_ || !wheel_motion_lease_active_ ||
        !wheel_motion_lease_seen_ ||
        now - last_wheel_motion_lease_ > std::chrono::milliseconds(command_timeout_ms_) ||
        now - last_heartbeat_ > std::chrono::milliseconds(command_timeout_ms_))
      {
        response->success = false;
        response->message = "定距动作或命令心跳已失效，拒绝 T 指令";
        return;
      }
      std::ostringstream command;
      command << "T " << std::uppercase << std::hex;
      for (std::size_t index = 0; index < 4; ++index) {
        const double revolutions = request->distance_m[index] /
          (kPi * wheel_diameter_);
        const double raw_delta = -revolutions * encoder_cpr_[index] * encoder_direction_[index];
        if (!std::isfinite(raw_delta) ||
          std::abs(raw_delta) > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
        {
          response->success = false;
          response->message = "定距目标无效或超出编码器范围";
          return;
        }
        const auto delta = static_cast<std::int32_t>(std::llround(raw_delta));
        const auto target = static_cast<std::uint32_t>(
          static_cast<std::uint32_t>(encoder_count_[index]) +
          static_cast<std::int32_t>(delta));
        encoder_motion_target_[index] = target;
        command << target << ' ';
      }
      command << '\n';
      std::string serial_response;
      response->success = dmc1_->transact(command.str(), serial_response);
      encoder_motion_active_ = response->success;
      if (!response->success) {
        // 响应丢失时控制板仍可能已接受 T，必须逐轴发 S 停车。
        encoder_motion_cancelled_ = true;
        wheel_motion_lease_active_ = false;
        for (const auto & name : kWheelJoints) command_velocity_[joint(name)] = 0.0;
        const bool stopped = stop_wheel_axes(dmc1_.get(), command_timeout_ms_);
        lock.unlock();
        if (!stopped) latch_fault("T 指令结果不明且 DMC1 停车失败");
        response->message = stopped ? "T 指令失败，DMC1 已停车" :
          "T 指令结果不明且停车失败，硬件已锁定";
        return;
      }
      response->message = "DMC1 已接受 T 绝对编码器目标";
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSerialSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!discover_controllers()) {
    latch_fault("DMC0、DMC1 或 SE2 初始化失败");
    return hardware_interface::CallbackReturn::ERROR;
  }
  outputs_enabled_ = true;
  manual_recovery_required_ = false;
  fault_.clear();
  last_heartbeat_ = std::chrono::steady_clock::now();
  last_write_ = last_heartbeat_;
  {
    std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
    encoder_motion_active_ = false;
    encoder_motion_cancelled_ = false;
    wheel_motion_lease_active_ = false;
    wheel_motion_lease_seen_ = false;
    wheel_motion_rearm_ready_ = false;
  }
  command_position_ = state_position_;
  std::fill(command_velocity_.begin(), command_velocity_.end(), 0.0);
  RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "三块底盘控制板已激活");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSerialSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  outputs_enabled_ = false;
  (void)stop_outputs();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSerialSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    for (const auto & interface : info_.joints[index].state_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(info_.joints[index].name, interface.name, &state_position_[index]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(info_.joints[index].name, interface.name, &state_velocity_[index]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSerialSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    for (const auto & interface : info_.joints[index].command_interfaces) {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        interfaces.emplace_back(info_.joints[index].name, interface.name, &command_position_[index]);
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        interfaces.emplace_back(info_.joints[index].name, interface.name, &command_velocity_[index]);
      }
    }
  }
  return interfaces;
}

hardware_interface::return_type RobotSerialSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!outputs_enabled_ || manual_recovery_required_) {
    publish_telemetry(time);
    return hardware_interface::return_type::ERROR;
  }
  const bool ok = read_wheels() && read_legs() &&
    read_servos_and_sonar(std::clamp(period.seconds(), 0.001, 0.1));
  if (!ok) {
    latch_fault("控制板状态查询超时或响应格式错误");
    publish_telemetry(time);
    return hardware_interface::return_type::ERROR;
  }
  publish_telemetry(time);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSerialSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!outputs_enabled_ || manual_recovery_required_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  bool heartbeat_expired = false;
  bool lease_expired = false;
  bool encoder_motion_active = false;
  {
    std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
    heartbeat_expired = now - last_heartbeat_ > std::chrono::milliseconds(command_timeout_ms_);
    encoder_motion_active = encoder_motion_active_;
    lease_expired = encoder_motion_active &&
      (!wheel_motion_lease_active_ ||
      now - last_wheel_motion_lease_ > std::chrono::milliseconds(command_timeout_ms_));
  }
  if (heartbeat_expired || lease_expired) {
    for (const auto & name : kWheelJoints) command_velocity_[joint(name)] = 0.0;
    if (lease_expired || (heartbeat_expired && encoder_motion_active)) {
      if (!stop_encoder_motion()) {
        latch_fault("定距动作失去心跳后 DMC1 停车失败");
        return hardware_interface::return_type::ERROR;
      }
    }
  }
  const double dt = std::clamp(period.seconds(), 0.001, 0.1);
  if (!write_wheels() || !write_legs() || !write_servos(dt)) {
    latch_fault("控制板写入失败");
    return hardware_interface::return_type::ERROR;
  }
  last_write_ = now;
  return hardware_interface::return_type::OK;
}

bool RobotSerialSystem::discover_controllers()
{
  dmc0_.reset();
  dmc1_.reset();
  se2_.reset();
  for (int attempt = 1; attempt <= discovery_retry_count_; ++attempt) {
    if (attempt > 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(discovery_retry_delay_ms_));
    }
    for (const auto & path : device_candidates_) {
      if ((dmc0_ && dmc0_->path() == path) ||
        (dmc1_ && dmc1_->path() == path) ||
        (se2_ && se2_->path() == path))
      {
        continue;
      }
      auto port = std::make_unique<SerialPort>(path, serial_timeout_ms_, inter_command_delay_ms_);
      if (!port->open_port()) continue;
      std::string response;
      if (!port->transact("$info\n", response)) continue;
      std::stringstream parser(response);
      std::string command;
      std::string model;
      parser >> command >> model;
      if (command != "$info") continue;
      if (model == "DMC") {
        int count = 0;
        int first_id = -1;
        parser >> count >> first_id;
        if (count != 4 || first_id < 0) continue;
        if (first_id / 4 == 0 && !dmc0_) {
          dmc0_ = std::move(port);
          RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "识别 DMC0: %s", path.c_str());
        } else if (first_id / 4 == 1 && !dmc1_) {
          dmc1_ = std::move(port);
          RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "识别 DMC1: %s", path.c_str());
        }
      } else if (model == "SE2" && !se2_) {
        se2_ = std::move(port);
        RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "识别 SE2: %s", path.c_str());
      }
    }
    if (dmc0_ && dmc1_ && se2_) {
      // 已验收 hw_ctrl 会在身份扫描和后续 I/R/S/f 查询之间完成配置加载；显式等待同等窗口。
      std::this_thread::sleep_for(std::chrono::milliseconds(discovery_retry_delay_ms_));
      if (query_encoder_parameters() && read_wheels() && read_legs() &&
        read_servos_and_sonar(0.025))
      {
        RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "三块控制板协议校验通过");
        return true;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("RobotSerialSystem"),
        "控制板身份已识别但协议校验失败，将关闭串口并重新发现");
      dmc0_.reset();
      dmc1_.reset();
      se2_.reset();
    }
    RCLCPP_WARN(
      rclcpp::get_logger("RobotSerialSystem"),
      "控制板发现第 %d/%d 轮未完成: DMC0=%d DMC1=%d SE2=%d",
      attempt, discovery_retry_count_, static_cast<bool>(dmc0_),
      static_cast<bool>(dmc1_), static_cast<bool>(se2_));
  }
  const bool ok = dmc0_ && dmc1_ && se2_;
  if (!ok) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "控制板发现失败: DMC0=%d DMC1=%d SE2=%d",
      static_cast<bool>(dmc0_), static_cast<bool>(dmc1_), static_cast<bool>(se2_));
  }
  return ok;
}

bool RobotSerialSystem::load_calibration(const std::string & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node items = root["servos"];
    if (!items || items.size() != servos_.size()) return false;
    for (std::size_t index = 0; index < servos_.size(); ++index) {
      auto & servo = servos_[index];
      const auto item = items[index];
      servo.zero_adc = item["zero_adc"].as<int>();
      servo.direction = item["direction"].as<int>();
      servo.minimum = item["min_deg"].as<double>();
      servo.maximum = item["max_deg"].as<double>();
      servo.kp = item["kp"].as<double>();
      servo.ki = item["ki"].as<double>();
      servo.kd = item["kd"].as<double>();
      servo.pwm_deadzone = item["pwm_deadzone"].as<int>();
      servo.brake_deadzone = item["brake_deadzone"].as<double>();
    }
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "读取舵机标定失败 %s: %s",
      path.c_str(), error.what());
    return false;
  }
}

bool RobotSerialSystem::query_encoder_parameters()
{
  encoder_cpr_.fill(0);
  encoder_direction_.fill(0);
  for (std::size_t index = 0; index < 4; ++index) {
    std::this_thread::sleep_for(std::chrono::milliseconds(encoder_query_delay_ms_));
    std::string response;
    if (!dmc1_->transact("I" + std::to_string(index + 4) + " \n", response)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotSerialSystem"),
        "DMC1 编码器参数 I%zu 查询失败", index + 4);
      return false;
    }
    std::stringstream parser(response);
    char command = 0;
    int id = -1;
    parser >> command >> id >> encoder_cpr_[index] >> encoder_direction_[index];
    if (command != 'I' || id != static_cast<int>(index + 4) ||
      encoder_cpr_[index] <= 0 || std::abs(encoder_direction_[index]) != 1)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotSerialSystem"),
        "DMC1 编码器参数响应无效: %s", response.c_str());
      return false;
    }
    encoder_initialized_[index] = false;
    RCLCPP_INFO(
      rclcpp::get_logger("RobotSerialSystem"),
      "已读取编码器 I%zu: CPR=%d direction=%d",
      index + 4, encoder_cpr_[index], encoder_direction_[index]);
  }
  return true;
}

bool RobotSerialSystem::read_wheels()
{
  std::string response;
  if (!dmc1_ || !dmc1_->transact("S \n", response)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "DMC1 S 状态查询失败");
    return false;
  }
  std::stringstream parser(response);
  char command = 0;
  parser >> command;
  if (command != 'S') {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "DMC1 S 响应无效: %s", response.c_str());
    return false;
  }
  std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
  for (std::size_t index = 0; index < 4; ++index) {
    int ticks_per_second = 0;
    std::string encoder_hex;
    int current = 0;
    parser >> ticks_per_second >> encoder_hex >> current;
    if (!parser) return false;
    std::uint32_t encoder = 0;
    try {
      encoder = static_cast<std::uint32_t>(std::stoul(encoder_hex, nullptr, 16));
    } catch (...) {
      return false;
    }
    const double side_sign = (index == 1 || index == 3) ? -1.0 : 1.0;
    const double linear_velocity = side_sign * ticks_per_second *
      (kPi * wheel_diameter_) / encoder_cpr_[index];
    state_velocity_[joint(kWheelJoints[index])] = linear_velocity / (wheel_diameter_ / 2.0);
    if (encoder_initialized_[index]) {
      const auto previous = static_cast<std::uint32_t>(encoder_count_[index]);
      const std::int32_t delta = static_cast<std::int32_t>(encoder - previous);
      state_position_[joint(kWheelJoints[index])] +=
        static_cast<double>(delta) * (-encoder_direction_[index]) * 2.0 * kPi /
        encoder_cpr_[index];
    }
    encoder_count_[index] = encoder;
    encoder_initialized_[index] = true;
    wheel_current_[index] = current;
  }
  if (encoder_motion_active_) {
    bool arrived = true;
    for (std::size_t index = 0; index < 4; ++index) {
      const auto current = static_cast<std::uint32_t>(encoder_count_[index]);
      const auto delta = static_cast<std::int32_t>(
        encoder_motion_target_[index] - current);
      arrived = arrived && std::abs(static_cast<std::int64_t>(delta)) <= 20;
    }
    encoder_motion_active_ = !arrived;
  }
  return true;
}

bool RobotSerialSystem::read_legs()
{
  std::string response;
  if (!dmc0_ || !dmc0_->transact("R \n", response)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "DMC0 R 状态查询失败");
    return false;
  }
  std::stringstream parser(response);
  char command = 0;
  parser >> command;
  if (command != 'R') {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotSerialSystem"), "DMC0 R 响应无效: %s", response.c_str());
    return false;
  }
  const std::array<double, 4> sign = {-1.0, 1.0, -1.0, 1.0};
  for (std::size_t index = 0; index < 4; ++index) {
    int fixed_angle = 0;
    int speed_deg_s = 0;
    int current = 0;
    parser >> fixed_angle >> speed_deg_s >> current;
    if (!parser) return false;
    state_position_[joint(kLegJoints[index])] = sign[index] * fixed_angle / 100.0 * kDegToRad;
    state_velocity_[joint(kLegJoints[index])] = sign[index] * speed_deg_s * kDegToRad;
    leg_current_[index] = current;
  }
  return true;
}

bool RobotSerialSystem::read_servos_and_sonar(double)
{
  std::string response;
  if (!se2_ || !se2_->transact("f    \n", response)) return false;
  std::stringstream parser(response);
  char command = 0;
  parser >> command;
  if (command != 'f') return false;
  std::array<int, 6> adc{};
  std::array<int, 8> ranges_mm{};
  for (auto & value : adc) parser >> value;
  for (auto & value : ranges_mm) parser >> value;
  if (!parser) return false;
  for (std::size_t index = 0; index < servos_.size(); ++index) {
    const double radians = adc_to_radians(index, adc[index]);
    if (index < 2) state_position_[joint(kHeadJoints[index])] = radians;
    else state_position_[joint(kSteerJoints[index - 2])] = radians;
  }
  for (std::size_t index = 0; index < sonar_m_.size(); ++index) {
    if (ranges_mm[index] < sonar_min_valid_mm_ ||
      ranges_mm[index] > sonar_max_valid_mm_)
    {
      // SE2 无回波或探头振铃时可能给出 0~20mm；不得把它当成真实近距离障碍。
      sonar_valid_[index] = false;
      sonar_initialized_[index] = false;
      continue;
    }
    const double raw = ranges_mm[index] / 1000.0;
    if (!sonar_initialized_[index]) {
      sonar_m_[index] = raw;
      sonar_initialized_[index] = true;
    } else {
      sonar_m_[index] = sonar_alpha_ * raw + (1.0 - sonar_alpha_) * sonar_m_[index];
    }
    sonar_valid_[index] = true;
  }
  return true;
}

bool RobotSerialSystem::write_wheels()
{
  std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
  // DMC1 的 T 指令由控制板闭环执行；到位前不能被周期 S 指令覆盖。
  if (encoder_motion_active_) return true;
  if (!dmc1_) return false;
  for (std::size_t index = 0; index < 4; ++index) {
    const double angular_velocity = command_velocity_[joint(kWheelJoints[index])];
    if (!std::isfinite(angular_velocity)) return false;
    const double ticks = angular_velocity * encoder_cpr_[index] / (2.0 * kPi);
    if (!std::isfinite(ticks) || std::abs(ticks) > 32767.0) return false;
    if (!command_wheel_axis(
        dmc1_.get(), index, std::lround(ticks), command_timeout_ms_))
    {
      return false;
    }
  }
  return true;
}

bool RobotSerialSystem::write_legs()
{
  const std::array<double, 4> minimum = {-45.0, -30.0, -30.0, -45.0};
  const std::array<double, 4> maximum = {30.0, 45.0, 45.0, 30.0};
  const std::array<double, 4> sign = {-1.0, 1.0, -1.0, 1.0};
  std::ostringstream command;
  command << "R ";
  for (std::size_t index = 0; index < 4; ++index) {
    const double degree = std::clamp(
      command_position_[joint(kLegJoints[index])] * kRadToDeg,
      minimum[index], maximum[index]);
    command << std::lround(sign[index] * degree * 100.0) << ' ';
  }
  command << '\n';
  std::string response;
  return dmc0_ && dmc0_->transact(command.str(), response);
}

bool RobotSerialSystem::write_servos(double dt)
{
  auto set_target = [this](std::size_t index, double target) {
      auto & servo = servos_[index];
      if (std::abs(target - servo.target) > 0.01) {
        servo.stuck_state = 0;
        servo.stuck_attempts = 0;
        servo.integral = 0.0;
      }
      servo.target = target;
    };
  for (std::size_t index = 0; index < 2; ++index) {
    set_target(
      index, command_position_[joint(kHeadJoints[index])] * kRadToDeg);
  }
  for (std::size_t index = 0; index < 4; ++index) {
    set_target(
      index + 2,
      command_position_[joint(kSteerJoints[index])] * kRadToDeg);
  }
  std::ostringstream command;
  command << "U ";
  for (std::size_t index = 0; index < servos_.size(); ++index) {
    command << servo_pwm(index, dt) << ' ';
  }
  command << '\n';
  std::string response;
  return se2_ && se2_->transact(command.str(), response);
}

bool RobotSerialSystem::stop_outputs()
{
  bool ok = stop_encoder_motion();
  std::string response;
  if (se2_) ok = se2_->transact("U 1500 1500 1500 1500 1500 1500 \n", response) && ok;
  return ok;
}

bool RobotSerialSystem::stop_encoder_motion()
{
  std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
  // 与 T 提交互斥；先逐轴停车，再释放软件的 T 运行状态。
  encoder_motion_cancelled_ = true;
  wheel_motion_lease_active_ = false;
  for (const auto & name : kWheelJoints) command_velocity_[joint(name)] = 0.0;
  const bool stopped = stop_wheel_axes(dmc1_.get(), command_timeout_ms_);
  encoder_motion_active_ = false;
  return stopped;
}

void RobotSerialSystem::latch_fault(const std::string & reason)
{
  fault_ = reason;
  outputs_enabled_ = false;
  manual_recovery_required_ = true;
  (void)stop_outputs();
  RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "%s；已锁定，需调用 /hardware/recover", reason.c_str());
}

void RobotSerialSystem::publish_telemetry(const rclcpp::Time & stamp)
{
  robot_interfaces::msg::ChassisState state;
  state.header.stamp = stamp;
  state.header.frame_id = "base_link";
  state.dmc0_connected = static_cast<bool>(dmc0_);
  state.dmc1_connected = static_cast<bool>(dmc1_);
  state.se2_connected = static_cast<bool>(se2_);
  state.outputs_enabled = outputs_enabled_;
  state.watchdog_ok = std::chrono::steady_clock::now() - last_heartbeat_ <=
    std::chrono::milliseconds(command_timeout_ms_);
  state.manual_recovery_required = manual_recovery_required_;
  state.fault = fault_;
  state.wheel_current_ma = wheel_current_;
  state.leg_current_ma = leg_current_;
  state_pub_->publish(state);

  for (std::size_t index = 0; index < sonar_pubs_.size(); ++index) {
    sensor_msgs::msg::Range range;
    range.header.stamp = stamp;
    range.header.frame_id = kSonarFrames[index];
    range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    range.field_of_view = 30.0 * kDegToRad;
    range.min_range = sonar_min_valid_mm_ / 1000.0;
    range.max_range = sonar_max_valid_mm_ / 1000.0;
    range.range = sonar_valid_[index] ? sonar_m_[index] :
      std::numeric_limits<float>::quiet_NaN();
    sonar_pubs_[index]->publish(range);
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus item;
  item.name = "robot_hardware/serial_boards";
  item.hardware_id = "DMC0+DMC1+SE2";
  item.level = outputs_enabled_ ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  item.message = outputs_enabled_ ? "三块控制板正常" : fault_;
  diagnostics.status.push_back(item);
  diagnostics_pub_->publish(diagnostics);
}

int RobotSerialSystem::servo_pwm(std::size_t index, double dt)
{
  auto & servo = servos_[index];
  const double bounded_target = std::clamp(servo.target, servo.minimum, servo.maximum);
  if (std::abs(bounded_target - servo.target) > 1e-9) servo.target = bounded_target;
  double error = shortest_angle_deg(servo.target - servo.current);
  if (std::abs(error) < kPidDeadzoneDeg + servo.brake_deadzone) {
    servo.integral = 0.0;
    servo.last_error = 0.0;
    servo.stuck_state = 0;
    servo.stuck_attempts = 0;
    return kServoNeutral;
  }
  const auto now = std::chrono::steady_clock::now();
  if (servo.stuck_state == 2 || servo.stuck_state == 3) {
    if (std::abs(error) < kStuckErrorDeg) {
      servo.stuck_state = 0;
      servo.stuck_attempts = 0;
    } else if (servo.stuck_state == 3 || now - servo.cooldown_since < kStuckCooldown) {
      return kServoNeutral;
    } else if (servo.stuck_attempts >= kStuckMaxAttempts) {
      servo.stuck_state = 3;
      return kServoNeutral;
    } else {
      servo.stuck_state = 1;
      servo.watch_since = now;
      servo.watch_position = servo.current;
      return kServoNeutral;
    }
  }
  if (std::abs(error) >= kStuckErrorDeg) {
    if (servo.stuck_state != 1) {
      servo.stuck_state = 1;
      servo.watch_since = now;
      servo.watch_position = servo.current;
    } else if (std::abs(shortest_angle_deg(servo.current - servo.watch_position)) >=
      kStuckMovementDeg)
    {
      servo.watch_since = now;
      servo.watch_position = servo.current;
    } else if (now - servo.watch_since >= kStuckDetect) {
      ++servo.stuck_attempts;
      servo.stuck_state = 2;
      servo.cooldown_since = now;
      servo.integral = 0.0;
      return kServoNeutral;
    }
  } else {
    servo.stuck_state = 0;
    servo.stuck_attempts = 0;
  }
  if (std::abs(error) < kIntegralSeparationDeg) {
    servo.integral = std::clamp(
      servo.integral + error * dt, -kIntegralMaximum, kIntegralMaximum);
  } else {
    servo.integral = 0.0;
  }
  const double derivative = (error - servo.last_error) / dt;
  servo.last_error = error;
  const double direction = servo.direction >= 0 ? 1.0 : -1.0;
  const double output = direction *
    (servo.kp * error + servo.ki * servo.integral + servo.kd * derivative);
  int pwm = static_cast<int>(std::lround(std::clamp(
    kServoNeutral + output, static_cast<double>(kServoMin), static_cast<double>(kServoMax))));
  if (pwm > kServoNeutral - servo.pwm_deadzone && pwm < kServoNeutral) {
    pwm = kServoNeutral - servo.pwm_deadzone;
  } else if (pwm > kServoNeutral && pwm < kServoNeutral + servo.pwm_deadzone) {
    pwm = kServoNeutral + servo.pwm_deadzone;
  }
  return pwm;
}

double RobotSerialSystem::adc_to_radians(std::size_t index, int adc)
{
  auto & servo = servos_[index];
  int relative = (adc - servo.zero_adc) % 4096;
  if (relative < 0) relative += 4096;
  double raw_degree = relative * 360.0 / 4096.0;
  if (servo.direction < 0) raw_degree = 360.0 - raw_degree;
  if (raw_degree > 180.0) raw_degree -= 360.0;
  if (!servo_initialized_[index]) {
    servo.current = raw_degree;
    servo_initialized_[index] = true;
  } else {
    servo.current += 0.8 * shortest_angle_deg(raw_degree - servo.current);
  }
  servo.current = std::clamp(servo.current, servo.minimum, servo.maximum);
  return servo.current * kDegToRad;
}

std::size_t RobotSerialSystem::joint(const std::string & name) const
{
  const auto iterator = joint_index_.find(name);
  if (iterator == joint_index_.end()) throw std::out_of_range("缺少关节: " + name);
  return iterator->second;
}

}  // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotSerialSystem, hardware_interface::SystemInterface)
