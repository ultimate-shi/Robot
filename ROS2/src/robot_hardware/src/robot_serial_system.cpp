// 文件作用：实现实机 ros2_control SystemInterface，连接 DMC0 腿部控制板、DMC1 轮驱动板和
// SE2 头部/转向舵机及超声波板。由 controller_manager 根据 robot_hardware_plugins.xml 加载，
// 不作为独立节点运行；串口路径、超时和标定文件由 hardware.xacro 中的硬件参数传入。
// 调用顺序：on_init 读取配置和标定 → on_configure 注册 ROS 接口 → on_activate 发现控制板；
// 激活后 controller_manager 按 update_rate 周期调用 read 读取反馈、write 下发命令，
// 停用时调用 on_deactivate。定距动作、手动恢复和心跳则由 on_configure 注册的 ROS 回调处理。
// 协议分工：DMC0 使用 R 查询/设置四腿；DMC1 使用 I 查询编码器、S 查询或设置轮速、
// D 设置已标定方向的 PWM、T 执行四轮定距；SE2 使用 f 读取六路舵机 ADC 与八路超声波、
// U 下发六路舵机 PWM。ROS 关节命令是目标值，实际串口输出由每轮反馈和标定计算。
// 每次串口写入同时发布 /hardware/serial_tx，供 MCAP 记录最终板端指令。
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
#include <functional>
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

// 关节顺序用于 ROS 数组到板端字段的转换；SE2 六通道顺序还须与标定文件一致。
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

// 将角度误差折回 [-180°, 180°]，避免编码器跨越一圈边界时选择错误方向。
double shortest_angle_deg(double value)
{
  while (value > 180.0) value -= 360.0;
  while (value < -180.0) value += 360.0;
  return value;
}
}  // namespace

// 每块板独占一个实例；所有命令和回包都经 transact 串行化，避免并发回调读错响应。
class SerialPort
{
public:
  using TraceCallback = std::function<void(
      const std::string &, const std::string &, const std::string &,
      std::size_t, const std::string &, std::int64_t)>;

  SerialPort(std::string path, int timeout_ms, int inter_command_delay_ms)
  : path_(std::move(path)), timeout_ms_(timeout_ms), delay_ms_(inter_command_delay_ms) {}

  ~SerialPort() {close_port();}

  void set_trace(TraceCallback trace) {trace_ = std::move(trace);}
  void set_board(std::string board) {board_ = std::move(board);}

  // discover_controllers 识别候选设备时调用；配置 115200 8N1、DTR/RTS 和读超时。
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

  // 实例销毁、重新发现或打开失败时关闭设备，释放串口独占句柄。
  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  // 所有板端命令共用的同步事务：保证指令间隔，写完整行，等待一条有效换行回包。
  // 返回 false 表示串口失败、超时或板端错误；调用方还须核对各协议字段及设备身份。
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
    const auto sent_at_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const char * cursor = command.data();
    std::size_t remaining = command.size();
    std::size_t bytes_written = 0;
    const auto report = [&](const std::string & result) {
        if (trace_) trace_(board_, path_, command, bytes_written, result, sent_at_ns);
      };
    while (remaining > 0) {
      const ssize_t written = ::write(fd_, cursor, remaining);
      if (written <= 0) {
        report(bytes_written ? "partial_write" : "write_failed");
        return false;
      }
      cursor += written;
      bytes_written += static_cast<std::size_t>(written);
      remaining -= static_cast<std::size_t>(written);
    }
    if (tcdrain(fd_) != 0) {
      report("drain_failed");
      return false;
    }
    response.clear();
    std::string line;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
    while (std::chrono::steady_clock::now() < deadline) {
      char byte = 0;
      const ssize_t count = ::read(fd_, &byte, 1);
      if (count < 0) {
        if (errno == EAGAIN) continue;
        report("read_failed");
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
          report(response.front() == '!' ? "board_error" : "response_ok");
          return response.front() != '!';
        }
        line.clear();
      } else if (line.size() >= 1023) {
        line.clear();
      }
    }
    last_transaction_ = std::chrono::steady_clock::now();
    report("response_timeout");
    return false;
  }

  const std::string & path() const {return path_;}

private:
  std::string path_;
  std::string board_{"UNIDENTIFIED"};
  TraceCallback trace_;
  int timeout_ms_{500};
  int delay_ms_{3};
  int fd_{-1};
  std::mutex mutex_;
  std::chrono::steady_clock::time_point last_transaction_{};
};

namespace
{
// write_wheels 的 S 速度分支、stop_wheel_axes 的停车分支调用；index=0..3 对应板端 S4..S7。
// ticks 是编码器计数/秒，duration_ms 是板端命令有效期；校验 ID、字段和错误码后才算成功。
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

// write_wheels 在目标轮速进入已标定 D 曲线时调用；发送 D4..D7 和带符号 PWM。
// D 模式仅用于已实测的方向，回包命令名必须与目标轴一致。
bool command_pwm_axis(SerialPort * port, std::size_t index, int pwm, int duration_ms)
{
  if (!port || index >= 4 || std::abs(pwm) > 100) return false;
  const int id = static_cast<int>(index + 4);
  std::string response;
  if (!port->transact("D" + std::to_string(id) + " " + std::to_string(pwm) +
    " " + std::to_string(duration_ms) + "\n", response)) return false;
  std::stringstream parser(response);
  std::string command;
  int absolute_encoder = 0;
  std::string incremental_encoder;
  int current = 0;
  parser >> command >> absolute_encoder >> incremental_encoder >> current;
  return static_cast<bool>(parser) && command == "D" + std::to_string(id);
}

// 停用、故障、T 动作取消或提交结果不明时调用；逐轴发 S 零速。
// 即使某一轴失败仍继续停车其余三轴，最终合并报告是否全部成功。
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

// controller_manager 创建插件后首先调用；解析 URDF 硬件参数、建立关节索引、加载六路舵机
// 与四轮标定。这里只校验静态配置，不打开串口；失败会阻止硬件进入后续生命周期阶段。
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
    if (!load_wheel_calibration(parameters.at("wheel_calibration_file"))) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "硬件参数无效: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

// controller_manager 配置硬件时调用一次；在硬件节点上注册状态/诊断/超声波发布器，
// 以及心跳、定距动作租约订阅和手动恢复、定距动作服务。此时尚未使能实机输出。
hardware_interface::CallbackReturn RobotSerialSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  state_pub_ = node->create_publisher<robot_interfaces::msg::ChassisState>(
    "/hardware/chassis_state", rclcpp::SystemDefaultsQoS());
  serial_tx_pub_ = node->create_publisher<robot_interfaces::msg::SerialCommand>(
    "/hardware/serial_tx", rclcpp::QoS(rclcpp::KeepLast(1000)).reliable());
  diagnostics_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::SystemDefaultsQoS());
  for (std::size_t index = 0; index < sonar_pubs_.size(); ++index) {
    sonar_pubs_[index] = node->create_publisher<sensor_msgs::msg::Range>(
      kSonarTopics[index], rclcpp::SensorDataQoS());
  }
  // 轮速保护节点定期发布心跳；write 发现心跳过期会清零轮速，并停止活动的 T 动作。
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
        // Action 运行期间定期发 true 维持租约；结束发 false 取消 T 并重新武装下一次动作。
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
  // Action 取消时由 wheel_motion_server 调用，立即向 DMC1 四轴发送零速。
  stop_encoder_motion_service_ = node->create_service<std_srvs::srv::Trigger>(
    "/hardware/stop_encoder_motion",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      response->success = stop_encoder_motion();
      response->message = response->success ? "DMC1 定距动作已停止" : "DMC1 停车失败，硬件已锁定";
      if (!response->success) latch_fault("DMC1 定距动作取消停车失败");
    });
  // 故障锁定后的人工恢复入口；不在普通控制周期中自动重连控制板。
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
      // 故障锁定后显式调用才重新发现三块板；发现失败保持锁定，不自动恢复输出。
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
  // wheel_motion_server 的 ExecuteWheelMotion Action 调用此内部服务提交 DMC1 定距目标。
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
      // ExecuteWheelMotion Action 经此服务提交一次四轮相对位移；要求运行租约和心跳均有效。
      // T 是 DMC1 的绝对编码器目标，执行期间周期 write_wheels 不再发送 S/D 覆盖它。
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
        const double revolutions = request->distance_m[axis_to_wheel_[index]] /
          (kPi * wheel_diameter_);
        const auto & wheel = wheels_[axis_to_wheel_[index]];
        const double raw_delta = revolutions * encoder_cpr_[index] * wheel.encoder_sign;
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

// controller_manager 激活硬件时调用；扫描串口并核验 I/S/R/f 协议，再开放输出。
// 位置命令先置为当前关节反馈、轮速置零，避免激活瞬间追逐未初始化的旧目标。
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

// controller_manager 停用硬件时调用；先关闭正常写入，再向轮轴发零速、舵机发中性 PWM。
hardware_interface::CallbackReturn RobotSerialSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  outputs_enabled_ = false;
  (void)stop_outputs();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// 初始化阶段向 ros2_control 暴露 URDF 声明的位置/速度反馈存储地址；read 会更新这些值。
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

// 初始化阶段向关节控制器暴露目标位置/速度存储地址；write 从这些值生成板端指令。
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

// 激活后的每个 controller_manager 控制周期调用；依次读取 DMC1 轮、DMC0 腿和 SE2 舵机/
// 超声波，再发布硬件状态。任一查询失败即锁定故障并返回 ERROR，阻止继续正常控制。
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

// 激活后的每个控制周期调用；检查轮速心跳和 T 动作租约，再依次写 DMC1、DMC0、SE2。
// 心跳过期时把轮速目标置零；定距动作租约失效时主动停车。串口写入失败会锁定硬件。
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

// on_activate 和 /hardware/recover 调用；按设备身份发现 DMC0、DMC1、SE2，不能仅靠端口号推断。
// 三块板齐全后查询 I/S/R/f 验证协议及初始反馈；失败关闭串口并按配置次数重试。
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
      port->set_trace([publisher = serial_tx_pub_](
          const std::string & board, const std::string & device,
          const std::string & command, std::size_t bytes_written,
          const std::string & result, std::int64_t sent_at_ns)
        {
          robot_interfaces::msg::SerialCommand trace;
          trace.header.stamp.sec = static_cast<std::int32_t>(sent_at_ns / 1000000000LL);
          trace.header.stamp.nanosec = static_cast<std::uint32_t>(sent_at_ns % 1000000000LL);
          trace.board = board;
          trace.device = device;
          trace.command = command;
          trace.bytes_written = static_cast<std::uint32_t>(bytes_written);
          trace.result = result;
          publisher->publish(trace);
        });
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
          port->set_board("DMC0");
          dmc0_ = std::move(port);
          RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "识别 DMC0: %s", path.c_str());
        } else if (first_id / 4 == 1 && !dmc1_) {
          port->set_board("DMC1");
          dmc1_ = std::move(port);
          RCLCPP_INFO(rclcpp::get_logger("RobotSerialSystem"), "识别 DMC1: %s", path.c_str());
        }
      } else if (model == "SE2" && !se2_) {
        port->set_board("SE2");
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

// on_init 调用；按 SE2 的 U/f 六通道顺序加载 ADC 零位、方向、角度限位和舵机 PID 参数。
// 顺序为头部 yaw/pitch、转向 FL/FR/RL/RR；只读配置，不向舵机发送命令。
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

// on_init 调用；校验四轮到 DMC1 轴号的唯一映射、编码器方向和 S/D 模式标定曲线。
// 换电机或轮位时更新 YAML；启动时拒绝重复或越界轴号，物理接线仍须现场核对。
bool RobotSerialSystem::load_wheel_calibration(const std::string & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path)["wheels"];
    std::array<bool, 4> used{{false, false, false, false}};
    for (std::size_t wheel = 0; wheel < 4; ++wheel) {
      const std::string name = std::array<std::string, 4>{{"fl", "fr", "rl", "rr"}}[wheel];
      const YAML::Node item = root[name];
      auto & config = wheels_[wheel];
      config.axis = item["axis"].as<int>();
      config.encoder_sign = item["encoder_sign"].as<int>();
      config.drive_sign = item["drive_sign"].as<int>();
      config.speed_command_sign = item["speed_command_sign"].as<int>();
      config.speed_mode_enabled = item["speed_mode_enabled"].as<bool>();
      config.cpr = item["cpr"].as<int>();
      config.pwm_enabled = item["pwm_enabled"].as<bool>();
      config.pwm = item["pwm"].as<std::vector<double>>();
      config.speed = item["speed_rad_s"].as<std::vector<double>>();
      config.negative_pwm_enabled = item["negative_pwm_enabled"] &&
        item["negative_pwm_enabled"].as<bool>();
      if (config.negative_pwm_enabled) {
        config.negative_drive_sign = item["negative_drive_sign"].as<int>();
        config.negative_pwm = item["negative_pwm"].as<std::vector<double>>();
        config.negative_speed = item["negative_speed_rad_s"].as<std::vector<double>>();
      }
      config.feedback_gain = item["feedback_gain"].as<double>();
      config.max_pwm_correction = item["max_pwm_correction"].as<double>();
      const int axis = config.axis - 4;
      if (axis < 0 || axis >= 4 || used[axis] ||
        std::abs(config.encoder_sign) != 1 || std::abs(config.drive_sign) != 1 ||
        std::abs(config.speed_command_sign) != 1 ||
        config.cpr <= 0 || !std::isfinite(config.feedback_gain) ||
        config.feedback_gain < 0 || !std::isfinite(config.max_pwm_correction) ||
        config.max_pwm_correction < 0 || config.max_pwm_correction > 100 ||
        config.pwm.size() < 2 ||
        config.pwm.size() != config.speed.size())
      {
        throw std::invalid_argument(name + " 轮位映射或曲线无效");
      }
      if (config.negative_pwm_enabled &&
        (std::abs(config.negative_drive_sign) != 1 ||
        config.negative_drive_sign == config.drive_sign ||
        config.negative_pwm.size() < 2 ||
        config.negative_pwm.size() != config.negative_speed.size()))
      {
        throw std::invalid_argument(name + " 负 PWM 曲线或方向无效");
      }
      for (std::size_t point = 0; point < config.pwm.size(); ++point) {
        if (!std::isfinite(config.pwm[point]) || !std::isfinite(config.speed[point]) ||
          config.pwm[point] <= 0 || config.pwm[point] > 100 || config.speed[point] <= 0 ||
          (point && (config.pwm[point] <= config.pwm[point - 1] ||
          config.speed[point] <= config.speed[point - 1])))
        {
          throw std::invalid_argument(name + " PWM 曲线必须正值且严格递增");
        }
      }
      for (std::size_t point = 0; point < config.negative_pwm.size(); ++point) {
        if (!std::isfinite(config.negative_pwm[point]) ||
          !std::isfinite(config.negative_speed[point]) ||
          config.negative_pwm[point] <= 0 || config.negative_pwm[point] > 100 ||
          config.negative_speed[point] <= 0 ||
          (point && (config.negative_pwm[point] <= config.negative_pwm[point - 1] ||
          config.negative_speed[point] <= config.negative_speed[point - 1])))
        {
          throw std::invalid_argument(name + " 负 PWM 曲线必须正值且严格递增");
        }
      }
      used[axis] = true;
      axis_to_wheel_[axis] = wheel;
    }
    return true;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"),
      "读取车轮标定失败 %s: %s", path.c_str(), error.what());
    return false;
  }
}

// discover_controllers 识别 DMC1 后调用；逐轴发送 I4..I7，取得编码器 CPR 和方向。
// CPR 必须与轮位标定一致；同时清除首次采样标志，避免旧计数形成虚假里程增量。
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
    if (encoder_cpr_[index] != wheels_[axis_to_wheel_[index]].cpr) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"),
        "I%zu CPR 与轮位标定不一致", index + 4);
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

// read 每周期以及 discover_controllers 首次协议校验时调用；发送无参数 S 查询四轮。
// 将板端计数/速度换算为 ROS 弧度和弧度每秒，记录电流；T 动作到位时清除活动标志。
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
    const std::size_t wheel_index = axis_to_wheel_[index];
    const auto & wheel = wheels_[wheel_index];
    state_velocity_[joint(kWheelJoints[wheel_index])] =
      wheel.encoder_sign * ticks_per_second * 2.0 * kPi / encoder_cpr_[index];
    if (encoder_initialized_[index]) {
      const auto previous = static_cast<std::uint32_t>(encoder_count_[index]);
      const std::int32_t delta = static_cast<std::int32_t>(encoder - previous);
      state_position_[joint(kWheelJoints[wheel_index])] +=
        static_cast<double>(delta) * wheel.encoder_sign * 2.0 * kPi /
        encoder_cpr_[index];
    }
    encoder_count_[index] = encoder;
    encoder_initialized_[index] = true;
    wheel_current_[wheel_index] = current;
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

// read 每周期以及启动校验时调用；向 DMC0 发送无参数 R，解析四腿角度、角速度和电流。
// 板端以百分之一度返回，按各腿安装方向换算为 ROS 弧度反馈。
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

// read 每周期以及启动校验时调用；向 SE2 发送 f，同时取得六路舵机 ADC 和八路超声波。
// ADC 经标定换算为头部/转向关节角；有效超声波换算为米并低通滤波，异常距离标为无效。
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

// write 每周期调用；T 定距动作运行时不覆盖板端目标，否则按四轮目标角速度逐轴下发。
// 有对应方向的实测 PWM 曲线且速度达到其首点时发 D4..D7；其余允许的情况发 S4..S7。
// 非零命令若既无可用 D 曲线又未验证 S 模式，会在发送任何轮轴命令前整体拒绝。
bool RobotSerialSystem::write_wheels()
{
  std::lock_guard<std::mutex> lock(encoder_motion_mutex_);
  // DMC1 的 T 指令由控制板闭环执行；到位前不能被周期 S 指令覆盖。
  if (encoder_motion_active_) return true;
  if (!dmc1_) return false;
  // 使用方法：先核对四轮命令是否都有已验证的执行模式，避免前三轴已转而末轴才发现不可用。
  for (std::size_t index = 0; index < 4; ++index) {
    const std::size_t wheel_index = axis_to_wheel_[index];
    const auto & wheel = wheels_[wheel_index];
    const double target = command_velocity_[joint(kWheelJoints[wheel_index])];
    if (!std::isfinite(target)) return false;
    const bool use_pwm = std::abs(target) >= wheel.speed.front() && wheel.pwm_enabled &&
      std::signbit(target) == (wheel.drive_sign < 0);
    const bool use_negative_pwm = wheel.negative_pwm_enabled &&
      std::abs(target) >= wheel.negative_speed.front() &&
      std::signbit(target) == (wheel.negative_drive_sign < 0);
    if (target != 0.0 && !use_pwm && !use_negative_pwm && !wheel.speed_mode_enabled) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"),
        "DMC1 轴 %d 的 S 速度模式未通过实测，拒绝非零关节命令", wheel.axis);
      return false;
    }
  }
  for (std::size_t index = 0; index < 4; ++index) {
    const std::size_t wheel_index = axis_to_wheel_[index];
    auto & wheel = wheels_[wheel_index];
    const double angular_velocity = command_velocity_[joint(kWheelJoints[wheel_index])];
    if (!std::isfinite(angular_velocity)) return false;
    const bool positive_pwm = wheel.pwm_enabled &&
      std::abs(angular_velocity) >= wheel.speed.front() &&
      std::signbit(angular_velocity) == (wheel.drive_sign < 0);
    const bool negative_pwm = wheel.negative_pwm_enabled &&
      std::abs(angular_velocity) >= wheel.negative_speed.front() &&
      std::signbit(angular_velocity) == (wheel.negative_drive_sign < 0);
    if (positive_pwm || negative_pwm)
    {
      // 曲线插值得到前馈 PWM，再根据最新轮速反馈做有界修正；板端只收到带符号 PWM。
      const auto & speeds = positive_pwm ? wheel.speed : wheel.negative_speed;
      const auto & pwms = positive_pwm ? wheel.pwm : wheel.negative_pwm;
      const int feedback_sign = positive_pwm ? wheel.drive_sign : wheel.negative_drive_sign;
      const int command_sign = positive_pwm ? 1 : -1;
      const double target = std::min(std::abs(angular_velocity), speeds.back());
      auto upper = std::lower_bound(speeds.begin(), speeds.end(), target);
      const std::size_t point = static_cast<std::size_t>(upper - speeds.begin());
      const double low_speed = point ? speeds[point - 1] : 0.0;
      const double low_pwm = point ? pwms[point - 1] : 0.0;
      const double desired_pwm = low_pwm + (target - low_speed) *
        (pwms[point] - low_pwm) / (speeds[point] - low_speed);
      const double measured = feedback_sign * state_velocity_[joint(kWheelJoints[wheel_index])];
      const double correction = std::clamp(wheel.feedback_gain * (target - measured),
        -wheel.max_pwm_correction, wheel.max_pwm_correction);
      const int pwm = static_cast<int>(std::lround(std::clamp(
        desired_pwm + correction, 0.0, pwms.back())));
      if (!command_pwm_axis(dmc1_.get(), index, pwm * command_sign, command_timeout_ms_)) {
        return false;
      }
      continue;
    }
    // S 模式将 ROS rad/s 换成板端编码器计数/秒；零轮速也走此分支以明确停车。
    const double ticks = angular_velocity * wheel.speed_command_sign *
      encoder_cpr_[index] / (2.0 * kPi);
    if (!std::isfinite(ticks) || std::abs(ticks) > 32767.0) return false;
    if (!command_wheel_axis(
        dmc1_.get(), index, std::lround(ticks), command_timeout_ms_))
    {
      return false;
    }
  }
  return true;
}

// write 每周期调用；把四腿位置目标限制在已验证的 ±30° 内，换成百分之一度后发送 DMC0 R。
// 这里负责关节目标下发；真实角度在下一轮 read_legs 才更新。
bool RobotSerialSystem::write_legs()
{
  // 使用方法：DMC0 实测在约 ±30° 停住，先以保守的软件限位拦截更远目标。
  const std::array<double, 4> minimum = {-30.0, -30.0, -30.0, -30.0};
  const std::array<double, 4> maximum = {30.0, 30.0, 30.0, 30.0};
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

// write 每周期调用；读取头部两轴和四个转向轴的位置目标，按 f 查询得到的 ADC 反馈计算 PWM。
// SE2 每次收到完整的 U 六通道命令，字段顺序为头部 yaw/pitch、转向 FL/FR/RL/RR。
// 相同目标不会清除 PID 状态；目标变化超过 0.01° 才重置积分和卡滞检测。
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

// on_deactivate 和 latch_fault 调用；先停止 DMC1 定距/轮速，再给 SE2 六路发 1500 中性 PWM。
// 这是停机输出，不修改 DMC0 腿部位置目标；返回值用于报告停车事务是否全部成功。
bool RobotSerialSystem::stop_outputs()
{
  bool ok = stop_encoder_motion();
  std::string response;
  if (se2_) ok = se2_->transact("U 1500 1500 1500 1500 1500 1500 \n", response) && ok;
  return ok;
}

// T 动作取消、租约失效、故障或停用时调用；加锁防止与服务端提交 T 同时操作 DMC1。
// 撤销动作租约、清零 ROS 轮速目标，并逐轴发送 S 零速，防止 T 在服务结束后继续运动。
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

// read/write 或控制板回调遇到不可继续控制的错误时调用；关闭正常输出并尝试停车。
// 错误保持锁定，只有显式调用 /hardware/recover 且重新发现三块板成功才恢复。
void RobotSerialSystem::latch_fault(const std::string & reason)
{
  fault_ = reason;
  outputs_enabled_ = false;
  manual_recovery_required_ = true;
  (void)stop_outputs();
  RCLCPP_ERROR(rclcpp::get_logger("RobotSerialSystem"), "%s；已锁定，需调用 /hardware/recover", reason.c_str());
}

// read 每周期调用，包含读失败路径；发布 /hardware/chassis_state、八路 /ultrasonic/*
// 和 /diagnostics。无效测距发 NaN，供下游安全层识别数据缺失。
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

// write_servos 对每个 SE2 通道调用；根据目标角和最近一次 f 反馈计算板端 PWM。
// 误差进入停止死区时输出 1500；卡滞时暂时冷却并限制重试；其余情况执行带限幅的 PID。
// index=0/1 为头部，2..5 为转向；参数来自 servo_calibration.yaml。
int RobotSerialSystem::servo_pwm(std::size_t index, double dt)
{
  auto & servo = servos_[index];
  const double bounded_target = std::clamp(servo.target, servo.minimum, servo.maximum);
  if (std::abs(bounded_target - servo.target) > 1e-9) servo.target = bounded_target;
  double error = shortest_angle_deg(servo.target - servo.current);
  // 目标附近不继续追逐 ADC 抖动；返回中性 PWM，同时清除积分和卡滞状态。
  if (std::abs(error) < kPidDeadzoneDeg + servo.brake_deadzone) {
    servo.integral = 0.0;
    servo.last_error = 0.0;
    servo.stuck_state = 0;
    servo.stuck_attempts = 0;
    return kServoNeutral;
  }
  const auto now = std::chrono::steady_clock::now();
  // 大误差却无足够位移时先冷却，再有限次数重试；超过次数保持中性输出。
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
  // 仅在接近目标时积累积分，防止远距离误差导致积分饱和。
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

// read_servos_and_sonar 对每路 ADC 调用；先扣零位并处理一圈回绕，再按安装方向换算角度。
// 首帧直接初始化，其后以最短角差平滑反馈，最后按机械范围裁剪并转成 ROS 弧度。
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

// 初始化校验及所有读写路径按关节名查数组下标；缺少 URDF 关节时抛错，避免静默错位。
std::size_t RobotSerialSystem::joint(const std::string & name) const
{
  const auto iterator = joint_index_.find(name);
  if (iterator == joint_index_.end()) throw std::out_of_range("缺少关节: " + name);
  return iterator->second;
}

}  // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotSerialSystem, hardware_interface::SystemInterface)
