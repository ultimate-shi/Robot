#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace robot_stereo_components
{

class StereoSplitterNode final : public rclcpp::Node
{
public:
  StereoSplitterNode()
  : Node("stereo_splitter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/stereo/image_raw");
    left_topic_ = declare_parameter<std::string>(
      "left_image_topic", "/stereo/left/image_raw");
    right_topic_ = declare_parameter<std::string>(
      "right_image_topic", "/stereo/right/image_raw");
    left_info_topic_ = declare_parameter<std::string>(
      "left_camera_info_topic", "/stereo/left/camera_info");
    right_info_topic_ = declare_parameter<std::string>(
      "right_camera_info_topic", "/stereo/right/camera_info");
    left_frame_ = declare_parameter<std::string>(
      "left_frame_id", "stereo_left_optical_frame");
    right_frame_ = declare_parameter<std::string>(
      "right_frame_id", "stereo_right_optical_frame");
    left_first_ = declare_parameter<bool>("left_first", true);
    calibration_mode_ = declare_parameter<bool>("calibration_mode", false);
    recognition_rate_ = declare_parameter<double>("recognition_rate", 10.0);
    const auto left_calibration = declare_parameter<std::string>(
      "left_calibration_file", "");
    const auto right_calibration = declare_parameter<std::string>(
      "right_calibration_file", "");

    left_camera_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, "stereo_left", to_url(left_calibration));
    right_camera_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, "stereo_right", to_url(right_calibration));
    if (!calibration_mode_ &&
      (!left_camera_info_->isCalibrated() || !right_camera_info_->isCalibrated()))
    {
      throw std::runtime_error("双目标定文件未正确加载，拒绝发布未标定图像");
    }

    // 大图输入采用传感器 QoS；图像和 CameraInfo 使用完全相同的可靠单帧队列，
    // 避免处理链繁忙时 CameraInfo 独自积压并触发 image_proc 不同步告警。
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_, output_qos);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_, output_qos);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      left_info_topic_, output_qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      right_info_topic_, output_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_, input_qos,
      std::bind(&StereoSplitterNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "C++ 双目拆分器已启动，输入 %s", input_topic_.c_str());
  }

private:
  static std::string to_url(const std::string & path)
  {
    if (path.empty() || path.find("://") != std::string::npos) {
      return path;
    }
    return "file://" + path;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (msg->width < 2 || msg->width % 2 != 0 || msg->height == 0 || msg->step == 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拼接图尺寸无效: %ux%u step=%u", msg->width, msg->height, msg->step);
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!calibration_mode_ && recognition_rate_ > 0.0 && has_last_publish_) {
      const auto minimum = std::chrono::duration<double>(1.0 / recognition_rate_);
      if (now - last_publish_ < minimum) {
        return;
      }
    }
    last_publish_ = now;
    has_last_publish_ = true;

    const std::size_t half_width = msg->width / 2;
    if (msg->step % msg->width != 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "拼接图 step 不能整除 width，无法安全拆分");
      return;
    }
    const std::size_t bytes_per_pixel = msg->step / msg->width;
    const std::size_t half_step = half_width * bytes_per_pixel;
    const std::size_t required = static_cast<std::size_t>(msg->step) * msg->height;
    if (msg->data.size() < required) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "拼接图数据长度不足");
      return;
    }

    auto first = make_half_image(*msg, 0, half_width, half_step);
    auto second = make_half_image(*msg, half_step, half_width, half_step);
    sensor_msgs::msg::Image left;
    sensor_msgs::msg::Image right;
    if (left_first_) {
      left = std::move(first);
      right = std::move(second);
    } else {
      left = std::move(second);
      right = std::move(first);
    }
    left.header.frame_id = left_frame_;
    right.header.frame_id = right_frame_;
    auto left_info = left_camera_info_->getCameraInfo();
    auto right_info = right_camera_info_->getCameraInfo();
    left_info.header.stamp = msg->header.stamp;
    right_info.header.stamp = msg->header.stamp;
    left_info.header.frame_id = left_frame_;
    right_info.header.frame_id = right_frame_;
    left_image_pub_->publish(std::move(left));
    right_image_pub_->publish(std::move(right));
    left_info_pub_->publish(std::move(left_info));
    right_info_pub_->publish(std::move(right_info));
  }

  sensor_msgs::msg::Image make_half_image(
    const sensor_msgs::msg::Image & source, std::size_t source_offset,
    std::size_t width, std::size_t half_step) const
  {
    sensor_msgs::msg::Image output;
    output.header = source.header;
    output.height = source.height;
    output.width = static_cast<std::uint32_t>(width);
    output.encoding = source.encoding;
    output.is_bigendian = source.is_bigendian;
    output.step = static_cast<std::uint32_t>(half_step);
    output.data.resize(half_step * source.height);
    for (std::size_t row = 0; row < source.height; ++row) {
      const auto * begin = source.data.data() + row * source.step + source_offset;
      std::memcpy(output.data.data() + row * half_step, begin, half_step);
    }
    return output;
  }

  std::string input_topic_;
  std::string left_topic_;
  std::string right_topic_;
  std::string left_info_topic_;
  std::string right_info_topic_;
  std::string left_frame_;
  std::string right_frame_;
  bool left_first_{true};
  bool calibration_mode_{false};
  double recognition_rate_{10.0};
  bool has_last_publish_{false};
  std::chrono::steady_clock::time_point last_publish_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> left_camera_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> right_camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
};

}  // namespace robot_stereo_components

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_stereo_components::StereoSplitterNode>());
  rclcpp::shutdown();
  return 0;
}
