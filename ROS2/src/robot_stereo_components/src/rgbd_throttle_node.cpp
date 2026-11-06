// 作用：限速并校验已配准的 RGB-D，统一供里程计、语义和点云使用；用法：astra_camera.launch.py 加载 rgbd_throttle_cpp。
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class RGBDThrottle : public rclcpp::Node
{
public:
  RGBDThrottle() : Node("rgbd_throttle_cpp")
  {
    rate_ = declare_parameter("max_rate", 8.0);
    max_age_ = declare_parameter("max_age", 1.5);
    skew_ = declare_parameter("max_rgb_depth_skew", 0.05);
    min_depth_ = declare_parameter("min_depth", 0.4);
    max_depth_ = declare_parameter("max_depth", 4.0);
    if (!(rate_ > 0 && max_age_ > 0 && skew_ > 0 && min_depth_ > 0 && max_depth_ > min_depth_)) {
      throw std::invalid_argument("RGB-D 频率、时间和深度范围必须有效");
    }
    auto qos = rclcpp::QoS(1).reliable();
    bundle_ = create_publisher<rtabmap_msgs::msg::RGBDImage>("rgbd_out", qos);
    rgb_ = create_publisher<sensor_msgs::msg::Image>("rgb_out", qos);
    depth_ = create_publisher<sensor_msgs::msg::Image>("depth_out", qos);
    info_ = create_publisher<sensor_msgs::msg::CameraInfo>("info_out", qos);
    preview_ = create_publisher<sensor_msgs::msg::Image>("preview_out", qos);
    input_ = create_subscription<rtabmap_msgs::msg::RGBDImage>("rgbd_in", qos,
      [this](rtabmap_msgs::msg::RGBDImage::ConstSharedPtr msg) { callback(msg); });
  }
private:
  void callback(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & msg)
  {
    const auto stamp = rclcpp::Time(msg->rgb.header.stamp);
    const auto depth_stamp = rclcpp::Time(msg->depth.header.stamp);
    const auto steady = std::chrono::steady_clock::now();
    const double age = (now() - stamp).seconds();
    const double depth_age = (now() - depth_stamp).seconds();
    if (stamp.nanoseconds() <= last_stamp_ || age < 0 || age > max_age_ ||
      depth_age < 0 || depth_age > max_age_ || std::abs((stamp-depth_stamp).seconds()) > skew_ ||
      msg->rgb.header.frame_id.empty() || msg->rgb.header.frame_id != msg->depth.header.frame_id ||
      msg->rgb.width == 0 || msg->rgb.width != msg->depth.width || msg->rgb.height != msg->depth.height ||
      !(msg->rgb_camera_info.k[0] > 0 && msg->rgb_camera_info.k[4] > 0))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "拒绝过时、未对齐或未标定的 RGB-D");
      return;
    }
    if (last_publish_.time_since_epoch().count() &&
      std::chrono::duration<double>(steady-last_publish_).count() < 1.0/rate_) {return;}
    try {
      auto view = cv_bridge::toCvCopy(msg->depth);
      cv::Mat meters;
      if (msg->depth.encoding == "16UC1") {view->image.convertTo(meters, CV_32FC1, 0.001);}
      else if (msg->depth.encoding == "32FC1") {meters = view->image;}
      else {throw std::runtime_error("深度只支持毫米 16UC1 或米制 32FC1");}
      cv::Mat preview(meters.rows, meters.cols, CV_8UC1, cv::Scalar(0));
      for (int row = 0; row < meters.rows; ++row) {
        for (int col = 0; col < meters.cols; ++col) {
          const float z = meters.at<float>(row, col);
          if (!std::isfinite(z) || z <= 0) {
            meters.at<float>(row, col) = std::numeric_limits<float>::quiet_NaN();
          }
          if (std::isfinite(z) && z >= min_depth_ && z <= max_depth_) {
            preview.at<unsigned char>(row, col) = static_cast<unsigned char>(
              1 + 254*(max_depth_-z)/(max_depth_-min_depth_));
          }
        }
      }
      // 所有输出保留各自采集时间；不为强行同步重打时间戳。
      bundle_->publish(*msg);
      rgb_->publish(msg->rgb);
      // 语义链沿用米制 32FC1 接口；RGB-D 原包仍保留设备原始深度单位，RTAB-Map 原生支持。
      depth_->publish(*cv_bridge::CvImage(msg->depth.header, "32FC1", meters).toImageMsg());
      auto rectified_info = msg->rgb_camera_info;
      if (rectified_info.p[0] > 0 && rectified_info.p[5] > 0) {
        // 语义节点按 K 反投影；校正后的图像应使用 P 的内参，不再使用原畸变 K/D。
        rectified_info.k = {rectified_info.p[0], rectified_info.p[1], rectified_info.p[2],
          rectified_info.p[4], rectified_info.p[5], rectified_info.p[6],
          rectified_info.p[8], rectified_info.p[9], rectified_info.p[10]};
        rectified_info.d.assign(rectified_info.d.size(), 0.0);
        rectified_info.r = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
      }
      info_->publish(rectified_info);
      preview_->publish(*cv_bridge::CvImage(msg->depth.header, "mono8", preview).toImageMsg());
      last_stamp_ = stamp.nanoseconds();
      last_publish_ = steady;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "RGB-D 输入无效：%s", e.what());
    }
  }
  double rate_, max_age_, skew_, min_depth_, max_depth_;
  int64_t last_stamp_{-1};
  std::chrono::steady_clock::time_point last_publish_{};
  rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr input_;
  rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr bundle_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_, depth_, preview_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RGBDThrottle>());
  rclcpp::shutdown();
  return 0;
}
