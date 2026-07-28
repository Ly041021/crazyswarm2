#include <cmath>
#include <memory>
#include <stdexcept>

#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "rclcpp/rclcpp.hpp"

class MocapPositionScaler : public rclcpp::Node
{
public:
  MocapPositionScaler()
  : Node("mocap_position_scaler")
  {
    scale_ = declare_parameter<double>("position_scale", 1.0);
    const double deadline_hz = declare_parameter<double>("deadline_hz", 100.0);
    if (!std::isfinite(scale_) || scale_ <= 0.0) {
      throw std::invalid_argument("position_scale must be finite and greater than zero");
    }
    if (!std::isfinite(deadline_hz) || deadline_hz <= 0.0) {
      throw std::invalid_argument("deadline_hz must be finite and greater than zero");
    }

    auto qos = rclcpp::SensorDataQoS();
    qos.deadline(rclcpp::Duration::from_seconds(1.0 / deadline_hz));
    publisher_ = create_publisher<NamedPoseArray>("poses", qos);
    subscription_ = create_subscription<NamedPoseArray>(
      "poses_raw", qos,
      [this](const NamedPoseArray::SharedPtr msg) {
        auto scaled = *msg;
        for (auto & named_pose : scaled.poses) {
          named_pose.pose.position.x *= scale_;
          named_pose.pose.position.y *= scale_;
          named_pose.pose.position.z *= scale_;
        }
        publisher_->publish(scaled);
      });

    RCLCPP_INFO(
      get_logger(), "Scaling mocap positions by %.6f at %.1f Hz: poses_raw -> poses",
      scale_, deadline_hz);
  }

private:
  using NamedPoseArray = motion_capture_tracking_interfaces::msg::NamedPoseArray;

  double scale_;
  rclcpp::Publisher<NamedPoseArray>::SharedPtr publisher_;
  rclcpp::Subscription<NamedPoseArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapPositionScaler>());
  rclcpp::shutdown();
  return 0;
}
