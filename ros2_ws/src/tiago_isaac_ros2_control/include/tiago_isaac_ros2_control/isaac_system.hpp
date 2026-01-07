#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <mutex>
#include <thread>
#include <vector>
#include <string>

namespace tiago_isaac_ros2_control
{

class IsaacSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  std::thread spin_thread_;

  std::string joint_states_topic_ = "/joint_states";
  std::string joint_command_topic_ = "/joint_command";

  std::vector<std::string> joint_names_;
  std::vector<double> state_pos_, state_vel_;
  std::vector<double> cmd_pos_;

  std::mutex mtx_;
  sensor_msgs::msg::JointState::SharedPtr last_js_;
  bool active_{false};
};

}  // namespace tiago_isaac_ros2_control
