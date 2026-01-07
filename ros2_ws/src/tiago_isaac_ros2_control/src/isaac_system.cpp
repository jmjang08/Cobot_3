#include "tiago_isaac_ros2_control/isaac_system.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace tiago_isaac_ros2_control
{

hardware_interface::CallbackReturn IsaacSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // topics from <param> in URDF ros2_control block (optional)
  if (info_.hardware_parameters.count("joint_states_topic")) {
    joint_states_topic_ = info_.hardware_parameters.at("joint_states_topic");
  }
  if (info_.hardware_parameters.count("joint_command_topic")) {
    joint_command_topic_ = info_.hardware_parameters.at("joint_command_topic");
  }

  joint_names_.clear();
  for (const auto & j : info_.joints) {
    joint_names_.push_back(j.name);
  }

  const size_t n = joint_names_.size();
  state_pos_.assign(n, 0.0);
  state_vel_.assign(n, 0.0);
  cmd_pos_.assign(n, 0.0);

  // rclcpp node for pub/sub
  node_ = std::make_shared<rclcpp::Node>("isaac_ros2_control_hw");

  // QoS: match Isaac side (RELIABLE, KEEP_LAST 10)
  auto sub_qos = rclcpp::SensorDataQoS(); // best_effort, volatile
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();


  sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, sub_qos,
    std::bind(&IsaacSystem::joint_state_cb, this, std::placeholders::_1));

  pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_command_topic_, pub_qos);

  exec_.add_node(node_);
  spin_thread_ = std::thread([this]() { exec_.spin(); });

  RCLCPP_INFO(node_->get_logger(),
    "[IsaacSystem] init: joints=%zu, sub=%s, pub=%s",
    joint_names_.size(), joint_states_topic_.c_str(), joint_command_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IsaacSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 2);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &state_pos_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &state_vel_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> IsaacSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn IsaacSystem::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;
  RCLCPP_INFO(node_->get_logger(), "[IsaacSystem] activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  RCLCPP_INFO(node_->get_logger(), "[IsaacSystem] deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void IsaacSystem::joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  last_js_ = msg;
}

hardware_interface::return_type IsaacSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  sensor_msgs::msg::JointState::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    msg = last_js_;
  }
  if (!msg) {
    return hardware_interface::return_type::OK;
  }

  // Map incoming name[] -> our joint order
  // (O(n^2) but n small; ok for now. If needed, build name->index map once.)
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto & jn = joint_names_[i];
    for (size_t k = 0; k < msg->name.size(); ++k) {
      if (msg->name[k] == jn) {
        if (k < msg->position.size()) state_pos_[i] = msg->position[k];
        if (k < msg->velocity.size()) state_vel_[i] = msg->velocity[k];
        break;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }

  // Always publish ALL joints (safe for Isaac implementations that expect full vectors)
  sensor_msgs::msg::JointState out;
  out.header.stamp = node_->now();
  out.name = joint_names_;
  out.position = cmd_pos_;  // only position commands
  // leave velocity/effort empty

  pub_->publish(out);
  return hardware_interface::return_type::OK;
}

}  // namespace tiago_isaac_ros2_control

PLUGINLIB_EXPORT_CLASS(tiago_isaac_ros2_control::IsaacSystem, hardware_interface::SystemInterface)
