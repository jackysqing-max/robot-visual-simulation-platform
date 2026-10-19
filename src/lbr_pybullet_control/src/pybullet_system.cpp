#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace lbr_pybullet_control
{

// This adapter deliberately reads simulator measurements, never echoing commands as state.
// The controller manager runs read -> controller update (including JTC interpolation) -> write.
class PyBulletSystem final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
    try {
      if (info_.joints.size() != kJoints) {
        throw std::runtime_error("Expected exactly seven LBR joints");
      }
      constexpr double deg = 3.14159265358979323846 / 180.0;
      const std::array<double, kJoints> upper_degrees{170, 120, 170, 120, 170, 120, 175};
      for (std::size_t i = 0; i < kJoints; ++i) {
        const auto & joint = info_.joints[i];
        if (joint.name != "lbr_A" + std::to_string(i + 1)) {
          throw std::runtime_error("Expected ordered joint names lbr_A1 through lbr_A7");
        }
        if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces.front().name != hardware_interface::HW_IF_POSITION)
        {
          throw std::runtime_error("Each joint requires one position command interface");
        }
        if (joint.state_interfaces.size() != 2 ||
          std::count_if(joint.state_interfaces.begin(), joint.state_interfaces.end(),
            [](const auto & iface) {return iface.name == hardware_interface::HW_IF_POSITION;}) != 1 ||
          std::count_if(joint.state_interfaces.begin(), joint.state_interfaces.end(),
            [](const auto & iface) {return iface.name == hardware_interface::HW_IF_VELOCITY;}) != 1)
        {
          throw std::runtime_error("Each joint requires position and velocity state interfaces");
        }
        const auto & command_interface = joint.command_interfaces.front();
        lower_[i] = command_interface.min.empty() ? -upper_degrees[i] * deg :
          std::stod(command_interface.min);
        upper_[i] = command_interface.max.empty() ? upper_degrees[i] * deg :
          std::stod(command_interface.max);
        if (!std::isfinite(lower_[i]) || !std::isfinite(upper_[i]) || lower_[i] >= upper_[i]) {
          throw std::runtime_error("Invalid joint position limits");
        }
      }
      command_topic_ = parameter("command_topic", "/iiwa7/joint_desired");
      state_topic_ = parameter("state_topic", "/iiwa7/joint_states");
      state_timeout_ = std::stod(parameter("state_timeout", "0.5"));
      activation_timeout_ = std::stod(parameter("activation_timeout", "10.0"));
      if (command_topic_.empty() || state_topic_.empty() ||
        !std::isfinite(state_timeout_) || state_timeout_ <= 0 ||
        !std::isfinite(activation_timeout_) || activation_timeout_ <= 0)
      {
        throw std::runtime_error("Topics must be nonempty and timeouts must be positive and finite");
      }

      // Avoid inheriting ros2_control_node's __node remapping or its parameter overrides.
      rclcpp::NodeOptions options;
      options.use_global_arguments(false);
      node_ = std::make_shared<rclcpp::Node>("lbr_pybullet_hardware", options);
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);
      publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        command_topic_, rclcpp::QoS(1));
      subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        state_topic_, rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {receive_state(*msg);});
      RCLCPP_INFO(node_->get_logger(), "PyBullet bridge: %s -> ros2_control -> %s",
        state_topic_.c_str(), command_topic_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger("lbr_pybullet_control"), "%s", error.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t i = 0; i < kJoints; ++i) {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_[i]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t i = 0; i < kJoints; ++i) {
      interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_[i]);
    }
    return interfaces;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    feedback_fault_ = false;
    have_state_ = false;
    const auto deadline = Clock::now() + std::chrono::duration<double>(activation_timeout_);
    RCLCPP_INFO(node_->get_logger(), "Waiting for measured PyBullet state (timeout %.1f s)",
      activation_timeout_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      executor_->spin_some();
      if (state_fresh()) {
        command_ = position_;
        active_ = true;
        RCLCPP_INFO(node_->get_logger(), "Activated at the measured joint positions");
        return hardware_interface::CallbackReturn::SUCCESS;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    feedback_fault_ = true;
    RCLCPP_ERROR(node_->get_logger(), "No fresh valid joint feedback received on %s",
      state_topic_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    feedback_fault_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    executor_->spin_some();
    if (active_ && feedback_fault_) {
      return hardware_interface::return_type::ERROR;
    }
    if (active_ && !state_fresh()) {
      feedback_fault_ = true;
      RCLCPP_ERROR(node_->get_logger(), "PyBullet feedback is stale (> %.3f s); commands disabled",
        state_timeout_);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) {
      return hardware_interface::return_type::OK;
    }
    if (feedback_fault_ || !state_fresh()) {
      feedback_fault_ = true;
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t i = 0; i < kJoints; ++i) {
      if (!std::isfinite(command_[i]) || command_[i] < lower_[i] || command_[i] > upper_[i]) {
        feedback_fault_ = true;
        RCLCPP_ERROR(node_->get_logger(), "Invalid position command for %s; commands disabled",
          info_.joints[i].name.c_str());
        return hardware_interface::return_type::ERROR;
      }
    }
    std_msgs::msg::Float64MultiArray msg;
    msg.data.assign(command_.begin(), command_.end());
    publisher_->publish(msg);
    return hardware_interface::return_type::OK;
  }

private:
  static constexpr std::size_t kJoints = 7;
  using Clock = std::chrono::steady_clock;

  std::string parameter(const std::string & name, const std::string & fallback) const
  {
    const auto found = info_.hardware_parameters.find(name);
    return found == info_.hardware_parameters.end() ? fallback : found->second;
  }

  bool state_fresh() const
  {
    return have_state_ && std::chrono::duration<double>(Clock::now() - last_state_).count() <=
           state_timeout_;
  }

  void receive_state(const sensor_msgs::msg::JointState & msg)
  {
    // A malformed packet must not partially overwrite the last complete measurement.
    if (msg.name.size() != msg.position.size() || msg.name.size() != msg.velocity.size()) {
      return;
    }
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (!indices.emplace(msg.name[i], i).second) {
        return;
      }
    }
    std::array<double, kJoints> next_position{}, next_velocity{};
    for (std::size_t i = 0; i < kJoints; ++i) {
      const auto found = indices.find("iiwa_joint_" + std::to_string(i + 1));
      if (found == indices.end()) {
        return;
      }
      const auto j = found->second;
      if (!std::isfinite(msg.position[j]) || !std::isfinite(msg.velocity[j])) {
        return;
      }
      next_position[i] = msg.position[j];
      next_velocity[i] = msg.velocity[j];
    }
    position_ = next_position;
    velocity_ = next_velocity;
    last_state_ = Clock::now();
    have_state_ = true;
  }

  std::array<double, kJoints> position_{}, velocity_{}, command_{}, lower_{}, upper_{};
  std::string command_topic_, state_topic_;
  double state_timeout_{0.5}, activation_timeout_{10.0};
  bool active_{false}, feedback_fault_{false}, have_state_{false};
  Clock::time_point last_state_{};
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

}  // namespace lbr_pybullet_control

PLUGINLIB_EXPORT_CLASS(lbr_pybullet_control::PyBulletSystem, hardware_interface::SystemInterface)
