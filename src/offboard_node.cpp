#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl() : Node("offboard_control")
  {
    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    warmup_setpoints_ = declare_parameter<int>("warmup_setpoints", 10);
    position_tolerance_ = declare_parameter<double>("position_tolerance", 0.1);
    local_position_topic_ =
      declare_parameter<std::string>("local_position_topic", "/fmu/out/vehicle_local_position_v1");

    target_system_ = static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
    target_component_ = static_cast<uint8_t>(declare_parameter<int>("target_component", 1));
    source_system_ = static_cast<uint8_t>(declare_parameter<int>("source_system", 1));
    source_component_ = static_cast<uint8_t>(declare_parameter<int>("source_component", 1));

    auto in_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
    auto out_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", in_qos);

    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", in_qos);

    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", in_qos);

    vehicle_local_position_sub_ =
      create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        local_position_topic_,
        out_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          current_position_ = {msg->x, msg->y, msg->z};
          has_position_ = true;

          RCLCPP_INFO(
            get_logger(),
            "Current position: x=%.3f, y=%.3f, z=%.3f",
            msg->x,
            msg->y,
            msg->z);
        });

    state_enter_time_ = now();
    log_state_entry();

    timer_ = create_wall_timer(100ms, [this]() { timer_callback(); });

    RCLCPP_INFO(
      get_logger(),
      "Offboard node ready. waypoint1=(0.00, 0.00, -1.00), waypoint2=(0.00, 1.50, -1.00), waypoint3=(1.50, 1.50, -1.00)");
  }

private:
  enum class State
  {
    Warmup,
    FlyToFirstPoint,
    FlyToSecondPoint,
    FlyToThirdPoint,
    Done,
  };

  void timer_callback()
  {
    publish_offboard_control_mode();
    publish_trajectory_setpoint(current_target());

    if (setpoint_counter_ < warmup_setpoints_) {
      ++setpoint_counter_;
      return;
    }

    if (state_ == State::Warmup) {
      if (auto_start_ && !offboard_command_sent_) {
        engage_offboard_mode();
        offboard_command_sent_ = true;
      }

      if (auto_arm_ && !arm_command_sent_) {
        arm();
        arm_command_sent_ = true;
      }

      enter_state(State::FlyToFirstPoint);
      return;
    }

    if (state_ == State::FlyToFirstPoint && reached_target(first_point_)) {
      RCLCPP_INFO(get_logger(), "Reached first target: (0.00, 0.00, -1.00)");
      enter_state(State::FlyToSecondPoint);
      return;
    }

    if (state_ == State::FlyToSecondPoint && reached_target(second_point_)) {
      RCLCPP_INFO(get_logger(), "Reached second target: (0.00, 1.50, -1.00)");
      enter_state(State::FlyToThirdPoint);
      return;
    }

    if (state_ == State::FlyToThirdPoint && reached_target(third_point_)) {
      RCLCPP_INFO(get_logger(), "Reached third target: (1.50, 1.50, -1.00)");
      enter_state(State::Done);
      return;
    }
  }

  bool reached_target(const std::array<float, 3> & target) const
  {
    if (!has_position_) {
      return false;
    }

    return std::abs(current_position_[0] - target[0]) <= position_tolerance_ &&
           std::abs(current_position_[1] - target[1]) <= position_tolerance_ &&
           std::abs(current_position_[2] - target[2]) <= position_tolerance_;
  }

  void enter_state(State next_state)
  {
    if (state_ == next_state) {
      return;
    }

    state_ = next_state;
    state_enter_time_ = now();
    log_state_entry();
  }

  void log_state_entry() const
  {
    const auto target = current_target();

    RCLCPP_INFO(get_logger(), "Enter state: %s", state_name(state_));
    RCLCPP_INFO(
      get_logger(),
      "Target setpoint: (%.2f, %.2f, %.2f)",
      target[0],
      target[1],
      target[2]);
  }

  const char * state_name(State state) const
  {
    switch (state) {
      case State::Warmup:
        return "Warmup";

      case State::FlyToFirstPoint:
        return "FlyToFirstPoint";

      case State::FlyToSecondPoint:
        return "FlyToSecondPoint";

      case State::FlyToThirdPoint:
        return "FlyToThirdPoint";

      case State::Done:
        return "Done";
    }

    return "Unknown";
  }

  std::array<float, 3> current_target() const
  {
    switch (state_) {
      case State::Warmup:
      case State::FlyToFirstPoint:
        return first_point_;

      case State::FlyToSecondPoint:
        return second_point_;

      case State::FlyToThirdPoint:
      case State::Done:
        return third_point_;
    }

    return first_point_;
  }

  void arm()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
      1.0F);

    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void engage_offboard_mode()
  {
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0F,
      6.0F);

    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  void publish_offboard_control_mode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = timestamp_us();

    offboard_control_mode_pub_->publish(msg);
  }

  void publish_trajectory_setpoint(const std::array<float, 3> & position)
  {
    const auto nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = position;
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.yaw = nan;
    msg.yawspeed = nan;
    msg.timestamp = timestamp_us();

    trajectory_setpoint_pub_->publish(msg);
  }

  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0F,
    float param2 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = target_system_;
    msg.target_component = target_component_;
    msg.source_system = source_system_;
    msg.source_component = source_component_;
    msg.from_external = true;
    msg.timestamp = timestamp_us();

    vehicle_command_pub_->publish(msg);
  }

  uint64_t timestamp_us() const
  {
    return static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;

  State state_{State::Warmup};
  rclcpp::Time state_enter_time_;

  int setpoint_counter_{0};
  int warmup_setpoints_{10};

  bool auto_start_{false};
  bool auto_arm_{false};
  bool offboard_command_sent_{false};
  bool arm_command_sent_{false};
  bool has_position_{false};

  double position_tolerance_{0.1};

  std::string local_position_topic_;

  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};

  const std::array<float, 3> first_point_{0.0F, 0.0F, -1.0F};
  const std::array<float, 3> second_point_{0.0F, 1.5F, -1.0F};
  const std::array<float, 3> third_point_{1.5F, 1.5F, -1.0F};

  std::array<float, 3> current_position_{0.0F, 0.0F, 0.0F};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}