#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

using namespace std::chrono_literals;

class SimpleTakeoff : public rclcpp::Node
{
public:
  SimpleTakeoff() : Node("simple_takeoff")
  {
    auto_start_ = declare_parameter<bool>("auto_start", false);
    auto_arm_ = declare_parameter<bool>("auto_arm", false);
    warmup_setpoints_ = declare_parameter<int>("warmup_setpoints", 10);
    target_z_ = declare_parameter<double>("target_z", -1.0);
    next_target_x_ = declare_parameter<double>("next_target_x", 1.5);
    next_target_y_ = declare_parameter<double>("next_target_y", 0.0);
    next_target_z_ = declare_parameter<double>("next_target_z", -1.0);
    hover_duration_ = declare_parameter<double>("hover_duration", 5.0);
    acceptance_xy_ = declare_parameter<double>("acceptance_xy", 0.15);
    acceptance_z_ = declare_parameter<double>("acceptance_z", 0.15);

    target_system_ = static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
    target_component_ = static_cast<uint8_t>(declare_parameter<int>("target_component", 1));
    source_system_ = static_cast<uint8_t>(declare_parameter<int>("source_system", 1));
    source_component_ = static_cast<uint8_t>(declare_parameter<int>("source_component", 1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();

    offboard_control_mode_pub_ =
      create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
    trajectory_setpoint_pub_ =
      create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
    vehicle_command_pub_ =
      create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);

    vehicle_local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        vehicle_local_position_ = *msg;
      });

    state_enter_time_ = now();
    timer_ = create_wall_timer(100ms, [this]() { timer_callback(); });

    RCLCPP_INFO(
      get_logger(),
      "Simple takeoff loaded. Convention: +x=front, +y=right, -z=up. takeoff=(0.0, 0.0, %.2f), right_target=(%.2f, %.2f, %.2f), hover=%.1fs, auto_start=%s, auto_arm=%s",
      target_z_,
      next_target_x_,
      next_target_y_,
      next_target_z_,
      hover_duration_,
      auto_start_ ? "true" : "false",
      auto_arm_ ? "true" : "false");
  }

private:
  enum class State
  {
    Warmup,
    Takeoff,
    Hover,
    FlyRight,
    Done,
  };

  void timer_callback()
  {
    publish_offboard_control_mode();
    publish_trajectory_setpoint(current_target_x(), current_target_y(), current_target_z());

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
      enter_state(State::Takeoff);
    }

    if (state_ == State::Takeoff && reached_target_z()) {
      RCLCPP_INFO(get_logger(), "Reached takeoff point: (0.00, 0.00, %.2f)", target_z_);
      enter_state(State::Hover);
    }

    if (state_ == State::Hover && elapsed_in_state() >= hover_duration_) {
      RCLCPP_INFO(get_logger(), "Hover finished: %.1fs", hover_duration_);
      enter_state(State::FlyRight);
    }

    if (state_ == State::FlyRight && reached_next_target()) {
      RCLCPP_INFO(
        get_logger(),
        "Reached right target: (%.2f, %.2f, %.2f)",
        next_target_x_,
        next_target_y_,
        next_target_z_);
      enter_state(State::Done);
    }
  }

  bool reached_target_z() const
  {
    if (!vehicle_local_position_) {
      return elapsed_in_state() > 3.0;
    }

    return std::abs(vehicle_local_position_->z - target_z_) <= acceptance_z_;
  }

  bool reached_next_target() const
  {
    if (!vehicle_local_position_) {
      return elapsed_in_state() > 3.0;
    }

    const auto dx = vehicle_local_position_->x - next_target_x_;
    const auto dy = vehicle_local_position_->y - next_target_y_;
    const auto dz = vehicle_local_position_->z - next_target_z_;
    return std::hypot(dx, dy) <= acceptance_xy_ && std::abs(dz) <= acceptance_z_;
  }

  void enter_state(State next_state)
  {
    if (state_ == next_state) {
      return;
    }

    state_ = next_state;
    state_enter_time_ = now();
    RCLCPP_INFO(get_logger(), "Simple takeoff state: %s", state_name(state_));
    RCLCPP_INFO(
      get_logger(),
      "Current target: (%.2f, %.2f, %.2f)",
      current_target_x(),
      current_target_y(),
      current_target_z());
  }

  const char * state_name(State state) const
  {
    switch (state) {
      case State::Warmup:
        return "Warmup";
      case State::Takeoff:
        return "Takeoff";
      case State::Hover:
        return "Hover";
      case State::FlyRight:
        return "FlyRight";
      case State::Done:
        return "Done";
    }
    return "Unknown";
  }

  double current_target_x() const
  {
    return state_ == State::FlyRight || state_ == State::Done ? next_target_x_ : 0.0;
  }

  double current_target_y() const
  {
    return state_ == State::FlyRight || state_ == State::Done ? next_target_y_ : 0.0;
  }

  double current_target_z() const
  {
    return state_ == State::FlyRight || state_ == State::Done ? next_target_z_ : target_z_;
  }

  double elapsed_in_state() const
  {
    return (now() - state_enter_time_).seconds();
  }

  void arm()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void engage_offboard_mode()
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
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

  void publish_trajectory_setpoint(double x, double y, double z)
  {
    const auto nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {
      static_cast<float>(x),
      static_cast<float>(y),
      static_cast<float>(z)};
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

  std::optional<px4_msgs::msg::VehicleLocalPosition> vehicle_local_position_;

  State state_{State::Warmup};
  rclcpp::Time state_enter_time_;
  int setpoint_counter_{0};
  int warmup_setpoints_{10};
  bool auto_start_{false};
  bool auto_arm_{false};
  bool offboard_command_sent_{false};
  bool arm_command_sent_{false};
  double target_z_{-1.0};
  double next_target_x_{0.0};
  double next_target_y_{1.5};
  double next_target_z_{-1.0};
  double hover_duration_{5.0};
  double acceptance_xy_{0.15};
  double acceptance_z_{0.15};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTakeoff>());
  rclcpp::shutdown();
  return 0;
}
