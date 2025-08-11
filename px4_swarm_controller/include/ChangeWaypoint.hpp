#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <yaml-cpp/yaml.h>

class ChangeWaypoint : public rclcpp::Node {
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

public:
    ChangeWaypoint();

private:
    void publish_offboard_control_mode();
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose);
    double coord(const size_t idx, const std::string &var, bool use_translated = false);
    void writeWP(const std::size_t idx);
    void update_trajectory_setpoint();

    enum class State { MOVING, WAITING, TRANSITIONING };
    State state = State::MOVING;
    bool proceed_received = false;
    bool use_translated_waypoints = false;
    bool enable_translation = false; // پارامتر برای فعال‌سازی انتقال
    double transition_start_time = 0.0;
    double transition_duration = 2.0;
    TrajectorySetpoint previous_waypoint;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pose_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr proceed_subscriber_;
    rclcpp::TimerBase::SharedPtr transition_timer;

    TrajectorySetpoint waypoint;
    std::size_t wp_idx{0u};
    YAML::Node waypoints;
    YAML::Node translated_waypoints;
    double threshold{};
    double threshold_angle{};
    double x_init{};
    double y_init{};
};