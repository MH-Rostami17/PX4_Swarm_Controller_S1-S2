#include "ChangeWaypoint.hpp"
#include <cmath>

ChangeWaypoint::ChangeWaypoint() : Node("waypoint") {
    const std::string name_space{this->get_namespace()};
    this->declare_parameter<std::string>("wp_path");
    this->declare_parameter<double>("x_init");
    this->declare_parameter<double>("y_init");
    this->declare_parameter<bool>("enable_translation", false);

    const auto wp_path{this->get_parameter("wp_path").as_string()};
    x_init = this->get_parameter("x_init").as_double();
    y_init = this->get_parameter("y_init").as_double();
    enable_translation = this->get_parameter("enable_translation").as_bool();

    RCLCPP_INFO(this->get_logger(), "enable_translation: %s, wp_path: %s", enable_translation ? "true" : "false", wp_path.c_str());

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
        name_space + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
        name_space + "/fmu/in/trajectory_setpoint", 10);
    waypoint_reached_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        name_space + "/waypoint_reached", 10);
    proceed_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/sync/proceed", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
            proceed_received = msg->data;
        });

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
    pose_subscriber = this->create_subscription<VehicleLocalPosition>(
        name_space + "/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            pose_subscriber_callback(msg);
        });

    // بارگذاری فایل نقاط اولیه
    try {
        waypoints = YAML::LoadFile(wp_path);
        threshold = waypoints["threshold"].as<double>();
        threshold_angle = waypoints["threshold_angle"].as<double>();
        waypoints = waypoints["wp"][name_space];
        RCLCPP_INFO(this->get_logger(), "Successfully loaded waypoints from: %s", wp_path.c_str());
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints: %s", e.what());
        return;
    }

    // بارگذاری فایل نقاط منتقل‌شده فقط اگر انتقال فعال باشد
    if (enable_translation) {
        std::string translated_wp_path = wp_path;
        size_t pos = translated_wp_path.find("waypoints_auto.yaml");
        if (pos != std::string::npos) {
            translated_wp_path.replace(pos, std::string("waypoints_auto.yaml").length(), "waypoints_translated.yaml");
        }
        RCLCPP_INFO(this->get_logger(), "Loading translated waypoints from: %s", translated_wp_path.c_str());
        try {
            translated_waypoints = YAML::LoadFile(translated_wp_path);
            translated_waypoints = translated_waypoints["wp"][name_space];
            RCLCPP_INFO(this->get_logger(), "Successfully loaded translated waypoints");
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load translated waypoints: %s", e.what());
            return;
        }

        // تنظیم تایمر برای شروع انتقال بعد از 30 ثانیه
        transition_timer = this->create_wall_timer(
            std::chrono::seconds(30),
            [this]() {
                use_translated_waypoints = true;
                transition_start_time = this->get_clock()->now().seconds();
                state = State::TRANSITIONING;
                previous_waypoint = waypoint;
                RCLCPP_INFO(this->get_logger(), "Transitioning to translated waypoints");
            });
    }

    // شروع با نقطه اول
    writeWP(wp_idx);
}

void ChangeWaypoint::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void ChangeWaypoint::pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose) {
    if (state == State::MOVING) {
        double distance = std::hypot(pose->x - (waypoint.position[0] + y_init),
                                     pose->y - (waypoint.position[1] + x_init),
                                     pose->z - waypoint.position[2]);
        double current_theta = pose->heading;
        double theta_d = waypoint.yaw;

        if (distance < threshold && std::abs(std::fmod(current_theta - theta_d + M_PI, 2 * M_PI) - M_PI) < threshold_angle) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            waypoint_reached_publisher_->publish(msg);
            state = State::WAITING;
        }
    } else if (state == State::WAITING && proceed_received) {
        proceed_received = false;
        wp_idx = (wp_idx + 1) % (use_translated_waypoints ? std::size(translated_waypoints) : std::size(waypoints));
        writeWP(wp_idx);
        std_msgs::msg::Bool msg;
        msg.data = false;
        waypoint_reached_publisher_->publish(msg);
        state = State::MOVING;
    }

    update_trajectory_setpoint();
}

double ChangeWaypoint::coord(const size_t idx, const std::string &var, bool use_translated) {
    auto wp_list = use_translated ? translated_waypoints : waypoints;
    return wp_list[idx][var].as<double>();
}

void ChangeWaypoint::writeWP(const std::size_t idx) {
    waypoint.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
    waypoint.position = {static_cast<float>(coord(idx, "x", use_translated_waypoints) - y_init),
                         static_cast<float>(coord(idx, "y", use_translated_waypoints) - x_init),
                         static_cast<float>(coord(idx, "z", use_translated_waypoints))};
    waypoint.yaw = static_cast<float>(coord(idx, "yaw", use_translated_waypoints));
}

void ChangeWaypoint::update_trajectory_setpoint() {
    if (enable_translation && state == State::TRANSITIONING) {
        double current_time = this->get_clock()->now().seconds();
        double elapsed = current_time - transition_start_time;
        double t = std::min(elapsed / transition_duration, 1.0);

        TrajectorySetpoint target_waypoint;
        target_waypoint.position = {static_cast<float>(coord(wp_idx, "x", true) - y_init),
                                   static_cast<float>(coord(wp_idx, "y", true) - x_init),
                                   static_cast<float>(coord(wp_idx, "z", true))};
        target_waypoint.yaw = static_cast<float>(coord(wp_idx, "yaw", true));

        waypoint.position[0] = previous_waypoint.position[0] + t * (target_waypoint.position[0] - previous_waypoint.position[0]);
        waypoint.position[1] = previous_waypoint.position[1] + t * (target_waypoint.position[1] - previous_waypoint.position[1]);
        waypoint.position[2] = previous_waypoint.position[2] + t * (target_waypoint.position[2] - previous_waypoint.position[2]);
        waypoint.yaw = previous_waypoint.yaw + t * (target_waypoint.yaw - previous_waypoint.yaw);

        RCLCPP_INFO(this->get_logger(), "Transitioning to translated waypoints at t=%f", t);

        if (t >= 1.0) {
            state = State::MOVING;
            RCLCPP_INFO(this->get_logger(), "Transition to translated waypoints completed");
        }
    }

    publish_offboard_control_mode();
    trajectory_setpoint_publisher_->publish(waypoint);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChangeWaypoint>());
    rclcpp::shutdown();
    return 0;
}