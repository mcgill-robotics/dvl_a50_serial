#include <string>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <marine_acoustic_msgs/msg/dvl.hpp>

#include "dvl_a50_serial/dvl_a50_serial.hpp"

using namespace dvl_a50_serial;

class DvlA50SerialNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    DvlA50SerialNode(std::string name)
    : rclcpp_lifecycle::LifecycleNode(name)
    {
        service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        this->declare_parameter<bool>("autostart", true);
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("frame", "dvl_a50_link");
        this->declare_parameter<double>("rate", 30.0); 
        this->declare_parameter<bool>("enable_on_activate", false);
        this->declare_parameter<int>("speed_of_sound", 1500);
        this->declare_parameter<bool>("led_enabled", true);
        this->declare_parameter<int>("mounting_rotation_offset", 0);
        this->declare_parameter<std::string>("range_mode", "auto");
        this->declare_parameter<int>("timeout_configure_ms", 3000);
        this->declare_parameter<int>("timeout_reset_dead_reckoning_ms", 3000);
        this->declare_parameter<int>("timeout_calibrate_gyro_ms", 15000);
        this->declare_parameter<int>("timeout_trigger_ping_ms", 3000);
        this->declare_parameter<int>("timeout_set_protocol_ms", 3000);
        this->declare_parameter<std::string>("topic_velocity", "dvl/velocity");
        this->declare_parameter<std::string>("topic_dead_reckoning", "dvl/dead_reckoning");
        this->declare_parameter<std::string>("topic_odometry", "dvl/odometry");

        velocity_msg_.velocity_mode = marine_acoustic_msgs::msg::Dvl::DVL_MODE_BOTTOM;
        velocity_msg_.dvl_type = marine_acoustic_msgs::msg::Dvl::DVL_TYPE_PISTON;

        velocity_msg_.beam_unit_vec[0].x = -0.6532814824381883;
        velocity_msg_.beam_unit_vec[0].y =  0.6532814824381883;
        velocity_msg_.beam_unit_vec[0].z =  0.38268343236508984;
        
        velocity_msg_.beam_unit_vec[1].x = -0.6532814824381883;
        velocity_msg_.beam_unit_vec[1].y = -0.6532814824381883;
        velocity_msg_.beam_unit_vec[1].z =  0.38268343236508984;

        velocity_msg_.beam_unit_vec[2].x =  0.6532814824381883;
        velocity_msg_.beam_unit_vec[2].y = -0.6532814824381883;
        velocity_msg_.beam_unit_vec[2].z =  0.38268343236508984;

        velocity_msg_.beam_unit_vec[3].x =  0.6532814824381883;
        velocity_msg_.beam_unit_vec[3].y =  0.6532814824381883;
        velocity_msg_.beam_unit_vec[3].z =  0.38268343236508984;

        dvl_.set_velocity_callback(std::bind(&DvlA50SerialNode::on_velocity_report, this, std::placeholders::_1));
        dvl_.set_dead_reckoning_callback(std::bind(&DvlA50SerialNode::on_dead_reckoning_report, this, std::placeholders::_1));
        dvl_.set_transducer_callback(std::bind(&DvlA50SerialNode::on_transducer_report, this, std::placeholders::_1));
        dvl_.set_error_callback([this](const std::string& err) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *this, 5000, "Serial line errors detected (" << err << "). Check physical connection.");
        });

        param_sub_ = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            bool reconfigure = false;
            for (const auto& param : parameters) {
                if (param.get_name() == "speed_of_sound" || 
                    param.get_name() == "led_enabled" || 
                    param.get_name() == "mounting_rotation_offset" || 
                    param.get_name() == "range_mode") {
                    reconfigure = true;
                }
            }
            if (reconfigure && dvl_active_) {
                int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
                bool led_enabled = this->get_parameter("led_enabled").as_bool();
                int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
                std::string range_mode = this->get_parameter("range_mode").as_string();
                int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
                dvl_.configure(speed_of_sound, true, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms);
            }
            return result;
        });
    }

    ~DvlA50SerialNode() {}

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/)
    {
        std::string port = this->get_parameter("port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        std::string frame = this->get_parameter("frame").as_string();
        
        RCLCPP_INFO(get_logger(), "Connecting to DVL A50 at %s (%d baud)", port.c_str(), baud_rate);

        std::string topic_velocity = this->get_parameter("topic_velocity").as_string();
        std::string topic_dead_reckoning = this->get_parameter("topic_dead_reckoning").as_string();
        std::string topic_odometry = this->get_parameter("topic_odometry").as_string();

        velocity_pub_ = this->create_publisher<marine_acoustic_msgs::msg::Dvl>(topic_velocity, rclcpp::SensorDataQoS());
        dead_reckoning_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_dead_reckoning, rclcpp::SensorDataQoS());
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(topic_odometry, rclcpp::SensorDataQoS());

        bool success = dvl_.connect(port, baud_rate);
        if (!success)
        {
            RCLCPP_ERROR(get_logger(), "Serial connection failed.");
            return CallbackReturn::FAILURE;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        enable_on_activate_ = this->get_parameter("enable_on_activate").as_bool();
        int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
        
        int attempts = 3;
        bool success = false;
        for (int i = 0; i < attempts; i++) {
            if (dvl_.configure(speed_of_sound, false, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms)) {
                success = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        if (!success) {
            RCLCPP_ERROR(get_logger(), "Initial DVL configure failed. Check serial connection or timeouts.");
            return CallbackReturn::FAILURE;
        }

        velocity_msg_.sound_speed = speed_of_sound;
        velocity_msg_.header.frame_id = frame;
        dead_reckoning_msg_.header.frame_id = frame;
        odometry_msg_.header.frame_id = frame;

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_activate(state);

        int timeout_set_protocol_ms = this->get_parameter("timeout_set_protocol_ms").as_int();
        if (!dvl_.set_protocol(3, timeout_set_protocol_ms)) {
            RCLCPP_ERROR(get_logger(), "Failed to set DVL serial protocol to 3. DVL might be ignoring ACKs.");
            return CallbackReturn::FAILURE;
        }

        if (enable_on_activate_) {
            int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
            bool led_enabled = this->get_parameter("led_enabled").as_bool();
            int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
            std::string range_mode = this->get_parameter("range_mode").as_string();
            int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
            if (!dvl_.configure(speed_of_sound, true, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms)) {
                RCLCPP_ERROR(get_logger(), "Failed to enable acoustics during activation. DVL rejecting hooks.");
                return CallbackReturn::FAILURE;
            }
        }
        
        velocity_pub_->on_activate();
        dead_reckoning_pub_->on_activate();
        odometry_pub_->on_activate();
        
        enable_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "enable", std::bind(&DvlA50SerialNode::srv_enable, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_callback_group_);
        disable_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "disable", std::bind(&DvlA50SerialNode::srv_disable, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_callback_group_);
        calibrate_gyro_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "calibrate_gyro", std::bind(&DvlA50SerialNode::srv_calibrate_gyro, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_callback_group_);
        reset_dead_reckoning_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "reset_dead_reckoning", std::bind(&DvlA50SerialNode::srv_reset_dead_reckoning, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_callback_group_);
        trigger_ping_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_ping", std::bind(&DvlA50SerialNode::srv_trigger_ping, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, service_callback_group_);

        dvl_active_ = true;
        RCLCPP_INFO(get_logger(), "DVL A50 Serial activated.");

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state)
    {
        LifecycleNode::on_deactivate(state);
        dvl_active_ = false;
        RCLCPP_INFO(get_logger(), "Deactivating DVL A50 Serial node");

        int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
        dvl_.configure(speed_of_sound, false, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms);

        velocity_pub_->on_deactivate();
        dead_reckoning_pub_->on_deactivate();
        odometry_pub_->on_deactivate();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/)
    {
        dvl_.disconnect();

        velocity_pub_.reset();
        dead_reckoning_pub_.reset();
        odometry_pub_.reset();

        enable_srv_.reset();
        disable_srv_.reset();
        calibrate_gyro_srv_.reset();
        reset_dead_reckoning_srv_.reset();
        trigger_ping_srv_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    void on_velocity_report(const VelocityReport& res)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!res.valid) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *this, 1000, "Received velocity report with invalid velocity data");
        }
        if (res.status > 0) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *this, 1000, "Received velocity report with error status " << res.status);
            if (res.status == 1) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *this, 500, "DVL may be overheating");
            }
        }

        velocity_msg_.header.stamp = this->now();
        velocity_msg_.velocity.x = res.velocity.x;
        velocity_msg_.velocity.y = res.velocity.y;
        velocity_msg_.velocity.z = res.velocity.z;
        for (size_t i = 0; i < 9; i++) {
            velocity_msg_.velocity_covar[i] = res.covariance[i];
        }
        if(res.altitude >= 0.0 && res.valid) {
            velocity_msg_.altitude = res.altitude;
        }

        velocity_msg_.course_gnd = std::atan2(res.velocity.y, res.velocity.x);
        velocity_msg_.speed_gnd = std::sqrt(res.velocity.x * res.velocity.x + res.velocity.y * res.velocity.y);
        velocity_msg_.beam_ranges_valid = true;
        velocity_msg_.beam_velocities_valid = res.valid;
        velocity_msg_.num_good_beams = 0;

        for (int beam = 0; beam < 4; beam++) {
            if (last_transducer_reports_[beam].distance > 0.0) {
                velocity_msg_.num_good_beams++;
                velocity_msg_.range[beam] = last_transducer_reports_[beam].distance;
                velocity_msg_.beam_quality[beam] = last_transducer_reports_[beam].rssi;
                velocity_msg_.beam_velocity[beam] = last_transducer_reports_[beam].velocity;
            } else {
                velocity_msg_.range[beam] = -1.0;
            }
            last_transducer_reports_[beam].distance = -1.0;
        }

        odometry_msg_.twist.twist.linear.x = res.velocity.x;
        odometry_msg_.twist.twist.linear.y = res.velocity.y;
        odometry_msg_.twist.twist.linear.z = res.velocity.z;
        for (size_t i = 0; i < 3; i++) {
            for (size_t j = 0; j < 3; j++) {
                odometry_msg_.twist.covariance[i*6 + j] = res.covariance[i*3 + j];
            }
        }
        
        if (velocity_pub_ && velocity_pub_->is_activated()) {
            velocity_pub_->publish(velocity_msg_);
        }
    }

    void on_dead_reckoning_report(const DeadReckoningReport& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (res.status > 0) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *this, 1000, "Received dead reckoning report with error status " << res.status);
        }

        auto now_stamp = this->now();
        dead_reckoning_msg_.header.stamp = now_stamp;
        dead_reckoning_msg_.pose.pose.position.x = res.position.x;
        dead_reckoning_msg_.pose.pose.position.y = res.position.y;
        dead_reckoning_msg_.pose.pose.position.z = res.position.z;

        double variance = res.pos_std * res.pos_std;
        dead_reckoning_msg_.pose.covariance[0] = variance;
        dead_reckoning_msg_.pose.covariance[7] = variance;
        dead_reckoning_msg_.pose.covariance[14] = variance;

        tf2::Quaternion quat;
        quat.setRPY(res.roll * M_PI / 180.0, res.pitch * M_PI / 180.0, res.yaw * M_PI / 180.0);
        dead_reckoning_msg_.pose.pose.orientation = tf2::toMsg(quat);

        if (dead_reckoning_pub_ && dead_reckoning_pub_->is_activated()) {
            dead_reckoning_pub_->publish(dead_reckoning_msg_);
        }

        odometry_msg_.header.stamp = now_stamp;
        odometry_msg_.pose = dead_reckoning_msg_.pose;            
        if (odometry_pub_ && odometry_pub_->is_activated()) {
            odometry_pub_->publish(odometry_msg_);
        }
    }

    void on_transducer_report(const TransducerReport& res) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (res.id >= 0 && res.id < 4) {
            last_transducer_reports_[res.id] = res;
        }
    }

    void srv_enable(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
        int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
        
        bool success = dvl_.configure(speed_of_sound, true, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms);
        res->success = success;
        res->message = success ? "Acoustics enabled" : "Failed to enable acoustics";
    }

    void srv_disable(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
        int speed_of_sound = this->get_parameter("speed_of_sound").as_int();
        bool led_enabled = this->get_parameter("led_enabled").as_bool();
        int mounting_rotation_offset = this->get_parameter("mounting_rotation_offset").as_int();
        std::string range_mode = this->get_parameter("range_mode").as_string();
        int timeout_configure_ms = this->get_parameter("timeout_configure_ms").as_int();
        
        bool success = dvl_.configure(speed_of_sound, false, led_enabled, mounting_rotation_offset, range_mode, timeout_configure_ms);
        res->success = success;
        res->message = success ? "Acoustics disabled" : "Failed to disable acoustics";
    }

    void srv_calibrate_gyro(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
        int timeout = this->get_parameter("timeout_calibrate_gyro_ms").as_int();
        bool success = dvl_.calibrate_gyro(timeout);
        res->success = success;
        res->message = success ? "Gyro calibrated" : "Failed to calibrate gyro";
    }

    void srv_reset_dead_reckoning(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
        int timeout = this->get_parameter("timeout_reset_dead_reckoning_ms").as_int();
        bool success = dvl_.reset_dead_reckoning(timeout);
        res->success = success;
        res->message = success ? "Dead reckoning reset" : "Failed to reset dead reckoning";
    }

    void srv_trigger_ping(std_srvs::srv::Trigger::Request::SharedPtr /*req*/, std_srvs::srv::Trigger::Response::SharedPtr res) {
        srv_disable(nullptr, res);
        int timeout = this->get_parameter("timeout_trigger_ping_ms").as_int();
        bool success = dvl_.trigger_ping(timeout);
        res->success = success;
        res->message = success ? "Ping triggered" : "Failed to trigger ping";
    }

private:
    DvlA50Serial dvl_;

    bool enable_on_activate_ = false;
    bool dvl_active_ = false;

    std::mutex data_mutex_;
    
    marine_acoustic_msgs::msg::Dvl velocity_msg_;
    geometry_msgs::msg::PoseWithCovarianceStamped dead_reckoning_msg_;
    nav_msgs::msg::Odometry odometry_msg_;
    TransducerReport last_transducer_reports_[4];
    
    rclcpp_lifecycle::LifecyclePublisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr velocity_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr dead_reckoning_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_dead_reckoning_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_ping_srv_;

    rclcpp::CallbackGroup::SharedPtr service_callback_group_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_sub_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;
    std::shared_ptr<DvlA50SerialNode> node = std::make_shared<DvlA50SerialNode>("dvl_a50_serial");
    exe.add_node(node->get_node_base_interface());
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
