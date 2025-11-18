#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"


struct Sample
{
    double t;
    double x;
    double y;
    double yaw;
    double v;
};

class MPCPathReplayer : public rclcpp::Node
{
public:
    MPCPathReplayer()
    : rclcpp::Node("mpc_path_replayer"),
      current_index_(0),
      dt_(0.1),
      playback_rate_(1.0),
      loop_(false)
    {
        // Defaults; override via --ros-args
        this->declare_parameter<std::string>("actual_csv_path", "/data/output_mpc.csv");
        this->declare_parameter<std::string>("ref_csv_path", "/data/output_ref.csv");
        this->declare_parameter<std::string>("lqr_csv_path", "/data/output_lqr.csv");
        this->declare_parameter<double>("playback_rate", 1.0);
        this->declare_parameter<bool>("loop", false);

        this->get_parameter("actual_csv_path", actual_csv_path_);
        this->get_parameter("ref_csv_path", ref_csv_path_);
        this->get_parameter("lqr_csv_path", lqr_csv_path_);
        this->get_parameter("playback_rate", playback_rate_);
        this->get_parameter("loop", loop_);

        RCLCPP_INFO(get_logger(), "Actual CSV: %s", actual_csv_path_.c_str());
        RCLCPP_INFO(get_logger(), "Ref CSV:    %s", ref_csv_path_.c_str());

        if (!load_csv(actual_csv_path_, actual_samples_, true)) {
            RCLCPP_FATAL(get_logger(), "Failed to load actual trajectory CSV");
            rclcpp::shutdown();
            return;
        }

        if (!load_csv(ref_csv_path_, ref_samples_, false)) {
            RCLCPP_WARN(get_logger(), "Failed to load reference CSV; continuing without ref path");
        }

        if(!load_csv(lqr_csv_path_, lqr_samples_, false))
        {
            RCLCPP_WARN(get_logger(), "Failed to load LQR CSV; continuing without LQR path");
        }

        if (actual_samples_.size() >= 2) {
            double dt_raw = actual_samples_[1].t - actual_samples_[0].t;
            if (dt_raw > 1e-6) {
                dt_ = dt_raw;
            }
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        path_pub_     = create_publisher<nav_msgs::msg::Path>("mpc/actual_path", 10);
        ref_path_pub_ = create_publisher<nav_msgs::msg::Path>("mpc/ref_path", 10);
        twist_pub_    = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_replay", 10);
        lqr_path_pub_ = create_publisher<nav_msgs::msg::Path>("lqr/actual_path", 10);
        lqr_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("lqr_robot_marker", 10);
        marker_pub_   = create_publisher<visualization_msgs::msg::Marker>("robot_marker", 10);

        

        actual_path_.header.frame_id = "map";
        lqr_path_.header.frame_id = "map";
        build_reference_path();

        double timer_period = dt_ / playback_rate_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(timer_period),
            std::bind(&MPCPathReplayer::on_timer, this));

        RCLCPP_INFO(get_logger(),
            "Loaded %zu actual samples, %zu ref samples, dt=%.3f, rate=%.2f, loop=%d",
            actual_samples_.size(), ref_samples_.size(), dt_, playback_rate_, loop_ ? 1 : 0);
    }

private:
    bool load_csv(const std::string & path,
                  std::vector<Sample> & out,
                  bool is_actual)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Could not open CSV: %s", path.c_str());
            return false;
        }

        std::string line;
        out.clear();

        while (std::getline(file, line)) {
            if (line.empty()) {
                continue;
            }

            std::stringstream ss(line);
            std::string cell;
            std::vector<double> vals;

            while (std::getline(ss, cell, ',')) {
                if (cell.empty()) {
                    vals.push_back(0.0);
                } else {
                    try {
                        vals.push_back(std::stod(cell));
                    } catch (const std::exception & e) {
                        RCLCPP_WARN(get_logger(), "Failed to parse cell '%s': %s",
                                    cell.c_str(), e.what());
                        vals.push_back(0.0);
                    }
                }
            }

            // For your files:
            // output_mpc: t,x,y,yaw,v, ...
            // output_ref: t,x,y,yaw,v
            if (vals.size() < 4) {
                continue;
            }

            Sample s;
            s.t   = vals[0];
            s.x   = vals[1];
            s.y   = vals[2];
            s.yaw = vals[3];
            if (vals.size() >= 5) {
                s.v = vals[4];
            } else {
                s.v = 0.0;
            }

            out.push_back(s);
        }

        if (out.empty()) {
            RCLCPP_ERROR(get_logger(), "No valid rows parsed from %s", path.c_str());
            return false;
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu samples from %s (%s)",
                    out.size(), path.c_str(),
                    is_actual ? "actual" : "ref");

        return true;
    }

    void build_reference_path()
    {
        ref_path_.header.frame_id = "map";
        ref_path_.poses.clear();

        for (const Sample & s : ref_samples_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = s.x;
            pose.pose.position.y = s.y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, s.yaw);
            pose.pose.orientation = tf2::toMsg(q);

            ref_path_.poses.push_back(pose);
        }
    }

    void on_timer()
    {
        if (actual_samples_.empty()) {
            return;
        }

        if (current_index_ >= actual_samples_.size()) {
            if (loop_) {
                current_index_ = 0;
                actual_path_.poses.clear();
            } else {
                RCLCPP_INFO_ONCE(get_logger(), "Finished playback");
                timer_->cancel();
                return;
            }
        }

        const Sample & s = actual_samples_[current_index_];
        rclcpp::Time now = this->now();

        // 1) TF: map -> base_link
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = s.x;
        tf_msg.transform.translation.y = s.y;
        tf_msg.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, s.yaw);
        tf_msg.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(tf_msg);

        // 2) Actual path
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";
        pose.pose.position.x = s.x;
        pose.pose.position.y = s.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf_msg.transform.rotation;

        actual_path_.header.stamp = now;
        actual_path_.poses.push_back(pose);
        path_pub_->publish(actual_path_);

        // 3) Reference path (static curve)
        if (!ref_path_.poses.empty()) {
            ref_path_.header.stamp = now;
            ref_path_pub_->publish(ref_path_);
        }

        // 4) Approximate cmd_vel
        geometry_msgs::msg::Twist twist;
        twist.linear.x = s.v;
        if (current_index_ + 1 < actual_samples_.size()) {
            double dyaw = actual_samples_[current_index_ + 1].yaw - s.yaw;
            double omega = dyaw / dt_;
            twist.angular.z = omega;
        } else {
            twist.angular.z = 0.0;
        }
        twist_pub_->publish(twist);

        current_index_++;

        // 5) Publish a simple robot footprint marker in base_link frame
        visualization_msgs::msg::Marker marker;
        marker.header.stamp = now;
        marker.header.frame_id = "base_link";
        marker.ns = "mpc_robot";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Car footprint size (meters) - tweak as you like 
        marker.scale.x = 4.5;  // length
        marker.scale.y = 2.0;  // width
        marker.scale.z = 0.5;  // height

        // Center the cube around base_link (shift it slightly forward if you want)
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.25; // half of height

        marker.pose.orientation.w = 1.0;

        // Coloar (RGBA)
        marker.color.r = 0.1f;
        marker.color.g = 0.8f;
        marker.color.b = 0.1f;
        marker.color.a = 0.8f;

        marker.lifetime = rclcpp::Duration::from_seconds(0.0); // 0 = forever, just updated
        marker_pub_->publish(marker);


    if(!lqr_samples_.empty() && current_index_ < lqr_samples_.size())
    {
        const Sample & s_lqr = lqr_samples_[current_index_];

        geometry_msgs::msg::PoseStamped pose_lqr;
        pose_lqr.header.stamp = now;
        pose_lqr.header.frame_id = "map";
        pose_lqr.pose.position.x = s_lqr.x;
        pose_lqr.pose.position.y = s_lqr.y;
        pose_lqr.pose.position.z = 0.0;
        
        tf2::Quaternion q_lqr;
        q_lqr.setRPY(0.0,0.0,s_lqr.yaw);
        pose_lqr.pose.orientation = tf2::toMsg(q_lqr);

        lqr_path_.header.stamp = now;
        lqr_path_.poses.push_back(pose_lqr);
        lqr_path_pub_->publish(lqr_path_);

        // Marker for LQR robot (blue)
        visualization_msgs::msg::Marker marker_lqr;
        marker_lqr.header.stamp = now;
        marker_lqr.header.frame_id = "base_link_lqr";
        marker_lqr.ns = "lqr_robot";
        marker_lqr.id = 0;
        marker_lqr.type = visualization_msgs::msg::Marker::CUBE;
        marker_lqr.action = visualization_msgs::msg::Marker::ADD;
        marker_lqr.scale.x = 4.5;
        marker_lqr.scale.y = 2.0;
        marker_lqr.scale.z = 0.5;
        marker_lqr.pose.position.z = 0.25;
        marker_lqr.color.r = 0.1;
        marker_lqr.color.g = 0.3;
        marker_lqr.color.b = 1.0;
        marker_lqr.color.a = 0.8;
        lqr_marker_pub_->publish(marker_lqr);

    }
    if(current_index_ < lqr_samples_.size()) {
        
        const Sample & s_lqr = lqr_samples_[current_index_];
        geometry_msgs::msg::TransformStamped tf_lqr;
        tf_lqr.header.stamp = now;
        tf_lqr.header.frame_id = "map";
        tf_lqr.child_frame_id = "base_link_lqr";

        tf_lqr.transform.translation.x = s_lqr.x;
        tf_lqr.transform.translation.y = s_lqr.y;
        tf_lqr.transform.translation.z = 0.0;

        tf2::Quaternion q_lqr;
        q_lqr.setRPY(0.0,0.0,s_lqr.yaw);
        tf_lqr.transform.rotation = tf2::toMsg(q_lqr);

        tf_broadcaster_->sendTransform(tf_lqr);
    }
    }

    // Parameters
    std::string actual_csv_path_;
    std::string ref_csv_path_;
    std::string lqr_csv_path_;


    // Data
    std::vector<Sample> actual_samples_;
    std::vector<Sample> ref_samples_;
    std::vector<Sample> lqr_samples_;

    // ROS
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lqr_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lqr_marker_pub_;


    nav_msgs::msg::Path actual_path_;
    nav_msgs::msg::Path ref_path_;
    nav_msgs::msg::Path lqr_path_;

    // State
    std::size_t current_index_;
    double dt_;
    double playback_rate_;
    bool loop_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MPCPathReplayer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

