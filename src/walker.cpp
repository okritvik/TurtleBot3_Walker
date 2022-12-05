/**
 * @file walker.cpp
 * @author Kumara Ritvik Oruganti (okritvik@umd.edu)
 * @brief A node that subscribes to /scan topic (lidar) and moves around the obstacles without colliding.
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using LIDAR = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

class Avoidance : public rclcpp::Node {
 public:
    Avoidance() :
        Node("walker") {
            auto callback = std::bind(&Avoidance::lidar_callback, this, _1);
            m_lidar_sub = this->create_subscription<LIDAR>
                                        ("scan", 10, callback);
            m_pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
        }

 private:
    void lidar_callback(const LIDAR& msg) {
        // RCLCPP_INFO(this->get_logger(), "Angle Min %f", msg.angle_min);
        // RCLCPP_INFO(this->get_logger(), "Angle Max %f", msg.angle_max);
        // RCLCPP_INFO(this->get_logger(), "Angle Inc %f", msg.angle_increment);
        if (msg.header.stamp.sec == 0) {
            return;
        }
        auto scan_data = msg.ranges;
        auto start_angle = 330;
        auto angle_range = 60;
        for (int i = start_angle; i < start_angle + angle_range; i++) {
            if (scan_data[i % 360] < 0.8) {
                perform_action(0.0, 0.1);
            } else {
                perform_action(0.05, 0.0);
            }
        }
    }
    void perform_action(auto x_vel, auto z_vel) {
        auto vel = TWIST();
        vel.linear.x = x_vel;
        // vel.linear.y = 0.0;
        // vel.linear.z = 0.0;
        // vel.angular.x = 0.0;
        // vel.angular.y = 0.0;
        vel.angular.z = -z_vel;
        m_pub_vel->publish(vel);
    }

    rclcpp::Subscription<LIDAR>::SharedPtr m_lidar_sub;
    rclcpp::Publisher<TWIST>::SharedPtr m_pub_vel;
    rclcpp::TimerBase::SharedPtr m_timer;
    LIDAR m_last_data;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Avoidance>());
    rclcpp::shutdown();
    return 0;
}
