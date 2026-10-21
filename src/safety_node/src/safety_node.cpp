#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>

#define ANGLE_MAX 2.3499999046325684
#define ANGLE_MIN -2.3499999046325684
#define ANGLE_STEP 0.004351851996034384
#define TTC_THRESHOLD 1 // (so if car is going 1m/sec towards a wall that is 0.2m away, we stop)

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages


        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        // create a subscriber for /scan
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
                                                    std::bind(&Safety::scan_callback, this, std::placeholders::_1));

        // create a subscriber for /ego_racer/odom
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10,
                                                    std::bind(&Safety::odom_callback, this, std::placeholders::_1));


        // create a publisher to /drive
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        
    }

private:
    /// TODO: create ROS subscribers and publishers

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed_ = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        float r_dt = 0.0;

        for (size_t i = 0; i < scan_msg->ranges.size(); i++)
        {
            if ((r_dt = speed_ * cosf(ANGLE_MIN + i * ANGLE_STEP)) > 0.0)
            {
                /// TODO: publish drive/brake message
                if ((scan_msg->ranges.at(i) / r_dt) < TTC_THRESHOLD)
                {
                    auto drive_msg_ = ackermann_msgs::msg::AckermannDriveStamped();
                    drive_msg_.drive.speed = 0.0;
                    drive_publisher_->publish(drive_msg_);
                    RCLCPP_INFO(this->get_logger(), "Stopped the veichle!\n");
                }
                
            }
            
        }

    }

    double speed_ = 0.0;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}