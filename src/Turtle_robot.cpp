#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class LidarSubscriberNode {
public:
    LidarSubscriberNode() {
        // Initialize the ROS node handle
        nh_ = ros::NodeHandle("~");

        // Subscribe to the LiDAR topic
        lidar_sub_ = nh_.subscribe("/scan", 1, &LidarSubscriberNode::lidarCallback, this);

        // Publish to the robot's velocity topic
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        // Set the initial state of the robot
        is_obstacle_detected_ = false;
    }

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Check for obstacles in the LiDAR data
        for (unsigned int i = 0; i < msg->ranges.size(); ++i) {
            // Check if any range is less than a threshold (indicating an obstacle)
            if (msg->ranges[i] < obstacle_distance_threshold_) {
                ROS_INFO("Obstacle detected! Stopping and turning 180 degrees.");
                stopAndTurn180();
                is_obstacle_detected_ = true;
                break;
            }
        }
    }

    void stopAndTurn180() {
        // Stop the robot
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        vel_pub_.publish(stop_cmd);

        // Wait for a short time to stop
        ros::Duration(1.0).sleep();

        // Turn the robot 180 degrees (counter-clockwise)
        geometry_msgs::Twist turn_cmd;
        turn_cmd.linear.x = 0.0;
        turn_cmd.angular.z = 1.0; // Set an appropriate angular velocity to achieve the turn
        vel_pub_.publish(turn_cmd);

        // Wait for the turn to complete
        ros::Duration(3.0).sleep();

        // Stop the robot after the turn is complete
        stop_cmd.angular.z = 0.0;
        vel_pub_.publish(stop_cmd);

        // Reset the obstacle detection flag
        is_obstacle_detected_ = false;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Publisher vel_pub_;
    bool is_obstacle_detected_;
    const double obstacle_distance_threshold_ = 0.5; // Adjust this threshold according to your LiDAR specifications
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_subscriber_node");
    LidarSubscriberNode node;
    ros::spin();
    return 0;
}