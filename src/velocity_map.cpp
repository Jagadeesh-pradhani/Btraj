#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>  // Custom message type for PositionCommand
#include <geometry_msgs/Twist.h>             // Twist message type
#include <sensor_msgs/Range.h>               // To read sonar height topic
#include <math.h>                            // For applying yaw and other control logic

ros::Publisher cmd_vel_pub;
double current_height = 0.0;  // To store current height of the drone

// Callback to get the current height from the sonar sensor
void sonarHeightCallback(const sensor_msgs::Range::ConstPtr& sonar_height_msg)
{
    current_height = sonar_height_msg->range;  // Get the current height
}

// Callback function to process PositionCommand message and map it to Twist
void positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& position_cmd)
{
    // Create a Twist message to store velocities
    geometry_msgs::Twist twist_msg;

    // Map the linear velocities from PositionCommand to Twist
    twist_msg.linear.x = position_cmd->velocity.x;
    twist_msg.linear.y = position_cmd->velocity.y;

    // Use current height from sonar sensor and position_cmd.z for better height control
    twist_msg.linear.z = position_cmd->position.z - current_height;

    // Map the yaw_dot (angular velocity around the Z-axis) to the Twist message's angular.z
    twist_msg.angular.z = position_cmd->yaw_dot;

    // Publish the Twist message to the /cmd_vel topic
    cmd_vel_pub.publish(twist_msg);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_mapper");

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Create a publisher for /cmd_vel topic (geometry_msgs/Twist)
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to the /position_cmd topic (quadrotor_msgs/PositionCommand)
    ros::Subscriber cmd_sub = nh.subscribe("/planning/pos_cmd", 10, positionCmdCallback);

    // Subscribe to the /sonar_height topic to get height information
    ros::Subscriber sonar_sub = nh.subscribe("/sonar_height", 10, sonarHeightCallback);

    // Keep the node running and processing incoming messages
    ros::spin();

    return 0;
}
