#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>  // Custom message type
#include <geometry_msgs/Twist.h>             // Twist message type

ros::Publisher cmd_vel_pub;

// Callback function to process PositionCommand message and map it to Twist
void positionCmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& position_cmd)
{
    // Create a Twist message to store velocities
    geometry_msgs::Twist twist_msg;

    // Map the linear velocities from PositionCommand to Twist
    twist_msg.linear.x = position_cmd->velocity.x;
    twist_msg.linear.y = position_cmd->velocity.y;
    twist_msg.linear.z = position_cmd->velocity.z;

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
    ros::Subscriber cmd_sub = nh.subscribe("/position_cmd", 10, positionCmdCallback);

    // Keep the node running and processing incoming messages
    ros::spin();

    return 0;
}
