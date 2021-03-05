#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"
//TODO: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;


bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
	//Creating a Twist Instance//
        geometry_msgs::Twist motor_command;
        
        //Indication that the Service Call Went Through//
	ROS_INFO("DriveToTarget Request Received - j1:%1.2f, j2:%1.2f", (float)req.linear_x, (float)req.angular_z);
	
	//Assigning the Requested Velocities to the Motor Command Instance//
	
	//The /cmd topic seems to reverse the translational and angular velocity hence they're reversed here
        motor_command.linear.x = req.angular_z;
        motor_command.angular.z = req.linear_x;
        
        // Publishing the Commanded Velocities//
        motor_command_publisher.publish(motor_command);
        
        //Feedback Indicating the Velocities Were Set//
	res.msg_feedback = "Linear Velocity Set To: " + std::to_string(req.linear_x) + " Angular Velocity Set To: " + std::to_string(req.angular_z);
	
	ROS_INFO_STREAM(res.msg_feedback);
	
	return true;
}

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "drive_bot");
	
	ros::NodeHandle n;
	
	//Carrying out the Publishing Action//
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	
	//Calling the Service//
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	
	ros::spin();
	
	return 0;	

}


