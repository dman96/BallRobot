#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

//Creating a Client Instance that Will Request a Service
ros::ServiceClient client;

//The Function that Will Call the command_robot service
void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Calling the command_robot service");
	
	//Creating a Service Request Instance
	ball_chaser::DriveToTarget srv;
	
	//Passing the Desired Direction to the Service
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	
	if(!client.call(srv))
	{
		ROS_ERROR("Failed to Call Service: DriveToTarget");
	}
}

//Callback Function that will read image data
void process_image_callback(const sensor_msgs::Image img)
{

	int white_pixel = 255;
	bool ball_found = false;
	
	int row;
	int step;
	int i;
	
	// Looking for White Pixels
	for (row=0; row<img.height && ball_found == false; row++)
	{
		for (step = 0; step < img.step && ball_found == false; step++)
		{
			i = (row*img.step)+step;
			
			if (img.data[i] == white_pixel)
			{
				ball_found = true;
				
			}
		}
	}
	
	//Ball Discovered
	if (ball_found==true)
	{
	
		//Separating the Image into Three Sections
		int section = img.width/3;
		int col = step/3;
		
		//Ball to the Left
		if (col<section)
		{
			ROS_INFO("Ball Found to the Left! Turning Robot to the Left");
			drive_robot(0.0,-0.5);
		}
		
		//Ball in the Middle
		else if (col >= section && col < 2*section)
		{
			ROS_INFO("Ball Found in the Center! Moving Robot Forward");
			drive_robot(-0.5,0.0);
		}
		
		//Ball to the Right
		else if (col > 2*section)
		{
			ROS_INFO("Ball Found to the Right! Turning Robot to the Right");
			drive_robot(0.0,0.5);
		}
	}
	
	else
	{
		//Ball Not Discovered
		ROS_INFO("Ball Not Found! Stopping Robot");
		drive_robot(0.0,0.0);
	}
		
}

int main(int argc, char** argv)
{

	//Initializing process_image node
	ros::init(argc,argv,"process_image");
	
	//Creating a Node Handle
	ros::NodeHandle n;
	
	//Creating a Client to Call the DriveToTarget Service
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
	
	//Retrieving a Camera Image
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw",10, process_image_callback);
	
	//ROS Communication Events
	ros::spin();
	
	return 0;
}
