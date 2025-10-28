#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
	// Request a service and pass the velocities to it to drive the robot
	ball_chaser::DriveToTarget service;
	service.request.linear_x = lin_x;
	service.request.angular_z = ang_z;

	// Call the safe_move service and pass the requested joint angles
	if (!client.call(service))
		ROS_ERROR("Failed to move the robot");

	// wait for 2 seconds to allow it to complete move
	ros::Duration(2).sleep();
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    ROS_INFO_STREAM("Inside process_image_callback");

	int white_px = 255;
	bool ball_found = false;

	int ball_position = img.width;	// divide the camera into 3 sections: left, centre, right
	int avg_pixel = 0;

	for (size_t j{ 0 }; j < img.height; j++)
	{
		for (size_t i{ 0 }; i < img.step; i++)
		{
			avg_pixel = (img.data[(j * img.step + i)] + img.data[(j * img.step + i + 1)] + img.data[(j * img.step + i + 2)]) / 3.0;

			if (avg_pixel == white_px)
			{
				//ROS_INFO("The ball position is (%d, %d)", i, j);
				int camera_pos = i + 1;

				// turn left if in the left section, camera_pos < 800
				if (camera_pos <= ball_position)
					drive_robot(0.5, 0.5);

				// go straight if in the central section, 800 < camera_pos <= 1600
				if (camera_pos > ball_position && camera_pos <= 2.0 * ball_position)
					drive_robot(1.5, 0.0);

				// turn right if in the right section, camera_pos > 1600
				if (camera_pos > 2.0 * ball_position)
					drive_robot(0.5, -0.5);

				ball_found = true;
				return;
			}
		}
	}

	ROS_INFO_STREAM("BALL NOT FOUND");

	// if the ball is not found, stop moving robot
	if (!ball_found)
		drive_robot(0.0, 0.0);
	
	return;
}


int main(int argc, char** argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 2, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}