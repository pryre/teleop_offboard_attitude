#include <ros/ros.h>
#include <teleop_offboard_attitude/teleop_offboard_attitude.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_offboard_attitude");
	TeleopAttitude teleop;

	ros::spin();

	return 0;
}
