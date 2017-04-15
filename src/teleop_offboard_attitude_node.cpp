#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>

class TeleopAttitude {
	public:
		TeleopAttitude();

	private:
	  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	private:
	  ros::NodeHandle nh_;

	  int linear_, angular_;
	  double l_scale_, a_scale_;
	  ros::Publisher att_pub_;
	  ros::Subscriber joy_sub_;
};

TeleopAttitude::TeleopAttitude():
	linear_(1),
	angular_(2) {

	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("setpoint_raw/attitude", 10);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAttitude::joyCallback, this);

}

void TeleopAttitude::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	mavros_msgs::AttitudeTarget sp_att;
	//twist.angular.z = a_scale_*joy->axes[angular_];
	//twist.linear.x = l_scale_*joy->axes[linear_];
	att_pub_.publish(sp_att);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_attitude");
	TeleopAttitude teleop_attitude;

	ros::spin();
}
