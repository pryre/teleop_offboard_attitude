#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string.h>

class TeleopAttitude {
	public:
		TeleopAttitude();

	private:
	  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	private:
	  ros::NodeHandle nh_;

	  int param_axis_roll_, param_axis_pitch_, param_axis_yaw_r_, param_axis_throttle_;
	  double param_scale_a, param_scale_r_;
	  
	  std::string param_joy_input;
	  
	  ros::Publisher att_pub_;
	  ros::Subscriber joy_sub_;
};

TeleopAttitude::TeleopAttitude():
	param_axis_roll_(1),
	param_axis_pitch_(2),
	param_axis_yaw_r_(3),
	param_axis_throttle_(4),
	param_joy_input("/joy") {
	
	ROS_INFO("[Teleop] Starting...");

	nh_.param("axis/roll", param_axis_roll_, param_axis_roll_);
	nh_.param("axis/pitch", param_axis_pitch_, param_axis_pitch_);
	nh_.param("axis/yaw_r", param_axis_yaw_r_, param_axis_yaw_r_);
	nh_.param("axis/throttle", param_axis_throttle_, param_axis_throttle_);
	
	nh_.param("scale/angular", param_scale_a, param_scale_a);	//Effectively sets the max roll/pitch angles (rad)
	nh_.param("scale/rate", param_scale_r_, param_scale_r_);	//Sets how quickly yaw will rotate (rad/s)

	att_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("setpoint_raw/attitude", 10);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(param_joy_input, 10, &TeleopAttitude::joyCallback, this);

	ROS_INFO("[Teleop] Listening to: %s", param_joy_input.c_str());
}

void TeleopAttitude::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
	mavros_msgs::AttitudeTarget sp_att;
	tf2::Quaternion sp_q;
	
	double roll = param_scale_a*joy->axes[param_axis_roll_];
	double pitch = param_scale_a*joy->axes[param_axis_pitch_];
	sp_q.setRPY(roll, pitch, 0.0f);
	
	sp_att.orientation.w = sp_q.getW();
	sp_att.orientation.x = sp_q.getX();
	sp_att.orientation.y = sp_q.getY();
	sp_att.orientation.z = sp_q.getZ();
	
	sp_att.body_rate.z = param_scale_r_*joy->axes[param_axis_yaw_r_];
	
	sp_att.thrust = joy->axes[param_axis_throttle_];
	
	sp_att.type_mask |= sp_att.IGNORE_ROLL_RATE | sp_att.IGNORE_PITCH_RATE;
	
	att_pub_.publish(sp_att);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_attitude");
	TeleopAttitude teleop_attitude;

	ros::spin();
}
