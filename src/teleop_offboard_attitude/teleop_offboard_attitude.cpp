#include <ros/ros.h>

#include <teleop_offboard_attitude/teleop_offboard_attitude.h>

#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>


TeleopAttitude::TeleopAttitude( void ):
	nh_(),
	nhp_( "~" ),
	param_axis_roll_(-1),
	param_axis_pitch_(-1),
	param_axis_yaw_(-1),
	param_axis_roll_r_(-1),
	param_axis_pitch_r_(-1),
	param_axis_yaw_r_(-1),
	param_axis_throttle_(-1),
	param_range_r_min_(-1.0),
	param_range_r_max_(1.0),
	param_range_p_min_(-1.0),
	param_range_p_max_(1.0),
	param_range_y_min_(-1.0),
	param_range_y_max_(1.0),
	param_range_t_min_(-1.0),
	param_range_t_max_(1.0),
	param_frame_id_("/world"),
	param_rate_(20.0f),
	param_viz_(false) {

	nhp_.param("axis/roll", param_axis_roll_, param_axis_roll_);
	nhp_.param("axis/pitch", param_axis_pitch_, param_axis_pitch_);
	nhp_.param("axis/yaw", param_axis_yaw_, param_axis_yaw_);
	nhp_.param("axis/roll_r", param_axis_roll_r_, param_axis_roll_r_);
	nhp_.param("axis/pitch_r", param_axis_pitch_r_, param_axis_pitch_r_);
	nhp_.param("axis/yaw_r", param_axis_yaw_r_, param_axis_yaw_r_);
	nhp_.param("axis/throttle", param_axis_throttle_, param_axis_throttle_);

	nhp_.param("scale/angular/roll", param_scale_a_r_, param_scale_a_r_);		//Effectively sets the max roll angle (rad)
	nhp_.param("scale/angular/pitch", param_scale_a_p_, param_scale_a_p_);		//Effectively sets the max pitch angle (rad)
	nhp_.param("scale/angular/yaw", param_scale_a_y_, param_scale_a_y_);		//Effectively sets the max yaw angle (rad)
	nhp_.param("scale/rate/roll", param_scale_r_r_, param_scale_r_r_);			//Sets how quickly roll will rotate (rad/s)
	nhp_.param("scale/rate/pitch", param_scale_r_p_, param_scale_r_p_);			//Sets how quickly pitch will rotate (rad/s)
	nhp_.param("scale/rate/yaw", param_scale_r_y_, param_scale_r_y_);			//Sets how quickly yaw will rotate (rad/s)

	nhp_.param("range/roll/min", param_range_r_min_, param_range_r_min_);
	nhp_.param("range/roll/max", param_range_r_max_, param_range_r_max_);
	nhp_.param("range/pitch/min", param_range_p_min_, param_range_p_min_);
	nhp_.param("range/pitch/max", param_range_p_max_, param_range_p_max_);
	nhp_.param("range/yaw/min", param_range_y_min_, param_range_y_min_);
	nhp_.param("range/yaw/max", param_range_y_max_, param_range_y_max_);
	nhp_.param("range/throttle/min", param_range_t_min_, param_range_t_min_);
	nhp_.param("range/throttle/max", param_range_t_max_, param_range_t_max_);

	nhp_.param("update_rate", param_rate_, param_rate_);
	nhp_.param("frame_id", param_frame_id_, param_frame_id_);
	nhp_.param("show_vizualizer", param_viz_, param_viz_);

	timer_att_ = nhp_.createTimer(ros::Duration(1.0/param_rate_), &TeleopAttitude::callback_att, this );

	sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopAttitude::callback_joy, this);

	pub_att_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/attitude", 10);
	pub_viz_ = nhp_.advertise<geometry_msgs::PoseStamped>("feedback/pose", 10);

	ROS_INFO("[Teleop] Listening to for joystick input");
}

TeleopAttitude::~TeleopAttitude( void ) {
}

double TeleopAttitude::normalize_input(double in, double min, double max) {
	return (in - min) / (max - min);
}

double TeleopAttitude::double_normalize_input(double in, double min, double max) {
	return 2*((in - min) / (max - min)) - 1;
}

void TeleopAttitude::callback_joy( const sensor_msgs::Joy::ConstPtr& joy ) {
	joy_last_ = *joy;
}

void TeleopAttitude::callback_att( const ros::TimerEvent& e ) {
	//Wait until we get at least 1 joy message
	if(joy_last_.header.stamp > ros::Time(0) ) {
		mavros_msgs::AttitudeTarget sp_out;
		geometry_msgs::PoseStamped viz_att_out;
		tf2::Quaternion sp_q;

		//==-- Attitude
		double roll = 0.0f;
		double pitch = 0.0f;
		double yaw = 0.0f;

		if(param_axis_roll_ >= 0)
			roll = param_scale_a_r_ * double_normalize_input(joy_last_.axes.at(param_axis_roll_), param_range_r_min_, param_range_r_max_);
		if(param_axis_pitch_ >= 0)
			pitch = param_scale_a_p_ * double_normalize_input(joy_last_.axes.at(param_axis_pitch_), param_range_p_min_, param_range_p_max_);
		if(param_axis_yaw_ >= 0)
			yaw = param_scale_a_y_ * double_normalize_input(joy_last_.axes.at(param_axis_yaw_), param_range_y_min_, param_range_y_max_);

		sp_q.setRPY(roll, pitch, yaw);

		sp_out.orientation.w = sp_q.getW();
		sp_out.orientation.x = sp_q.getX();
		sp_out.orientation.y = sp_q.getY();
		sp_out.orientation.z = sp_q.getZ();

		if( ( param_axis_roll_ < 0 ) && ( param_axis_pitch_ < 0 ) && ( param_axis_yaw_ < 0 ) )
			sp_out.type_mask |= sp_out.IGNORE_ATTITUDE;

		//==-- Rates
		if(param_axis_roll_r_ >= 0) {
			sp_out.body_rate.x = param_scale_r_r_ * double_normalize_input(joy_last_.axes.at(param_axis_roll_r_), param_range_r_min_, param_range_r_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_ROLL_RATE;
		}

		if(param_axis_pitch_r_ >= 0) {
			sp_out.body_rate.y = param_scale_r_p_ * double_normalize_input(joy_last_.axes.at(param_axis_pitch_r_), param_range_p_min_, param_range_p_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_PITCH_RATE;
		}

		if(param_axis_yaw_r_ >= 0) {
			sp_out.body_rate.z = param_scale_r_y_ * double_normalize_input(joy_last_.axes.at(param_axis_yaw_r_), param_range_y_min_, param_range_y_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_YAW_RATE;
		}

		//==-- Thottle
		if(param_axis_throttle_ >= 0) {
			sp_out.thrust = normalize_input(joy_last_.axes.at(param_axis_throttle_), param_range_t_min_, param_range_t_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_THRUST;
		}

		//==-- Publish
		sp_out.header.stamp = e.current_real;
		sp_out.header.frame_id = param_frame_id_;
		pub_att_.publish(sp_out);

		//==-- Visualization
		if( param_viz_ ) {
			//Attitude
			if( !( sp_out.type_mask & sp_out.IGNORE_ATTITUDE ) ) {
				viz_att_out.pose.orientation = sp_out.orientation;

				viz_att_out.header.stamp = e.current_real;
				viz_att_out.header.frame_id = param_frame_id_;
				pub_viz_.publish(viz_att_out);
			}

			//TODO: Rates?

			//TODO: Throttle?
		}
	}
}
