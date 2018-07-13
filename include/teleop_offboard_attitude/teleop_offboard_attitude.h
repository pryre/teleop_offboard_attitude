#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class TeleopAttitude {
	public:
		TeleopAttitude( void );
		~TeleopAttitude( void );

	private:
		void callback_att( const ros::TimerEvent& e );
		void callback_joy( const sensor_msgs::Joy::ConstPtr& joy );
		double normalize_input(double in, double min, double max);
		double double_normalize_input(double in, double min, double max);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;

		ros::Timer timer_att_;
		ros::Subscriber sub_joy_;

		ros::Publisher pub_att_;
		ros::Publisher pub_viz_;

		double param_rate_;
		std::string param_frame_id_;
		bool param_viz_;

		int param_axis_roll_, param_axis_pitch_, param_axis_yaw_;
		int param_axis_roll_r_, param_axis_pitch_r_, param_axis_yaw_r_;
		int param_axis_throttle_;

		double param_scale_a_r_, param_scale_a_p_, param_scale_a_y_;
		double param_scale_r_r_, param_scale_r_p_, param_scale_r_y_;

		double param_range_r_min_, param_range_p_min_, param_range_y_min_, param_range_t_min_;
		double param_range_r_max_, param_range_p_max_, param_range_y_max_, param_range_t_max_;

		sensor_msgs::Joy joy_last_;
};
