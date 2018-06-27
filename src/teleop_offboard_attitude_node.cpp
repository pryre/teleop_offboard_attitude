#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <string.h>

sensor_msgs::Joy joy_last_;

class TeleopAttitude {
	public:
		TeleopAttitude( ros::NodeHandle *nh );
		void spin( void );

	private:
		void joyCallback( const sensor_msgs::Joy::ConstPtr& joy );
		double normalizeInput(double in, double min, double max);
		double doubleNormalizeInput(double in, double min, double max);

	private:
		ros::NodeHandle *nh_;

		std::string param_input_joy_;
		std::string param_output_setpoint_;
		std::string param_output_viz_att_;

		int param_axis_roll_, param_axis_pitch_, param_axis_yaw_;
		int param_axis_roll_r_, param_axis_pitch_r_, param_axis_yaw_r_;
		int param_axis_throttle_;

		double param_scale_a_r_, param_scale_a_p_, param_scale_a_y_;
		double param_scale_r_r_, param_scale_r_p_, param_scale_r_y_;


		double param_range_r_min_, param_range_p_min_, param_range_y_min_, param_range_t_min_;
		double param_range_r_max_, param_range_p_max_, param_range_y_max_, param_range_t_max_;

		double param_spin_rate_;
		std::string param_frame_id_;
		bool param_viz_;

		ros::Subscriber joy_sub_;

		ros::Publisher sp_pub_;
		ros::Publisher viz_att_pub_;

		ros::Rate rate_;
};

TeleopAttitude::TeleopAttitude( ros::NodeHandle *nh ):
	nh_( nh ),
	param_input_joy_("/joy"),
	param_output_setpoint_("setpoint_raw"),
	param_output_viz_att_("goal/attitude"),
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
	param_spin_rate_(20.0f),
	rate_(20.0f),
	param_viz_(false) {

	ROS_INFO("[Teleop] Starting...");

	nh_->param("input_joy", param_input_joy_, param_input_joy_);
	nh_->param("output_setpoint", param_output_setpoint_, param_output_setpoint_);
	nh_->param("output_viz/attitude", param_output_viz_att_, param_output_viz_att_);

	nh_->param("axis/roll", param_axis_roll_, param_axis_roll_);
	nh_->param("axis/pitch", param_axis_pitch_, param_axis_pitch_);
	nh_->param("axis/yaw", param_axis_yaw_, param_axis_yaw_);
	nh_->param("axis/roll_r", param_axis_roll_r_, param_axis_roll_r_);
	nh_->param("axis/pitch_r", param_axis_pitch_r_, param_axis_pitch_r_);
	nh_->param("axis/yaw_r", param_axis_yaw_r_, param_axis_yaw_r_);
	nh_->param("axis/throttle", param_axis_throttle_, param_axis_throttle_);

	nh_->param("scale/angular/roll", param_scale_a_r_, param_scale_a_r_);		//Effectively sets the max roll angle (rad)
	nh_->param("scale/angular/pitch", param_scale_a_p_, param_scale_a_p_);		//Effectively sets the max pitch angle (rad)
	nh_->param("scale/angular/yaw", param_scale_a_y_, param_scale_a_y_);		//Effectively sets the max yaw angle (rad)
	nh_->param("scale/rate/roll", param_scale_r_r_, param_scale_r_r_);			//Sets how quickly roll will rotate (rad/s)
	nh_->param("scale/rate/pitch", param_scale_r_p_, param_scale_r_p_);			//Sets how quickly pitch will rotate (rad/s)
	nh_->param("scale/rate/yaw", param_scale_r_y_, param_scale_r_y_);			//Sets how quickly yaw will rotate (rad/s)

	nh_->param("range/roll/min", param_range_r_min_, param_range_r_min_);
	nh_->param("range/roll/max", param_range_r_max_, param_range_r_max_);
	nh_->param("range/pitch/min", param_range_p_min_, param_range_p_min_);
	nh_->param("range/pitch/max", param_range_p_max_, param_range_p_max_);
	nh_->param("range/yaw/min", param_range_y_min_, param_range_y_min_);
	nh_->param("range/yaw/max", param_range_y_max_, param_range_y_max_);
	nh_->param("range/throttle/min", param_range_t_min_, param_range_t_min_);
	nh_->param("range/throttle/max", param_range_t_max_, param_range_t_max_);


	nh_->param("spin_rate", param_spin_rate_, param_spin_rate_);
	nh_->param("frame_id", param_frame_id_, param_frame_id_);
	nh_->param("show_vizualizer", param_viz_, param_viz_);

	joy_sub_ = nh_->subscribe<sensor_msgs::Joy>(param_input_joy_, 10, &TeleopAttitude::joyCallback, this);

	sp_pub_ = nh_->advertise<mavros_msgs::AttitudeTarget>(param_output_setpoint_, 10);
	viz_att_pub_ = nh_->advertise<geometry_msgs::PoseStamped>(param_output_viz_att_, 10);

	rate_ = ros::Rate(param_spin_rate_);

	ROS_INFO("[Teleop] Listening to: %s", param_input_joy_.c_str());
}

double TeleopAttitude::normalizeInput(double in, double min, double max) {
	return (in - min) / (max - min);
}

double TeleopAttitude::doubleNormalizeInput(double in, double min, double max) {
	return 2*((in - min) / (max - min)) - 1;
}

void TeleopAttitude::joyCallback( const sensor_msgs::Joy::ConstPtr& joy ) {
	joy_last_ = *joy;
}

void TeleopAttitude::spin( void ) {
	//Wait until we get at least 1 joy message
	if(joy_last_.header.stamp > ros::Time(0.0f) ) {
		ros::Time time_now = ros::Time::now();
		mavros_msgs::AttitudeTarget sp_out;
		geometry_msgs::PoseStamped viz_att_out;
		tf2::Quaternion sp_q;

		//==-- Attitude
		double roll = 0.0f;
		double pitch = 0.0f;
		double yaw = 0.0f;

		if(param_axis_roll_ >= 0)
			roll = param_scale_a_r_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_roll_), param_range_r_min_, param_range_r_max_);
		if(param_axis_pitch_ >= 0)
			pitch = param_scale_a_p_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_pitch_), param_range_p_min_, param_range_p_max_);
		if(param_axis_yaw_ >= 0)
			yaw = param_scale_a_y_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_yaw_), param_range_y_min_, param_range_y_max_);

		sp_q.setRPY(roll, pitch, yaw);

		sp_out.orientation.w = sp_q.getW();
		sp_out.orientation.x = sp_q.getX();
		sp_out.orientation.y = sp_q.getY();
		sp_out.orientation.z = sp_q.getZ();

		if( ( param_axis_roll_ < 0 ) && ( param_axis_pitch_ < 0 ) && ( param_axis_yaw_ < 0 ) )
			sp_out.type_mask |= sp_out.IGNORE_ATTITUDE;

		//==-- Rates
		if(param_axis_roll_r_ >= 0) {
			sp_out.body_rate.x = param_scale_r_r_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_roll_r_), param_range_r_min_, param_range_r_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_ROLL_RATE;
		}

		if(param_axis_pitch_r_ >= 0) {
			sp_out.body_rate.y = param_scale_r_p_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_pitch_r_), param_range_p_min_, param_range_p_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_PITCH_RATE;
		}

		if(param_axis_yaw_r_ >= 0) {
			sp_out.body_rate.z = param_scale_r_y_ * doubleNormalizeInput(joy_last_.axes.at(param_axis_yaw_r_), param_range_y_min_, param_range_y_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_YAW_RATE;
		}

		//==-- Thottle
		if(param_axis_throttle_ >= 0) {
			sp_out.thrust = normalizeInput(joy_last_.axes.at(param_axis_throttle_), param_range_t_min_, param_range_t_max_);
		} else {
			sp_out.type_mask |= sp_out.IGNORE_THRUST;
		}

		//==-- Publish
		sp_out.header.stamp = time_now;
		sp_out.header.frame_id = param_frame_id_;
		sp_pub_.publish(sp_out);

		//==-- Visualization
		if( param_viz_ ) {
			//Attitude
			if( !( sp_out.type_mask & sp_out.IGNORE_ATTITUDE ) ) {
				viz_att_out.pose.orientation = sp_out.orientation;

				viz_att_out.header.stamp = time_now;
				viz_att_out.header.frame_id = param_frame_id_;
				viz_att_pub_.publish(viz_att_out);
			}

			//TODO: Rates?

			//TODO: Throttle?
		}
	}

	ros::spinOnce();
	rate_.sleep();
}


int main(int argc, char** argv) {
	ros::init( argc, argv, "teleop_attitude" );
	ros::NodeHandle nh( ros::this_node::getName() );

	TeleopAttitude teleop_attitude( &nh );

	while( ros::ok() )
		teleop_attitude.spin();

	return 0;
}
