#include <serial_communication/cmd_vel.h>
#define ENCODERCIR 16384.
#include <cmath>

CmdVel::CmdVel()
{

}

bool CmdVel::readParameters(ros::NodeHandle &node_handle)
{
	if (!node_handle.getParam("calibration_cmd_ang", calibration_cmd_ang_))
		ROS_WARN_STREAM("Parameter calibration_cmd_ang not set for serial_communication. Using default setting: " << calibration_cmd_ang_);
	if (!node_handle.getParam("calibration_cmd_lin", calibration_cmd_lin_))
		ROS_WARN_STREAM("Parameter calibration_cmd_lin not set for serial_communication. Using default setting: " << calibration_cmd_lin_);
    if (!node_handle.getParam("apply_vel_filter", apply_vel_filter_))
		ROS_WARN_STREAM("Parameter apply_vel_filter not set for serial_communication. Using default setting: " << apply_vel_filter_);
	if (!node_handle.getParam("vel_filter_size", vel_filter_size_))
		ROS_WARN_STREAM("Parameter vel_filter_size not set for serial_communication. Using default setting: " << vel_filter_size_);
	if (!node_handle.getParam("wheel_acc_limit", wheel_acc_limit_))
		ROS_WARN_STREAM("Parameter wheel_acc_limit not set for serial_communication. Using default setting: " << wheel_acc_limit_);        
	
	return true;
}

void CmdVel::cmdCallback(const geometry_msgs::Twist msg_cmd)
{
	cmd_vel.linear.x = msg_cmd.linear.x;
	cmd_vel.angular.z = msg_cmd.angular.z;
	time_last_cmd = ros::Time::now();
}

ros::Subscriber CmdVel::getCmdVelSub(ros::NodeHandle &nh)
{
    return nh.subscribe("/cmd_vel", 1, &CmdVel::cmdCallback, this);
}

void CmdVel::getCmdVel(int16_t velocitybuf[3])
{
		ros::Duration timediff_cmd = timestamp - time_last_cmd; // negative if message received in last spinonce
		if (timediff_cmd.toSec() > max_dt_cmd)
		{
			v_left_cmd = 0;
			v_right_cmd = 0;
		}
		else
		{
			v_left_cmd = cmd_vel.linear.x * calibration_cmd_lin_ - cmd_vel.angular.z * (base_width ) * calibration_cmd_ang_;
			v_right_cmd = cmd_vel.linear.x * calibration_cmd_lin_ + cmd_vel.angular.z * (base_width ) * calibration_cmd_ang_;
		}

		//Account for motor deadzone
		velocitybuf[0] = v_left_cmd / VelocityMax * 500 + 130; // Convert to MCU velocity range: sends pwm signals at 1500, +/- 500
		velocitybuf[1] = v_right_cmd / VelocityMax * 500 + 130;

		if(velocitybuf[0] > 2000) velocitybuf[0] = 2000;
		if(velocitybuf[1] > 2000) velocitybuf[1] = 2000;

		velocitybuf[2] = 0xFFFB;
}


void CmdVel::medianFilter(float &x, float &y)
{
	//Moving window of values to average
	buffer_x.push_back(x);
	buffer_y.push_back(y);
	if (buffer_x.size() > vel_filter_size_)
	{
		buffer_x.pop_front();
		buffer_y.pop_front();
	}

	//Get average
	float average_x = 0, average_y = 0;
	for(int i = 0; i < buffer_x.size(); i++){
		average_x += buffer_x[i];
		average_y += buffer_y[i];
	}

	//Set readings to average
	x = average_x / buffer_x.size();
	y = average_y / buffer_y.size();
}

std::vector<float> CmdVel::getVelFromEncoder(std::vector<float> encoder)
{
	//! To do implememt realiable formulaus for computing velocity
    encoder_left = encoder[1];  // encoder1 is left
    encoder_right = encoder[0]; // encoder2 is right

    //Get difference in encoder
    float diff_enc_left = encoder_left - encoder_left_prev;
    float diff_enc_right = encoder_right - encoder_right_prev;

	//Get difference in time
	float dt = (std::chrono::system_clock::now() - time_prev).count() / 1000000000.0;

    if (fabs(diff_enc_right) <= ENCODERCIR / 2)
        f_right = diff_enc_right;
    else if (fabs(diff_enc_right) < -ENCODERCIR / 2)
        f_right = diff_enc_right + ENCODERCIR;
    else
        f_right = diff_enc_right - ENCODERCIR;

    if (fabs(diff_enc_left) <= ENCODERCIR / 2)
        f_left = diff_enc_left;
    else if (fabs(diff_enc_left) < -ENCODERCIR / 2)
        f_left = diff_enc_left + ENCODERCIR;
    else
        f_left = diff_enc_left - ENCODERCIR;

    v_right = f_right / ENCODERCIR * M_PI * wheel_diameter / dt;
    v_left = -f_left / ENCODERCIR * M_PI * wheel_diameter / dt;
//	ROS_INFO("%f\t%f\t%f\t%f", f_right, f_right/ENCODERCIR, f_right/ENCODERCIR * M_PI, wheel_diameter / dt);
//	ROS_INFO("Time difference: %f \t vleft: %f \t vright: %f", dt, v_left, v_right);
    // Sometimes data gets lost and spikes are seen in the velocity readouts.
    // This is solved by limiting the max difference between subsequent velocity readouts.
    if (fabs(v_right - v_right_prev) / dt > wheel_acc_limit_)
        v_right = v_right_prev;
    if (fabs(v_left - v_left_prev) / dt > wheel_acc_limit_)
        v_left = v_left_prev;

    // Deadzone the velocities, to avoid accumulation of noise in steady position
    if (fabs(v_right) < 0.01 || fabs(v_right) > 5 || std::isnan(v_right))
        v_right = 0.0;
    if (fabs(v_left) < 0.01 || fabs(v_left) > 5 || std::isnan(v_left))
        v_left = 0.0;

    // Apply medianfilter to velocities
    if (apply_vel_filter_)
        medianFilter(v_left, v_right);

    //Set all previous values to current values
    encoder_left_prev = encoder_left;
    encoder_right_prev = encoder_right;
    v_right_prev = v_right;
    v_left_prev = v_left;
	time_prev = std::chrono::system_clock::now();

    return std::vector<float>({v_left, v_right});
}
