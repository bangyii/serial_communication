#include <serial_communication/cmd_vel.h>
#define ENCODERCIR 16384.
#include <cmath>

CmdVel::CmdVel()
{
}

bool CmdVel::readParameters(ros::NodeHandle &node_handle)
{
	if (!node_handle.getParam("calibration_cmd_ang", calibration_cmd_ang_))
		ROS_WARN_STREAM("Parameter calibration_cmd_ang not set for controller cmd vel. Using default setting: " << calibration_cmd_ang_);
	if (!node_handle.getParam("calibration_cmd_lin", calibration_cmd_lin_))
		ROS_WARN_STREAM("Parameter calibration_cmd_lin not set for controller cmd vel. Using default setting: " << calibration_cmd_lin_);
	if (!node_handle.getParam("apply_vel_filter", apply_vel_filter_))
		ROS_WARN_STREAM("Parameter apply_vel_filter not set for controller cmd vel. Using default setting: " << apply_vel_filter_);
	if (!node_handle.getParam("vel_filter_size", vel_filter_size_))
		ROS_WARN_STREAM("Parameter vel_filter_size not set for controller cmd vel. Using default setting: " << vel_filter_size_);
	if (!node_handle.getParam("wheel_acc_limit", wheel_acc_limit_))
		ROS_WARN_STREAM("Parameter wheel_acc_limit not set for controller cmd vel. Using default setting: " << wheel_acc_limit_);
	if (!node_handle.getParam("deadzone_pulse_width", deadzone_pulse_width))
		ROS_WARN_STREAM("Parameter deadzone_pulse_width not set for controller cmd vel. Using default setting: " << deadzone_pulse_width);
	if (!node_handle.getParam("motor_kp", motor_kp))
		ROS_WARN_STREAM("Parameter motor_kp not set for controller cmd vel. Using default setting: " << motor_kp);
	if (!node_handle.getParam("motor_ki", motor_ki))
		ROS_WARN_STREAM("Parameter motor_ki not set for controller cmd vel. Using default setting: " << motor_ki);
	if (!node_handle.getParam("motor_kd", motor_kd))
		ROS_WARN_STREAM("Parameter motor_kd not set for controller cmd vel. Using default setting: " << motor_kd);
	if (!node_handle.getParam("motor_f", motor_f))
		ROS_WARN_STREAM("Parameter motor_f not set for controller cmd vel. Using default setting: " << motor_f);
	if (!node_handle.getParam("frequency", frequency))
		ROS_WARN_STREAM("Parameter frequency not set for controller cmd vel. Using default setting: " << frequency);
	if (!node_handle.getParam("motor_max_accel", ramp_rate))
		ROS_WARN_STREAM("Parameter motor_max_accel not set for controller cmd vel. Using default setting: " << ramp_rate);
	if (!node_handle.getParam("motor_max_decel", descent_rate))
		ROS_WARN_STREAM("Parameter motor_max_decel not set for controller cmd vel. Using default setting: " << descent_rate);
	if (!node_handle.getParam("calibration_v_angular", calibration_v_angular_))
		ROS_WARN_STREAM("Parameter calibration_v_angular not set for controller odom. Using default setting: " << calibration_v_angular_);
	if (!node_handle.getParam("w_tolerance", w_tolerance))
		ROS_WARN_STREAM("Parameter w_tolerance not set for controller odom. Using default setting: " << w_tolerance);
	if (!node_handle.getParam("pid_filter", pid_filter))
		ROS_WARN_STREAM("Parameter pid_filter not set for controller odom. Using default setting: " << pid_filter);

	//Setup PID
	left_motor_pid.setPID(motor_kp, motor_ki, motor_kd);
	left_motor_pid.setMaxIOutput(VelocityMax);
	left_motor_pid.setOutputLimits(-VelocityMax, VelocityMax);
	left_motor_pid.setF(motor_f);
	left_motor_pid.setFreq(frequency);
	left_motor_pid.setOutputRampRate(ramp_rate); //ms-2
	left_motor_pid.setOutputDescentRate(descent_rate);
	left_motor_pid.setOutputFilter(pid_filter);

	right_motor_pid.setPID(motor_kp, motor_ki, motor_kd);
	right_motor_pid.setMaxIOutput(VelocityMax);
	right_motor_pid.setOutputLimits(-VelocityMax, VelocityMax);
	right_motor_pid.setF(motor_f);
	right_motor_pid.setFreq(frequency);
	right_motor_pid.setOutputRampRate(ramp_rate); //ms-2
	right_motor_pid.setOutputDescentRate(descent_rate);
	right_motor_pid.setOutputFilter(pid_filter);

	return true;
}

void CmdVel::setCurrentAngular(float val)
{
	current_angular = val;
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
		v_left_cmd = cmd_vel.linear.x * calibration_cmd_lin_ - cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_;
		v_right_cmd = cmd_vel.linear.x * calibration_cmd_lin_ + cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_;
	}

	//PID for motor controls, getOutput(current reading, target)
	//Reset when 0 commanded or when change of direction
	//if (v_left_cmd == 0 || v_left_cmd * v_left_cmd_prev < 0 || v_right_cmd == 0 || v_right_cmd * v_right_cmd_prev < 0)
	//if (v_left_cmd == 0 && v_right_cmd == 0 || v_left_cmd * v_left_cmd_prev < 0 || v_right_cmd * v_right_cmd_prev < 0)
	if(v_left_cmd == 0 && v_right_cmd == 0)
	{
		left_motor_pid.reset();
		right_motor_pid.reset();
	}

	//Set a tolerance for acceptable deviation of angular velocity from commanded angular velocity
	//Used to prevent swirling motion before reaching a steady straight motion
	float target_angular = (v_right_cmd - v_left_cmd) / (base_width)*calibration_v_angular_;
	float left_pid_out, right_pid_out;

	//Right wheel is too fast, ie turning left when commanded to go straight
	// if (current_angular - target_angular > w_tolerance && w_tolerance != 0 && v_left_cmd*v_right_cmd > 0)
	// {
	// 	ROS_WARN("Right wheel is too fast, exceeded angular speed deviation tolerance. Skipping right wheel PID cycle once");
	// 	left_pid_out = left_motor_pid.getOutput(v_left, v_left_cmd);
	// 	right_pid_out = right_motor_pid.skipCycle();
	// }

	// //Left wheel is too fast, ie turning right when commanded to go straight
	// else if (current_angular - target_angular < -w_tolerance && w_tolerance != 0 && v_left_cmd*v_right_cmd > 0)
	// {
	// 	ROS_WARN("Left wheel is too fast, exceeded angular speed deviation tolerance. Skipping right wheel PID cycle once");
	// 	right_pid_out = right_motor_pid.getOutput(v_right, v_right_cmd);
	// 	left_pid_out = left_motor_pid.skipCycle();
	// }
	//ROS_INFO("%f %f", current_angular, target_angular);
	if (w_tolerance != 0 && v_left_cmd * v_right_cmd > 0)
	{
		//ROS_INFO("Controlling angular tolerance");
		if (current_angular - target_angular > w_tolerance)
		{
			ROS_WARN("Right wheel is too fast, exceeded angular speed deviation tolerance. Skipping right wheel PID cycle once");
			left_pid_out = left_motor_pid.getOutput(v_left, v_left_cmd);
			right_pid_out = right_motor_pid.skipCycle();
		}

		else if (current_angular - target_angular < -w_tolerance)
		{
			ROS_WARN("Left wheel is too fast, exceeded angular speed deviation tolerance. Skipping right wheel PID cycle once");
			right_pid_out = right_motor_pid.getOutput(v_right, v_right_cmd);
			left_pid_out = left_motor_pid.skipCycle();
		}

		else
        	{
                	left_pid_out = left_motor_pid.getOutput(v_left, v_left_cmd);
        	        right_pid_out = right_motor_pid.getOutput(v_right, v_right_cmd);
	        }

	}

	else
	{
		left_pid_out = left_motor_pid.getOutput(v_left, v_left_cmd);
		right_pid_out = right_motor_pid.getOutput(v_right, v_right_cmd);
	}

	//ROS_INFO("Post PID output left: %f \t right: %f", left_pid_out + v_left_cmd, right_pid_out + v_right_cmd);
	//PID_out = F*setpoint + P*e + I*e_sum*dt + D*de/dt
	velocitybuf[0] = (left_pid_out) / VelocityMax * 500;
	velocitybuf[1] = (right_pid_out) / VelocityMax * 500;

	//Left velocity is within deadzone
	if (velocitybuf[0] < 0 && velocitybuf[0] > -deadzone_pulse_width)
		velocitybuf[0] = -deadzone_pulse_width;
	if (velocitybuf[0] > 0 && velocitybuf[0] < deadzone_pulse_width)
		velocitybuf[0] = deadzone_pulse_width;

	//Right velocity is within deadzone
	if (velocitybuf[1] < 0 && velocitybuf[1] > -deadzone_pulse_width)
		velocitybuf[1] = -deadzone_pulse_width;
	if (velocitybuf[1] > 0 && velocitybuf[1] < deadzone_pulse_width)
		velocitybuf[1] = deadzone_pulse_width;

	velocitybuf[2] = 0xFFFB;

	v_left_cmd_prev = v_left_cmd;
	v_right_cmd_prev = v_right_cmd;
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
	for (int i = 0; i < buffer_x.size(); i++)
	{
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
	encoder_left = encoder[1];	// encoder1 is left
	encoder_right = encoder[0]; // encoder2 is right

	//Get difference in encoder
	float diff_enc_left = 0;
	float diff_enc_right = 0;
	if (encoder_left_prev != -1 && encoder_right_prev != -1)
	{
		diff_enc_left = encoder_left - encoder_left_prev;
		diff_enc_right = encoder_right - encoder_right_prev;
	}

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

	// Sometimes data gets lost and spikes are seen in the velocity readouts.
	// This is solved by limiting the max difference between subsequent velocity readouts.
	// If acceleration is passed, just update velocity within acceleration limits
	float right_acc = (v_right - v_right_prev) / dt;
	float left_acc = (v_left - v_left_prev) / dt;
	if (fabs(right_acc) > wheel_acc_limit_)
	{
		v_right = v_right_prev + wheel_acc_limit_ * dt * (right_acc / fabs(right_acc));
		ROS_WARN("Right wheel acceleration limit reached, limiting right velocity change");
	}

	if (fabs(left_acc) > wheel_acc_limit_)
	{
		v_left = v_left_prev + wheel_acc_limit_ * dt * (left_acc / fabs(left_acc));
		ROS_WARN("Left wheel acceleration limit reached, limiting left velocity change");
	}

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
