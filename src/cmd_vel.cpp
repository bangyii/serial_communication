#include <serial_communication/cmd_vel.h>
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

	return true;
}

void CmdVel::cmdCallback(const geometry_msgs::Twist msg_cmd)
{
	cmd_vel.linear.x = msg_cmd.linear.x;
	cmd_vel.angular.z = msg_cmd.angular.z;
}

ros::Subscriber CmdVel::getCmdVelSub(ros::NodeHandle &nh)
{
	return nh.subscribe("/cmd_vel", 1, &CmdVel::cmdCallback, this);
}

void CmdVel::getCmdVel(int16_t velocitybuf[3])
{
	v_left_cmd = cmd_vel.linear.x * calibration_cmd_lin_ - cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_;
	v_right_cmd = cmd_vel.linear.x * calibration_cmd_lin_ + cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_;

	//Send velocity commands directly to STM, up to 3 decimal places
	velocitybuf[0] = (int16_t)(v_left_cmd * 1000);
	velocitybuf[1] = (int16_t)(v_right_cmd * 1000);

	velocitybuf[2] = (int16_t)0xFFFB;

	v_left_cmd_prev = v_left_cmd;
	v_right_cmd_prev = v_right_cmd;
}