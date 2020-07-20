#include <serial_communication/odom.h>
#include <tf/transform_datatypes.h>
#include <vector>

Odom::Odom()
{
	time_prev = std::chrono::system_clock::now();
}

bool Odom::readParameters(ros::NodeHandle &node_handle)
{
	if (!node_handle.getParam("calibration_v_angular", calibration_v_angular_))
		ROS_WARN_STREAM("Parameter calibration_v_angular not set for serial_communication. Using default setting: " << calibration_v_angular_);
	if (!node_handle.getParam("calibration_v_linear", calibration_v_linear_))
		ROS_WARN_STREAM("Parameter calibration_v_linear not set for serial_communication. Using default setting: " << calibration_v_linear_);
	if (!node_handle.getParam("bias_acc_x", bias_acc_x_))
		ROS_WARN_STREAM("Parameter bias_acc_x not set for serial_communication. Using default setting: " << bias_acc_x_);
	if (!node_handle.getParam("bias_acc_y", bias_acc_y_))
		ROS_WARN_STREAM("Parameter bias_acc_y not set for serial_communication. Using default setting: " << bias_acc_y_);
	if (!node_handle.getParam("bias_acc_z", bias_acc_z_))
		ROS_WARN_STREAM("Parameter bias_acc_z not set for serial_communication. Using default setting: " << bias_acc_z_);
	if (!node_handle.getParam("bias_gyro_x", bias_gyro_x_))
		ROS_WARN_STREAM("Parameter bias_gyro_x not set for serial_communication. Using default setting: " << bias_gyro_x_);
	if (!node_handle.getParam("bias_gyro_y", bias_gyro_y_))
		ROS_WARN_STREAM("Parameter bias_gyro_y not set for serial_communication. Using default setting: " << bias_gyro_y_);
	if (!node_handle.getParam("bias_gyro_z", bias_gyro_z_))
		ROS_WARN_STREAM("Parameter bias_gyro_z not set for serial_communication. Using default setting: " << bias_gyro_z_);
	if (!node_handle.getParam("cal_imu_gyro", cal_imu_gyro_))
		ROS_WARN_STREAM("Parameter cal_imu_gyro not set for serial_communication. Using default setting: " << cal_imu_gyro_);
	if (!node_handle.getParam("cal_imu_acc", cal_imu_acc_))
		ROS_WARN_STREAM("Parameter cal_imu_acc not set for serial_communication. Using default setting: " << cal_imu_acc_);
    if (!node_handle.getParam("odom_update_method", odom_update_method_))
		ROS_WARN_STREAM("Parameter odom_update_method not set for serial_communication. Using default setting: " << odom_update_method_);
	if (!node_handle.getParam("publish_odom_tf", publish_odom_tf_))
		ROS_WARN_STREAM("Parameter publish_odom_tf not set for serial_communication. Using default setting: " << publish_odom_tf_);

	return true;
}


void Odom::odomCallback(const nav_msgs::Odometry msg_odom)
{
	x_odom = msg_odom.pose.pose.position.x;
	y_odom = msg_odom.pose.pose.position.y;
	theta_odom = tf::getYaw(msg_odom.pose.pose.orientation);
}

void Odom::poseCallback(const geometry_msgs::Pose2D msg_pose)
{
	x_odom = msg_pose.x;
	y_odom = msg_pose.y;
	theta_odom = msg_pose.theta;
}

ros::Subscriber Odom::getOdomUpdateSub(ros::NodeHandle& nh){
	if (odom_update_method_ == "pose2D")
		return nh.subscribe("/pose2D", 1, &Odom::poseCallback, this);
	else if (odom_update_method_ == "odom")
		return nh.subscribe("/ekf/odom", 1, &Odom::odomCallback, this);
}

std::vector<float> Odom::getIMU(std::vector<float> raw)
{
	ax = raw[0] / 16384.0 * cal_imu_acc_ - bias_acc_x_;
	ay = raw[1] / 16384.0 * cal_imu_acc_ - bias_acc_y_;
	az = raw[2] / 16384.0 * cal_imu_acc_ - bias_acc_z_;
	gx = raw[3] / 131.0 / 180.0 * 3.14159265359 * cal_imu_gyro_ - bias_gyro_x_;
	gy = raw[4] / 131.0 / 180.0 * 3.14159265359 * cal_imu_gyro_ - bias_gyro_y_;
	gz = raw[5] / 131.0 / 180.0 * 3.14159265359 * cal_imu_gyro_ - bias_gyro_z_;
	return std::vector<float> ({ax, ay, az, gx, gy, gz});
}

std::vector<float> Odom::getOdom(std::vector<float> velocity)
{
	float v_left = velocity[0];
	float v_right = velocity[1];
	// Odometry calculation
	v_linear = (v_right + v_left) / 2 * calibration_v_linear_;
	v_angular = (v_right - v_left) / ( base_width / 4) * calibration_v_angular_;
	std::chrono::duration<float> elapsed_seconds = std::chrono::system_clock::now() - time_prev;
	float dt = elapsed_seconds.count();

	float delta_x = v_linear * cos(theta_odom) * dt;
	float delta_y = v_linear * sin(theta_odom) * dt;
	float delta_th = v_angular * dt;
	x_odom += delta_x;
	y_odom += delta_y;
	theta_odom += delta_th;

//	ROS_INFO("Time difference since last cmd calc: %f \t dx: %f \t dy: %f", dt, delta_x, delta_y);

	//Set previous time to now
	time_prev = std::chrono::system_clock::now();
	return std::vector<float>({x_odom, y_odom, theta_odom});
}

std::vector<float> Odom::getVelocity()
{
	return std::vector<float>({v_linear, v_angular});
}
