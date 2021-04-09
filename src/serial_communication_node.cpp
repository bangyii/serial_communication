#include <serial_communication/serial_communication.h>
#include <serial_communication/odom.h>
#include <serial_communication/cmd_vel.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/base_utils.h>
#include <exception>

#include <sensor_msgs/Imu.h> // ROS message used for linear and angular accelerations
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h> // ROS message used for joystick data
#include <std_msgs/Bool.h>
#include <boost/asio.hpp>				// Include boost library function
#include <chrono>

#define BUFFER_SIZE 12
#define TEMP_BUFFER_SIZE 24

std::vector<double> CovOrient = {0.05, 0, 0,
								 0, 0.05, 0,
								 0, 0, 0.05};
std::vector<double> CovGyro = {0.05, 0, 0,
							   0, 0.05, 0,
							   0, 0, 0.05};
std::vector<double> CovAcc = {0.05, 0, 0,
							  0, 0.05, 0,
							  0, 0, 0.05};
std::vector<double> CovPose = {0.05, 0, 0, 0, 0, 0,
							   0, 0.05, 0, 0, 0, 0,
							   0, 0, 0.05, 0, 0, 0,
							   0, 0, 0, 0.05, 0, 0,
							   0, 0, 0, 0, 0.05, 0,
							   0, 0, 0, 0, 0, 0.05};
std::vector<double> CovTwist = {0.05, 0, 0, 0, 0, 0,
								0, 0.05, 0, 0, 0, 0,
								0, 0, 0.05, 0, 0, 0,
								0, 0, 0, 0.05, 0, 0,
								0, 0, 0, 0, 0.05, 0,
								0, 0, 0, 0, 0, 0.05};
float xJoyBias = 0.0, yJoyBias = 0.0, joystickScale = 0.95;
float joystick_deadzone = 0.05;
float base_width = 0.5;

bool readParameters(ros::NodeHandle &nh);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_communication"); // Initialize the node
	ros::NodeHandle node_handle("~");
	SerialComm serial("/dev/ttyUSB0");
	Odom odom;
	CmdVel cmdVel;

	// Read Parameters
	if (!readParameters(node_handle) || !serial.readParameters(node_handle) || !odom.readParameters(node_handle) || !cmdVel.readParameters(node_handle))
	{
		ROS_ERROR("Could not read parameters for serial_communication");
		ros::requestShutdown();
	}

	// Subscribers
	ros::Subscriber cmd_vel_sub = cmdVel.getCmdVelSub(node_handle);
	ros::Subscriber odom_update_sub = odom.getOdomUpdateSub(node_handle);

	// Publishers
	ros::Publisher joystick_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/user/joy", 1);
	ros::Publisher velocity_pub = node_handle.advertise<geometry_msgs::Twist>("/velocities", 1);
	ros::Publisher acceleration_pub = node_handle.advertise<sensor_msgs::Imu>("/imu", 1);
	ros::Publisher velocity_raw_pub = node_handle.advertise<std_msgs::Float32MultiArray>("/velocities_raw", 1);
	ros::Publisher odom_pub = node_handle.advertise<nav_msgs::Odometry>("/encoders/odom", 1);
	ros::Publisher odom2d_pub = node_handle.advertise<geometry_msgs::Pose2D>("/encoders/pose2D", 1);
	ros::Publisher estop_pub = node_handle.advertise<std_msgs::Bool>("/e_stop", 1, true);
	tf::TransformBroadcaster odom_broadcaster;

	// Variables
	int16_t buf[BUFFER_SIZE];		// Define the received data
	int16_t velocitybuf[3]; //define wheelchair speed including side1 x and side2 y as well as one header for data verification.
	std::vector<float> JoystickValue = {0, 0};
	int16_t e_stop = 0;

	//TODO: unnecessary duplicate of variable
	odom.base_width = cmdVel.base_width = base_width;

	// Set up
	boost::asio::serial_port sp = serial.setupPort();
	if (!sp.is_open())
	{
		ROS_ERROR("Unable to open serial port, shutting down");
		ros::shutdown();
	}

	while (ros::ok())
	{
		ros::spinOnce();

		uint8_t buf_temp[TEMP_BUFFER_SIZE];
		try
		{
			//Get cmd_vel from topic and then write to MCU
			cmdVel.getCmdVel(velocitybuf);
			write(sp, boost::asio::buffer(velocitybuf));

			//Read data from MCU and place into temp buffer
			read(sp, boost::asio::buffer(buf_temp)); // block
		}

		catch (const std::exception &e)
		{
			ROS_WARN("Error: %s", e.what());
			break;
		}

		//Decode bytes received from MCU
		for (uint8_t i = 0; i < BUFFER_SIZE; i++)
			buf[i] = buf_temp[2 * i + 1] << 8 | buf_temp[2 * i];

		//Check if last byte received is correct
		if (buf[BUFFER_SIZE - 1] != int16_t(0xabcd))
		{
			std::cout << "Data lost, skip this transmission with buf " << std::hex << buf[BUFFER_SIZE - 1] << ". Initializing re-synchronisation\n";
			serial.syncSerial(&sp);
			std::cout << "Re-synchronisation complete\n";
			continue;
		}

		// Implicit conversion from uint16 to float
		JoystickValue[0] = (-(buf[1] - 2047.0) / 1500.0 - xJoyBias) / joystickScale; //x
		JoystickValue[1] = (-(buf[0] - 2047.0) / 1500.0 - yJoyBias) / joystickScale; //y
		if (JoystickValue[0] > 1)
			JoystickValue[0] = 1;
		else if (JoystickValue[0] < -1)
			JoystickValue[0] = -1;
		else if (fabs(JoystickValue[0]) < joystick_deadzone)
			JoystickValue[0] = 0.0;

		if (JoystickValue[1] > 1)
			JoystickValue[1] = 1;
		else if (JoystickValue[1] < -1)
			JoystickValue[1] = -1;
		else if (fabs(JoystickValue[1]) < joystick_deadzone)
			JoystickValue[1] = 0.0;

		//Get estop status from MCU
		e_stop = buf[10];

		//Get acc and gyro from buffer
		//imuReadings : {ax, ay, az, gx, gy, gz}
		std::vector<float> imuReadings = odom.getIMU(std::vector<float>(buf + 4, buf + 10));

		//Get raw velocity from encoders
		//velocity_raw : {v_left, v_right}, data received in buffer is multiplied by 1000
		std::vector<float> velocity_raw = {buf[2] / (float)1000.0, buf[3] / (float)1000.0};

		//currentOdom : {x_odom, y_odom, theta}
		std::vector<float> currentOdom = odom.getOdom(velocity_raw);
		
		// Publish estop state
		std_msgs::Bool e_stop_status;
		e_stop_status.data = e_stop == 1.0;
		estop_pub.publish(e_stop_status);

		// Publish Joystick data
		joystick_pub.publish(rosmsg::makeFloat32MultiArray(JoystickValue));

		// Publish raw Velocity data
		velocity_raw_pub.publish(rosmsg::makeFloat32MultiArray(velocity_raw));

		// Publish Velocity data
		//velocity : {v_linear, v_angular}
		std::vector<float> velocity = odom.getVelocity();
		velocity_pub.publish(rosmsg::makeTwist(velocity[0], velocity[1]));

		// Publish Odometry Pose2D
		odom2d_pub.publish(rosmsg::makePose2D(currentOdom[0], currentOdom[1], currentOdom[2]));

		// Publish Odometry message
		odom_pub.publish(rosmsg::makeOdometry(rosmsg::makeHeader("odom", ros::Time::now()), "base_link",
											  currentOdom[0], currentOdom[1], currentOdom[2], velocity[0], velocity[1],
											  CovPose, CovTwist));

		// Publish IMU data
		acceleration_pub.publish(rosmsg::makeIMU(rosmsg::makeHeader("imu", ros::Time::now()),
												 0, 0, 0,
												 imuReadings[3], imuReadings[4], imuReadings[5],
												 imuReadings[0], imuReadings[1], imuReadings[2],
												 CovOrient, CovGyro, CovAcc));

		// Send Odom transform (ONLY IF NO OTHER ODOM PUBLISHER ACTIVE!!)
		// (e.g. laser_scan_matcher or robot_pose_ekf)
		if (odom.publish_odom_tf_)
		{
			tf::Quaternion odom_quat = tf::createQuaternionFromYaw(currentOdom[2]);
			odom_broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(odom_quat, tf::Vector3(currentOdom[0], currentOdom[1], 0.0)),
					ros::Time::now(), "odom", "base_link"));
		}
	}

	ROS_INFO("Shutting down serial comms node, write 0.0 to motor");
	velocitybuf[0] = 0.0;
	velocitybuf[1] = 0.0;
	write(sp, boost::asio::buffer(velocitybuf));
	serial.runIOService();
	return 0;
}

bool readParameters(ros::NodeHandle &nh)
{
	if (!nh.getParam("orientation_cov", CovOrient))
		ROS_WARN_STREAM("Parameter orientation_cov not set. Using default setting: 0.05 * (3x3) Identity matrix");
	if (!nh.getParam("gyro_cov", CovGyro))
		ROS_WARN_STREAM("Parameter gyro_cov not set. Using default setting: 0.05 * (3x3) Identity matrix");
	if (!nh.getParam("acc_cov", CovAcc))
		ROS_WARN_STREAM("Parameter acc_cov not set. Using default setting: 0.05 * (3x3) Identity matrix");
	if (!nh.getParam("pose_cov", CovPose))
		ROS_WARN_STREAM("Parameter odom_cov not set. Using default setting: 0.05 * (6x6) Identity matrix");
	if (!nh.getParam("twist_cov", CovTwist))
		ROS_WARN_STREAM("Parameter velocity_cov not set. Using default setting: 0.05 * (6x6) Identity matrix");
	if (!nh.getParam("x_joy_bias", xJoyBias))
		ROS_WARN_STREAM("Parameter x_joy_bias not set. Using default setting: " << xJoyBias);
	if (!nh.getParam("y_joy_bias", yJoyBias))
		ROS_WARN_STREAM("Parameter y_joy_bias not set. Using default setting: " << yJoyBias);
	if (!nh.getParam("joystick_factor", joystickScale))
		ROS_WARN_STREAM("Parameter joystick_factor not set. Using default setting: " << joystickScale);
	if (!nh.getParam("base_width", base_width))
		ROS_WARN_STREAM("Parameter base_width not set. Using default setting: " << base_width);
	if (!nh.getParam("joystick_deadzone", joystick_deadzone))
		ROS_WARN_STREAM("Parameter joystick_deadzone not set. Using default setting: " << joystick_deadzone);
	return true;
}
