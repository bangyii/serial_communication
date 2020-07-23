#include <serial_communication/serial_communication.h>
#include <serial_communication/odom.h>
#include <serial_communication/cmd_vel.h>
#include <scat_libs/rosmsg.h>
#include <scat_libs/base_utils.h>

#include <sensor_msgs/Imu.h>			// ROS message used for linear and angular accelerations
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h> // ROS message used for joystick data
#include <boost/asio.hpp> // Include boost library function
#include <chrono>

// IMU covariance matrices, measured using rosbag during experiment while driving around and calculated using numpy. 0 entry means negligible.
std::vector<double> CovOrient = {-1, -1, -1, // Orientation not given by IMU, set Covariance to -1 and estimates to 0. 
								-1, -1, -1,
								-1, -1, -1};
// std::vector<double> CovOrient = {1, 0, 0,
// 								0, 1, 0,
// 								0, 0, 1};								
std::vector<double> CovGyro = {0.002, 0, 0,
								0, 0.002, 0,
								0, 0, 0.002};
std::vector<double> CovAcc = {0.002, 0, 0,
								0, 0.002, 0,
								0, 0, 0.34};
std::vector<double> CovOdom = {0.2, 0, 0, 0, 0, 0, 
								0, 0.2, 0, 0, 0, 0, 
								0, 0, 0.01, 0, 0, 0,
								0, 0, 0, 0.01, 0, 0,
								0, 0, 0, 0, 0.01, 0,
								0, 0, 0, 0, 0, 0.4};
std::vector<double> CovVel = {0.1, 0, 0, 0, 0, 0, 
								0, 0.01, 0, 0, 0, 0, 
								0, 0, 0.01, 0, 0, 0,
								0, 0, 0, 0.01, 0, 0,
								0, 0, 0, 0, 0.01, 0,
								0, 0, 0, 0, 0, 0.1};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_communication"); // Initialize the node
	ros::NodeHandle node_handle("~");
	SerialComm serial("/dev/ttyUSB0");
	Odom odom;
	CmdVel cmdVel;

	// Read Parameters
	if (!serial.readParameters(node_handle) || !odom.readParameters(node_handle) || !cmdVel.readParameters(node_handle))
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
	tf::TransformBroadcaster odom_broadcaster;

	// Variables
	int16_t buf[11];		// Define the received data
	int16_t velocitybuf[3]; //define wheelchair speed including side1 x and side2 y as well as one header for data verification.
	std::vector<float> JoystickValue = {0, 0};

	// Parameters
	// // Data quality check
	// float variance_x,variance_y,mean_x,mean_y;
	// int Period_Count=500;
	// int count=0;
	// std::vector<float> x_data(Period_Count);
	// std::vector<float> y_data(Period_Count);

	//TODO: unnecessary duplicate of variable 
	odom.base_width = cmdVel.base_width = 0.5;

	// Set up
	boost::asio::serial_port sp = serial.setupPort();
	if(!sp.is_open()){
		ROS_ERROR("Unable to open serial port, shutting down");
		ros::shutdown();
	}

	while (ros::ok())
	{
		// time_now = std::chrono::system_clock::now();

		ros::spinOnce(); // check for incoming messages
		// static uint32_t cnt = 0;
		// ROS_INFO_STREAM("cnt: " << (cnt++) / 200.0 ) ;

		//Get cmd_vel from topic and then write to MCU
		cmdVel.getCmdVel(velocitybuf);
		write(sp, boost::asio::buffer(velocitybuf)); 


		//Read data from MCU and place into temp buffer
		static uint8_t buf_temp[22];
		read(sp, boost::asio::buffer(buf_temp)); // block

		//Decode bytes received from MCU
		for (uint8_t i = 0; i < 11; i++)
		{
			//ROS_INFO_STREAM((uint16_t)buf_temp[i]);
			buf[i] = buf_temp[2 * i + 1] << 8 | buf_temp[2 * i];
			// ROS_INFO_STREAM(buf[i]);
		}

		if (buf[10] != int16_t(0xabcd))
		{
			std::cout << "data lost skip this transmission, with buf " << std::hex << buf[10] << std::endl;
			continue;
		}

		// Implicit conversion from uint16 to float
		JoystickValue[0] = -(buf[1] / 2048.0 - 1.0)/0.7;
		JoystickValue[1] = -(buf[0] / 2048.0 - 1.0)/0.7;

		//Get acc and gyro from buffer
		//imuReadings = {ax, ay, az, gx, gy, gz}
		std::vector<float> imuReadings = odom.getIMU(std::vector<float>(buf + 4, buf + 10));

		//Get raw velocity from encoders
		//velocity_raw = {v_left, v_right}
		std::vector<float> velocity_raw = cmdVel.getVelFromEncoder(std::vector<float>(buf + 2, buf + 4));

		//currentOdom = {x_odom, y_odom, theta}
		std::vector<float> currentOdom = odom.getOdom(velocity_raw);


		// if(count>=Period_Count){
		// 	mean_x=mean_y=variance_x=variance_y=0.;
		//     for(int i=0;i<Period_Count;i++){
		// 		mean_x+=x_data[i];
		// 		mean_y+=y_data[i];
		// 	}
		// 	mean_x/=Period_Count;
		// 	mean_y/=Period_Count;
		// 	for(int i=0;i<Period_Count;i++){
		// 		variance_x+=(x_data[i]-mean_x)*(x_data[i]-mean_x);
		// 		variance_y+=(y_data[i]-mean_y)*(y_data[i]-mean_y);
		// 	}
		// 	variance_x/=Period_Count;
		// 	variance_y/=Period_Count;
		// 	count=0;
		// 	std::cout<<"x variance "<<std::sqrt(variance_x)<<" y variance "<<std::sqrt(variance_y)<<std::endl;
		// }else{
		// 	x_data[count]=JoystickValue[0];
		// 	y_data[count]=JoystickValue[1];
		// 	count+=1;
		// }

		// Publish Joystick data
		joystick_pub.publish(rosmsg::makeFloat32MultiArray(JoystickValue));

		// Publish raw Velocity data
		velocity_raw_pub.publish(rosmsg::makeFloat32MultiArray(velocity_raw));

		// Publish Velocity data
		//velocity = {v_linear, v_angular}
		std::vector<float> velocity = odom.getVelocity();
		velocity_pub.publish(rosmsg::makeTwist(velocity[0], velocity[1]));

		// Publish Odometry Pose2D
		odom2d_pub.publish(rosmsg::makePose2D(currentOdom[0], currentOdom[1], currentOdom[2]));

		// Publish Odometry message
		odom_pub.publish(rosmsg::makeOdometry(rosmsg::makeHeader("odom", ros::Time::now()),
											  currentOdom[0], currentOdom[1], currentOdom[2], velocity[0], velocity[1],
											  CovOdom, CovVel));

		// Publish IMU data
		acceleration_pub.publish(rosmsg::makeIMU(rosmsg::makeHeader("base_footprint", ros::Time::now()),
												0,0,0,
												imuReadings[3],imuReadings[4],imuReadings[5],
												imuReadings[0],imuReadings[1],imuReadings[2],
												CovOrient,CovGyro,CovAcc));

		// Send Odom transform (ONLY IF NO OTHER ODOM PUBLISHER ACTIVE!!)
		// (e.g. laser_scan_matcher or robot_pose_ekf)
		if (odom.publish_odom_tf_)
		{
			tf::Quaternion odom_quat = tf::createQuaternionFromYaw(currentOdom[2]);
			odom_broadcaster.sendTransform(
				tf::StampedTransform(
					tf::Transform(odom_quat, tf::Vector3(currentOdom[0], currentOdom[1], 0.0)),
					ros::Time::now(), "odom", "base_footprint"));
		}
	}
	serial.runIOService();
	return 0;
}
