# serial_communcation

## Brief description
This nodes makes the connection to STMF32 MCU,
publishes all sensor data, and sends velocity commands to the robot.

## Node description

# Subscribers
/cmd_vel [geometry_msgs/Twist]:
Command velocity. These commands are forwarded directly to the actuators. 

/pose2D [geometry_msgs/Pose2D]:
Can be used to improve odometry.
/odom [nav_msgs/Odometry]:
Can alternatively be used to improve odometry.

# Publishers
/user/joy [std_msgs/Float32MultiArray]:
Joystick output from the wheelchair joystick

/velocities [geometry_msgs/Twist]:
linear and angular velocity based on wheel encoders

/velocities_raw [std_msgs/Float32MultiArray]:
individual velocities left and right encoders

/imu [sensor_msgs/IMU]:
imu data

/encoders/odom [nav_msgs/Odometry]:
Odometry estimation based on previous odometry estimation and current velocity measurement. 
If available, the odometry estimation of e.g. EKF can be used to improve the guess. 

/encoders/pose2D [geometry_msgs/Pose2D]:
2d pose estimation based on encoder odometry. More convenient format for some applications


## Troubleshooting:

Check which USB port the MCU is connected.
ls /dev/ttyUSB*

Make sure you have the rights to use USB ports. 
sudo chmod 777 /dev/ttyUSB0

