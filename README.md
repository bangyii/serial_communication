# serial_communcation

## Brief description
This nodes makes the connection to STMF32 MCU,
publishes all sensor data, and sends velocity commands to the robot.

## Node description

## Subscribers
**/cmd_vel [geometry_msgs/Twist]:** Command velocity. These commands are forwarded directly to the actuators. 

**/pose2D [geometry_msgs/Pose2D]:**
Can be used to improve odometry.

**/ekf/odom [nav_msgs/Odometry]:**
Can alternatively be used to improve odometry.

## Publishers
**/user/joy [std_msgs/Float32MultiArray]:**
Joystick output from the wheelchair joystick. First element is x direction, maximum when joystick pushed forward, value range [-1.0, 1.0]. Second element is y direction, maximum when pushed left, value range [-1.0, 1.0]

**/velocities [geometry_msgs/Twist]:**
linear and angular velocity based on wheel encoders

**/velocities_raw [std_msgs/Float32MultiArray]:**
individual velocities left and right encoders

**/imu [sensor_msgs/IMU]:**
imu data

**/encoders/odom [nav_msgs/Odometry]:**
Odometry estimation based on previous odometry estimation and current velocity measurement. 
If available, the odometry estimation of e.g. EKF can be used to improve the guess. 

**/encoders/pose2D [geometry_msgs/Pose2D]:**
2d pose estimation based on encoder odometry. More convenient format for some applications

## TF Broadcaster
odom -> base_link TF can be broadcasted by this node, just enable *publish_odom_tf_* in config file. 

## Parameters
**usb_port:** Full path to USB port that MCU is connected to, .e.g. "/dev/ttyUSB0"

**orientation_cov:** 3x3 covariance matrix of the orientation that is published by this node in the topic */imu*. Not necessary to be defined as this node does not publish orientation of robot in the */imu* topic.

**gyro_cov:** 3x3 covariance matrix of gyroscope readings in */imu* topic. Values should be as accurate as possible to ensure that any ekf algorithms that take in the */imu* topic is able to make accurate estimations

**acc_cov:** 3x3 covariance matrix of acceleration readings in */imu* topic.

**pose_cov:** 6x6 covariance matrix of pose (position and orientation) in */encoders/odom* topic. Values should be set accurately if pose is fed into ekf algorithm.

**twist_cov:** 6x6 covariance matrix of twist (linear and angular) in */encoders/odom* topic. Values should be set accurately if twist is fed into ekf algorithm. 

**bias_acc_x:** This value offsets the x-acceleration in */imu*. Units are in m/s^2. Calibration formula is as below:

        ax = ax * cal_imu_acc_ - bias_acc_x_

This means that if the IMU is reporting an x-acceleration of 10 when it should be reporting 9, then this parameter should be set to 1, and vice versa if the IMU is under-reporting.

**bias_acc_y:** Same as *bias_acc_x*

**bias_acc_z:** Same as *bias_acc_x*

**bias_gyro_x:** This value offsets the angular velocity around x in */imu*. Units are in rad/s. Calibration formula is as below:

        gx = gx * cal_imu_gyro_ - bias_gyro_x_

This means that if the IMU is reporting an x angular velocity of 1 when it should be reporting 0.5, then this parameter should be set to 0.5, and vice versa if the IMU is under-reporting.

**bias_gyro_y:** Same as *bias_gyro_x*

**bias_gyro_z:** Same as *bias_gyro_x*

**cal_imu_gyro:** Check formula shown in *bias_acc_x*. Note that this will affect acceleration readings in all 3 axes equally. This parameter is not usually used and is kept at 1.0 to have no effect.

**cal_imu_acc:** Check formula shown in *bias_gyro_x*. Note that this will affect gyroscope readings in all 3 axes equally. This parameter is not usually used and is kept at 1.0 to have no effect.

**odom_update_method:** Either "pose2D" or "odom". This determines what topic this node will subscribe to for odometry updates. If "pose2D", /pose2D is subscribed for odometry updates. If "odom", /ekf/odom is subscribed.

**publish_odom_tf:** Set true to publish odom -> base_link tf

**calibration_cmd_ang:** Scales how much the commanded left/right wheel velocity is affected by the commanded angular velocity. This value is usually not used and kept at 1.0 to negate its effect.

        v_left_cmd = cmd_vel.linear.x * calibration_cmd_lin_ - cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_
        v_right_cmd = cmd_vel.linear.x * calibration_cmd_lin_ + cmd_vel.angular.z * (base_width / 2) * calibration_cmd_ang_

**calibration_cmd_lin:** Same as *calibration_cmd_ang*. Usually kept at 1.0 as well.

## Troubleshooting:

Check which USB port the MCU is connected. Check using the command below:

        ls /dev/ttyUSB*

Make sure you have the rights to use USB ports. If you receive permission denied errors, run:

        sudo chmod 777 /dev/ttyUSB*

replace * with the port number that the MCU is connected to, for example "/dev/ttyUSB0"

