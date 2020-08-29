# serial_communcation

## Brief description
This nodes makes the connection to STMF32 MCU,
publishes all sensor data, and sends velocity commands to the robot.

## Node description

## Subscribers
/cmd_vel [geometry_msgs/Twist]:
Command velocity. These commands are forwarded directly to the actuators. 

/pose2D [geometry_msgs/Pose2D]:
Can be used to improve odometry.

/ekf/odom [nav_msgs/Odometry]:
Can alternatively be used to improve odometry.

## Publishers
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

## TF Broadcaster
odom -> base_link TF can be broadcasted by this node, just enable *publish_odom_tf_* in config file. 

## Parameters
**usb_port:** Full path to USB port that MCU is connected to, .e.g. "/dev/ttyUSB0"

**orientation_cov:** 3x3 covariance matrix of the orientation that is published by this node in the topic */imu*. Not necessary to be defined as this node does not publish orientation of robot in the */imu* topic.

**gyro_cov:** 3x3 covariance matrix of gyroscope readings in */imu* topic. Values should be as accurate as possible to ensure that any ekf algorithms that take in the */imu* topic is able to make accurate estimations

**acc_cov:** 3x3 covariance matrix of acceleration readings in */imu* topic.

**pose_cov:** 6x6 covariance matrix of pose (position and orientation) in */encoders/odom* topic. Values should be set accurately if pose is fed into ekf algorithm.

**twist_cov:** 6x6 covariance matrix of twist (linear and angular) in */encoders/odom* topic. Values should be set accurately if twist is fed into ekf algorithm. 

**calibration_v_angular:** This value affects the angular velocity reported by this node based on the encoder readings from MCU. Formula is as follows:

        v_angular = (v_right - v_left) / (base_width) * calibration_v_angular_

Can be used to reduce angular drift of robot. To calibratate, turn the robot 1 round and make sure that */encoders/odom* reports the orientation as [0,0,0,1] where *w* is 1. If not, increase or decrease this parameter to calibrate it.

**calibration_v_linear:** This value affects the linear velocity reported by this node based on the encoder readings from MCU. Formula is as follows:

        v_linear = (v_right + v_left) / 2 * calibration_v_linear_

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

**apply_vel_filter:** Set true to enable moving average filter on the left/right wheel velocities read from processing raw encoder readings from MCU.

**vel_filter_size:** Integer value defining the size of the moving average filter window. A value of 5 will keep 5 values in the moving average window.

**wheel_acc_limit:** Manually defined acceleration limit of the wheels. This is also used to reduce noise in the readings of left/right wheel velocities. If the velocities are found the have too high an acceleration, then the left/right velocities will only be incremeneted/deceremented by *wheel_acc_limit * dt*.

        v_right = v_right_prev + wheel_acc_limit_ * dt * sign(acc_right)
        v_left = v_left_prev + wheel_acc_limit_ * dt * sign(acc_left)

**deadzone_pulse_width:** Pulsewidth that is considered deadzone for the wheels. Value ranges from 0-500. If set to 500, motors will move at full speed no matter what velocity is commanded. 0 means this parameter has no effect, and the motors might experience deadzones, where the commanded velocity is not high enough to move the wheels. This parameter is not recommeneded to be used if PID values below are defined.

**motor_kp:** Proportional gain for PID of the wheels. Applies to both wheels. 

**motor_ki:** Integral gain for the PID of the wheels. Applies to both wheels.

**motor_kd:** Derivative gain for the PID of the wheels. Applies to both wheels.

**motor_f:** Feedforward factor for the PID of the wheels. Applies to both wheels. Value should be at least 1. PID formula is as below:

        Foutput = motor_f * motor_cmd
        Poutput = motor_kp * error
        Ioutput = motor_ki * errorSum * dt
        Doutput = motor_kd * errorRate

        motorPID = Foutput + Poutput + Ioutput + Doutput

        motorPWM = motorPID / pwmScaling

**frequency:** Frequency at which to run the PID loop.

**motor_max_accel:** Ramping rate of the PID output, this is used to prevent jerky motions of the motor. If the motorPID calculated exceeds the motor_max_accel, then motorPID is limited to and increase of motor_max_accel * dt.

        motorPID = clamp(motorPID, lastMotorPID - motor_max_accel * dt, lastMotorPID + motor_max_accel * dt)

**w_tolerance:** Tolerance for deviation in current yaw velocity from commanded yaw velocity. For example, if the motors are commanded to go straight, the yaw velocity is 0. However, due to different timings in the ramping of each motor, the robot may turn left slightly before going straight. Meaning at the beggining of the motion, yaw velocity may not actually be 0, and it may be a positive number (turning clockwise, left).

This parameter should stop the ramping of the faster wheel to allow the other wheel to catch up. 

*Note that this parameter does not function correctly yet*

## Troubleshooting:

Check which USB port the MCU is connected. Check using the command below:

        ls /dev/ttyUSB*

Make sure you have the rights to use USB ports. If you receive permission denied errors, run:

        sudo chmod 777 /dev/ttyUSB*

replace * with the port number that the MCU is connected to, for example "/dev/ttyUSB0"

