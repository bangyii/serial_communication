#ifndef ODOM_H
#define ODOM_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <chrono>


class Odom
{
public:
    Odom();
    bool readParameters(ros::NodeHandle &node_handle);
    void poseCallback(const geometry_msgs::Pose2D msg_pose);
    void odomCallback(const nav_msgs::Odometry msg_odom);
    ros::Subscriber getOdomUpdateSub(ros::NodeHandle& nh);
    std::vector<float> getIMU(std::vector<float> raw);
    std::vector<float> getOdom(std::vector<float> velocity);
    std::vector<float> getVelocity();

    std::string odom_update_method_ = "odom";
    bool publish_odom_tf_ = false;    

    // Calibrated sensor parameters
    // Odometry parameters
    float calibration_v_angular_ = 1.0;
    float calibration_v_linear_ = 1.0;

    // IMU parameters
    float bias_acc_x_ = 0;
    float bias_acc_y_ = 0;
    float bias_acc_z_ = 0;
    float bias_gyro_x_ = 0;
    float bias_gyro_y_ = 0;
    float bias_gyro_z_ = 0;
    float cal_imu_gyro_ = 1.0;
    float cal_imu_acc_ = 1.0;
    float base_width = 0.5;

    float x_odom = 0, y_odom = 0, theta_odom = 0;
    float ax, ay, az, gx, gy, gz;
    float v_linear, v_angular;

	std::chrono::time_point<std::chrono::system_clock> time_prev = std::chrono::system_clock::now();
};

#endif