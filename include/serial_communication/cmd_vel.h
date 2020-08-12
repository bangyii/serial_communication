#ifndef CMD_VEL_H
#define CMD_VEL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <deque>
#include <chrono>
#include "serial_communication/MiniPID.h"

class CmdVel
{
public:
    CmdVel();
    ros::Subscriber getCmdVelSub(ros::NodeHandle &nh);
    void cmdCallback(const geometry_msgs::Twist msg_cmd);
    void getCmdVel(int16_t velocitybuf[3]);
    void medianFilter(float &x, float &y);
    bool readParameters(ros::NodeHandle &node_handle);
    std::vector<float> getVelFromEncoder(std::vector<float> encoder);

    //Motor pid
    float motor_kp = 1, motor_ki = 0, motor_kd = 0, motor_f = 0, frequency = 5.0;
    float ramp_rate = 0.3;
    MiniPID left_motor_pid, right_motor_pid;

    //Parameters
    ros::Time time_last_cmd;
    geometry_msgs::Twist cmd_vel;
    float calibration_cmd_ang_ = 1.0;
    float calibration_cmd_lin_ = 1.0;
    float wheel_diameter = 0.25;
    float base_width = 0.5;
    float VelocityMax = 1.125; //m/s
    float deadzone_pulse_width = 0;
    float max_dt_cmd = 0.5;
    float f_right, f_left, v_right, v_left, v_right_prev, v_left_prev, v_right_cmd, v_left_cmd;
    float v_left_cmd_prev, v_right_cmd_prev;
    bool apply_vel_filter_ = true;
    int vel_filter_size_ = 5;
    float wheel_acc_limit_ = 10.0;

private:
    ros::Time timestamp;
    std::deque<float> buffer_x;
    std::deque<float> buffer_y;
    float encoder_left, encoder_right, encoder_left_prev = -1, encoder_right_prev = -1;
    std::chrono::time_point<std::chrono::system_clock> time_prev = std::chrono::system_clock::now();
};

#endif
