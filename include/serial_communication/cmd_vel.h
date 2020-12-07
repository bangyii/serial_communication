#ifndef CMD_VEL_H
#define CMD_VEL_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <deque>
#include <chrono>

class CmdVel
{
public:
    CmdVel();
    ros::Subscriber getCmdVelSub(ros::NodeHandle &nh);
    void cmdCallback(const geometry_msgs::Twist msg_cmd);
    void getCmdVel(int16_t velocitybuf[3]);
    bool readParameters(ros::NodeHandle &node_handle);

    //Parameters
    // ros::Time time_last_cmd;
    geometry_msgs::Twist cmd_vel;
    float calibration_cmd_ang_ = 1.0;
    float calibration_cmd_lin_ = 1.0;
    float base_width = 0.5;
    float v_right_cmd, v_left_cmd;
    float v_left_cmd_prev, v_right_cmd_prev;

private:
    ros::Time timestamp;
    std::deque<float> buffer_x;
    std::deque<float> buffer_y;
    float encoder_left, encoder_right, encoder_left_prev = -1, encoder_right_prev = -1;
    std::chrono::time_point<std::chrono::system_clock> time_prev = std::chrono::system_clock::now();
};

#endif
