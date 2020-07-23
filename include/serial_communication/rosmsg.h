#ifndef ROSMSG_H
#define ROSMSG_H

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <algorithm>
#include <std_msgs/Header.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <boost/array.hpp>

namespace rosmsg{
    std_msgs::Float32MultiArray makeFloat32MultiArray(std::vector<float> input)
    {   std_msgs::MultiArrayDimension dimVar;
        dimVar.size = input.size();
        dimVar.stride = 0;

        std_msgs::Float32MultiArray ret;
        ret.layout.dim.push_back(dimVar);
        ret.data = input;
        return ret;
    }

    geometry_msgs::Twist makeTwist(float linear, float angular)
    {
        geometry_msgs::Twist newTwist;
        newTwist.linear.x = linear;
        newTwist.angular.z = angular;

        return newTwist;
    }

    geometry_msgs::Pose2D makePose2D(float x, float y, float theta)
    {
        geometry_msgs::Pose2D newPose;
        newPose.x = x;
        newPose.y = y;
        newPose.theta = theta;

        return newPose;
    }

    std_msgs::Header makeHeader(std::string frame_id, ros::Time stamp)
    {
        std_msgs::Header newHeader;
        newHeader.frame_id = frame_id;
        newHeader.stamp = stamp;

        return newHeader;
    }

    nav_msgs::Odometry makeOdometry(std_msgs::Header header, std::string child_frame, float x, float y, float theta, float v_linear, float v_angular, 
                                    std::vector<double> odomCov, std::vector<double> velCov)
    {
        nav_msgs::Odometry newOdom;
        newOdom.header = header;
        newOdom.child_frame_id = child_frame;
        newOdom.pose.pose.position.x = x;
        newOdom.pose.pose.position.y = y;

        tf::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
        newOdom.pose.pose.orientation.w = odom_quat.w();
        newOdom.pose.pose.orientation.x = odom_quat.x();
        newOdom.pose.pose.orientation.y = odom_quat.y();
        newOdom.pose.pose.orientation.z = odom_quat.z();

        boost::array<double, 36> odomCovBoost;
        std::copy(odomCov.begin(), odomCov.end(), odomCovBoost.begin());
        newOdom.pose.covariance = odomCovBoost;

        newOdom.twist.twist.linear.x = v_linear;
        newOdom.twist.twist.angular.z = v_angular;
        boost::array<double, 36> velCovBoost;
        std::copy(velCov.begin(), velCov.end(), velCovBoost.begin());
        newOdom.twist.covariance = velCovBoost;

        return newOdom;
    }

    sensor_msgs::Imu makeIMU(std_msgs::Header header, float gx, float gy, float gz, float ax, float ay, float az,
                            std::vector<double> orientCov, std::vector<double> gyroCov, std::vector<double> accCov)
    {
        sensor_msgs::Imu newIMU;
        newIMU.header = header;

        newIMU.angular_velocity.x = gx;
        newIMU.angular_velocity.y = gy;
        newIMU.angular_velocity.z = gz;

        newIMU.linear_acceleration.x = ax;
        newIMU.linear_acceleration.y = ay;
        newIMU.linear_acceleration.z = az;

        boost::array<double, 9> orientCovBoost;
        std::copy(orientCov.begin(), orientCov.end(), orientCovBoost.begin());
        newIMU.orientation_covariance = orientCovBoost;

        boost::array<double, 9> gyroCovBoost;
        std::copy(gyroCov.begin(), gyroCov.end(), gyroCovBoost.begin());
        newIMU.angular_velocity_covariance = gyroCovBoost;

        boost::array<double, 9> accCovBoost;
        std::copy(accCov.begin(), accCov.end(), accCovBoost.begin());
        newIMU.linear_acceleration_covariance = accCovBoost;

        return newIMU;
    }
}

#endif