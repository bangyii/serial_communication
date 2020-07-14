#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <ros/ros.h>
#include <deque>
#include <boost/asio.hpp> // Include boost library function

class SerialComm
{
public:
    SerialComm();
    SerialComm(std::string usb_port);
    bool readParameters(ros::NodeHandle &node_handle);
    boost::asio::serial_port setupPort();
    void getCmdVel(int16_t velocitybuf[3]);
    void runIOService();

    /**
     * Param variables
     **/
    std::string usb_port_ = "/dev/ttyUSB0";

private:
	boost::asio::io_service iosev;
};

#endif