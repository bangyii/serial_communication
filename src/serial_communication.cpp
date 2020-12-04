#include <serial_communication/serial_communication.h>
#include <exception>

SerialComm::SerialComm()
{
}

SerialComm::SerialComm(std::string usb_port)
{
	usb_port_ = usb_port;
}

bool SerialComm::readParameters(ros::NodeHandle &node_handle)
{
	if (!node_handle.getParam("usb_port", usb_port_))
		ROS_WARN_STREAM("Parameter usb_port not set for serial_communication. Using default setting: " << usb_port_);
	return true;
}

boost::asio::serial_port SerialComm::setupPort()
{
	boost::asio::serial_port sp(iosev);
	try{
		sp.open(usb_port_); // Define the serial port of the transmission // serial_port sp(iosev, "/dev/USB_MCU");
		sp.set_option(boost::asio::serial_port::baud_rate(115200));
		sp.set_option(boost::asio::serial_port::flow_control());
		sp.set_option(boost::asio::serial_port::parity());
		sp.set_option(boost::asio::serial_port::stop_bits());
		sp.set_option(boost::asio::serial_port::character_size(8));
		ROS_INFO_STREAM("Initializing serial connection");

		//Clear RX buffer until end bytes are received, to synchronize data packets
		bool flag_beginning = false;
		while (!flag_beginning)
		{
			uint8_t temp[1];
			read(sp, boost::asio::buffer(temp));
			if (temp[0] == 0xcd)
			{
				read(sp, boost::asio::buffer(temp));
				if (temp[0] == 0xab)
				{
					flag_beginning = true;
				}
			}
		}
		ROS_INFO_STREAM("Achieved connection to MCU");
	}

	catch(const boost::system::system_error& e)
	{
		ROS_ERROR("Error occurred: %s", e.what());
	}

	return sp;
}

void SerialComm::runIOService()
{
	iosev.run();
}
