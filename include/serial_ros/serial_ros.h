#include <ros/ros.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

using namespace LibSerial;

class Serial_ros
{
public:
	Serial_ros(std::string name);
	~Serial_ros();

	void spin();

private:
    ros::NodeHandle nh_;

    SerialPort serial_port_;
    std::string serial_port_name_;
    int parity_;
    int flowcontrol_;
    int baund_rate_;
    int character_size_;
    int stop_bits_;

    std::string serial_read;

    void serial_baund_rate(int baund_rate);
    void serial_character_size(int character_size);
    void serial_stop_bits(int stop_bits);
    void serial_parity(int parity);
    void serial_flowcontrol(int flowcontrol);
};