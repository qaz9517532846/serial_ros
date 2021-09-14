#include <serial_ros/serial_ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_ros_node");
    
    Serial_ros Serial_ros(ros::this_node::getName());
    Serial_ros.spin();

	return 0;
}