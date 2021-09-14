#include <serial_ros/serial_ros.h>

Serial_ros::Serial_ros(std::string name)
{
    ros::NodeHandle nh_("~/" + name);
    nh_.param<std::string>("Serial_Port", serial_port_name_, "/dev/ttyACM0");
    nh_.param<int>("Baund_rate", baund_rate_, 115200);
    nh_.param<int>("character_size", character_size_, 8);
    nh_.param<int>("stop_bits", stop_bits_, 1);
    nh_.param<int>("parity", parity_, 3);
    nh_.param<int>("flow_control", flowcontrol_, 0);

    serial_port_.Open(serial_port_name_.c_str());
    if(!serial_port_.IsOpen())
    {
        ROS_INFO("Find not device: %s", serial_port_name_.c_str());
        return;
    }

    serial_baund_rate(baund_rate_);
    serial_character_size(character_size_);
    serial_stop_bits(stop_bits_);
    serial_parity(parity_);
    serial_flowcontrol(flowcontrol_);
}

Serial_ros::~Serial_ros()
{

}

void Serial_ros::serial_baund_rate(int baund_rate)
{
    switch(baund_rate)
    { 
        case 300:
            serial_port_.SetBaudRate(BaudRate::BAUD_300);
            break;
        case 1200:
            serial_port_.SetBaudRate(BaudRate::BAUD_1200);
            break; 
        case 2400:
            serial_port_.SetBaudRate(BaudRate::BAUD_2400);
            break;
        case 9600:
            serial_port_.SetBaudRate(BaudRate::BAUD_9600);
            break; 
        case 19200: 
            serial_port_.SetBaudRate(BaudRate::BAUD_19200);
            break; 
        case 38400: 
            serial_port_.SetBaudRate(BaudRate::BAUD_38400);
            break;
        case 57600: 
            serial_port_.SetBaudRate(BaudRate::BAUD_57600);
            break; 
        case 115200: 
            serial_port_.SetBaudRate(BaudRate::BAUD_115200);
            break; 
        default: 
            serial_port_.SetBaudRate(BaudRate::BAUD_DEFAULT);
            break;
    }
}

void Serial_ros::serial_character_size(int character_size)
{
    switch(character_size)
    { 
        case 5:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_5);
            break;
        case 6:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_6);
            break; 
        case 7:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_7);
            break;
        case 8:
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            break; 
        default: 
            serial_port_.SetCharacterSize(CharacterSize::CHAR_SIZE_DEFAULT);
            break;
    }
}

void Serial_ros::serial_stop_bits(int stop_bits)
{
    switch(stop_bits)
    { 
        case 1:
            serial_port_.SetStopBits(StopBits::STOP_BITS_1);
            break;
        case 2:
            serial_port_.SetStopBits(StopBits::STOP_BITS_2);
            break; 
        default: 
            serial_port_.SetStopBits(StopBits::STOP_BITS_DEFAULT);
            break;
    }
}

void Serial_ros::serial_parity(int parity)
{
    switch(parity)
    { 
        case 1:
            serial_port_.SetParity(Parity::PARITY_EVEN);
            break;
        case 2:
            serial_port_.SetParity(Parity::PARITY_ODD);
            break; 
        case 3:
            serial_port_.SetParity(Parity::PARITY_NONE);
            break; 
        default: 
            serial_port_.SetParity(Parity::PARITY_DEFAULT);
            break;
    }
}

void Serial_ros::serial_flowcontrol(int flowcontrol)
{
    switch(flowcontrol)
    { 
        case 0:
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            break; 
        case 1:
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_HARDWARE);
            break;
        default: 
            serial_port_.SetFlowControl(FlowControl::FLOW_CONTROL_DEFAULT);
            break;
    }
}

void Serial_ros::spin()
{
    std::string read_string_1 ;
    
    // Variable to hold user input.
    std::string user_input ;
    user_input.clear() ;
    
    // Print to the terminal what will take place next.
    std::cout << "Using Write() and ReadLine() to write a string and "
              << "read a line of data:" << std::endl << std::endl ;

    // Prompt the user for input.
    std::cout << R"(Enter something you would like to send over )"
              << R"(serial, (enter "Q" or "q" to quit): )" << std::flush ;
    
    while(true)
    {
        // Get input from the user.
        std::getline(std::cin, user_input) ;

        if (user_input == "q" ||
            user_input == "Q" ||
            user_input == "")
        {
            break ;
        }

        // Write the user input to the serial port.
        serial_port_.Write(user_input + "\n") ;

        // Read the data transmitted from the corresponding serial port.
        serial_port_.ReadLine(read_string_1) ;

        // Print to the terminal what was sent and what was received.
        std::cout << "\tSerial Port 1 sent:\t"     << user_input   << std::endl
                  << "\tSerial Port 2 received:\t" << read_string_1 << std::endl ;
    }

    // Close the serial ports and end the program.
    serial_port_.Close() ;

    // Successful program completion.
    std::cout << "The example program successfully completed!" << std::endl ;
}