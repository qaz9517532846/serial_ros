# serial_ros
Serial Port Conmmunication C++ program under Robot Operating System.

- Software: Robot Operating System and LibSerial.

- Version: kinetic, melodic, noetic.

------

###  Install LibSerial.

  ``` $ sudo apt-get install -y libserial-dev ```
  
------

### Clone serial_ros

``` bash
$ cd <catkin_workspace>/src
```

``` bash
$ git clone https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

``` bash
$ cd ..
```

``` bash
$ catkin_make
```

------

### Run serial_ros

``` bash
$ roslaunch serial_ros serial_ros_node.launch
```

------

### ROS Parameter

- Serial_Port(type: string, default: "/dev/ttyACM0") : Serial Comunication with device(ex. Arduino, STM32, Renease MCU).

- Baund_rate(type: int, default: 115200): setting device baund rate.

- character_size(type: int, default: 8): setting device character size.

- stop_bits(type: int, default: 1): setting device stop bits.

- parity(type: int, default: 0): setting device parity.

   - 1 is even.

   - 2 is odd.

   - 3 is none.

- flow_control(type: int, default: 1): setting device flow control.

   - 0 is none.

   - 1 is hardware.

------

### Reference

[1]. LibSerial. https://github.com/crayzeewulf/libserial

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
