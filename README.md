# Drone
Drone projects including PCB design, Frame, and Software

## Design

battery size

motors brushless or brushed

drone physical size


## PCB
single-board design with accelerometers, motor speed controllers, battery management, and LEDs.

NodeMCU socket for controller with integrated wifi.

![image](https://user-images.githubusercontent.com/13946498/227780319-d5eada6b-10d0-42fe-b5f7-ef49a47baa42.png)

![image](https://user-images.githubusercontent.com/13946498/228086294-8e9f67d7-536d-4cf0-87ef-0eea5961e807.png)


## Frame




## Software
Arduino-based C++ program with ROS/serial interface. Wifi controlled.

### ROS
from PC run:

```
roscore
```

in another terminal from PC run:

```
roslaunch rosserial_server socket.launch

```

This will create a rosserial server to interface with the nodeMCU controller.

In another terminal launch:

```
rosrun imu_complementary_filter complementary_filter_node
```

## User Interface

Commands available:

Up, Down, Left, Right, Forward, Backward, Rotate Left, Rotate Right

If not command is given, the drone will hold in place.
