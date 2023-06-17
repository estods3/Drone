# Drone
Drone projects including PCB design, Frame, and Software

## Design

Battery Size: 3S

Motors: brushless

ESC: Speedybee 4-in-1 ESC

drone physical size


## PCB
single-board design with accelerometers, motor speed controllers, battery management, and LEDs.
<img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_top.png" alt="drawing" width="400"/>
<img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_bottom.png" alt="drawing" width="400"/>

NodeMCU socket for controller with integrated wifi.

### PCB Features:
Antenna clearance to prevent interference.

IMU orientation aligned with drone frame.

<img src="https://user-images.githubusercontent.com/13946498/227780319-d5eada6b-10d0-42fe-b5f7-ef49a47baa42.png" alt="drawing" width="400"/><img src="https://user-images.githubusercontent.com/13946498/228086294-8e9f67d7-536d-4cf0-87ef-0eea5961e807.png" alt="drawing" width="400"/>


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
