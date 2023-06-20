# Drone
This project is for a NodeMCU (ESP32) based DIY quadcopter that you can control from your phone over WiFi. This repo provides the design files for the custom flight controller hardware and software as well as the frame.

## Features

Physical Size: TBD

Battery Size: 3S

Motors: brushless

ESC: [Speedybee 4-in-1 ESC](https://www.speedybee.com/speedybee-f7-v3-bl32-50a-4-in-1-esc/)

For full BOM, click [here](https://github.com/estods3/Drone/blob/main/HW/BOM.md)

## PCB
Single-board design with accelerometers, battery monitoring, and status LEDs.

<img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_top.png" alt="drawing" width="400"/><img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_bottom.png" alt="drawing" width="400"/>

### PCB Features:
1. 2-Layer design with top and bottom ground plane.
2. Antenna clearance complies with ESP32 specification to help prevent interference to WiFi Signal.
3. IMU orientation aligned with drone frame. Design supports both ISO-20600 and MPU2060 IMU components.
<img src="https://user-images.githubusercontent.com/13946498/227780319-d5eada6b-10d0-42fe-b5f7-ef49a47baa42.png" alt="drawing" width="400"/>  <img src="https://user-images.githubusercontent.com/13946498/228086294-8e9f67d7-536d-4cf0-87ef-0eea5961e807.png" alt="drawing" width="400"/>
4. ESC mounting holes and JST-SH 8 Pin connector layout to allow for SpeedyBee BL32 50A V3 ESC to mount directly to PCB (Stacked ESC/Flight Controller Design).
5. Single Supply (3S LiPo) powering both 4-in-1 ESC as well as PCB.
6. Status LEDs to quickly see if Wifi is connected, if software is recieving commands from remote controller, etc.
   
## Frame
TBD

## Software
In-house flight controller based on NodeMCU microcontroller compatible Arduino IDE.

### Software Features
1. Arduino-based C++.
2. Wifi controlled using a phone with browser, provides battery life status as well as controls GUI.
3. ROS serial interface provides live feed of IMU data, flight controller status, user commands, and ESC throttle to each motor. (must have variable ros_is_used = True).

### ESC Firmware with BLHeli
Using Arduino Nano as an interface (4wArduino_Nano__16_MULTIv20005.hex) with ESC plugged into pin D4.

<img src="https://github.com/estods3/Drone/blob/main/BLHeliSuite32xlESC%20Setup.png" alt="drawing" width="400"/>


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
