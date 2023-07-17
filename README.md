# Quadcopter with ROS/Wifi Flight Controller
This project is for a quadcopter using a WiFi controlled flight controller that you can control from your phone. The flight controller also interfaces with Robotic Operating System (ROS) to support live debugging and data logging with open-sourced industry tools.

<img src="https://github.com/estods3/Drone/blob/main/phone_remote_controller.png" alt="drawing" height="200"/><img src="https://github.com/estods3/Drone/blob/main/ros_live_visualization.png" alt="drawing" height="200"/>

This repo provides the design files for the custom flight controller hardware and software as well as a parts list for building a working quadcopter with the custom flight controller.

## Flight Controller
The flight controller is a single-board design with accelerometers, microcontroller, wifi, battery monitoring, ESC connector, and status LEDs.

| Top | Bottom |
|---|---|
| <img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_top.png" alt="drawing" width="400"/> | <img src="https://github.com/estods3/Drone/blob/main/HW/pcb/drone_controller_bottom.png" alt="drawing" width="400"/> |

Gerber files used to order this PCB are included in the latest release, [here](https://github.com/estods3/Drone/releases/tag/V1.0)

Parts list to assembly final board can be found in the BOM, [here](https://github.com/estods3/Drone/blob/main/HW/BOM.md)

### Hardware Features:
1. 2-Layer design with top and bottom ground plane.
2. Antenna clearance complies with ESP32 specification to help prevent interference to WiFi Signal.
3. IMU orientation aligned with drone frame. Design supports both ISO-20600 and MPU2060 IMU components.
<img src="https://user-images.githubusercontent.com/13946498/227780319-d5eada6b-10d0-42fe-b5f7-ef49a47baa42.png" alt="drawing" width="400"/>  <img src="https://user-images.githubusercontent.com/13946498/228086294-8e9f67d7-536d-4cf0-87ef-0eea5961e807.png" alt="drawing" width="400"/>
4. Standard 30.5mm by 30.5mm flight controller mounting pattern
5. Eectrical interface using JST-SH 8 Pin connector (compatible with SpeedyBee BL32 50A V3 ESC for stacked ESC/Flight Controller layout).
6. Single Supply (3S LiPo) powering both ESC as well as flight controller (Powered through JST-SH Connector).
7. Status LEDs to quickly see if Wifi is connected, if software is recieving commands from remote controller, etc.
8. Built-in ESC Firmware debug interface with BLHeli Suite (Coming Soon).

### Software Features:
1. Arduino-based C++.
2. Wifi controlled using a phone with browser, provides battery life status as well as controls for Up, Down, Left, Right, etc.
3. Robotic Operating System (ROS) interface provides live feed of IMU data, flight controller status, user commands, and ESC throttle to each motor. (must have variable USE_ROS = True).
4. PID-based flight stability controller (Coming Soon).
5. "Testing Mode" allows user to test throttle of each motor, calibrate PID controller without reflashing flight controller (Coming Soon).

#### IMU Calibration:
Run script to compute IMU calibration:

```
Reading sensors for first time...
Results of measurements a/g:	0.23	0.53	12.09	-0.03	-0.02	0.01

Calculating offsets...
Results of measurements a/g:	-0.00	-0.00	9.81	0.00	0.00	0.00
...

Calibration successful!
Results of measurements a/g:	-0.01	0.00	9.81	0.00	0.00	-0.00

FINISHED!

Sensor readings with offsets:	-0.01	0.00	9.81	0.00	0.00	-0.00
Your offsets:	-0.23	-0.53	-2.29	0.03	0.02	-0.01
```

#### ROS Interface:
Create a ROS interface where you can view and log data from your quadcopter live.

From a PC with ROS installed, run:

```
roscore
```

In another terminal from PC run:

```
roslaunch rosserial_server socket.launch

```

This will create a rosserial server to interface with the nodeMCU controller.

In another terminal launch:

```
rosrun imu_complementary_filter complementary_filter_node
```

In another terminal launch:
```
rqt
```

to see the following visualization of your drone's data!

<img src="https://github.com/estods3/Drone/blob/main/ros_live_visualization.png" alt="drawing" width="1000"/>

#### User Interface/Remote Control
The UI for remote control is deployed over WiFi to a static IP address accessible to a phone logged on to the Flight Controller's hosted WiFi network.

<img src="https://github.com/estods3/Drone/blob/main/phone_remote_controller.png" alt="drawing" width="1000"/>

Commands available:

Up, Down, Left, Right, Forward, Backward, Rotate Left, Rotate Right

If not command is given, the drone will hold in place.

## Prototype
This flight controller was tested with a 5 inch FPV Frame, although other frames could be used with tuning of the flight controller.

#### Test Quadcopter Specs:

Physical Size: ~450 grams, 5 inch FPV

Frame: 

Battery Size: 3S Lipo

Motors: brushless DC

ESC: [Speedybee 4-in-1 ESC](https://www.speedybee.com/speedybee-f7-v3-bl32-50a-4-in-1-esc/)

For full BOM, click [here](https://github.com/estods3/Drone/blob/main/HW/BOM.md)

#### ESC Firmware with BLHeli:
Using Arduino Nano as an interface (4wArduino_Nano__16_MULTIv20005.hex) with ESC plugged into pin D4. The opposite corner motors were set to clockwise and counter-clockwise, respectively. NOTE: this can be done in BLHeli software to correct the direction of the motor if the motor was wired incorrectly to the ESC. Brushless DC motors have 3 wires A, B, and C and the order determines the direction.

<img src="https://github.com/estods3/Drone/blob/main/BLHeliSuite32xlESC%20Setup.png" alt="drawing" width="1000"/>

## Resources:
PID Controller: https://github.com/0xekez/arduino-drone-pid/blob/master/pid_v2_in_arduino/pid_v2_in_arduino.ino
