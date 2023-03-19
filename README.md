# Drone
Drone projects including PCB design, Frame, and Software

## Design

battery size

motors brushless or brushed

drone physical size


## PCB
single-board design with accelerometers, motor speed controllers, battery management, and LEDs.

NodeMCU socket for controller with integrated wifi.


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
roslaunch rosserial_server socket.launch port:=/dev/ttyUSB0
```

This will create a rosserial server to interface with the nodeMCU controller.
