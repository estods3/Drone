# Drone
Drone projects including PCB design, Frame, and Software

## Design

battery size

motors brushless or brushed

drone physical size


## PCB



## Frame




## Software


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
