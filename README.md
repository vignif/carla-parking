# Carla parking with ROS
this package contains a ros node capable of simple rule based parking with an ego vehicle in the carla simulator v 0.9.7 and ROS melodic

### Example
this gif shows the implemented routine with a time scale x5
![](.figure/m1_comp.gif)


## Installation
<catkin_ws> = name of your catkin workspace

```cd ~/<catkin_ws>/src```

```git clone https://github.com/vignif/carla_parking.git```

```cd ..```

```catkin_make ```

## Environment for running

run carla with ```./CarlaUE4.sh``` as from https://carla.readthedocs.io/en/latest/start_quickstart/

run carla ros bridge with
```roslaunch carla_ros_bridge carla_ros_bridge.launch```

## Running

```source devel/setup.bash```

```rosrun carla_park park.py```

### Specs
park.py implements a basic parking policy for autonomous cars based on geometric information
This script spawns:
    -one ego vehicle in x=61.4, y=-7.62, z=0.05
    -two vehicle inside the parking location a side of the ego vehicle respectively in x=60.4, y=-10.62, z=0.05 and x=47.0, y=-10.62, z=0.05
    -a camera attached to the ego vehicle for future sensor based parking policy

all the vehicles are rotated of 180' in order to be correctly in line with the street

requirements:
running localhost environment of carla
running carla-ros-bridge

tested env:
-Ubuntu 18.04.3 LTS
-UnrealEngine 4.22
-ROS melodic
-carla 0.9.7


### License
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) available at [LICENSE](LICENSE)
