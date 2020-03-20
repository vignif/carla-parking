# Carla parking with ROS
Carla is an open source simulator for autonomous driving research available at http://carla.org/

This package contains an implemantation of a **rule based parking motion**. It is an open-loop approach for the task, given that no information are retrieved from sensors and crashes can occur.

The example spawns 2 cars in predefined positions (leaving enough space between them) and the ego vehicle is blindly controlled to fit the parking spot.

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

tested env:
1. Ubuntu 18.04.3 LTS
2. UnrealEngine 4.22
3. ROS melodic
4. carla 0.9.7


### License
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) available at [LICENSE](LICENSE)
