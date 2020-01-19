# carla_parking
this package contains a ros node capable of simple rule based parking with an ego vehicle in the carla simulator v 0.9.7 and ROS melodic

## installation
<catkin_ws> = name of your catkin workspace

cd ~/<catkin_ws>/src

git clone https://github.com/vignif/carla_parking.git

cd ..

catkin_make


## running

source devel/setup.bash

rosrun carla_park park.py
