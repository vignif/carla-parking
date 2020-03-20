#!/usr/bin/env python

"""
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

"""

import rospy
from std_msgs.msg import String
import carla
import time
import random
import numpy as np
from carla_msgs.msg import CarlaWorldInfo


invert_steering_point = -7.8
parked_locations = [carla.Transform(carla.Location(x=60.4, y=-10.62, z=0.05), carla.Rotation(yaw=180)),
                    carla.Transform(carla.Location(x=47.0, y=-10.62, z=0.05), carla.Rotation(yaw=180))]

class CarlaParkVehicle(object):
    """
    class responsable of:
        -spawning 3 vehicles of which one ego
        -interact with ROS and carla server
        -destroy the created objects
        -execute the parking manoeuvre
    """
    def __init__(self):
        # construct object with server connection and ros node initiation

        rospy.init_node('park_vehicle', anonymous=True)
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)
        self.world = client.get_world()
        self.actor_list = []
        blueprint_library = self.world.get_blueprint_library()

        #create ego vehicle
        bp = random.choice(blueprint_library.filter('vehicle.tesla.*'))
        init_pos = carla.Transform(carla.Location(x=61.4, y=-7.62, z=0.05), carla.Rotation(yaw=180))
        self.vehicle = self.world.spawn_actor(bp, init_pos)
        self.actor_list.append(self.vehicle)

        #create 2 parked vehicles
        for pos in parked_locations:
            v = self.world.spawn_actor(bp, pos)
            self.actor_list.append(v)

        rospy.loginfo('created %s' % self.vehicle.type_id)


    def move_to_init_parking(self):
        """
        function to move the ego vehicle into 'start parking' position
        """
        while self.vehicle.get_location().x > 50:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.4, brake=0.0))
        rospy.loginfo("parking spot found!")
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(2)

    def park(self):
        """
        function enables the ego vehicle to enter the parking spot also actuating on steering wheels

        todo:
        -a step function is applied on the steering wheels it would be more realistic to actuate the steering
        wheels with an increasing(decreasing) function i.e steer += (-)0.1
        -above mentioned approach requires a more sofisticated approach i.e. sensor based
        -last manoeuvre of the parking procedure (go forward for completing the parking) is based only
        on temporal information, variability can be caused by server delays
        """
        while True:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.3, steer=0.6, brake=0.0, reverse=True))
            time.sleep(0.1)
            if self.vehicle.get_location().y < invert_steering_point:
                break
        while self.vehicle.get_location().y < invert_steering_point:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.2, steer=-0.6, brake=0.0, reverse=True))
            time.sleep(0.1)
            if abs(self.vehicle.get_transform().rotation.yaw) > 180 - 2:
                rospy.loginfo("in parking line")
                self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0, reverse=True))
                break
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(0.5)
        rospy.loginfo("go forward to complete the parking")
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0, brake=0.0, reverse=False))
        time.sleep(1.2)
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
        time.sleep(0.5)
        rospy.loginfo("parking complete")
        return

    def destroy(self):
        """
        destroy all the actors
        """
        print('destroying actors')
        for actor in self.actor_list:
            actor.destroy()
        print('done.')


    def run(self):
        """
        main loop
        """
        # wait for ros-bridge to set up CARLA world
        rospy.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            rospy.wait_for_message("/carla/world_info", CarlaWorldInfo, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr("Timeout while waiting for world info!")
            raise e
        rospy.loginfo("CARLA world available. Spawn ego vehicle...")

        self.move_to_init_parking()
        self.park()

        print("press CTRL+C to terminate the node")
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    ego_vehicle = CarlaParkVehicle()
    try:
        ego_vehicle.run()
    finally:
        if ego_vehicle is not None:
            ego_vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
