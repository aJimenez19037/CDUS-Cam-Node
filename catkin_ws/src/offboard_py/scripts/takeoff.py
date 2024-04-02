#! /usr/bin/env python

import rospy
import math
import numpy as np
from drone import Drone
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from utils import const as fc

def main():
    rospy.init_node("offb_node_py")
    try:
        drone = Drone(fc.NS)  
    except rospy.ROSInterruptException:
        pass
    drone.arm()
    drone.takeoff(1)
    print(drone.transform_point((1,0,0)))
    print(drone.pose)
    drone.turn(np.pi/2, mode = 'relative')
    print(drone.transform_point((1,0,0)))
    print(drone.pose)

    # drone.goTo([0,1,0], mode='relative')
    # drone.goTo([0,-1,0], mode='relative')
    # drone.goTo([1,0,0], mode='relative')
    # drone.goTo([-1,0,0], mode='relative')

    drone.land()
    while not rospy.is_shutdown():
        drone.rate.sleep()

    #drone.takeoff(0.75)
    # while not rospy.is_shutdown():
    #     pose = drone.pose
    #     drone.hover(1)
    #     drone.rate.sleep()
    #     drone.rate.sleep()

if __name__ == "__main__":
    main()
