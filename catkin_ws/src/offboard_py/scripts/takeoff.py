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
    drone.takeoff(0.5)
    drone.turn(0)
    drone.turn(np.pi/2)
    drone.turn(3*np.pi/2)
    drone.turn(0)
    drone.goTo([0,0.3,0], mode='relative')
    drone.goTo([0,-0.3,0], mode='relative')
    drone.goTo([0.3,0,0], mode='relative')
    drone.goTo([-0.3,0,0], mode='relative')

    drone.land()
    while not rospy.is_shutdown():
        drone.rate.sleep()

if __name__ == "__main__":
    main()
