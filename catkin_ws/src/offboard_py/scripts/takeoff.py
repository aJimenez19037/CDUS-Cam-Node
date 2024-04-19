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
    drone.takeoff(0.7)
    drone.turn(0)
    droene.goTo([2,1.5,0.7], mode="global")
    drone.land()
    while not rospy.is_shutdown():
        drone.rate.sleep()

if __name__ == "__main__":
    main()
