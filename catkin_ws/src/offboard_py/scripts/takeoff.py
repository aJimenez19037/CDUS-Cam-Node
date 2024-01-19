#! /usr/bin/env python

import rospy
import math
from drone import Drone
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from utils import const as fc

def main():
    rospy.init_node("offb_node_py")
    try:
        drone = Drone()  
    except rospy.ROSInterruptException:
        pass
    drone.arm()
    drone.takeoff(0.75)
    drone.hover(5)
    drone.land()

    #drone.takeoff(0.75)
    # while not rospy.is_shutdown():
    #     pose = drone.pose
    #     drone.hover(1)
    #     drone.rate.sleep()
    #     drone.rate.sleep()

if __name__ == "__main__":
    main()