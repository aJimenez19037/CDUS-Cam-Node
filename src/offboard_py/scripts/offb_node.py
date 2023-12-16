#! /usr/bin/env python

import rospy
import math
from drone import Drone
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


DRONE_WIDTH = 1
DIST_TO_GOAL_TOL = .1

drone = None
obs_detected = False


def eucl_dist(x,y,z):
    pose = drone.pose
    eucl_dist = math.sqrt(pow(pose[0]-x,2) + pow(pose[1]-y,2) + pow(pose[2]-z,2))
    return eucl_dist
    

def cam_cb(msg):
    
    # Callback function of the subscriber.
    cXYZ = msg.data
    cX = cXYZ[0,8]
    cY = cXYZ[8,16]
    cZ = cXYZ[16,24]
    # CXYZ is 1x24 vector
    # x gives depth, y gives width, z gives height. 
    min_x = min(cX)
    min_y = min(cY)
    min_z = min(cZ)
    max_x = max(cX)
    max_y = max(cY)
    max_z = max(cZ)

    center = [(max_x-min_x)/2,(max_y-min_y)/2,(max_z-min_z)/2]

    drone_x, drone_y, drone_z = drone.pose()
    #change alt to alt of obj, 
    drone.takeoff(center[2])
    #Ensure that an obs has been detected
    while obs_detected == True:
        drone.goTo(drone_x, min_y-DRONE_WIDTH, center[2])
        drone.goTo(drone_x, min_y-DRONE_WIDTH, center[2])

    
def obs_found_cb(msg):
    obs_detected = msg.data

def main():
    rospy.init_node("offb_node_py")
    try:
        drone = Drone()
        # Your main code...
    except rospy.ROSInterruptException:
        pass
    # rospy.init_node("cam_node")
    # rospy.Subscriber("obstacle_flag", Bool, obs_found_cb)
    # rospy.Subscriber('obs_corners_data', Float32MultiArray, cam_cb)

    drone.arm()
    drone.takeoff(1)
    
    drone.rate.sleep()



if __name__ == "__main__":
    main()