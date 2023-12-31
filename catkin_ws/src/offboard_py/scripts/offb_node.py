#! /usr/bin/env python

import rospy
import math
from drone import Drone
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from utils import const as fc

drone = None
obs_detected = False
land_flag = False


def eucl_dist(x,y,z):
    pose = drone.pose
    eucl_dist = math.sqrt(pow(pose[0]-x,2) + pow(pose[1]-y,2) + pow(pose[2]-z,2))
    return eucl_dist
    

def cam_cb(msg):
    
    #data contains all coordinates of all 8 points. 
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
        #  x = depth , y = width, z = height
        drone.goTo([drone_x, min_y-fc.DRONE_WIDTH, center[2]],'relative')
        drone.goTo([max_x, min_y-fc.DRONE_WIDTH, center[2]], 'relative')


    
def obs_found_cb(msg):
    obs_detected = msg.data
    if obs_detected == True:
        drone.hover(5)
        rospy.loginfo("Obstacle detected")

def main():
    rospy.init_node("offb_node_py")
    obs_found_sub = rospy.Subscriber('/obstacle_flag', Bool, obs_found_cb)
    obs_corners_sub = rospy.Subscriber('/obs_corners_data',Float32MultiArray, cam_cb )
    try:
        drone = Drone()
        # Your main code...
    except rospy.ROSInterruptException:
        pass
    # rospy.Subscriber("obstacle_flag", Bool, obs_found_cb)
    # rospy.Subscriber('obs_corners_data', Float32MultiArray, cam_cb)

    # add a wait for camera 
    try:
        # Wait for a message on the specified topic with a timeout
        rospy.loginfo("Waiting for a message on topic /obstacle flag")
        rospy.wait_for_message("/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)
    except rospy.ROSException:
        rospy.logwarn("Timeout reached. No message received.")
        rospy.signal_shutdown("Timeout reached. No message received.")


    drone.arm()
    drone.takeoff(0.75)
    drone.hover(5.0)

    while not rospy.is_shutdown():
        while obs_detected == False and land_flag == False:
            drone.hover(1)
            drone.rate.sleep()
        if land_flag == True:
            drone.land()



if __name__ == "__main__":
    main()