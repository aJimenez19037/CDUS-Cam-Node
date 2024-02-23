#! /usr/bin/env python
# Need to make cam node into into a class

import rospy
import math
from drone import Drone
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from utils import const as fc
global land_flag
global pose 
drone = None
pose = [0,0,0]

def cam_cb(msg):
    #data contains all coordinates of all 8 points. 
    print("cam callback called")
    rospy.loginfo("cam callabck called")
    cXYZ = msg.data
    cX = cXYZ[:8]
    cY = cXYZ[8:16]
    cZ = cXYZ[16:24]
    # CXYZ is 1x24 vector
    # x gives depth, y gives width, z gives height. 
    min_x = min(cX)
    min_y = min(cY)
    min_z = min(cZ)
    max_x = max(cX)
    max_y = max(cY)
    max_z = max(cZ)

    center = [(max_x-min_x)/2,(max_y-min_y)/2,(max_z-min_z)/2]

    
    drone_x, drone_y, drone_z = pose
    #change alt to alt of obj, 
    print("height change")
    print(drone.NS)
    drone.goTo([0, center[2]+0.1, 0],'relative')
    #Ensure that an obs has been detected
    # while obs_detected == True:
    #  x = depth , y = width, z = height
    print("moving left")
    drone.goTo([0, min_y-fc.DRONE_WIDTH, 0],'relative')
    print("moving past")
    drone.goTo([max_x+fc.DRONE_WIDTH, 0, 0], 'relative')
    print("moving right") 
    drone.goTo([0, -(min_y-fc.DRONE_WIDTH), 0], 'relative')
    print("done with obs avoidance")
    drone.land()
    land_flag = True
    
def obs_found_cb(msg):
    global obs_detected
    global obs_detected_once
    obs_detected = msg.data
    if obs_detected == True:
        obs_detected_once = True
        print("Obstacle detected")

def main():
    global obs_detected
    global obs_detected_once
    global drone
    obs_detected = False
    obs_detected_once = False

    rospy.init_node("offb_node_py")
    obs_found_sub = rospy.Subscriber('/cam_node/obstacle_flag', Bool, obs_found_cb)
    cam = rospy.get_param('~cam', default=True)
    Namespace = rospy.get_param('~NS', default="Samwise")
    try:
        drone = Drone(Namespace)
        print(drone.NS)
    except rospy.ROSInterruptException:
        pass
   
    if cam:
        try:
            # Wait for a message on the specified topic with a timeout
            rospy.loginfo("Waiting for a message on topic /camera/pose_flag")
            rospy.wait_for_message("/cam_node/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)

        except rospy.ROSException:
            rospy.logwarn("Timeout reached. No message received.")
            rospy.signal_shutdown("Timeout reached. No message received.")

    obs_corners_sub = rospy.Subscriber('/cam_node/obs_corners_data', Float32MultiArray, cam_cb, queue_size=10)
    land_flag = False
    print(drone.NS)
    drone.arm()
    drone.takeoff(1)
    drone.hover(2)
    while not rospy.is_shutdown():
        while obs_detected == False and obs_detected_once == False:
            print(obs_detected)
            print(obs_detected_once)
            drone.hover(5)
        drone.rate.sleep()


if __name__ == "__main__":
    main()