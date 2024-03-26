#! /usr/bin/env python
# Need to make cam node into into a class

import rospy
import math
from drone import Drone_Avoidance
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from utils import const as fc


def main():
    rospy.init_node("offb_node_py")
    cam = rospy.get_param('~cam', default=True)
    Namespace = rospy.get_param('~NS', default="Samwise")
    try:
        drone = Drone_Avoidance(Namespace)
        print(drone.NS)
    except rospy.ROSInterruptException:
        pass
   
    if cam:
        try:
            # Wait for a message on the specified topic with a timeout
            rospy.loginfo("Waiting for a message on topic /camera/pose_flag")
            print("Waiting for a message on topic /camera/pose_flag")
            rospy.wait_for_message("/cam_node/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)

        except rospy.ROSException:
            rospy.logwarn("Timeout reached. No message received.")
            rospy.signal_shutdown("Timeout reached. No message received.")

    obs_corners_sub = rospy.Subscriber('/cam_node/obs_corners_data', Float32MultiArray, drone.cam_cb, queue_size=10)
    
    drone.arm()
    drone.takeoff(0.8)
    drone.hover(1)
    while not rospy.is_shutdown():
        while drone.land_flag == False and drone.doing_obs_avoid == False:
            print("[offb_node.py] OBJECT NOT DETECTED ... RETRYING")
            drone.turn(0)
            drone.hover(0.5)

        while drone.doing_obs_avoid == True:
            min_x, min_y, min_z, max_x, max_y, max_z = drone.get_corners()
            
            print("[offb_node.py] MOVING LEFT")
            drone.goTo([0, min_y-fc.DRONE_WIDTH, 0],'relative')
            drone.turn(0)

            #print("[offb_node.py] MOVING PAST")
            #drone.goTo([max_x+fc.DRONE_WIDTH, 0, 0], 'relative')
            #drone.turn(0)

            #print("[offb_node.py] MOVING RIGHT") 
            #drone.goTo([0, -(min_y-fc.DRONE_WIDTH), 0], 'relative')
            #drone.turn(0)

            print("[offb_node.py] FINISHED ... LANDING")
            drone.land() 
            drone.set_obstacle_avoid_status(False)
        
        drone.rate.sleep()



if __name__ == "__main__":
    main()
