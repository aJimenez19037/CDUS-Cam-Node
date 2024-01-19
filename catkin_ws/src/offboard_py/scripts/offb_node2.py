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

class Offb_Node:
        def __init__(self):
            obs_detected = False
            land_flag = False
            rospy.init_node('offb_node_py')
            prev = rospy.get_time
            try:
                self.drone = Drone()
                cam = rospy.get_param('~cam', default=True)

                if cam:
                    try:
                        # Wait for a message on the specified topic with a timeout
                        rospy.loginfo("Waiting for a message on topic /cam_node/pose_flag")
                        rospy.wait_for_message("/cam_node/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)
                        cam = False 

                    except rospy.ROSException:
                        rospy.logwarn("Timeout reached. No message received.")
                        rospy.signal_shutdown("Timeout reached. No message received.")
                        
                obs_found_sub = rospy.Subscriber('/cam_node/obstacle_flag', Bool, self.obs_found_cb)
                obs_corners_sub = rospy.Subscriber('/cam_node/obs_corners_data', Float32MultiArray, self.cam_cb, queue_size=10)
                self.drone.arm()
                self.drone.takeoff(0.75)
                self.drone.hover(1)
                while not rospy.is_shutdown():
                    if obs_detected == False:
                        self.drone.hover(1)
                    self.drone.rate.sleep()
            except rospy.ROSInterruptException:
                pass
        def obs_found_cb(self, msg):
            self.obs_detected = msg.data
            
        def cam_cb(self, msg):
                #data contains all coordinates of all 8 points. 
                print("cam_cbb")
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
                drone_x, drone_y, drone_z = self.drone.pose
                print("changing alt")
                self.drone.goTo([0,center[2],0],'relative')
                #  x = depth , y = width, z = height
                print("beginnign to avoid obs")
                self.drone.goTo([0, min_y-fc.DRONE_WIDTH, center[2]],'relative')
                print("moving past obs")
                self.drone.goTo([max_x+fc.DRONE_WIDTH, min_y-fc.DRONE_WIDTH, center[2]], 'relative')
                print("realigning")
                self.drone.goTo([max_x+fc.DRONE_WIDTH, 0, center[2]], 'relative')
                self.obs_detected = False




                

if __name__ == '__main__':
    node = Offb_Node()
    rospy.spin()