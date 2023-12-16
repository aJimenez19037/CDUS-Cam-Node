#! /usr/bin/env python

import rospy
import mavros
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

class Drone:
    def __init__(self):
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        
        self.current_state = None
        self.pose = None
        self.yaw = 0
        self.hz = 20
        self.rate = rospy.Rate(self.hz)


    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z ]

    def arm(self):

        for i in range(100):
            self.publish_pose([0,0,-1])
            self.rate.sleep()
        # Wait for Flight Controller connection

        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):

            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                    if(self.arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                    last_req = rospy.Time.now()
            if self.current_state.armed:
                # Update timestamp and publish sp 
                break
        self.publish_pose([0,0,-1])
        self.rate.sleep()
    
    def publish_pose(self, position,yaw = np.pi/2):
        new_pose = PoseStamped()
        new_pose.pose.position.x = position[0]
        new_pose.pose.position.y = position[1]
        new_pose.pose.position.z = position[2]
        self.local_pos_pub.publish(new_pose)

    def takeoff(self, alt):
        print("Takeoff...")
        position = self.pose
        while self.pose[2] < alt:
            position[2] += 0.1
            self.publish_pose(position)
            self.rate.sleep()


        



    