#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.transformations import *
import message_filters

from math import *
import numpy as np
from numpy.linalg import norm
import time
from utils import const as fc

class Drone:
    def __init__(self,NS = 'None'):
        self.NS = NS
        self.pose = None
        self.yaw = 0
        self.sp = None
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        self.current_state = State()
        self.prev_request = None
        self.prev_state = None
        self.state = None

        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Subscriber('/mavros/state', State, self.state_callback)

        if self.NS == 'Samwise': #Sim
            rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)
        # else:
        #     rospy.Subscriber('vicon/' + self.NS + '/' + self.NS,TransformStamped, self.vicon_callback)



    def state_callback(self, state):
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])

    def arm(self):
        print("arming")
        for i in range(self.hz):
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()
    
        # wait for FCU connection
        while not self.current_state.connected:
            print('Waiting for FCU connection...')
            self.rate.sleep()

        prev_request = rospy.get_time()
        prev_state = self.current_state
        while not rospy.is_shutdown():
            now = rospy.get_time()
            if self.current_state.mode != "OFFBOARD" and (now - prev_request > 2.):
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                prev_request = now 
            else:
                if not self.current_state.armed and (now - prev_request > 2.):
                   self.arming_client(True)
                   prev_request = now 

            # older versions of PX4 always return success==True, so better to check Status instead
            if prev_state.armed != self.current_state.armed:
                print("Vehicle armed: %r" % self.current_state.armed)

            if prev_state.mode != self.current_state.mode: 
                print("Current mode: %s" % self.current_state.mode)
            prev_state = self.current_state

            if self.current_state.armed:
                break
            # Update timestamp and publish sp 
            self.publish_setpoint([0,0,-1])
            self.rate.sleep()

    @staticmethod
    def get_setpoint(x, y, z, yaw=0):
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        q = quaternion_from_euler(0, 0, yaw)
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        return set_pose
    def publish_setpoint(self, sp, yaw=0):
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)

    def takeoff(self, height):
        print("Takeoff...")
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.02
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def hover(self, t_hold):
        print('Position holding...')
        t0 = time.time()
        hold_pose = self.pose
        while not rospy.is_shutdown():
            t = time.time()
            if t - t0 > t_hold and t_hold > 0: break
            # Update timestamp and publish sp 
            self.publish_setpoint(hold_pose)
            self.rate.sleep()

    # def kill_node(self):
    #     print("shutdown time!")

    def land(self):
        
        print("Landing...")
        self.sp = self.pose
        while self.sp[2] > - 1.0:
            self.sp[2] -= 0.05
            self.publish_setpoint(self.sp)
            self.rate.sleep()
        resp = self.landing_client(min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=0.0)

        if resp.success:
            rospy.loginfo("Landing command sent successfully!")

        else:
            rospy.logerr("Failed to send landing command: %s" % resp.result)
        
        #self.stop()
        
    # def stop(self):
    #     while self.current_state.armed or self.current_state.mode == "OFFBOARD":
    #         if self.current_state.armed:
    #             print(self.current_state)
    #             self.arming_client(False)
    #         if self.current_state.mode == "OFFBOARD":
    #             self.set_mode_client(base_mode=0, custom_mode="MANUAL")
    #             print(self.current_state)
    #         self.rate.sleep()

    @staticmethod
    def transform(pose):
        # transformation: x - forward, y - left, z - up (ENU - MoCap frame)
        pose_new = np.zeros(3)
        pose_new[0] = - pose[1]
        pose_new[1] = pose[0]
        pose_new[2] = pose[2]
        return pose_new
    def goTo(self, wp, mode='global', tol = None):
        if tol == None:
            tol = fc.DIST_TO_GOAL_TOL
        wp = self.transform(wp)
        if mode=='global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        if abs(goal[0]) > fc.X_BOUND:
            print("Waypoint is outside of X bounds...landing")
            self.land()
        elif abs(goal[1]) > fc.Y_BOUND:
            print("Waypoint is outside of Y bounds...landing")
            self.land()

        rospy.loginfo("Going to a waypoint...")
        self.sp = self.pose
        while norm(goal - self.pose) > tol:
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.03 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()
    # def vicon_callback(self, data):
    #     translations = data.transform.translation
    #     vicon_x = translations.x
    #     vicon_y = translations.y
    #     vicon_z = translations.z
    #     self.pose = np.array([vicon_x,vicon_y,vicon_z])
        