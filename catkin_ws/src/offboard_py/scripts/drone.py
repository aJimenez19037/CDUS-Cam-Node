#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped, Quaternion
from mavros_msgs.msg import State, AttitudeTarget 
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.transformations import *
from tf.transformations import quaternion_from_euler
import message_filters


from math import *
import numpy as np
from numpy.linalg import norm
import time
from utils import const as fc

class Drone(object):
    def __init__(self,NS = 'None'):
        self.NS = NS
        self.pose = None
        self.quaternion = None
        self.yaw = 0
        self.sp = None
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
        self.waypoints = []
        self.land_flag = False

        self.current_state = State()
        self.prev_request = None
        self.prev_state = None
        self.state = None

        self.setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.landing_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pub_attitude = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        rospy.Subscriber('/mavros/state', State, self.state_callback)

        if self.NS == None:
            self.NS = fc.NS
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

    def state_callback(self, state):
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])
        self.quaternion = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w ])
        euler = euler_from_quaternion(self.quaternion)
        self.yaw = euler[2] 
        
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

    def get_setpoint(self, x, y, z, yaw_goal=None):
        set_pose = PoseStamped()
        set_pose.pose.position.x = x
        set_pose.pose.position.y = y
        set_pose.pose.position.z = z
        
        if yaw_goal is not None:
            q = quaternion_from_euler(0, 0, yaw_goal)
        else:
            q = quaternion_from_euler(0, 0, self.yaw)
        
        set_pose.pose.orientation.x = q[0]
        set_pose.pose.orientation.y = q[1]
        set_pose.pose.orientation.z = q[2]
        set_pose.pose.orientation.w = q[3]
        
        return set_pose
    
    def publish_setpoint(self, sp):
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2])
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
            self.land_flag = True

        else:
            rospy.logerr("Failed to send landing command: %s" % resp.result)

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
            self.sp += 0.01 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()
    # def vicon_callback(self, data):
    #     translations = data.transform.translation
    #     vicon_x = translations.x
    #     vicon_y = translations.y
    #     vicon_z = translations.z
    #     self.pose = np.array([vicon_x,vicon_y,vicon_z])

    def goToVelocity(self, wp, mode='global', tol = None):
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
        #Position controller gains (adjust as needed)
        k_p = 1  # Proportional gain
        while norm(goal - self.pose) > tol:            
            # Calculate position error
                        
            # Calculate velocity command using position controller
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = rospy.Time.now()
            velocity_msg.twist.linear.x = k_p * (goal[0] - self.pose[0])
            velocity_msg.twist.linear.y = k_p * (goal[1] - self.pose[1])
            velocity_msg.twist.linear.z = k_p * (goal[2] - self.pose[2])
            if abs(velocity_msg.twist.linear.x) > fc.MAX_AXIS_VELOCITY:
                if velocity_msg.twist.linear.x < 0:
                    velocity_msg.twist.linear.x = -fc.MAX_AXIS_VELOCITY
                else: 
                    velocity_msg.twist.linear.x = fc.MAX_AXIS_VELOCITY
            if abs(velocity_msg.twist.linear.y) > fc.MAX_AXIS_VELOCITY:
                if velocity_msg.twist.linear.y < 0:
                    velocity_msg.twist.linear.y = -fc.MAX_AXIS_VELOCITY
                else: 
                    velocity_msg.twist.linear.y = fc.MAX_AXIS_VELOCITY
            if abs(velocity_msg.twist.linear.z) > fc.MAX_AXIS_VELOCITY:
                if velocity_msg.twist.linear.z < 0:
                    velocity_msg.twist.linear.z = -fc.MAX_AXIS_VELOCITY
                else: 
                    velocity_msg.twist.linear.z = fc.MAX_AXIS_VELOCITY
            # Publish the velocity message
            self.velocity_publisher.publish(velocity_msg)

            # Wait for a short time to allow the message to be received
            self.rate.sleep()

    def turn(self, yaw, mode = 'global'):
        print("Turning")
        sp = self.pose
        if mode == 'global':
            while abs(self.yaw - yaw) > fc.DEGREE_TOL:
                setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw)
                setpoint.header.stamp = rospy.Time.now()
                self.setpoint_publisher.publish(setpoint)
                self.rate.sleep()

        elif mode == 'relative':
            goal_yaw = self.yaw + yaw
            while abs(self.yaw - goal_yaw) > fc.DEGREE_TOL:
                setpoint = self.get_setpoint(sp[0], sp[1], sp[2], goal_yaw)
                setpoint.header.stamp = rospy.Time.now()
                self.setpoint_publisher.publish(setpoint)
                self.rate.sleep()

class Drone_Avoidance(Drone):
    # created this class to avoid having to copy over the cam_node object to different scripts.
    def __init__(self,NS = None):
        super(Drone_Avoidance, self).__init__(NS=NS)
        # inherit all properties and methods from the Drone parent class
        self.doing_obs_avoid = False
        self.center = None
        self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z = None, None, None, None, None, None
    
    def cam_cb(self,msg):
        #data contains all coordinates of all 8 points. 
        if  self.doing_obs_avoid == False:
            print("[offb_node.py] RECIEVED BB CORNER INFO")
            cXYZ = msg.data
            cX = cXYZ[:8]
            cY = cXYZ[8:16]
            cZ = cXYZ[16:24]
            # CXYZ is 1x24 vector
            # x gives depth, y gives width, z gives height. 
            self.min_x = min(cX)
            self.min_y = min(cY)
            self.min_z = min(cZ)
            self.max_x = max(cX)
            self.max_y = max(cY)
            self.max_z = max(cZ)
            self.center = np.array([(self.max_x-self.min_x)/2,(self.max_y-self.min_y)/2,(self.max_z-self.min_z)/2])
            self.doing_obs_avoid = True

    def get_is_avoiding(self):
        return self.doing_obs_avoid
    
    def get_corners(self):
        return ( self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z )
    
    def set_obstacle_avoid_status(self, boolean=False):
        self.doing_obs_avoid = boolean








        


 
  


        
            
