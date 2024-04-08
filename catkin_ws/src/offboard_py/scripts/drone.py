#!/usr/bin/env python

#Todo - Check transformation
#Todo - check global x,y vs relative x,y in vicon
# Add max velocity argument to goToVelocity
# Need to create const variables for angles and goTo command. And proportional const
# need to add tol to turn command


import rospy
import mavros
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, TwistStamped, Quaternion
from mavros_msgs.msg import State, AttitudeTarget 
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.broadcaster import TransformBroadcaster
from tf.listener import TransformListener
from tf.transformations import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import message_filters


from math import *
import numpy as np
from numpy.linalg import norm
import time
from utils import const as fc

class Drone(object):
    def __init__(self,NS = 'None'):
        """Constructor for drone object
            Args: 
                NS = Namespace used in ROS/Vicon
            Returns:
            None
        """
        self.NS = NS
        self.pose = None # x,y,z
        self.quaternion = None #x,y,z,w
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
        """ Updates drone object's state
            state message:

            header: 
                seq: 5
                stamp: 
                    secs: 5
                    nsecs: 320000000
                frame_id: ''
                connected: True
                armed: False
                guided: False
                manual_input: False
                mode: "OFFBOARD"
                system_status: 0
        """
        self.current_state = state

    def drone_pose_callback(self, pose_msg):
        """ Updates drone object's pose
            pose message:

            header: 
                seq: 80
                stamp: 
                    secs: 3
                    nsecs: 352000000
                frame_id: "map"
                pose: 
                position: 
                    x: -0.000699795258697
                    y: -0.0551643483341
                    z: 0.14720441401
                orientation: 
                    x: 0.00583348306994
                    y: 0.000859421985614
                    z: -0.698356772749
                    w: -0.715725628632
        """
        
        self.pose = np.array([ pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z ])
        self.quaternion = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w ])
        euler = euler_from_quaternion(self.quaternion)
        self.yaw = euler[2] 
        
    def arm(self):
        """Arm drone
        """
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
        """Create PoseStamped() method

            PoseStamped message

            header: 
                seq: 83
                stamp: 
                    secs: 3
                    nsecs: 460000000
                frame_id: "map"
                pose: 
                position: 
                    x: -0.000625429267529
                    y: -0.0605959072709
                    z: 0.127905875444
                orientation: 
                    x: 0.00581461972105
                    y: 0.000828222781004
                    z: -0.69836536873
                    w: -0.715717454119

            Args: 
                x, y, z = drone pose goal in x, y, z coordinates
                yaw_goal = drone yaw angle goal
            Returns:
                Pose message
        """
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
        """Publish setpoing
            Args: 
                sp = [x, y, z]
            Returns:
                None
        """
        setpoint = self.get_setpoint(sp[0], sp[1], sp[2])
        setpoint.header.stamp = rospy.Time.now()
        self.setpoint_publisher.publish(setpoint)

    def takeoff(self, height):
        """Publish desired height / go to desired height. Works similarly to goTo command
            Args: 
                height = desired height 
            Returns:
                None
        """
        print("Takeoff...")
        self.sp = self.pose
        while self.pose[2] < height:
            self.sp[2] += 0.02
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def hover(self, t_hold):
        """Publish hold pose for given duration t_hold
            Args: 
                t_hold = time to remain hovering 
            Returns:
                None
        """
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
        """Begin landing, slowly decreases altitude then calls landing client
            Args: 
                t_hold = time to remain hovering 
            Returns:
                None
        """
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
    
    def goTo(self, wp, mode='global', tol = None):
        """ Drone goes to given waypoint using positional commands.
            Args: 
                wp =  [x,y,z]
                mode = global or relative 
                tol = distance tolerance to be considered having reached wp (meters)
            Returns:
                None
            Notes:
                If using relative, errors accumulate
                Relative, deos not change x,y,z axis, simply (rel_wp + pose) instead of (global_wp)
                Overtakes any other command. 
                Drone will begin moving to waypoint until within the specified tolerance. 
                Will land if setpoint is outside of specified boundaries. 
                Boundary in const was determined by CDUS vicon cage and its blindspots.
                Does not allow for you to fully control velocity except for changing the proportional const (0.1)

        """
        if tol == None:
            tol = fc.DIST_TO_GOAL_TOL
        goal = wp
        if mode == 'global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        elif mode=='drone':
            #transform point to be in the drone reference frame
            rotation_matrix = np.array([[np.cos(self.yaw), -np.sin(self.yaw),0],
                                [np.sin(self.yaw), np.cos(self.yaw), 0],
                                [0, 0,1]])
            
            rotated_wp = np.matmul(rotation_matrix,wp) * wp
            goal = self.pose + rotated_wp
            
            rotated_wp = np.matmul(rotation_matrix,wp) * wp
            goal = self.pose + rotated_wp

        if abs(goal[0]) > fc.X_BOUND:
            print("Waypoint is outside of X bounds...landing")
            self.land()
        elif abs(goal[1]) > fc.Y_BOUND:
            print("Waypoint is outside of Y bounds...landing")
            self.land()

        rospy.loginfo("Going to a waypoint..." + str(goal))
        self.sp = self.pose
        # creates waypoint that is (.01 * error) closer to the goal. Increasing .01 leads to faster moving drone.  
        while norm(goal - self.pose) > tol:
            n = (goal - self.sp) / norm(goal - self.sp)
            self.sp += 0.01 * n
            self.publish_setpoint(self.sp)
            self.rate.sleep()

    def goToVelocity(self, wp, mode='global', tol = None):
        """ Drone goes to given waypoint using velocity commands

            Args: 
                wp =  [x,y,z]
                mode = global or relative 
                    [Note] if navigating using relative mode, errors do accumulate. Small tolerance may help reduce this issue
                tol = distance tolerance to be considered having reached wp (meters)
            Returns:
                None

            Notes:
                *** Similar to goTo command but is controlled using velocity commands instead of waypoints allowing us to better control velocity. In this case it is a max axis velocity set in const.

        """
        if tol == None:
            tol = fc.DIST_TO_GOAL_TOL
        if mode == 'global':
            goal = wp
        elif mode=='relative':
            goal = self.pose + wp
        elif mode=='drone':
            #transform point to be in the drone reference frame
            rotation_matrix = np.array([[np.cos(self.yaw), -np.sin(self.yaw),0],
                                [np.sin(self.yaw), np.cos(self.yaw), 0],
                                [0, 0,1]])
            
            rotated_wp = np.matmul(rotation_matrix,wp) * wp
            goal = self.pose + rotated_wp

        if abs(goal[0]) > fc.X_BOUND:
            print("Waypoint is outside of X bounds...landing")
            self.land()
        elif abs(goal[1]) > fc.Y_BOUND:
            print("Waypoint is outside of Y bounds...landing")
            self.land()
        
        rospy.loginfo("Going to a waypoint...")
        self.sp = self.pose
        #Position controller gain const
        k_p = 1  
        while norm(goal - self.pose) > tol:            
                        
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = rospy.Time.now()

            # velocity in x,y,z = gain const * error
            velocity_msg.twist.linear.x = k_p * (goal[0] - self.pose[0])
            velocity_msg.twist.linear.y = k_p * (goal[1] - self.pose[1])
            velocity_msg.twist.linear.z = k_p * (goal[2] - self.pose[2])

            # apply upper bound to velocity
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

            self.velocity_publisher.publish(velocity_msg)
            self.rate.sleep()

    def turn(self, yaw, mode = 'global'):
        """Turn drone to given yaw
            Args: 
                yaw = desired yaw
                mode = global or relative 
                    [Note] if navigating using relative mode, errors do accumulate. Small tolerance may help reduce this issue
            Returns:
                None

            Notes:
                Works similar to goTo command in that it makes small waypoints based (0.1 * error). Adjusting 0.1 will change how quickly it turns.
                Turns in shortest direction
                Normalized angles range from -180 to 180 degrees. 
                All angles are in rad
        """
        sp = self.pose
        yaw_sp = self.yaw

        if mode == 'global':
            goal_yaw = yaw
            normalized_goal_yaw = math.atan2(math.sin(goal_yaw), math.cos(goal_yaw))
            normalized_yaw_sp = math.atan2(math.sin(yaw_sp), math.cos(yaw_sp))
        elif mode == 'relative':
            goal_yaw = self.yaw + yaw
            normalized_goal_yaw = math.atan2(math.sin(goal_yaw), math.cos(goal_yaw))
            normalized_yaw_sp = math.atan2(math.sin(yaw_sp), math.cos(yaw_sp))

        print("Start turning")

        while abs(normalized_goal_yaw - normalized_yaw_sp) > fc.DEGREE_TOL:
            n = normalized_goal_yaw - normalized_yaw_sp
            #normalize diff to ensure drone turns shortest direction
            if n < -np.pi:
                n += 2 * np.pi
            elif n >= np.pi:
                n -= 2 * np.pi
            #create mini waypoints. yaw_sp = proportional const * error
            yaw_sp += 0.1 * n
            setpoint = self.get_setpoint(sp[0], sp[1], sp[2], yaw_sp)
            self.setpoint_publisher.publish(setpoint)
            normalized_yaw_sp = math.atan2(math.sin(yaw_sp), math.cos(yaw_sp))
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
        #while not currently avoiding an obstacle take in obstacle corner data 
        if  self.doing_obs_avoid == False:
            print("[offb_node.py] RECIEVED BB CORNER INFO")
            cXYZ = msg.data
            cX = cXYZ[:8] # first 8 values are the 8 corners x position relative to cam
            cY = cXYZ[8:16] # 8 corners y position relative to cam
            cZ = cXYZ[16:24] # 8 corners z position relative to cam
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
        # returns if drone is currently avoiding an obstacle
        return self.doing_obs_avoid
    
    def get_corners(self):
        # returns corners
        return ( self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z )
    
    def set_obstacle_avoid_status(self, boolean=False):
        # setter for if avoiding an obstacle or not
        self.doing_obs_avoid = boolean            
