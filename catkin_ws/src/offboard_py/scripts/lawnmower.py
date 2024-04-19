#! /usr/bin/env python
# Might be better to use previous wp instead of current pose. To prevent deviation
import rospy
import numpy as np
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Float32MultiArray, Bool
from drone import Drone_Avoidance
from utils import const as fc

def lawnmower_global(pose,x_dist,y_dist):
    #direction does not change based on orientation of drone
    print(type(pose))
    wp1 = [-x_dist,y_dist,1]
    wp2 = [-x_dist,0,1]
    wp3 = [x_dist,0,1]
    wp4 = [x_dist,-y_dist,1]
    lawnmower_wp = [wp1,[np.pi/2],wp2,[np.pi], 
                    wp3,[np.pi/2],wp4,[0]]
    print("WP:" + str(lawnmower_wp))
    return lawnmower_wp

def main():
    rospy.init_node("offb_node_py")
    cam = rospy.get_param('cam')
    Namespace = rospy.get_param('~NS', default="Samwise")
    #create drone obj
    try:
        drone = Drone_Avoidance(Namespace)
        print(drone.NS)
    except rospy.ROSInterruptException:
        pass
    # if cam node is running wait for node to start
    if cam:
        try:
            # Wait for a message on the specified topic with a timeout
            rospy.loginfo("Waiting for a message on topic /camera/pose_flag")
            rospy.wait_for_message("/cam_node/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)

        except rospy.ROSException:
            rospy.logwarn("Timeout reached. No message received.")
            rospy.signal_shutdown("Timeout reached. No message received.")
    # subscribe to obs_corners_sub
    obs_corners_sub = rospy.Subscriber('/cam_node/obs_corners_data', Float32MultiArray, drone.cam_cb, queue_size=10)
    drone.arm()
    drone.takeoff(0.6)
    drone.goToVelocity([2,1.6,0.6], 'global')
    drone.turn(-np.pi, "global") # turn to -x
    #generate wp assuming to be starting at 1.5,1.5,1 
    wp1 = [-0.9,1.6,0.6] # move -x aka closer to us
    wp2 = [-0.9,0,0.6] # move toward center aka right
    wp3 = [0.2,0,0.6]
    wp4 = [2,0,0.6] # move past sven 
    wp5 = [2,-1.5,1]# move farther right
    wp6 = [-0.9,-1.5,1] # mover towards us
    drone.waypoints = [wp1,[3*np.pi/2],wp2,[0], 
                    wp3,[0],wp4,[3*np.pi/2],wp5,[-np.pi], wp6]

    while not rospy.is_shutdown():
        print("[offb_node.py] Lawmower Patter Start")
        while drone.land_flag == False and drone.doing_obs_avoid == False and len(drone.waypoints) != 0:
            print("Remaining WPs = ", str(len(drone.waypoints)))
            if len(drone.waypoints[0]) == 1:# turn command: yaw
                drone.turn(drone.waypoints[0][0],'global')
                drone.waypoints.pop(0)

            elif len(drone.waypoints[0]) == 3:# go to position: x,y,z
                drone.goToVelocity(drone.waypoints[0],'global') 
                drone.waypoints.pop(0)

            else:
                print("[ERROR] Waypoint is not correct shape")
                drone.land()

            drone.rate.sleep()
        if len(drone.waypoints) == 0:
            drone.land()
        while drone.doing_obs_avoid == True:
            min_x, min_y, min_z, max_x, max_y, max_z = drone.get_corners()
            
            print("[offb_node.py] MOVING Right")
            drone.goTo([0, min_y-fc.DRONE_WIDTH, 0],'relative')

            print("[offb_node.py] MOVING PAST")
            drone.goTo([max_x+fc.DRONE_WIDTH, 0, 0], 'relative')

            print("[offb_node.py] MOVING RIGHT") 
            drone.goTo([0, -(min_y-fc.DRONE_WIDTH), 0], 'relative')

            print("[offb_node.py] FINISHED ... LANDING")
            drone.set_obstacle_avoid_status(False)
        
        drone.rate.sleep()


if __name__ == "__main__":
    main()
