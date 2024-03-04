#! /usr/bin/env python
# Need to make cam node into into a class
import rospy
import numpy as np
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Float32MultiArray, Bool
from drone import Drone_Avoidance
from utils import const as fc

def lawnmower_global(pose,x_dist,y_dist):
    #direction does not change based on orientation of drone
    print(type(pose))
    wp1 = np.add(pose,[0, -y_dist, 0])
    wp2 = np.add(wp1,[x_dist,0,0])
    wp3 = np.add(wp2,[0, y_dist, 0])
    wp4 = np.add(wp3,[x_dist,0,0])
    lawnmower_wp = [wp1,[np.pi/2],wp2,[-np.pi], 
                    wp3,[np.pi/2],wp4,[0]]
    return lawnmower_wp



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
            rospy.wait_for_message("/cam_node/pose_flag", Bool, timeout=fc.CAM_TIMEOUT)

        except rospy.ROSException:
            rospy.logwarn("Timeout reached. No message received.")
            rospy.signal_shutdown("Timeout reached. No message received.")

    obs_corners_sub = rospy.Subscriber('/cam_node/obs_corners_data', Float32MultiArray, drone.cam_cb, queue_size=10)
    drone.arm()
    print(drone.pose)
    drone.takeoff(0.5)
    drone.waypoints = lawnmower_global(drone.pose,1.5,1.5)#want to make the points at take off height
    drone.hover(0.5)

    while not rospy.is_shutdown():
        while drone.land_flag == False and drone.doing_obs_avoid == False and len(drone.waypoints) != 0:
            print("[offb_node.py] Lawmower Patter Start")
            if len(drone.waypoints[0]) == 1:# turn command: yaw
                drone.turn(drone.waypoints[0][0],'global')
                drone.waypoints.pop(0)

            elif len(drone.waypoints[0]) == 3:# go to position: x,y,z
                drone.goTo(drone.waypoints[0],'global') # global 
                drone.waypoints.pop(0)
            else:
                print("[ERROR] Waypoint is not correct shape")
                drone.land()
            drone.rate.sleep()
        if len(drone.waypoints) == 0:
            drone.land()
        while drone.doing_obs_avoid == True:
            min_x, min_y, min_z, max_x, max_y, max_z = drone.get_corners()
            
            print("[offb_node.py] MOVING LEFT")
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