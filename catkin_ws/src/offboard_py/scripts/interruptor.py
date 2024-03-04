#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def publish_flag():
    # This node was created to interrupt a flight path, replicating the way the cam_node would interrupt a flight path. 

    rospy.init_node('interruptor')
    
    pub = rospy.Publisher('interrupt', Bool, queue_size=10)
    
    # Set the rate at which to publish (in Hz)
    rate = rospy.Rate(0.1)  # Publish every 10 seconds (adjust as needed)
    
    # Wait for some initial time (adjust as needed)
    rospy.sleep(25)  # Wait for 10 seconds before publishing the flag
    print("Published true flag")
    pub.publish(True)  # Set flag to True

    while not rospy.is_shutdown():
        # Publish the flag
        pub.publish(False)
        rospy.sleep(5)
        
        # Log message
        rospy.loginfo("Published False!")
        
        # Sleep to maintain the desired publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_flag()
    except rospy.ROSInterruptException:
        pass

#need to fix goTo Functino as its makes it so that it doesnt reach wp to needed specificity