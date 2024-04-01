#!/usr/bin/env python3

import rospy;
from std_msgs.msg import Empty;

## main
if __name__ == '__main__':
    try:
        ## initialize the node
        rospy.init_node('tello_emergency_node');
        tello_ns = rospy.get_namespace();
        print(tello_ns);
        
        ## emergency publisher
        # all motor shutdown
        EM_pub = rospy.Publisher(tello_ns + 'emergency', Empty, queue_size=1);

        rate = rospy.Rate(10);
        while not rospy.is_shutdown():
            
            ## ask for keyboard input to publish emergency
            tmp = input(tello_ns + " emergency case?...");
            if tmp == "Y":
                EM_pub.publish(Empty());
            
            rate.sleep();
    except rospy.ROSInterruptException:
        pass;
        
        