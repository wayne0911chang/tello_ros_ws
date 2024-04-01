#!/usr/bin/env python3

import rospy;
from geometry_msgs.msg import Twist


###############################################################
# TODO:
#   for tello are not shutdown normally
#   cmd_vel commmand would be latched in the flying controller
#   use this node to reset to 0
###############################################################



def cmd_vel_cb(msg):
    rospy.loginfo("========show the cmd_vel info========\nlinear: x={}, y={}, z={}\nangular: x={}, y={}, z={}".format(
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z));

## main
if __name__ == '__main__':
    rospy.init_node("reset_velocity");
    tello_ns = rospy.get_namespace();

    vel_pub = rospy.Publisher(tello_ns + 'cmd_vel', Twist, queue_size=10);
    vel_sub = rospy.Subscriber(tello_ns + 'cmd_vel', Twist, cmd_vel_cb);
    ## 10 hz
    rate = rospy.Rate(10);

    while not rospy.is_shutdown():

        rospy.loginfo("========reset the cmd_vel========");
        twist_msg = Twist();
        twist_msg.linear.x = 0; twist_msg.linear.y = 0; twist_msg.linear.z = 0;
        twist_msg.angular.x = 0; twist_msg.angular.y = 0; twist_msg.angular.z = 0;
        vel_pub.publish(twist_msg);

        ## wait
        rate.sleep();