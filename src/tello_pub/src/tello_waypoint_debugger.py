#!/usr/bin/env python3
import rospy;
# import numpy as np

# for numpy array deserialization and serialization
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Empty;

global TELLO_SOT;
def myTelloSOTCallback(msg):
    global TELLO_SOT;
    rospy.loginfo("========receive Start of Testing command========");
    TELLO_SOT = True;

class WaypointPub:
    def __init__(self, mode=None):
        self.mode = mode;
        self.queue_size = 1;
        self.exp_mode_pub = rospy.Publisher("/exp_mode/debug", Empty, queue_size=1);
        self.EOT_pub = rospy.Publisher("/EOT", Empty, queue_size=self.queue_size);


        if self.mode is None:
            rospy.loginfo("========[wp debugger]========\nDebugger mode is not set up!!");
            self.reset_vel();
            self.EOT_pub.publish(Empty());
            return;
        elif self.mode == "velocity":
            # self.vel_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=self.queue_size);
            self.vel_pub = rospy.Publisher("/tello_0/cmd_vel", Twist, queue_size=self.queue_size);
        elif self.mode == "position":
            self.pos_pub = rospy.Publisher("tello_waypoint", Point, queue_size=1);

    def waypoint_publishing(self):
        if self.mode is None:
            rospy.logdebug("========[wp debugger]========\nUnknown mode!!");
        
        elif self.mode == "velocity":
            twist_msg = Twist();
            twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0;
            tmp = eval(input("input vx, vy, vz, az in drone command frame: "));
            print(tmp);

            if len(tmp) != 4:
                rospy.loginfo("========[wp debugger]========\nwrong format of velocity command!");
                self.reset_vel();
                return;
        
            else:
                twist_msg.linear.x = tmp[0];
                twist_msg.linear.y = tmp[1];
                twist_msg.linear.z = tmp[2];
                twist_msg.angular.z = tmp[3];
                self.vel_pub.publish(twist_msg);
                return;
    
        elif self.mode == "position":
            waypoint_msg = Point();
            tmp = eval(input("input x, y, z in world frame: "));

            if len(tmp) != 3:
                rospy.loginfo("========[wp debugger]========\nwrong format of position command!");
                self.reset_vel();
                return;
        
            else:
                waypoint_msg.x = tmp[0];
                waypoint_msg.y = tmp[1];
                waypoint_msg.z = tmp[2];
                self.pos_pub.publish(waypoint_msg);
    

    def reset_vel(self):
        twist_msg = Twist();
        twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0; twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0; twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0;
        self.vel_pub.publish(twist_msg);
        rospy.loginfo("========[wp debugger]========\nVELOCITY COMMAND RESET!!");

if __name__ == '__main__':
    # global TELLO_SOT;
    TELLO_SOT = False;
    rospy.init_node("waypoint_publisher_test");
    rate = rospy.Rate(2);
    rospy.Subscriber("/SOT", Empty, myTelloSOTCallback);
    # tmp = np.array([0, 0, 0], dtype=np.float32);
    # # get parameters
    # tmp[0] = rospy.get_param('~wp_x', default=2.0);
    # tmp[1] = rospy.get_param('~wp_y', default=0.0);
    # tmp[2] = rospy.get_param('~wp_z', default=1.0);
    # mode_tmp = input("input the debugging mode: position or velocity: ");
    wp_pub = WaypointPub(mode="velocity");

    while not rospy.is_shutdown():
        # ready for exp
        wp_pub.exp_mode_pub.publish(Empty());

        if TELLO_SOT:
            wp_pub.waypoint_publishing();
        else:
            rospy.loginfo("========[wp debugger]========\n Experiment not started!");
        
        rate.sleep();