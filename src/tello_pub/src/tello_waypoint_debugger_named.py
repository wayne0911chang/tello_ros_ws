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
        self.tello_ns = rospy.get_namespace();
        self.TELLO_SOT = False;
        self.TELLO_FLYING = False;
        self.traj_type = "none"
        self.square_vel = 0.5;

        print(self.tello_ns);
        rospy.loginfo("========[wp debugger]========\n %s", self.tello_ns);
        print(rospy.get_name());
        self.traj_type = rospy.get_param(rospy.get_name() + "/traj_type");
        self.square_vel = rospy.get_param(rospy.get_name() + "/square_vel");
        print(self.traj_type);
        print(self.square_vel);
        
        self.land_pub = rospy.Publisher(self.tello_ns + "land", Empty, queue_size=self.queue_size);
        self.land_sub = rospy.Subscriber(self.tello_ns + "land", Empty, self.myTelloLandCb);
        self.takeoff_sub = rospy.Subscriber(self.tello_ns + "takeoff", Empty, self.myTelloTakeoffCb);
        rospy.Subscriber("/SOT", Empty, self.myTelloSOTCb);

        if self.mode is None:
            rospy.loginfo("========[wp debugger]========\nDebugger mode is not set up!!");
            self.reset_vel();
            # self.EOT_pub.publish(Empty());
            self.land_pub.publish(Empty());
            return;
        elif self.mode == "velocity":
            self.vel_pub = rospy.Publisher(self.tello_ns + "cmd_vel", Twist, queue_size=self.queue_size);            

        elif self.mode == "position":
            self.pos_pub = rospy.Publisher(self.tello_ns + "tello_waypoint", Point, queue_size=1);

    def myTelloSOTCb(self, msg):
        if not self.TELLO_SOT:
            rospy.loginfo("receive SOT!");
            self.TELLO_SOT = True;
    
    def myTelloLandCb(self, msg):
        self.TELLO_FLYING = False;
        self.reset_vel();
        rospy.loginfo("receive LAND!");
    
    def myTelloTakeoffCb(self, msg):
        self.TELLO_FLYING = True;
        rospy.loginfo("receive TAKEOFF");

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

    '''
    @square_traj
        positive y
        negative x
        negative y
        positive x
    '''
    def square_traj(self):
        if self.mode == "velocity":
            twist_msg = Twist();
            twist_msg.angular.x = 0.0; 
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = 0.0;
            tmp = input("input:\n\tforward: w\n\tbackward: s\n\tleftward: a\n\trightward: d\n");
            print(tmp);

            if len(tmp) != 1:
                rospy.loginfo("========[wp debugger]========\nwrong format of velocity command!");
                self.reset_vel();
                return;
            else:
                if tmp == "w":
                    twist_msg.linear.x = 0.0;
                    twist_msg.linear.y = self.square_vel;
                    twist_msg.linear.z = 0.0;
                elif tmp == "s":
                    twist_msg.linear.x = 0.0;
                    twist_msg.linear.y = -self.square_vel;
                    twist_msg.linear.z = 0.0;
                elif tmp == "a":
                    twist_msg.linear.x = -self.square_vel;
                    twist_msg.linear.y = 0.0;
                    twist_msg.linear.z = 0.0;
                elif tmp == "d":
                    twist_msg.linear.x = self.square_vel;
                    twist_msg.linear.y = 0.0;
                    twist_msg.linear.z = 0.0;
                else:
                    twist_msg.linear.x = 0.0;
                    twist_msg.linear.y = 0.0;
                    twist_msg.linear.z = 0.0;

            self.vel_pub.publish(twist_msg);
        else:
            self.reset_vel();

    def reset_vel(self):
        twist_msg = Twist();
        twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0; twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0; twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0;
        self.vel_pub.publish(twist_msg);
        rospy.loginfo("========[wp debugger]========\nVELOCITY COMMAND RESET!!");

if __name__ == '__main__':
    # global TELLO_SOT;
    # TELLO_SOT = False;
    rospy.init_node("waypoint_publisher_test");
    rate = rospy.Rate(20);
    wp_pub = WaypointPub(mode="velocity");
    
    while not rospy.is_shutdown():
        # ready for exp
        # wp_pub.exp_mode_pub.publish(Empty());

        if wp_pub.TELLO_SOT and wp_pub.TELLO_FLYING:
            if wp_pub.traj_type == "circle":
                wp_pub.waypoint_publishing();
            elif wp_pub.traj_type == "square":
                wp_pub.square_traj();
            else:
                wp_pub.reset_vel();
        # else:
        #     rospy.loginfo("========[wp debugger]========\n Experiment not started!");
        
        rate.sleep();