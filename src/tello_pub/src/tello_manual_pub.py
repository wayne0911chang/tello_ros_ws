#!/usr/bin/env python3
import rospy;
import numpy as np
# test the keyboard function
# import keyboard;

from std_msgs.msg import Empty;
from geometry_msgs.msg import Twist;

global TELLO_SOT;
TELLO_SOT = False;


# SOT callback
def telloSOTCallback(msg):
    global TELLO_SOT;
    TELLO_SOT = True;
    pass;


# main
if __name__ == '__main__':   
    rospy.init_node("manaul_flight_single_drone");

    vel_pub_0 = rospy.Publisher("/tello_0/cmd_vel", Twist, queue_size=1);
    vel_pub_1 = rospy.Publisher("/tello_1/cmd_vel", Twist, queue_size=1);
    
    SOT_sub = rospy.Subscriber("/SOT", Empty, callback=telloSOTCallback);
    
    manual_pub_0 = rospy.Publisher("/tello_0/manual_takeoff", Empty, queue_size=1);
    manual_pub_1 = rospy.Publisher("/tello_1/manual_takeoff", Empty, queue_size=1);
    # manual_pub_2 = rospy.Publisher("/tello_2/manual_takeoff", Empty, queue_size=1);
    
    takeoff_pub_0 = rospy.Publisher("/tello_0/takeoff", Empty, queue_size=1);
    takeoff_pub_1 = rospy.Publisher("/tello_1/takeoff", Empty, queue_size=1);
    # takeoff_pub_2 = rospy.Publisher("/tello_2/takeoff", Empty, queue_size=1);
    
    
    land_pub_0 = rospy.Publisher("/tello_0/land", Empty, queue_size=1);
    land_pub_1 = rospy.Publisher("/tello_1/land", Empty, queue_size=1);
    # land_pub_2 = rospy.Publisher("/tello_2/land", Empty, queue_size=1);


    rate = rospy.Rate(10);

    counter = 0;
    while not rospy.is_shutdown():

        # check if SOT
        if (TELLO_SOT):
            # manual_pub_0.publish(Empty());
            # manual_pub_1.publish(Empty());
            # manual_pub_2.publish(Empty());

            # takeoff_pub_0.publish(Empty());
            takeoff_pub_1.publish(Empty());
            # takeoff_pub_2.publish(Empty());
            
            counter += 1;

        if (counter >= 10):
            # kb_event = keyboard.read_event();
            user_input = input("input the command direction\nw: forward\ns: backward\na: left-shift\nd: right-shift\nup: upward\ndown: downward\nr: positive yaw\nq: negative yaw\n")
            
            # publish flight command
            if user_input == 'w':
                twist_msg = Twist();
                twist_msg.linear.y = 0.4;
                # vel_pub_0.publish(twist_msg);
                vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("pressed w and publish forward command!");
            
            elif user_input == 's':
                twist_msg = Twist();
                twist_msg.linear.y = -0.4;
                # vel_pub_0.publish(twist_msg);
                vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("pressed s and publish backward command!");

            elif user_input == 'a':
                twist_msg = Twist();
                twist_msg.linear.x = -0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("pressed a and publish left command!");

            elif user_input == 'd':
                twist_msg = Twist();
                twist_msg.linear.x = 0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("pressed d and publish right command!");

            elif user_input == 'h':
                twist_msg = Twist();
                twist_msg.linear.z = 0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("press h and publish positive z command!");
            
            elif user_input == 'l':
                twist_msg = Twist();
                twist_msg.linear.z = -0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("press l and publish negative z command!");
            
            elif user_input == 'r':
                twist_msg = Twist();
                twist_msg.angular.z = 0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("press r and publish positive yaw command!");
            
            elif user_input == 'q':
                twist_msg = Twist();
                twist_msg.angular.z = -0.4;
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("press q and publish negative yaw command!");

            elif user_input == '':
                twist_msg = Twist();
                vel_pub_0.publish(twist_msg);
                # vel_pub_1.publish(twist_msg);
                # debug
                rospy.loginfo("press '' and publish zero command!");

            else:
                rospy.loginfo("the key pressed: {} but nothing done:(".format(user_input));

        '''
        if (counter >= 100):
            # publish landing
            # land_pub_0.publish(Empty());
            land_pub_1.publish(Empty());
            # land_pub_2.publish(Empty());
        '''

        rate.sleep();



