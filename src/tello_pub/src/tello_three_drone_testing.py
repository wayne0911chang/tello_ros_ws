#!/usr/bin/env python3
import rospy;
# import numpy as np

from geometry_msgs.msg import Twist
from tello_driver.msg import TelloStatus;
from std_msgs.msg import Empty;

class TestingPub:
    def __init__(self, drone_name, mode=None):
        # self.tello_ns
        self.mode = mode;
        self.queue_size = 1;
        # self.tello_ns = rospy.get_namespace();
        self.tello_ns = drone_name;
        self.FORMATION_TAKEOFF = False;
        self.tello_state = {};
        self.des_vel = 0.4;
        self.form_dir = None;

        # print(self.tello_ns);
        # rospy.loginfo("========[three drone testing]========\n %s", self.tello_ns);

        # self.exp_mode_pub = rospy.Publisher(self.tello_ns + "exp_mode/debug", Empty, queue_size=1);
        # self.EOT_pub = rospy.Publisher(self.tello_ns + "EOT", Empty, queue_size=self.queue_size);
        self.vel_pub = rospy.Publisher(self.tello_ns + "cmd_vel", Twist, queue_size=self.queue_size);
        
        self.land_pub = rospy.Publisher(self.tello_ns + "land", Empty, queue_size=self.queue_size);
        self.takeoff_pub = rospy.Publisher(self.tello_ns + "takeoff", Empty, queue_size=self.queue_size);
        
        self.formation_takeoff_sub = rospy.Subscriber("/takeoff", Empty, self.formation_takeoff_Cb);
        self.EOT_sub = rospy.Subscriber("/EOT", Empty, self.myTelloEOTCb);

        self.tello_state_sub = rospy.Subscriber(self.tello_ns + "status", TelloStatus, self.myTelloStateCb);

        if self.mode is None:
            rospy.loginfo("========[three drone testing]========\nTesting mode is not set up!!");
            # self.reset_vel();
            # self.EOT_pub.publish(Empty());
            self.land_pub.publish(Empty());
            return;
    
        elif self.mode == "static":
            # self.vel_pub = rospy.Publisher(self.tello_ns + "cmd_vel", Twist, queue_size=self.queue_size);
            rospy.loginfo(self.tello_ns + " current mode: " + self.mode);
    
        elif self.mode == "hover":
            # self.pos_pub = rospy.Publisher(self.tello_ns + "tello_waypoint", Point, queue_size=1);
            rospy.loginfo(self.tello_ns + " current mode: " + self.mode);
        
        elif self.mode == "move":
            rospy.loginfo(self.tello_ns + " current mode: " + self.mode);
            # set velocity
            # self.des_vel = rospy.get_param('~des_vel', default=0.0);
            # set direction: 'x', 'y', 'z', 'yaw'
            # self.form_dir = rospy.get_param('~des_dir', default=None);
            self.in_formation = rospy.get_param('~in_form', default=False);
        
        elif self.mode == "form_ctrl":
            rospy.loginfo(self.tello_ns + " current mode: " + self.mode);
            self.in_formation = rospy.get_param('~in_form', default=True);


    '''
    @move
        publish velocity to move
    '''
    def move(self, form_dir):
        if self.FORMATION_TAKEOFF:
            twist_msg = Twist();
            if not self.in_formation:
                if form_dir == "x":
                    twist_msg.linear.x = self.des_vel;
                    self.vel_pub.publish(twist_msg)
                elif form_dir == "y":
                    twist_msg.linear.y = self.des_vel;
                    self.vel_pub.publish(twist_msg)
                elif form_dir == "z":
                    twist_msg.linear.z = self.des_vel;
                    self.vel_pub.publish(twist_msg)
                
                elif form_dir == "yaw":
                    twist_msg.angular.z = self.des_vel;
                    self.vel_pub.publish(twist_msg)
                else:
                    self.reset_vel();
                    rospy.loginfo("========[three drone testing]========\nUnknown Direction!");
            else:
                if ('0' in self.tello_ns):
                    if form_dir == "x":
                        twist_msg.linear.x = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    
                    elif form_dir == "y":
                        twist_msg.linear.y = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "z":
                        twist_msg.linear.z = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "yaw":
                        twist_msg.angular.z = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    else:
                        self.reset_vel();
                        rospy.loginfo("========[three drone testing]========\nUnknown Direction!");
                elif ('1' in self.tello_ns):
                    if form_dir == "x":
                        twist_msg.linear.x = -self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "y":
                        twist_msg.linear.y = -self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "z":
                        twist_msg.linear.z = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "yaw":
                        twist_msg.angular.z = -self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    else:
                        self.reset_vel();
                        rospy.loginfo("========[three drone testing]========\nUnknown Direction!");
                elif ('2' in self.tello_ns):
                    if form_dir == "x":
                        twist_msg.linear.y = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "y":
                        twist_msg.linear.x = -self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "z":
                        twist_msg.linear.z = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    elif form_dir == "yaw":
                        twist_msg.angular.z = self.des_vel;
                        self.vel_pub.publish(twist_msg)
                    else:
                        self.reset_vel();
                        rospy.loginfo("========[three drone testing]========\nUnknown Direction!");
    '''
    @reset_vel
        publish zero velocity
    '''
    def reset_vel(self):
        twist_msg = Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;

        self.vel_pub.publish(twist_msg);

    '''
    @myTelloSOTCb
        only publish takeoff in hover, move mode
    '''
    def formation_takeoff_Cb(self, msg):
        if (not self.FORMATION_TAKEOFF) and (self.mode != "static"):
            self.takeoff_pub.publish(Empty());
            rospy.loginfo(self.tello_ns + " receive takeoff!");
            self.FORMATION_TAKEOFF = True;

    '''
    @myTelloEOTCb
        reset velocities for the drone
        publish landing command in case of EOT received
    '''
    def myTelloEOTCb(self, msg):
        self.reset_vel();
        if self.FORMATION_TAKEOFF:
            self.land_pub.publish(Empty());
            rospy.loginfo(self.tello_ns + " receive EOT!");
            self.FORMATION_TAKEOFF = False;

    '''
    @myTelloStateCb
        receive tello state
    '''
    def myTelloStateCb(self, msg):       
        # self.tello_state['fly_mode'] = msg.fly_mode;
        self.tello_state['battery_state'] = msg.battery_state;
        self.tello_state['is_flying'] = msg.is_flying;
        self.tello_state['is_on_ground'] = msg.is_on_ground;
        

    '''
    @logTelloState
        print the tello state in case sub receive msg
        raise KeyError in case no matched key found
    '''
    def logTelloState(self):

        if self.tello_state_sub.get_num_connections() > 0:
            # catch KeyError
            try:
                rospy.loginfo(self.tello_ns + " status logging:\nfly mode: {}\nbattery state: {}\nis flying: {}\nis on ground: {}".format(
                    # self.tello_state['fly_mode'], 
                    self.tello_state['battery_state'],
                    self.tello_state['is_flying'],
                    self.tello_state['is_on_ground']
                ));
            except KeyError as ke:
                rospy.logerr("Received KeyError - reason:{}".format(ke));


if __name__ == '__main__':
    # global TELLO_SOT;
    # TELLO_SOT = False;
    rospy.init_node("three_drone_flight_ctrl");
    rate = rospy.Rate(10);
    
    test_mode = rospy.get_param('~mode', default='static');
    
    test_pub_0 = TestingPub(drone_name="/tello_0/", mode=test_mode)
    test_pub_1 = TestingPub(drone_name="/tello_1/", mode=test_mode)
    test_pub_2 = TestingPub(drone_name="/tello_2/", mode=test_mode)
    
    # test_pub = TestingPub(mode=test_mode);

    counter = 0;
    while not rospy.is_shutdown():
        # ready for exp
        # wp_pub.exp_mode_pub.publish(Empty());
        if test_pub_0.FORMATION_TAKEOFF and test_pub_1.FORMATION_TAKEOFF and test_pub_2.FORMATION_TAKEOFF:

            if test_mode == "move":
                formation_direction = input("key in x, y, z, yaw to control formation:     ");
                test_pub_0.move(formation_direction);
                test_pub_1.move(formation_direction);
                test_pub_2.move(formation_direction);
        
            # if (counter % 10 == 0):
            #     test_pub_0.logTelloState();
            #     test_pub_1.logTelloState();
            #     test_pub_2.logTelloState();
        

        counter += 1;
        rate.sleep();