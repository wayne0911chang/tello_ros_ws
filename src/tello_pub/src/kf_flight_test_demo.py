#!/usr/bin/env python3
import rospy;
import numpy as np

# for numpy array deserialization and serialization
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Empty;
# ros msgs
from geometry_msgs.msg import Twist, Point;
from tello_sub.msg import PointArr

########################################################
# global variables
########################################################

# global variables for flight control
global TELLO_LAND, TELLO_SOT;
TELLO_LAND = False; TELLO_SOT = False;

global CMD_YAW_RATE, CMD_VEL;
CMD_YAW_RATE = 0.3; CMD_VEL = 0.2;

# global variables for markers
global IMG_Y_MAX, IMG_X_MAX;
IMG_Y_MAX = 720; IMG_X_MAX = 960;

global MARKER_LEFT, MARKER_RIGHT, MARKER_UP, MARKER_DOWN;
MARKER_LEFT = IMG_X_MAX; MARKER_RIGHT = -1; MARKER_UP = IMG_Y_MAX; MARKER_DOWN = -1;

global MARKER_L_INIT, MARKER_R_INIT, MARKER_U_INIT, MARKER_D_INIT;
MARKER_L_INIT = 0; MARKER_R_INIT = IMG_X_MAX; MARKER_U_INIT = 0; MARKER_D_INIT = IMG_Y_MAX;

global MARKER_COUNT, MARKER_THR;
MARKER_COUNT = 0; MARKER_THR = 20;

# global variable for marker loss tolerance
global LOSS_TOL;
LOSS_TOL = 30;

####################################################
# functions
####################################################

# set corner to initial position
def corner_reinit():
    global MARKER_LEFT, MARKER_RIGHT, MARKER_UP, MARKER_DOWN;
    MARKER_LEFT = IMG_X_MAX; MARKER_RIGHT = -1;
    MARKER_UP = IMG_Y_MAX; MARKER_DOWN = -1;

# callbacks
def myTelloLandCallback(msg):
    global TELLO_LAND;
    rospy.loginfo("========receive landing command========");
    TELLO_LAND = True;

def myTelloSOTCallback(msg):
    global TELLO_SOT;
    rospy.loginfo("========receive Start of Testing command========");
    TELLO_SOT = True;

def myPointArrCb(msg):
    global MARKER_LEFT, MARKER_RIGHT, MARKER_UP, MARKER_DOWN;
    global MARKER_L_INIT, MARKER_R_INIT, MARKER_U_INIT, MARKER_D_INIT;
    global MARKER_COUNT;
    global LOSS_TOL;

    # initialize the marker corners
    corner_reinit();
    # rospy.loginfo("========receive marker corner========\nlength: {}".format(msg.size));

    # show data and assign marker corner
    for i in range(len(msg.ids)):
        # rospy.loginfo("{}th marker id: {}".format(i, msg.ids[i]));
        for j in range(4*i, 4*(i+1)):
            # rospy.loginfo("{}th marker corners: ({}, {})".format(i, msg.points[j].x, msg.points[j].y));
            # assign marker position
            MARKER_LEFT = msg.points[j].x if (msg.points[j].x < MARKER_LEFT) else MARKER_LEFT;
            MARKER_RIGHT = msg.points[j].x if (msg.points[j].x > MARKER_RIGHT) else MARKER_RIGHT;
            MARKER_UP = msg.points[j].y if (msg.points[j].y < MARKER_UP) else MARKER_UP;
            MARKER_DOWN = msg.points[j].y if (msg.points[j].y > MARKER_DOWN) else MARKER_DOWN;

    # check for initial
    if (MARKER_COUNT == 0):
        MARKER_L_INIT = MARKER_LEFT;
        MARKER_R_INIT = MARKER_RIGHT;
        MARKER_D_INIT = MARKER_DOWN;
        MARKER_U_INIT = MARKER_UP;

    MARKER_COUNT += 1;
    
    # 50 -> 5 sec
    LOSS_TOL = 50;


####################################################
# main controller class
####################################################

# PD controller
class DemoCtrller:


    def __init__(self, dist2wall, yaw_init):
        # dist2wall is a float number

        # param
        self.queue_size = 1;
        self.queue_size_waypoint = 1;
        self.pose_init = np.hstack((dist2wall, yaw_init)).astype(np.float32);

        # maximum command
        self.LinVelMax = 2.0;
        self.AngVelMax = 2.0;
        self.LinVelMin = -2.0;
        self.AngVelMin = -2.0;

        # PD controller gain
        ##################################
        # K_p = [[0.9,   0,   0],
        #        [  0, 0.9,   0],
        #        [  0,   0, 0.9]]
        ##################################
        # K_D = [[0.1,   0,   0],
        #        [  0, 0.1,   0],
        #        [  0,   0, 0.1]]
        ##################################
        # 0930 case
        # self.K_prop = 0.5 * np.eye(3, dtype=np.float32);
        # self.K_diff = 0.05 * np.eye(3, dtype=np.float32);
        # self.K_prop_yaw = 0.5;
        # self.K_diff_yaw = 0.05;
        # 1005 case 1
        # self.K_prop = 1.0 * np.eye(3, dtype=np.float32);
        # self.K_diff = 0.1 * np.eye(3, dtype=np.float32);
        # self.K_prop_yaw = 1.0;
        # self.K_diff_yaw = 0.1;
        # 1005 case 2
        # self.K_prop = 1.0 * np.eye(3, dtype=np.float32);
        # self.K_diff = 0.25 * np.eye(3, dtype=np.float32);
        # self.K_prop_yaw = 1.35;
        # self.K_diff_yaw = 0.25;
        # 1005 case 3
        # self.K_prop = 1.0 * np.eye(3, dtype=np.float32);
        # self.K_diff = 0.25 * np.eye(3, dtype=np.float32);
        # self.K_prop_yaw = 1.35;
        # self.K_diff_yaw = 0.25;
        # 1005 case 4
        self.K_prop = 0.75 * np.eye(3, dtype=np.float32);
        self.K_diff = 0.1 * np.eye(3, dtype=np.float32);
        self.K_prop_yaw = 1.0;
        self.K_diff_yaw = 0.1;

        # subscribers
        self.kf_sub = rospy.Subscriber("tello_pose_kf", numpy_msg(Floats), callback=self.mykfcb, queue_size=self.queue_size);
        self.waypoint_sub = rospy.Subscriber("tello_waypoint", Point, callback=self.myWayPointcb, queue_size=self.queue_size_waypoint);    

        # publishers
        self.exp_mode_pub = rospy.Publisher("/exp_mode/tracking", Empty, queue_size=1);
        self.vel_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=self.queue_size);
        # self.vel_pub = rospy.Publisher("/cmd_vel_debug", Twist, queue_size=self.queue_size);
        self.EOT_pub = rospy.Publisher("/EOT", Empty, queue_size=1);

        # state variables
        # [x, y, z, yaw] in world frame & body frame
        self.kf_state = np.zeros((4, ), dtype=np.float32);
        self.pose_err = np.zeros((4, ), dtype=np.float32);
        self.vel_err = np.zeros((4, ), dtype=np.float32);

        # initialize the yaw angle with a shift of positive pi
        # then the rotated odom frame have x-axis pointed to marker
        # that is, parallel to z-axis in camera frame
        # self.yaw_shift = np.pi;   # annotate for testing
        self.yaw_shift = 0;         # for testing

        # [vx, vy, vz] in world frame
        self.kf_LinVel = np.zeros((3, ), dtype=np.float32);

        # [roll_rate, pitch_rate, yaw_rate] in body frame
        self.kf_AngVel = np.zeros((3, ), dtype=np.float32);

        # waypoint
        self.waypoint = None;

        # experimental status
        #   None:       on ground
        #   forward:    flying forward waypoint
        #   backward:   flying backward waypoint
        #   hover:      hover waypoint
        self.exp_status = None;
        self.status_record = [];

        # loop count
        self.loop_count = -1;


    def mykfcb(self, msg):
    ####################################################################
    ## ref
    # tello_pose_kf[0] = drone_x.X[0, 0]
    # tello_pose_kf[1] = drone_y.X[0, 0]
    # tello_pose_kf[2] = drone_z.X[0, 0]
    # tello_pose_kf[3] = drone_yaw.X_yaw[0, 0]
    # tello_pose_kf[4] = drone_x.X[1, 0] # x velocity
    # tello_pose_kf[5] = drone_x.X[2, 0] # x velocity drift
    # tello_pose_kf[6] = drone_y.X[1, 0] # y velocity
    # tello_pose_kf[7] = drone_y.X[2, 0] # y velocity drift
    # tello_pose_kf[8] = drone_z.X[1, 0] # z velocity
    # tello_pose_kf[9] = drone_z.X[2, 0] # z velocity drift
    # tello_pose_kf[10] = temp_imu[0] # x angular velocity
    # tello_pose_kf[11] = temp_imu[1] # y angular velocity
    # tello_pose_kf[12] = temp_imu[2] # z angular velocity
    # pub_pose_kf.publish(tello_pose_kf)
    #####################################################################  
        # assign states
        # transform to marker frame => tobecheck
        # pure yaw rotation model
        # shift yaw with pi
        self.kf_state[3] = self.adjust_yaw(msg.data[3]);
        
        # 4th
        rot_mat = np.array([
            [-np.cos(self.kf_state[3]), 0, np.sin(self.kf_state[3])],
            [0, 1, 0],
            [np.sin(self.kf_state[3]), 0, np.cos(self.kf_state[3])]]);

        kf_raw = np.array([msg.data[0], msg.data[1], msg.data[2]]);
        self.kf_state[:3] = rot_mat.dot(kf_raw);
        
        # rospy.loginfo("========marker frame drone position & pose (x, y, z, yaw) = ({}, {}, {}, {})========".format(
        #     self.kf_state[0], self.kf_state[1], self.kf_state[2], self.kf_state[3]
        # ));

        # marker frame to world frame
        (self.kf_state[0], self.kf_state[1], self.kf_state[2]) = (self.kf_state[2], self.kf_state[0], self.kf_state[1]+1);
        rospy.loginfo("========world frame drone position (x, y, z) = ({}, {}, {})========".format(
            self.kf_state[0], self.kf_state[1], self.kf_state[2]
        ));


        # assign linear velocity
        # marker frame
        kf_raw = np.array([msg.data[4], msg.data[6], msg.data[8]]);
        self.kf_LinVel = rot_mat.dot(kf_raw);
        # world frame
        # after examination y direction is reversed
        # so we add -1 term
        (self.kf_LinVel[0], self.kf_LinVel[1], self.kf_LinVel[2]) = (self.kf_LinVel[2], -1*self.kf_LinVel[0], self.kf_LinVel[1]);
        # rospy.loginfo("========world frame drone linear velocity (vx, vy, vz) = ({}, {}, {})========".format(
        #     self.kf_LinVel[0], self.kf_LinVel[1], self.kf_LinVel[2]
        # ));

        # assign angular velocity
        # body frame
        j = 0;
        for i in [10, 11, 12]:
            self.kf_AngVel[j] = msg.data[i];
            j += 1;
        # rospy.loginfo("========body frame drone angular velocity (wx, wy, wz) = ({}, {}, {})========".format(
        #     self.kf_AngVel[0], self.kf_AngVel[1], self.kf_AngVel[2]
        # ));

    def myWayPointcb(self, msg):
        # input:
        #       geometry_msgs.Point in world frame
        # do:
        #       assign waypoint from msg
        #       determine current status from waypoint
        #           hover => (xw, yw, zw) = (d, 0, 1)
        #       yaw_deisred always 0
        self.waypoint = np.array([msg.x, msg.y, msg.z, 0.0], dtype=np.float32);
        rospy.loginfo("========world frame waypoint: (x, y, z, yaw) = ({})".format(self.waypoint));
        
        self.exp_status = 'fly';


    def update_loopcount(self, loop_count):
        self.loop_count = loop_count;


    def adjust_yaw(self, yaw_data):
        if self.loop_count == 0:
            # check the direction of msg.data[3]
            # if yaw_data < -0.5*np.pi:
            #     yaw_data += 0.5 * np.pi;
            #     self.yaw_shift = 0.5 * np.pi;
            # elif yaw_data > 0.5*np.pi:
            #     yaw_data -= 0.5 * np.pi;
            #     self.yaw_shift = -0.5 * np.pi;
            if yaw_data < -3.0:
                self.yaw_shift = np.pi;
            elif yaw_data > 3.0:
                self.yaw_shift = -np.pi;
        yaw_data += self.yaw_shift;
        return yaw_data;


    def reset_vel(self):
        twist_msg = Twist();
        twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0; twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0; twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0;
        self.vel_pub.publish(twist_msg);
        rospy.loginfo("========VELOCITY COMMAND RESET!!========");

    def get_error(self, pose_d):
        # input:
        #       desired pose (4, )
        #       desired velocity are zeros
        # output:
        #       current pose and desired pose error in world frame & body frame
        self.pose_err = self.kf_state - pose_d;
        rospy.loginfo("========pose error: (ex, ey, ez, eyaw) = ({})".format(self.pose_err));

        self.vel_err = np.hstack((self.kf_LinVel, self.kf_AngVel[2]));
        rospy.loginfo("========velocity error: (e_vx, e_vy, e_vz, e_vyaw = ({}))".format(self.vel_err));

    def PositionTracking_PD(self):
        # TODO:
        #       determine state based on waypoint
        #       publish velocity command based on pose_error
        twist_msg = Twist();
        # rotation from world to current cmd frame
        # yaw rotation included
        rot_w2cmd = np.array([[np.sin(self.kf_state[3]), np.cos(self.kf_state[3]), 0], 
                            [-1*np.cos(self.kf_state[3]), np.sin(self.kf_state[3]), 0], 
                            [0, 0, 1]]);
        #################################### add yaw rotation
        # hovering
        if (self.exp_status is not None):
            
            # get distance from waypoint
            self.get_error(self.waypoint);
            
            # assign cmd_vel
            cmd_linear = -1*self.K_prop.dot(rot_w2cmd.dot(self.pose_err[:3])) + -1*self.K_diff.dot(rot_w2cmd.dot(self.vel_err[:3]));
            cmd_angular = np.array([0, 0, -1*self.K_prop_yaw * self.pose_err[3] + -1*self.K_diff_yaw * self.vel_err[3]]);
            
            # invert the cmd_angular due to yaw command direction
            cmd_angular = -1 * cmd_angular;

            # checking commands
            rospy.loginfo("========twist msg before clipped:\nlinear:\t{}\nangular:\t{}".format(cmd_linear, cmd_angular));

            cmd_linear = np.clip(cmd_linear, self.LinVelMin, self.LinVelMax);
            cmd_angular = np.clip(cmd_angular, self.AngVelMin, self.AngVelMax);
            
            rospy.loginfo("========twist msg after clipped:\nlinear:\t{}\nangular:\t{}".format(cmd_linear, cmd_angular));

            # assign back to Vector3
            twist_msg.linear.x = cmd_linear[0];
            twist_msg.linear.y = cmd_linear[1];
            twist_msg.linear.z = cmd_linear[2];
            twist_msg.angular.x = cmd_angular[0];
            twist_msg.angular.y = cmd_angular[1];
            twist_msg.angular.z = cmd_angular[2];
            
            # publish
            self.vel_pub.publish(twist_msg)

            # normal eot
            if self.exp_status == 'finish':
                self.reset_vel();
                self.EOT_pub.publish(Empty());
                rospy.loginfo("========End of experiment========");

        else:
            # reset velocity to 0
            # publish 0 velocity
            self.reset_vel();



# main
if __name__ == '__main__':    
    # ros node
    rospy.init_node("tello_tracking_demo_node");
    
    # subscribers
    rospy.Subscriber("/tello/land", Empty, myTelloLandCallback);
    rospy.Subscriber("/SOT", Empty, myTelloSOTCallback);
    rospy.Subscriber("/aruco_corners", PointArr, myPointArrCb);     # annotate for kf testing

    loop_count = 0;
    # controller
    # dist2wall: 3x1 np.float32
    # yaw_init: 1x1 np.float32
    dist2wall = np.array([2, 0, 1.0], dtype=np.float32);
    # dist2wall = np.array([2, 0, 1.5], dtype=np.float32);
    yaw_init = np.array([0.0], dtype=np.float32);
    
    tello_ctrller = DemoCtrller(dist2wall, yaw_init);

    # timing
    rate = rospy.Rate(10);

    # marker
    marker_loss = True;
    

    while not rospy.is_shutdown():
        # outer loop determine start condition
        # that is: receive SOT & not receive land & eot_count == 0 & not aruco markers are seen
        
        # update loop count
        tello_ctrller.update_loopcount(loop_count);

        # showing the experiment is ready
        tello_ctrller.exp_mode_pub.publish(Empty());

        # check markers
        marker_loss = False if LOSS_TOL > 0 else True;

        # outer loop conditions 
        if (TELLO_SOT and (not TELLO_LAND) and (not marker_loss)):
            rospy.loginfo("========experiment mode========");
            tello_ctrller.PositionTracking_PD();
        
        ############################################################################################
        # forced landing
        elif (TELLO_LAND or marker_loss):
            
            if marker_loss:
                rospy.loginfo("========marker loss for 5 secs========");
                TELLO_LAND = True;

            if TELLO_LAND:
                rospy.loginfo("========forced landing========");
                tello_ctrller.reset_vel();
                break;

        # marker loss tolerance
        LOSS_TOL -= 1;        # annotate for testing
        loop_count += 1;

        rate.sleep();