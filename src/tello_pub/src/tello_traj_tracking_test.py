#!/usr/bin/env python3
import rospy;
import numpy as np

# for numpy array deserialization and serialization
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Empty;
# ros msgs
from geometry_msgs.msg import Twist, Point;
from tello_sub.msg import PointArr;
from tf.transformations import euler_from_quaternion;
# trajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint;

########################################################
# global variables
########################################################

# global variables for flight control
global TELLO_LAND, TELLO_SOT;
TELLO_LAND = False; TELLO_SOT = False;

# global CMD_YAW_RATE, CMD_VEL;
# CMD_YAW_RATE = 0.3; CMD_VEL = 0.2;

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
LOSS_TOL = 50;

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


# PD controller
class TrajTracker:


    def __init__(self, dist2wall, yaw_init):
        # dist2wall is a float number

        # ================================param==============================
        self.queue_size = 1;
        self.queue_size_waypoint = 1;
        self.xw_init = dist2wall;
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
        # self.K_prop = 0.9 * np.eye(3, dtype=np.float32);
        # self.K_diff = 0.1 * np.eye(3, dtype=np.float32);
        # self.K_prop = 0.45 * np.eye(3, dtype=np.float32);     # undershoot in both 2m and 1m case
        # self.K_prop = 0.75 * np.eye(3, dtype=np.float32);       # overshoot in 2m case, okay in 1m case
        self.K_prop = 0.6 * np.eye(3, dtype=np.float32);
        self.K_diff = 0.1 * np.eye(3, dtype=np.float32);
        self.K_prop_yaw = 1.0;
        self.K_diff_yaw = 0.1;

        # subscribers
        self.kf_sub = rospy.Subscriber("tello_pose_kf", numpy_msg(Floats), callback=self.mykfcb, queue_size=self.queue_size);
        self.waypoint_sub = rospy.Subscriber("tello_waypoint", Point, callback=self.myWayPointcb, queue_size=self.queue_size_waypoint);    
        # trajectory subscriber
        self.traj_sub = rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint, self.myTrajCallback, queue_size=1);

        # publishers
        self.exp_mode_pub = rospy.Publisher("/exp_mode/tracking", Empty, queue_size=1);
        self.vel_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=self.queue_size);
        self.EOT_pub = rospy.Publisher("/EOT", Empty, queue_size=self.queue_size);

        # state variables
        # [x, y, z, yaw] in world frame & body frame
        self.kf_state = np.zeros((4, ), dtype=np.float32);
        self.pose_err = np.zeros((4, ), dtype=np.float32);
        self.vel_err = np.zeros((4, ), dtype=np.float32);

        # initialize the yaw angle with a shift of positive pi
        # then the rotated odom frame have x-axis pointed to marker
        # that is, parallel to z-axis in camera frame
        self.yaw_shift = 0;

        # [vx, vy, vz] in world frame
        self.kf_LinVel = np.zeros((3, ), dtype=np.float32);

        # [roll_rate, pitch_rate, yaw_rate] in body frame
        self.kf_AngVel = np.zeros((3, ), dtype=np.float32);

        # ==============================waypoint=======================================
        self.waypoint = None;
        # self.hover_wp = np.array([self.xw_init, 0.0, 1.0, 0.0], dtype=np.float32);
        # self.forward_wp = np.array([self.xw_init-1, 0.0, 1.0, 0.0], dtype=np.float32);
        # self.backward_wp = np.array([self.xw_init+1, 0.0, 1.0, 0.0], dtype=np.float32);
        # for trajectory tracking
        self.vel_waypoint = None;


        # experimental status
        #   None: on ground
        #   forward: flying forward waypoint
        #   backward: flying backward waypoint
        #   hover: hover waypoint
        self.exp_status = None;
        self.status_record = [];

        # loop count
        self.loop_count = -1;
        self.small_vel_count = 0;

        
    def mykfcb(self, msg):
        ################# kf publish world frame now #######################
        # self.kf_state[3] = self.adjust_yaw(msg.data[3]);
        # self.kf_state[:3] = np.array([msg.data[0], msg.data[1], msg.data[2]]);
        self.kf_state = msg.data[:4];
        rospy.loginfo("========world frame drone position (x, y, z) = ({}, {}, {})========".format(
            self.kf_state[0], self.kf_state[1], self.kf_state[2]
        ));

        # assign linear velocity
        self.kf_LinVel = np.array([msg.data[4], msg.data[6], msg.data[8]]);

        # world frame
        # after examination y direction is reversed
        # so we add -1 term
        # (self.kf_LinVel[0], self.kf_LinVel[1], self.kf_LinVel[2]) = (self.kf_LinVel[2], -1*self.kf_LinVel[0], self.kf_LinVel[1]);
        rospy.loginfo("========world frame drone linear velocity (vx, vy, vz) = ({}, {}, {})========".format(
            self.kf_LinVel[0], self.kf_LinVel[1], self.kf_LinVel[2]
        ));

        # assign angular velocity
        # body frame
        j = 0;
        for i in [10, 11, 12]:
            self.kf_AngVel[j] = msg.data[i];
            j += 1;
        # rospy.loginfo("========body frame drone angular velocity (wx, wy, wz) = ({}, {}, {})========".format(
        #     self.kf_AngVel[0], self.kf_AngVel[1], self.kf_AngVel[2]
        # ));


        # TODO:
        # input:
        #       geometry_msgs.Point in world frame
        # do:
        #       assign waypoint from msg
        #       determine current status from waypoint
        #           hover => (xw, yw, zw) = (d, 0, 1)
        #           forward => (xw, yw, zw) = (d-1, 0, 1)
        #           backward => (xw, yw ,zw) = (d+1, 0, 1)
        #       yaw_deisred always 0
    def myWayPointcb(self, msg):
        self.waypoint = np.array([msg.x, msg.y, msg.z, 0.0], dtype=np.float32);
        rospy.loginfo("========world frame waypoint: (x, y, z, yaw) = ({})".format(self.waypoint));
        
        # determine status
        if np.array_equal(self.waypoint, self.hover_wp):
            self.exp_status = 'hover';
            rospy.loginfo("========HOVER MODE========");

            # record
            if len(self.status_record) == 0:
                self.status_record.append(self.exp_status);

        elif np.array_equal(self.waypoint, self.forward_wp):
            self.exp_status = 'forward';
            rospy.loginfo("========FORWARD MODE========");

            # record
            if len(self.status_record) == 1:
                self.status_record.append(self.exp_status);

        elif np.array_equal(self.waypoint, self.backward_wp):
            self.exp_status = 'backward';
            rospy.loginfo("========BACKWARD MODE========");

            # record
            if len(self.status_record) == 2:
                self.status_record.append(self.exp_status);

        else:
            # unknown or landing
            if len(self.status_record) == 3:
                self.exp_status = 'finish';
                self.status_record.append(self.exp_status);
            else:
                self.exp_status = None;


    # TODO:
    #   given trajectory points
    #   set as tracked waypoints of velocity and position
    def myTrajCallback(self, msg):
        # get yaw angle from quaternion
        axes_str = 'rzyx';
        quat = (msg.transforms[0].rotation.x, msg.transforms[0].rotation.y, msg.transforms[0].rotation.z, msg.transforms[0].rotation.w);
        rospy.loginfo("========[traj]========\nrotation (x,y,z,w): {}".format(quat));
        euler = euler_from_quaternion(quat, axes_str);
        self.waypoint = np.array([
            msg.transforms[0].translation.x,
            msg.transforms[0].translation.y,
            msg.transforms[0].translation.z,
            euler[2]
        ], dtype=np.float32);
        rospy.loginfo("========[traj]========\nwaypoint received: {}\n RPY from quaternion: {}".format(self.waypoint, euler));
        self.vel_waypoint = np.array([
            msg.velocities[0].linear.x,
            msg.velocities[0].linear.y,
            msg.velocities[0].linear.z,
            msg.velocities[0].angular.z
        ]);
        rospy.loginfo("========[traj]========\nvelocity waypoint received: {}\nvelocity norm: {}".format(
            self.vel_waypoint, np.linalg.norm(self.vel_waypoint)));

        # changing status
        self.exp_status = 'tracking';
        # set the status to finish if the desired velocity is very small for a period of time
        if (np.linalg.norm(self.vel_waypoint) < 1e-5) and (self.loop_count > 0):

            if self.small_vel_count >= 10:
                self.exp_status = 'finish';
                rospy.loginfo("========[traj]========\nexperiment finished!");
            self.small_vel_count += 1;
    
    
    def update_loopcount(self, loop_count):
        self.loop_count = loop_count;

    # TODO:
    #   manually set the cmd to 0
    def reset_vel(self):
        twist_msg = Twist();
        twist_msg.angular.x = 0.0; twist_msg.angular.y = 0.0; twist_msg.angular.z = 0.0;
        twist_msg.linear.x = 0.0; twist_msg.linear.y = 0.0; twist_msg.linear.z = 0.0;
        self.vel_pub.publish(twist_msg);
        rospy.loginfo("========[traj]========\nVELOCITY COMMAND RESET!!");

    # input:
    #       desired pose (4, )
    #       desired velocity are zeros
    # output:
    #       current pose and desired pose error in world frame & body frame
    def get_error(self, pose_d):
        self.pose_err = self.kf_state - pose_d;
        rospy.loginfo("========pose error: (ex, ey, ez, eyaw) = ({})".format(self.pose_err));

        self.vel_err = np.hstack((self.kf_LinVel, self.kf_AngVel[2]));
        rospy.loginfo("========velocity error: (e_vx, e_vy, e_vz, e_vyaw = ({}))".format(self.vel_err));


    # TODO:
    #   calculate current position and velocity error to desired from optimization
    def get_error_from_traj(self, pose_d, vel_d):
        self.pose_err = self.kf_state - pose_d;
        rospy.loginfo("========[traj]========\npose error: {}".format(self.pose_err));
        self.vel_err = np.hstack((
                self.kf_LinVel - vel_d[:3], 
                self.kf_AngVel[2] - vel_d[3]
            ));
        rospy.loginfo("========[traj]========\nvelocity error: {}".format(self.vel_err));


    # TODO:
    #       determine state based on waypoint
    #       publish velocity command based on pose_error
    def PositionTracking_PD(self):
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
            cmd_angular = np.array([0, 0, -0.9 * self.pose_err[3] + -0.1 * self.vel_err[3]]);

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
            # on land
            # reset velocity to 0
            # publsih 0 velocity
            self.reset_vel();

    # TODO:
    #   control the drone using waypoint from trajectory
    def WaypointTracking(self, flag='direct'):
        twist_msg = Twist();
        
        if (self.exp_status is not None):
            # normal eot
            if self.exp_status == 'finish':
                self.reset_vel();
                self.EOT_pub.publish(Empty());
                rospy.loginfo("========End of experiment========");

            # directly use the optimized velocity from minimum snap traj
            elif (flag == "direct"):
                rospy.loginfo("========[traj]========\ndirect control from waypoints");
                twist_msg.linear.x = self.vel_waypoint[0];
                twist_msg.linear.y = self.vel_waypoint[1];
                twist_msg.linear.z = self.vel_waypoint[2];
                twist_msg.angular.x = 0;
                twist_msg.angular.y = 0;
                twist_msg.angular.z = self.vel_waypoint[3];

                self.vel_pub.publish(twist_msg);

            elif (flag == "PD"):
                # rotation from world to current cmd frame
                # yaw rotation included
                rospy.loginfo("========[traj]========\nPD control law");
                rot_w2cmd = np.array([[np.sin(self.kf_state[3]), np.cos(self.kf_state[3]), 0], 
                                    [-1*np.cos(self.kf_state[3]), np.sin(self.kf_state[3]), 0], 
                                    [0, 0, 1]]);
                
                # get distance from waypoint
                self.get_error_from_traj(self.waypoint, self.vel_waypoint);
                
                # assign cmd_vel
                cmd_linear = -1*self.K_prop.dot(rot_w2cmd.dot(self.pose_err[:3])) + -1*self.K_diff.dot(rot_w2cmd.dot(self.vel_err[:3]));
                cmd_angular = np.array([
                        0,
                        0, 
                        -self.K_prop_yaw * self.pose_err[3] + -self.K_diff_yaw * self.vel_err[3]
                    ]);
                
                # invert the cmd angular due to yaw cmd direction
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
                self.vel_pub.publish(twist_msg);
        else:
            self.reset_vel();


# main
if __name__ == '__main__':    
    # ros node
    rospy.init_node("tello_traj_track_node");
    
    # subscribers
    rospy.Subscriber("/tello/land", Empty, myTelloLandCallback);
    rospy.Subscriber("/SOT", Empty, myTelloSOTCallback);
    rospy.Subscriber("/aruco_corners", PointArr, myPointArrCb);     # annotate for kf testing

    loop_count = 0;
    dist2wall = np.array([2, 0, 1.0], dtype=np.float32);
    yaw_init = np.array([0.0], dtype=np.float32);

    # controller
    tello_ctrller = TrajTracker(dist2wall, yaw_init);

    # timing
    rate = rospy.Rate(10);

    # marker
    marker_loss = True;
    

    while not rospy.is_shutdown():
        # outer loop determine start condition
        # that is: receive SOT & not receive land & eot_count == 0 & not aruco markers are seen

        # update loop count
        tello_ctrller.update_loopcount(loop_count);

        # showing the exp is ready
        tello_ctrller.exp_mode_pub.publish(Empty());

        # check markers
        marker_loss = False if LOSS_TOL > 0 else True;
        # marker_loss = False;             # for testing

        # outer loop conditions 
        if (TELLO_SOT and (not TELLO_LAND) and (not marker_loss)):
            rospy.loginfo("========experiment mode========");
            # tello_ctrller.PositionTracking_PD();
            tello_ctrller.WaypointTracking(flag="PD");
            loop_count += 1;
        
        ############################################################################################
        # forced landing
        elif (TELLO_LAND or marker_loss):
            if TELLO_LAND:
                rospy.loginfo("========forced landing========");
                tello_ctrller.reset_vel();
                break;
            if marker_loss:
                rospy.loginfo("========marker loss for 5 secs========");
                TELLO_LAND = True;
        
        # marker loss tolerance
        LOSS_TOL -= 1;        # annotate for testing
        

        rate.sleep();