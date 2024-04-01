#!/usr/bin/env python3

import threading
import numpy as np;
import rospy

from rospy.numpy_msg import numpy_msg;
from rospy_tutorials.msg import Floats;

from tello_driver.msg import TelloStatus;
from tello_sub.msg import PointArr;

from std_msgs.msg import Empty;

from geometry_msgs.msg import Point, Twist, PoseArray;

from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint;

import smach, smach_ros

'''
@TODO:
    monitor node 
    wrap up all the states
    record the flag status along experiment time
        how to get the flag status from the main class...?
            global container with data append from subscribers
            set flags in the state constructor and update flags when executing states
'''
class TelloMonitor():
    def __init__(self):
        # ros node 
        rospy.init_node('tello_monitor_node');

        self.tello_sm = smach.StateMachine(outcomes=['terminate', 'emergency']);
        self.tello_sm.userdata.redo_flag = False;
        # flags record
        self.tello_sm.userdata.monitor = {};

        # Open the container
        with self.tello_sm:
            # Add states to the container
            smach.StateMachine.add(
                'Resting', Resting(), transitions={
                    'takeoff':      'Hovering',
                    'idle_rest':    'Resting',
                    'terminate':    'terminate',
                    'redo_rest':    'Hovering'},
                remapping={
                    'redo_flag_in': 'redo_flag',
                    'redo_flag_out':'redo_flag',
                    'monitor_in':   'monitor',
                    'monitor_out':  'monitor'}
            );
            smach.StateMachine.add(
                'Hovering', Hovering(), transitions={
                    'landing':      'Resting',
                    'emergency':    'emergency',
                    'idle_hover':   'Hovering',
                    'test':         'Testing',
                    'redo_hover':   'Testing'},
                remapping={
                    'redo_flag_in': 'redo_flag',
                    'redo_flag_out':'redo_flag',
                    'monitor_in':   'monitor',
                    'monitor_out':  'monitor'}
            );
            smach.StateMachine.add(
                'Testing', Testing(), transitions={
                    'emergency':    'emergency',
                    'eot':          'Hovering',
                    'idle_test':    'Testing',
                    'redo_test':    'Resting'},
                remapping={
                    'redo_flag_in': 'redo_flag',
                    'redo_flag_out':'redo_flag',
                    'monitor_in':   'monitor',
                    'monitor_out':  'monitor'}
            );
    

    '''
    @TODO:
        combine the redo flags together in to one plot
    '''
    def plot_flags(self):

        # save the data in npz format
        fpath = '/home/chungyu/wayne_temp/tello_ws/';
        np.savez_compressed(
                fpath + 'flags_record_' + get_current_time() + '.npz',
                redo_rest_in = self.tello_sm.userdata.monitor['redo_rest_in'],
                redo_hover_in = self.tello_sm.userdata.monitor['redo_hover_in'],
                redo_test_in = self.tello_sm.userdata.monitor['redo_test_in'],
                aruco_corner = self.tello_sm.userdata.monitor['aruco_corner'],
                aruco_np = self.tello_sm.userdata.monitor['aruco_np'],
                imu_np = self.tello_sm.userdata.monitor['imu_np'],
                kf_np = self.tello_sm.userdata.monitor['kf_np'],
                wp_debug = self.tello_sm.userdata.monitor['wp_debug'],
                wp_fixed = self.tello_sm.userdata.monitor['wp_fixed'],
                traj_smpl = self.tello_sm.userdata.monitor['traj_smpl'],
                is_takeoff = self.tello_sm.userdata.monitor['is_takeoff'],
                is_land = self.tello_sm.userdata.monitor['is_land'],
                is_em = self.tello_sm.userdata.monitor['is_em'],
                is_eot = self.tello_sm.userdata.monitor['is_EOT'],
                is_sot = self.tello_sm.userdata.monitor['is_SOT'],
                exp_mode = self.tello_sm.userdata.monitor['exp_mode']
            );

        import matplotlib.pyplot as plt;
        plt.rcParams['text.usetex'] = True;
        
        ############################## setup redo flags v.s. time
        fig, axes = plt.subplots(4, 1, sharey=True);
        redo_flags = ['redo_rest_in', 'redo_hover_in', 'redo_test_in'];
        tmp = np.array([-1], dtype=np.uint8).flatten();
        for i in range(len(redo_flags)):
            flag_arr = np.array(self.tello_sm.userdata.monitor[redo_flags[i]], dtype=np.uint8).reshape(-1);     # flatten in 1 dim
            tmp = np.hstack((tmp, flag_arr));
            mline, sline, bline = axes[i].stem(
                0.1 * np.arange(len(self.tello_sm.userdata.monitor[redo_flags[i]])),
                self.tello_sm.userdata.monitor[redo_flags[i]],
                linefmt='b--', markerfmt='bo', bottom=0
            );
            bline.set_color('b');
            mline.set_markerfacecolor('none');
            
            axes[i].set_ylabel(redo_flags[i]);
            axes[i].grid();

        # plot the concatenated data
        tmp = tmp[1:];
        mline, sline, bline = axes[3].stem(
                0.1 * np.arange(tmp.shape[0]), tmp,
                linefmt='g--', markerfmt='go', bottom=0
            );
        bline.set_color('g');
        mline.set_markerfacecolor('none');
        axes[3].set_ylabel('redo_concat');
        axes[3].set_xlabel('time(s)');
        axes[3].set_ylim(0, 1);
        axes[3].grid();

        plt.tight_layout();
        plt.savefig(fpath + 'redo_flag_' + get_current_time() + '.svg');

        ######################################## plot sensor flags
        fig, axes = plt.subplots(4, 1, sharey=True);
        sensor_flags = ['aruco_corner', 'aruco_np', 'imu_np', 'kf_np'];
        for i in range(len(sensor_flags)):
            mline, sline, bline = axes[i].stem(
                0.1 * np.arange(len(self.tello_sm.userdata.monitor[sensor_flags[i]])),
                self.tello_sm.userdata.monitor[sensor_flags[i]],
                linefmt='r--', markerfmt='ro', bottom=0
            );
            bline.set_color('r');
            mline.set_markerfacecolor('none');
            
            axes[i].set_ylabel(sensor_flags[i]);
            axes[i].grid();
        axes[3].set_xlabel('time(s)');
        axes[3].set_ylim(0, 1);


        plt.tight_layout();
        plt.savefig(fpath + 'sensor_flag_0'+get_current_time()+'.svg');

        #############################3# plot planner flags
        fig, axes = plt.subplots(3, 1, sharey=True);
        planner_flags = ['wp_debug', 'wp_fixed', 'traj_smpl'];
        for i in range(len(planner_flags)):
            mline, sline, bline = axes[i].stem(
                0.1 * np.arange(len(self.tello_sm.userdata.monitor[planner_flags[i]])),
                self.tello_sm.userdata.monitor[planner_flags[i]],
                linefmt='m--', markerfmt='mo', bottom=0
            );
            bline.set_color('m');
            mline.set_markerfacecolor('none');
            
            axes[i].set_ylabel(planner_flags[i]);
            axes[i].grid();
        axes[2].set_xlabel('time(s)');
        axes[2].set_ylim(0, 1);


        plt.tight_layout();
        plt.savefig(fpath + 'planner_flag_'+get_current_time()+'.svg');

        #######################################3 plot exp flow flags
        fig, axes = plt.subplots(6, 1, sharey=True);
        exp_flags = ['is_land', 'is_takeoff', 'is_em', 'is_EOT', 'is_SOT', 'exp_mode'];
        for i in range(len(exp_flags)):
            mline, sline, bline = axes[i].stem(
                0.1 * np.arange(len(self.tello_sm.userdata.monitor[exp_flags[i]])),
                self.tello_sm.userdata.monitor[exp_flags[i]],
                linefmt='c--', markerfmt='co', bottom=0
            );
            bline.set_color('c');
            mline.set_markerfacecolor('none');
            
            axes[i].set_ylabel(exp_flags[i]);
            axes[i].grid();
        axes[5].set_xlabel('time(s)');
        axes[5].set_ylim(0, 1);

        plt.tight_layout();
        plt.savefig('exp_status_flag_'+get_current_time()+'.svg');

        plt.show();

    
    def execute_viz(self):
        # visualization
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('tello_smach_test', self.tello_sm, '/TELLO_SM_ROOT')
        sis.start()
        # Execute SMACH plan
        outcome = self.tello_sm.execute();
        print(outcome);
        # print(self.tello_sm.userdata.monitor);

        sis.stop();
        self.plot_flags();


def get_current_time():
    ## add date and time
    from datetime import date, datetime;
    timer = date.today();
    time_stamp = str(timer);
    timer = datetime.now().time();
    timer = timer.strftime("%H-%M-%S")
    time_stamp = time_stamp + "-" + timer;
    return time_stamp;

'''
@TODO:
    define state Resting
    takeoff     -> Hovering
    idel_rest   -> Resting
    terminate   -> DONE
    redo_rest   -> Hovering


    from Resting to Hovering
        count 10 sec and takeoff
        redo case
    from Hovering to Resting
        use input to determine re-do testing or not


    redo case:
        msg flow
            Resting -> Hovering -> Testing -> Resting
'''
class Resting(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['takeoff', 'idle_rest', 'terminate', 'redo_rest'],
            input_keys=['redo_flag_in', 'monitor_in'],
            output_keys=['redo_flag_out', 'monitor_out']);

        self.count = 0;
        self.is_takeoff = False;
        self.is_land = True;

        # add the ros sub and pub
        self.rate = rospy.Rate(10);

        self.takeoff_pub = rospy.Publisher('/tello/takeoff', Empty, queue_size=1);

        rospy.Subscriber('/tello/land', Empty, callback=self.myTelloLandCb, queue_size=1);


    def myTelloLandCb(self, msg):
        if not self.is_land:
            self.is_land = True;
        rospy.loginfo("========[Hovering]========\nland msg received");
    
    '''
    @TODO:
        update the monitor data
        keys of the data is specified with states
        if check_keys = True:
            only do checking keys
        else:
            append data
    '''
    def update_status(self, ud, check_keys=False):
        if check_keys:
            # check the containers and record flags
            ud_keys = ud.monitor_in.keys();
            if 'redo_rest_in' not in ud_keys:
                ud.monitor_in.update({'redo_rest_in':   []});
            # if 'redo_rest_out' not in ud_keys:
            #     ud.monitor_in.update({'redo_rest_out':  []})
            if 'is_land' not in ud_keys:
                ud.monitor_in.update({'is_land':        []});
            if 'is_takeoff' not in ud_keys:
                ud.monitor_in.update({'is_takeoff':     []});
        else:
            # update the state
            ud.monitor_in['redo_rest_in'].append(int(ud.redo_flag_in));
            # ud.monitor_in['redo_rest_out'].append(int(ud.redo_flag_out));
            ud.monitor_in['is_land'].append(int(self.is_land));
            ud.monitor_in['is_takeoff'].append(int(self.is_takeoff));
            ud.monitor_out = ud.monitor_in;

    '''
    @TODO:
        takeoff after 10 second counted
    '''
    def execute(self, ud):

        while not rospy.is_shutdown():
            self.rate.sleep();

            self.update_status(ud=ud, check_keys=True);

            if ud.redo_flag_in:
                self.is_land = False;
                self.is_takeoff = False;
                # close the redo loop
                ud.redo_flag_out = False;
                ud.redo_flag_in = False;
                self.count = 0;
                
                # update the state
                self.update_status(ud=ud, check_keys=False);
            else:
                # initialize the redo flag out
                ud.redo_flag_out = False;

                if not self.is_takeoff:
                    # counting and takeoff
                    if self.count < 50:
                        self.count += 1;

                        # update the state
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_rest';
                    else:
                        # takeoff
                        rospy.loginfo("========[Resting]========\nTAKEOFF!");
                        self.takeoff_pub.publish(Empty());
                        self.is_takeoff = True;
                        self.is_land = False;
                        # clear the counter
                        self.count = 0;
                        # update the state
                        self.update_status(ud=ud, check_keys=False);
                        return 'takeoff';
                else:
                    if self.is_land:
                        # hovering -> landing
                        # check if re-exp is required
                        # hang the main thread and take input
                        # DO NOT set is_takeoff = False
                        # if do so the script would start counting
                        rospy.loginfo("========[Resting]========\nRedo the experiment?\t(Y/N/None)");
                        tmp = input();

                        if tmp == 'Y':
                            # redo the exp
                            ud.redo_flag_out = True;

                            # update the state
                            self.update_status(ud=ud, check_keys=False);
                            return 'redo_rest';

                        elif tmp == 'N':
                            # terminate the whole process
                            rospy.loginfo("========[Resting]========\nDONE!");
                            # update the state
                            self.update_status(ud=ud, check_keys=False);
                            return 'terminate';
                            
                        else:
                            # idle again
                            # update the state
                            self.update_status(ud=ud, check_keys=False);
                            return 'idle_rest';

'''
@TODO:
    define state Hovering
    landing     -> Resting
    emergency   -> Emergency
    idle_hover  -> Hovering
    test        -> Testing
    redo_hover  -> Testing

    from Hovering to Resting
        publish landing topic
    from Hovering to Testing
        publish test topic
        redo case
    from Testing to Hovering
        receive EOT topic and publish landing topic
'''
class Hovering(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['landing', 'test', 'emergency', 'idle_hover', 'redo_hover'],
            input_keys=['redo_flag_in', 'monitor_in'],
            output_keys=['redo_flag_out', 'monitor_out']);

        self.is_takeoff = False;
        self.is_em = False;
        self.is_EOT = False;
        self.count = 0;

        # add the ros sub and pub
        self.rate = rospy.Rate(10);

        rospy.Subscriber('/tello/takeoff', Empty, callback=self.myTelloTakeoffCb, queue_size=1);
        rospy.Subscriber('/tello/emergency', Empty, callback=self.myTelloEMCb, queue_size=1);
        rospy.Subscriber('/EOT', Empty, callback=self.myTelloEOTCb, queue_size=1);

        # self.SOT_pub = rospy.Publisher('/SOT', Empty, queue_size=1);
        self.land_pub =  rospy.Publisher('/tello/land', Empty, queue_size=1);

    '''
    @TODO:
        update the monitor data
        keys of the data is specified with states
        if check_keys = True:
            only do checking keys
        else:
            append data
    '''
    def update_status(self, ud, check_keys=False):
        if check_keys:
            # check the containers and record flags
            ud_keys = ud.monitor_in.keys();
            if 'redo_hover_in' not in ud_keys:
                ud.monitor_in.update({'redo_hover_in':  []});
            # if 'redo_hover_out' not in ud_keys:
            #     ud.monitor_in.update({'redo_hover_out': []});
            if 'is_em' not in ud_keys:
                ud.monitor_in.update({'is_em':          []});
            if 'is_takeoff' not in ud_keys:
                ud.monitor_in.update({'is_takeoff':     []});
            if 'is_EOT' not in ud_keys:
                ud.monitor_in.update({'is_EOT':         []});
        else:
            # update the state
            ud.monitor_in['redo_hover_in'].append(int(ud.redo_flag_in));
            # ud.monitor_in['redo_hover_out'].append(int(ud.redo_flag_out));
            ud.monitor_in['is_em'].append(int(self.is_em));
            ud.monitor_in['is_takeoff'].append(int(self.is_takeoff));
            ud.monitor_in['is_EOT'].append(int(self.is_EOT));
            ud.monitor_out = ud.monitor_in;

    def myTelloTakeoffCb(self, msg):
        if not self.is_takeoff:
            self.is_takeoff = True;
            rospy.loginfo("========[Hovering]========\ntakeoff msg received");

    def myTelloEMCb(self, msg):
        self.is_em = True;
        rospy.loginfo("========[Hovering]========\nEMERGENCY msg received!!\nSHOTDOWN");

    def myTelloEOTCb(self, msg):
        if not self.is_EOT:
            self.is_EOT = True;
            rospy.loginfo("========[Hovering]========\nEnd Of Test msg received");

    '''
    @TODO:
        takeoff and waiting for the SOT command to start testing
    '''
    def execute(self, ud):
        # debug
        # print(ud.monitor_in);

        while not rospy.is_shutdown():
            self.rate.sleep();

            self.update_status(ud=ud, check_keys=True);

            # check if it is redo
            if ud.redo_flag_in:
                self.is_EOT = False;
                self.is_takeoff = False;
                ud.redo_flag_out = True;
                # update and record data
                self.update_status(ud=ud, check_keys=False);
                return 'redo_hover';
            else:
                # initialize the redo_flag_out
                ud.redo_flag_out = False;

                if self.is_em:
                    # update and record data
                    self.update_status(ud=ud, check_keys=False);
                    return 'emergency';

                elif not self.is_takeoff:
                    # update and record data
                    self.update_status(ud=ud, check_keys=False);
                    return 'landing';

                elif not self.is_EOT:
                    # wait for 5 second
                    if self.count < 50:
                        self.count += 1;
                        # update and record data
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_hover';
                    else:
                        rospy.loginfo("========[Hovering]========\nTransition to testing state");
                        self.count = 0;
                        # update and record data
                        self.update_status(ud=ud, check_keys=False);
                        return 'test';

                elif self.is_EOT:
                    rospy.loginfo("========[Hovering]========\nLanding");
                    self.land_pub.publish(Empty());
                    self.is_takeoff = False;
                    # update and record data
                    self.update_status(ud=ud, check_keys=False);
                    return 'landing';

'''
@TODO:
    the state doing experiment content
    setup all the sensors and subscribe to them

    emergency       -> emergency
    eot             -> Hovering
    idle_test       -> Testing
    redo_testing    -> Resting
'''
class Testing(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['eot', 'emergency', 'idle_test', 'redo_test'],
            input_keys=['redo_flag_in', 'monitor_in'],
            output_keys=['redo_flag_out', 'monitor_out']);

        # flight control
        self.is_SOT = False;
        self.is_EOT = False;
        self.is_em = False;
        # sensors
        self.aruco_corner_flag = False;
        self.aruco_np_flag = False;
        self.imu_np_flag = False;
        self.kf_np_flag = False;
        # planners
        self.wp_debug_flag = False;
        self.wp_fixed_flag = False;
        self.traj_smpl_flag = False;
        # trackers
        self.exp_mode_flag = False;

        self.rate = rospy.Rate(10);
        
        # rospy.Subscriber('/SOT', Empty, callback=self.myTelloSOTCb, queue_size=1);
        rospy.Subscriber('/EOT', Empty, callback=self.myTelloEOTCb, queue_size=1);
        rospy.Subscriber('/tello/emergency', Empty, callback=self.myTelloEMCb, queue_size=1);
        
        # sensors
        rospy.Subscriber('aruco_corners', PointArr, callback=self.myPointArrCb, queue_size=1);
        rospy.Subscriber('aruco_pose_numpy', numpy_msg(Floats), callback=self.myPoseNpCb, queue_size=1);
        rospy.Subscriber('imu_numpy', numpy_msg(Floats), callback=self.myImuNpCb, queue_size=1);
        rospy.Subscriber('tello_pose_kf', numpy_msg(Floats), callback=self.myKFCb, queue_size=1);
        
        # planners
        # testing from input
        # rospy.Subscriber('tello_waypoint', Point, callback=self.myTelloWpCb, queue_size=1);
        # testing by fixed point in xml file
        rospy.Subscriber('/fixed_wp/wp_pub_test', PoseArray, callback=self.myFixedWpCb, queue_size=1);
        # trajectory generated from the fixed point array
        rospy.Subscriber('/desired_state', MultiDOFJointTrajectoryPoint, callback=self.myTrajCb, queue_size=1);
        rospy.Subscriber('/static_planner', Empty, callback=self.myStaticTrajCb, queue_size=1);

        # trackers
        # rospy.Subscriber('/exp_mode/tracking', Empty, callback=self.myExpModeCb, queue_size=1);
        rospy.Subscriber('/exp_mode/debug', Empty, callback=self.myExpModeCb, queue_size=1);

        # publisher
        self.SOT_pub = rospy.Publisher('/SOT', Empty, queue_size=1);

    '''
    @TODO:
        update the monitor data
        keys of the data is specified with states
        if check_keys = True:
            only do checking keys
        else:
            append data
    '''
    def update_status(self, ud, check_keys=False):
        if check_keys:
            # check the containers and record flags
            ud_keys = ud.monitor_in.keys();
            if 'redo_test_in' not in ud_keys:
                ud.monitor_in.update({'redo_test_in':   []});
            # if 'redo_test_out' not in ud_keys:
            #     ud.monitor_in.update({'redo_test_out':  []});
            if 'is_em' not in ud_keys:
                ud.monitor_in.update({'is_em':          []});
            if 'is_SOT' not in ud_keys:
                ud.monitor_in.update({'is_SOT':         []});
            if 'is_EOT' not in ud_keys:
                ud.monitor_in.update({'is_EOT':         []});
            # sensor
            if 'aruco_corner' not in ud_keys:
                ud.monitor_in.update({'aruco_corner':   []});
            if 'aruco_np' not in ud_keys:
                ud.monitor_in.update({'aruco_np':       []});
            if 'imu_np' not in ud_keys:
                ud.monitor_in.update({'imu_np':         []});
            if 'kf_np' not in ud_keys:
                ud.monitor_in.update({'kf_np':          []});
            # planners
            if 'wp_debug' not in ud_keys:
                ud.monitor_in.update({'wp_debug':       []});
            if 'wp_fixed' not in ud_keys:
                ud.monitor_in.update({'wp_fixed':       []});
            if 'traj_smpl' not in ud_keys:
                ud.monitor_in.update({'traj_smpl':      []});
            # trackers
            if 'exp_mode' not in ud_keys:
                ud.monitor_in.update({'exp_mode':       []});
            
        else:
            # update the state
            ud.monitor_in['redo_test_in'].append(int(ud.redo_flag_in));
            # ud.monitor_in['redo_test_out'].append(int(ud.redo_flag_out));
            ud.monitor_in['is_em'].append(int(self.is_em));
            ud.monitor_in['is_SOT'].append(int(self.is_SOT));
            ud.monitor_in['is_EOT'].append(int(self.is_EOT));
            # sensor
            ud.monitor_in['aruco_corner'].append(int(self.aruco_corner_flag));
            ud.monitor_in['aruco_np'].append(int(self.aruco_np_flag));
            ud.monitor_in['imu_np'].append(int(self.imu_np_flag));
            ud.monitor_in['kf_np'].append(int(self.kf_np_flag));
            # planners
            ud.monitor_in['wp_debug'].append(int(self.wp_debug_flag));
            ud.monitor_in['wp_fixed'].append(int(self.wp_fixed_flag));
            ud.monitor_in['traj_smpl'].append(int(self.traj_smpl_flag));
            # trackers
            ud.monitor_in['exp_mode'].append(int(self.exp_mode_flag));

            ud.monitor_out = ud.monitor_in;

    def myTelloEMCb(self, msg):
        self.is_em = True;
        rospy.loginfo("========[Testing]========\nEMERGENCY msg received!!\nSHOTDOWN");

    def myTelloEOTCb(self, msg):
        self.is_EOT = True;
        rospy.loginfo("========[Testing]========\nEnd Of Test msg received");

    def myPointArrCb(self, msg):
        if not self.aruco_corner_flag:
            rospy.loginfo("========[Testing]========\nArUco setup");
        self.aruco_corner_flag = True;

    def myPoseNpCb(self, msg):
        if not self.aruco_np_flag:
            rospy.loginfo("========[Testing]========\nArUco pose in numpy setup");
        self.aruco_np_flag = True;

    def myImuNpCb(self, msg):
        if not self.imu_np_flag:
            rospy.loginfo("========[Testing]========\nIMU in numpy setup");
        self.imu_np_flag = True;

    def myKFCb(self, msg):
        if not self.kf_np_flag:
            rospy.loginfo("========[Testing]========\nKalman Filter setup");
        self.kf_np_flag = True;

    def myTelloWpCb(self, msg):
        if not self.wp_debug_flag:
            rospy.loginfo("========[Testing]========\nWaypoint debugger set up");
        self.wp_debug_flag = True;

    def myFixedWpCb(self, msg):
        if not self.wp_fixed_flag:
            rospy.loginfo("========[Testing]========\nFixed waypoint set up");
        self.wp_fixed_flag = True;

    def myTrajCb(self, msg):
        if not self.traj_smpl_flag:
            rospy.loginfo("========[Testing]========\nTrajectory generation set up");
        self.traj_smpl_flag = True;
    
    def myStaticTrajCb(self, msg):
        if not self.traj_smpl_flag:
            rospy.loginfo("========[Testing]========\nStatic trajectory generation set up");
        self.traj_smpl_flag = True;

    def myExpModeCb(self, msg):
        if not self.exp_mode_flag:
            rospy.loginfo("========[Testing]========\nExp mode tracking is set up");
        self.exp_mode_flag = True;

    '''
    @TODO:
        check each sensor is working or not
        check planner is working or not
        check exp is done or not
    '''
    def execute(self, ud):
        
        while not rospy.is_shutdown():
            self.rate.sleep();

            self.update_status(ud=ud,check_keys=True);

            if ud.redo_flag_in:
                # reset the flags for flying
                self.is_EOT = False;
                self.is_SOT = False;

                self.aruco_corner_flag = False;
                self.aruco_np_flag = False;
                self.imu_np_flag = False;
                self.kf_np_flag = False;

                self.wp_debug_flag = False;
                self.wp_fixed_flag = False;
                self.traj_smpl_flag = False;

                self.exp_mode_flag = False;

                ud.redo_flag_out = True;

                self.update_status(ud=ud, check_keys=False);
                return 'redo_test';
            else:
                # initialize the redo_flag_out
                ud.redo_flag_out = False;
                
                if self.is_em:
                    self.update_status(ud=ud, check_keys=False);
                    return 'emergency';

                # elif not self.is_SOT:
                #     rospy.loginfo("========[Testing]========\nno SOT msg received");
                #     return 'eot';
                elif self.is_EOT:
                    rospy.loginfo("========[Testing]========\nEnd Of Testing!");
                    self.update_status(ud=ud, check_keys=False);
                    return 'eot';

                # elif self.is_SOT and not self.is_EOT:
                elif not self.is_EOT:
                    # TODO:
                    #       check current state
                    #       open the sensor nodes and send back checking
                    #       open the planner nodes and send back checking
                    #       open the exp node and send back checking

                    sensor_setup = self.aruco_corner_flag and self.aruco_np_flag and self.imu_np_flag and self.kf_np_flag;
                    # planner_setup = self.wp_debug_flag or (self.wp_fixed_flag and self.traj_smpl_flag);
                    # planner_setup = self.traj_smpl_flag;        # for testing static trajectory
                    planner_setup = True;                         # for testing velocity performance
                    exp_setup = self.exp_mode_flag;

                    if not sensor_setup:
                        rospy.loginfo("========[Testing]========\nSensor is not yet set up......");
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_test';
                    elif not planner_setup:
                        rospy.loginfo("========[Testing]========\nPlanner is not yet set up......");
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_test';
                    elif not exp_setup:
                        rospy.loginfo("========[Testing]========\nExp node is not yet set up......");
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_test';
                    else:
                        if self.wp_debug_flag:
                            rospy.loginfo("========[Testing]========\nWaypoint debug mode exp");
                        elif (self.wp_fixed_flag and self.traj_smpl_flag):
                            rospy.loginfo("========[Testing]========\nTrajectory generation mode exp");
                        # testing the static planning
                        elif (self.traj_smpl_flag):
                            rospy.loginfo("========[Testing]========\nStatic trajectory generation mode exp");

                        if not self.is_SOT:
                            rospy.loginfo("========[Testing]========\nStart Of Testing!");
                            self.SOT_pub.publish(Empty());
                            self.is_SOT = True;
                            
                        self.update_status(ud=ud, check_keys=False);
                        return 'idle_test';
                    


# main
def test():
    # for tesitng
    rospy.init_node('tello_monitor_node');
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['emergency', 'terminate'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add(
            'Resting', Resting(), transitions={
                'takeoff':      'Hovering',
                'idle_rest':    'Resting',
                'terminate':    'terminate'
        });
        smach.StateMachine.add(
            'Hovering', Hovering(), transitions={
                'landing':      'Resting',
                'emergency':    'emergency',
                'idle_hover':   'Hovering',
                'sot':          'Testing'
        });
        smach.StateMachine.add(
            'Testing', Testing(), transitions={
                'emergency':    'emergency',
                'eot':          'Hovering',
                'idle_test':    'Testing',
        });
    
    
    # visualization
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('smach_test', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute();
    print(outcome);

    sis.stop()


if __name__ == '__main__':
    # test()
    monitor = TelloMonitor();
    monitor.execute_viz();

    