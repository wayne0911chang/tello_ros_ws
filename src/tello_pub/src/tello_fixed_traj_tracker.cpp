// std-lib
# include <iostream>
# include <string>
# include <vector>
// ros-dep
# include <ros/ros.h>
# include <tf/tf.h>
# include <std_msgs/Empty.h>
# include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
# include <geometry_msgs/Accel.h>
// Eigen
# include <eigen3/Eigen/Dense>
// # include <eigen3/Eigen/Core>
// # include <eigen3/Eigen/Geometry>
# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

// math utils
# include <tello_pub/math_utils.h>

class FixedTrajTracker
{
    /*================[sub and pub]==========================*/
    ros::Subscriber eskf_sub, eskf_bias_sub, SOT_sub;
    ros::Publisher  ref_vel_pub, land_pub, takeoff_pub;
    ros::Time       trackerStartTime;
    ros::Timer      trackerTimer;

    /*=================[param]===============================*/
    std::string     ros_ns;
    std::string     ros_nodeName;

    bool            useESKF = true;
    bool            useOpLCmd = true;
    bool            rotated_traj = false;
    
    double          smpl_time, square_side_length, traj_height, cmd_vel_y, cmd_vel_x, cmd_angvel_z;

    // SOT, wp0 -> wp1, wp1 -> wp2, wp2 -> wp3, wp3 -> wp0, EOT
    bool            smach_flag[6] = {false, false, false, false, false, false};

    int             state_counter = 0;
    int             opl_state_counter = 0;

    Eigen::Vector3d     droneAngVel, biasAngVel;
    Eigen::Quaterniond  quaternion_des, droneQuat, droneQuatInit;
    Eigen::VectorXd     dronePosVel;
    

    /*=================[func]=================================*/
    /*
    @telloSOTCb
        set the start flag
    */
    void telloSOTCb(std_msgs::Empty const& msg){
        if (!smach_flag[0]){
            smach_flag[0] = true;
            takeoff_pub.publish(std_msgs::Empty());
            trackerStartTime = ros::Time::now();
            trackerTimer.start();
        }
    }
    /*
    @eskfCb
        get drone state
    */
    void eskfCb(
        const trajectory_msgs::MultiDOFJointTrajectoryPoint& traj_msg 
    ){
        dronePosVel << traj_msg.transforms[0].translation.x,
                       traj_msg.transforms[0].translation.y,
                       traj_msg.transforms[0].translation.z,
                       traj_msg.velocities[0].linear.x,
                       traj_msg.velocities[0].linear.y,
                       traj_msg.velocities[0].linear.z;
        tf::quaternionMsgToEigen(traj_msg.transforms[0].rotation, droneQuat);
        droneAngVel << traj_msg.velocities[0].angular.x,
                       traj_msg.velocities[0].angular.y,
                       traj_msg.velocities[0].angular.z;
    }
    /*
    @eskfBiasCb
        get bias for calculate the ang vel.
    */
    void eskfBiasCb(geometry_msgs::Accel const& bias_msg){
        biasAngVel << bias_msg.angular.x,
                      bias_msg.angular.y,
                      bias_msg.angular.z;   
    }
    /*
    @cmdTimerCb
        call the waypointGenerator to have current waypoint
        publish reference velocity command by the controller
        update the smach according to the error
    */
    void cmdTimerCb(ros::TimerEvent const& ev){
        
        // check SOT
        if (smach_flag[0]){
            // get the pose waypoint
            Eigen::Vector3d p_des, err_p, err_q;
            Eigen::Quaterniond q_des;
            waypointGenerator(p_des, q_des);

            // get the error, orientation in rotation matrix
            err_p = dronePosVel.head(3) - p_des;
            err_q = quaterniondToRvec(q_des * droneQuat.conjugate());
            
            // TODO:
            //      use rotation matrix instead
            //      derive the quaternion geometric controller yourself...?
        }
    }
    /*
    @cmdTimerOpLCb
        give open loop command after raise to a desired height
    */
    void cmdTimerOplCb(ros::TimerEvent const& ev){
        if (smach_flag[0]){
            // TODO
            //      use the derived controller to track the desired height

            // Eigen::Vector3d err_p = dronePosVel.head(3) - Eigen::Vector3d(0, 0, traj_height);
            // Eigen::Vector3d err_q =
            
            // debug
            ROS_INFO_STREAM("========[fixed traj track]========\nopen loop cmd!");

            state_counter++;

            // wait for 5 sec
            if (state_counter > 5*(opl_state_counter+1)/smpl_time) { opl_state_counter++; }

            // determine the state and cmd
            geometry_msgs::Twist twist_msg;
            
            // check trajectory type
            if (!rotated_traj){
                switch (opl_state_counter){
                    case 1:
                        // positive y for fixed vel
                        twist_msg.linear.y = cmd_vel_y;
                        ref_vel_pub.publish(twist_msg);
                        break;

                    case 2:
                        // positive x for fixed vel
                        twist_msg.linear.x = cmd_vel_x;
                        ref_vel_pub.publish(twist_msg);
                        break;

                    case 3:
                        // negative y for fixed vel
                        twist_msg.linear.y = -cmd_vel_y;
                        ref_vel_pub.publish(twist_msg);
                        break;
                    
                    case 4:
                        // negative x for fixed vel
                        twist_msg.linear.x = -cmd_vel_x;
                        ref_vel_pub.publish(twist_msg);
                        break;

                    case 5:
                        // zero vel
                        ref_vel_pub.publish(twist_msg);
                        land_pub.publish(std_msgs::Empty());
                        break;

                    default:
                        break;
                }
            }
            // stop and rotation
            else {
                // rotate around the circle
                switch (opl_state_counter)
                {
                    case 0: 
                        break;
                    
                    case 100:
                        // zero velocity
                        ref_vel_pub.publish(twist_msg);
                        land_pub.publish(std_msgs::Empty());
                        break;

                    default:
                        // run 4 sec with fixed linear & angular vel.
                        twist_msg.linear.y = cmd_vel_y;
                        twist_msg.angular.z = cmd_angvel_z;
                        ref_vel_pub.publish(twist_msg);
                        break;
                }
            }
        }
    }
    /*
    @waypointGenerator
        generate the desired waypoint according to 
            current state from smach_flag
            square traj length from square_side_length
            rotated traj flag
    */
    void waypointGenerator(
        Eigen::Vector3d& p_des, Eigen::Quaterniond& quat_des
    ){
        if (!rotated_traj){
            // pure translation
            quat_des = droneQuatInit;

            if (smach_flag[0]) { p_des << 0.0, 0.0, traj_height; }
            if (smach_flag[1]) { p_des << 0.0, square_side_length, traj_height; }
            if (smach_flag[2]) { p_des << square_side_length, square_side_length, 1.0; }
            if (smach_flag[3]) { p_des << square_side_length, 0.0, traj_height; }
            if (smach_flag[4]) { p_des << 0.0, 0.0, traj_height; }
        }
        else if (rotated_traj){

            if (smach_flag[0]){
                // hover
                quat_des = droneQuatInit;
                p_des << 0.0, 0.0, traj_height;
            }
            if (smach_flag[1]){
                // rotate by 90 deg
                quat_des = Eigen::Quaterniond(
                    Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1))
                ) * droneQuatInit;
                p_des << 0.0, square_side_length, traj_height;
            }
            if (smach_flag[2]){
                // rotate by 180 deg
                quat_des = Eigen::Quaterniond(
                    Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0, 0, 1))
                ) * droneQuatInit;
                p_des << square_side_length, square_side_length, traj_height;
            }
            if (smach_flag[3]){
                // rotate by 270 deg
                quat_des = Eigen::Quaterniond(
                    Eigen::AngleAxisd(M_PI*3/2, Eigen::Vector3d(0, 0, 1))
                ) * droneQuatInit;
                p_des << square_side_length, 0.0, traj_height;
            }
            if (smach_flag[4]){
                // back to the origin and hover
                quat_des = droneQuatInit;
                p_des << 0.0, 0.0, traj_height;
            }
        }
    }
    /*
    @changeState
        change the state according to current state
    */
    void changeState(){
        // condition: SOT only
        if (
            smach_flag[0] && !smach_flag[1]
                          && !smach_flag[2] 
                          && !smach_flag[3]
                          && !smach_flag[4]
        ){ smach_flag[1] = true; }
        // condition: SOT & 1st wp
        else if (
            smach_flag[0] && smach_flag[1] && !smach_flag[2] 
                                           && !smach_flag[3]
                                           && !smach_flag[4]
        ){ smach_flag[1] = false; smach_flag[2] = true; }
        // condition: SOT & 2nd wp
        else if (
            smach_flag[0] && smach_flag[2] && !smach_flag[1] 
                                           && !smach_flag[3]
                                           && !smach_flag[4]
        ){ smach_flag[2] = false; smach_flag[3] = true; }
        // condition: SOT & 3rd wp
        else if (
            smach_flag[0] && smach_flag[3] && !smach_flag[1] 
                                           && !smach_flag[2]
                                           && !smach_flag[4]
        ){ smach_flag[3] = false; smach_flag[4] = true; }
        // condition: SOT & 4th wp == 1st wp
        else if (
            smach_flag[0] && smach_flag[4] && !smach_flag[1] 
                                           && !smach_flag[2]
                                           && !smach_flag[3]
        ){ smach_flag[4] = false; }
    }

public:
    /*
    default constructor & destructor
    */
    FixedTrajTracker(){}
    ~FixedTrajTracker(){}

    /*
    @FixedTrajTracker
    */
    explicit FixedTrajTracker(ros::NodeHandle& nh_){   

        // get the namespace
        ros_ns = ros::this_node::getNamespace();
        ros_nodeName = ros::this_node::getName();

        // debug
        std::cout << ros_ns << std::endl << ros_nodeName << std::endl;

        // get the rosparam
        nh_.getParam(ros_nodeName + "/useESKF", useESKF);
        nh_.getParam(ros_nodeName + "/opl_cmd", useOpLCmd);
        nh_.getParam(ros_nodeName + "/cmd_vel_y", cmd_vel_y);
        nh_.getParam(ros_nodeName + "/cmd_vel_x", cmd_vel_x);
        nh_.getParam(ros_nodeName + "/cmd_angvel_z", cmd_angvel_z);
        nh_.getParam(ros_nodeName + "/rotated_traj", rotated_traj);
        nh_.getParam(ros_nodeName + "/square_side_length", square_side_length);
        nh_.getParam(ros_nodeName + "/traj_height", traj_height);
        nh_.getParam(ros_nodeName + "/smpl_time", smpl_time);
        
        // debug
        std::cout << "params: \n" << useESKF << std::endl
                  << useOpLCmd << std::endl
                  << cmd_vel_y << std::endl
                  << cmd_vel_x << std::endl
                  << rotated_traj << std::endl
                  << square_side_length << std::endl
                  << traj_height << std::endl
                  << smpl_time << std::endl;

        // setting subscribers
        int buffer_size = 1;

        SOT_sub = nh_.subscribe(
            "/SOT", buffer_size, 
            &FixedTrajTracker::telloSOTCb, this
        );
        land_pub = nh_.advertise<std_msgs::Empty>(
            ros_ns + "/land", buffer_size
        );
        takeoff_pub = nh_.advertise<std_msgs::Empty>(
            ros_ns + "/takeoff", buffer_size
        );
        if (useESKF){
            eskf_sub = nh_.subscribe(
                ros_ns + "/eskf_state_full", buffer_size, 
                &FixedTrajTracker::eskfCb, this
            );
            eskf_bias_sub = nh_.subscribe(
                ros_ns + "/bias", buffer_size,
                &FixedTrajTracker::eskfBiasCb, this
            );
        }
        else {
            // TODO
            // use marker for drone pose instead...?
        }
        ref_vel_pub = nh_.advertise<geometry_msgs::Twist>(
            ros_ns + "/cmd_vel", buffer_size
        );
        if (useOpLCmd){
            trackerTimer = nh_.createTimer(
                ros::Duration(smpl_time), &FixedTrajTracker::cmdTimerOplCb, this, false
            );
        }
        else {
            trackerTimer = nh_.createTimer(
                ros::Duration(smpl_time), &FixedTrajTracker::cmdTimerCb, this, false
            );
        }
        // initialize parameters
        dronePosVel = Eigen::VectorXd::Zero(6);
        droneQuat = Eigen::Quaterniond::Identity();
        droneAngVel = Eigen::Vector3d::Zero(3);
        biasAngVel = Eigen::Vector3d::Zero(3);

        // assign the ideal quaternion orientation as 
        // x, y, z, w = (sqrt(2), sqrt(2), 0.0, 0.0)
    }
};

int main(int argc, char **argv){
    
    ros::init(argc, argv, "fixed_traj_tracker", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    FixedTrajTracker test(nh);
    ros::spin();

    return 0;
}