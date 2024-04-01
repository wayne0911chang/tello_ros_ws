#ifndef QUAT_ESKF_H
#define QUAT_ESKF_H

// ros-dep
# include <ros/ros.h>
# include <tf/tf.h>
// # include <image_transport/image_transport.h>
# include <std_msgs/Empty.h>
// debug
# include <std_msgs/Float64MultiArray.h>
# include <geometry_msgs/PoseArray.h>
# include <geometry_msgs/Pose.h>
// # include <geometry_msgs/Point.h>
# include <nav_msgs/Odometry.h>
# include <sensor_msgs/Imu.h>
# include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
# include <tf/transform_datatypes.h>
# include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// for self-defined msg
// #include "tello_sub/PointArr.h"
#include "tello_sub/ArUcoPoseArr.h"

// Eigen
# include <eigen3/Eigen/Dense>
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>

# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

// std
# include <stdio.h>
# include <iostream>
// # include <typeinfo>    // for debugging
# include <vector>
# include <string>

// utils and parameters
# include "tello_pub/math_utils.h"


/*=========================== global variables =========================*/
enum types {
    DRONE_STATE_0,
    DRONE_STATE_1,
    DRONE_STATE_2,
    TARGET_STATE_0,
    TARGET_STATE_2,
    TARGET_FACE,
    TARGET_ODOM
};

// define parameters
#define T_SMPL double(0.01);

/*====================[LKF]=========================================*/
/*
@LinearKalmanFilter
    produce a constant acceleration linear KF with noisy acceleration
    for the given topic of state
    the available measurement contains
        position from marker
        velocity from odom
        acceleration from IMU

        drone marker => ros_ns + "/aruco_pose_arr"
        target face  => ros_ns + "/face_pose_arr"
*/
class LinearKalmanFilter
{
    // ros-dep
    // ros::NodeHandle nh_;

    // pose, vel, acc
    ros::Subscriber pose_sub, odom_sub, imu_sub;
    ros::Publisher state_pub;
    ros::Time startTimeKF;
    ros::Timer predictStateTimer;
    
    // process control for testing
    bool StartOfTest;
    ros::Subscriber SOT_sub;

    // KF settings
    double t_pred;
    double nAccVar, nPositionVar, nVelVar, nAngVelVar;
    int stateSize, measureSize, measureSizeAng;

    geometry_msgs::Quaternion q_prevMarker, q_prevFace, q_prevImu, q_prevYOLO;
    geometry_msgs::Quaternion q_initImu, q_initYOLO;
    // Eigen::Vector3d p_prev;

    // parameters
    std::vector<int > wall_ids;
    std::vector<int > board_ids;
    int desiredArUcoID;
    int myType;
    bool JosephForm;
    double quatThr;
    
public:
    // try all public
    Eigen::VectorXd state;
    Eigen::MatrixXd F_, Q_, P_;                 // state transition matrix, process noise gain matrix
    Eigen::MatrixXd H_p, H_v, H_imu;
    Eigen::MatrixXd R_p, R_v, R_a;
    Eigen::MatrixXd kalmanGain;

    Eigen::Matrix3d omegaHat;
    Eigen::Matrix3d R_imu_to_cam;
    /*
    @LinearKalmanFilter
        empty constructor for overload
    */
    LinearKalmanFilter(){}
    ~LinearKalmanFilter(){}
    /*
    @LinearKalmanFilter
        state: 
            position, velocity, acceleration
            orientation, angular velocity
        setup the process equation, the measurement equation
        
        the predict process follows the timer event
        the measurement update follows the subscriber
    */
    LinearKalmanFilter(int targetType, bool useJoseph, ros::NodeHandle& nh_);
    /*
    @getStateSize
        return the size of state as int
    */
    int getStateSize(){ return stateSize; }
    /*
    @getErrCov
        return the err cov P_ as Eigen::MatrixXd
    */
    Eigen::MatrixXd getErrCov(){ return P_.eval(); }
    /*
    @getKalmanGain
        return the kalmanGain as Eigen::MatrixXd
    */
    Eigen::MatrixXd getKalmanGain(){ return kalmanGain.eval(); }
    /*
    @getStateTransit
        return the F_ as Eigen::MatrixXd
    */
    Eigen::MatrixXd getStateTransit(){ return F_.eval(); }
    /*
    @getProcessNoiseCov
        return Q_ as Eigen::MatrixXd
    */
    Eigen::MatrixXd getProcessNoiseCov(){ return Q_.eval(); }
    /*
    @getCam0toMarker
        Give the rotation matrix only
        for the drone pose 1 => desiredArUcoID = 25
        for the drone pose 2 => desiredArUcoID = 22
    */
    void getCam0toMarker(Eigen::Matrix3d& R_m_to_c0);
    /*
    @getCamInitToHumanInit
        camera initial frame to the leader's frame
    */
    void getCamInitToHumanInit(Eigen::Matrix3d& R_c_init_to_L_init);
    /*
    @getOrientationToInit
        for drone 2 and drone 1
        get the filtered orientation relative to initial condition 
    */
    void getOrientationToCamInit(Eigen::Matrix3d& R_c_to_init);
    /*
    @telloSOTCallback
        trigger the initialization of timer and start flag
        only when SOT is false
    */
    void telloSOTCallback(std_msgs::Empty const& msg);
    /*
    @publishPosteriorState
        publish the posterior estimated state
    */
    void publishPosteriorState();
    /*
    @publishPosteriorStateFace
        publish the posterior estimated state for face pose
    */
    void publishPosteriorStateFace();
    /*
    @ArUcoPoseArrCB
        update the aurco pose with received pose array
        and then publish the posterior estimation w.r.t. pose as
            trajectory_msgs::MultiDOFJointTrajectoryPoint 
    */
    void ArUcoPoseArrCB(tello_sub::ArUcoPoseArr const& arucoPoseArr);
    /*
    @FacePoseArrCB
        update the face pose 
        and publish the posterior state w.r.t. face pose measurement as
            geometry_msgs::Pose
    */
    void FacePoseArrCB(geometry_msgs::PoseArray const& poseArr);
    /*
    @YOLOPoseArrCB
        update the human pose from yolo 
        no need to publish the state
    */
    void YOLOPoseArrCB(geometry_msgs::PoseArray const& poseArr);
    /*
    @odomCB
        update the vel with received pose array
    */
    void odomCB(nav_msgs::Odometry const& odom);
    /*
    @imuCB
        update the acc, yaw, yaw rate with received pose array
    */
    void imuCB(sensor_msgs::Imu const& imu);
    /*
    @predictState
        publish the last posterior state update
        compute the prior state and prior error cov. only
    */
    void predictState(ros::TimerEvent const& ev);
    /*
    @estimateDroneOdom
        estimate the drone0 odometry
    */
    void estimateDroneOdom();
};


/*=============================[quaternion-based eskf]==========================*/
class ErrorStateKalmanFilter
{
    /*======================================[ros-dep]====================*/
    // sub and pub
    ros::Subscriber imu_sub, odom_sub, marker_sub, SOT_sub;
    ros::Publisher eskf_pub;
    int queue_size = 1;

    // debug 
    ros::Publisher covTr_pub;



    // timing
    ros::Timer predictStateTimer;
    ros::Time startTimeEskf;
    double t_pred = T_SMPL;
    
    // process
    bool StartOfTest = false;

    /*===========================[parameters]=================================*/
    std::string droneName;
    int desiredArUcoID;

    int stateSize = 10;         // pos, vel, orientation in quat
    int errStateSize = 9;       // pos, vel, rvec
    int inputSize = 6;          // lin acc, ang vel
    double nAngVelVar = 1e-6;   // rad^2 s^-2
    double nAccVar = 1e-6;      // m^2 s^-4 --> g^2
    double nVelVar = 1e-6;      // m^2 s^-2

    // nominal state
    Eigen::VectorXd nominalState, imuInput;     // nominalState = [p, v]
    Eigen::Quaterniond nominalQuat;
    
    // error state 
    Eigen::VectorXd errState;                   // error state mean = [dp, dv, dtheta]
    Eigen::MatrixXd errCov_, nProcCov_;
    Eigen::Matrix3d nGyroCov_;                  // rvec measurement noise cov
    Eigen::Matrix3d nOdomCov_;

    // constant Jacobians
    Eigen::Quaterniond quatGyro2Iner;           // the transformation quat
    Eigen::MatrixXd jacobErrStateRvec;          // errRvec w.r.t. errState
    Eigen::MatrixXd nF_;                        // noise state transit Jacob

    // frame transform
    Eigen::Matrix3d gyroToOdom;                 // gyro to oom

    // KF components
    Eigen::MatrixXd kalmanGain;

    /*=============================[functions]==================================*/
    /*====[callbacks]====*/
    /*
    @telloSOTCallback
        trigger the initialization of timer and start flag
    */
    void telloSOTCallback(std_msgs::Empty const& msg);
    /*
    @imuCB
        input
        imu input (acc, ang vel) to Eigen::Vector3d
        
        measurement
        correction and injection
        imu orientation to Eigen::Quaterniond
    */
    void imuCB(sensor_msgs::Imu const& imu);
    /*
    @odomCB
        measurement
        correction and injection
        reset...?
        odom velocity to Eigen::Vector3d
    */
   void odomCB(nav_msgs::Odometry const& odom);
    /*
    @ArUcoPoseArrCB
        measurement
        correction and injection
        aruco relative position to Eigen::Vector3d
    */
    void ArUcoPoseArrCB(tello_sub::ArUcoPoseArr const& arucoPoseArr);
public:

    /*========[constructor destructor]========*/
    /*
    @ErrorStateKalmanFilter
        empty constructor 
    */ 
    ErrorStateKalmanFilter(){};
    /*
    @ErrorStateKalmanFilter
        normal constructor with nodehandles
        initialize the subs, pubs, and timers
    */
    ErrorStateKalmanFilter(int drone_id, ros::NodeHandle& nh_);
    /*
    @~ErrorStateKalmanFilter
        destructor
    */
    ~ErrorStateKalmanFilter(){};
    /*========[eskf functions]========*/
    /*
    @predictState
        publish the previous posterior states
        predict the nominal state with DT kinematics
        predict the error state with DT kinematics (zero mean)
        propagate the error covariance on of error state
    */
    void predictState(ros::TimerEvent const& ev);
    /*
    @stateTransitJacob
        find the state transition Jacobian for error states
    */
    void stateTransitJacob(Eigen::MatrixXd& errF_);
    /*
    @true2errJacob
        find the true-state to error-state Jacobian
    */
    Eigen::MatrixXd true2errJacob();
    /*
    @injectNominal
        adopt the group composition for the state
    */
    void injectNominal();
    /*
    @resetErrState
        reset error state to 0
        calculate the Jacobian
    */
    void resetErrState();
    /*
    @publishTrueState
        publish the mean of the true state as nominal state
            if nominal state is corrected and injected ==> then being true state
            if not ==> then just nominal prediction
    */
   void publishTrueState();
};

 
#endif