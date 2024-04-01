#ifndef FACEODOM_H
#define FACEODOM_H

// ros-dep and eigen-dep
# include "tello_pub/ros_eigen_dep.h"
# include "tello_pub/SE3Pose.h"

// full eskf depends
# include "tello_pub/quat_eskf_full.h"

/*================== define global variables ============================*/
/*
@FaceOdom
    provide the odometry for face coordinate
    re-initialize if target face is not detected and covTr is too large
*/
class FaceOdom : public FacePoseESKF
{
    /*==================[ros-dep]=================*/
    // ros::Subscriber drone1_sub, facePose_sub, SOT_sub;
    ros::Subscriber drone1_sub, facePose_sub;

    [[maybe_unused]] ros::Subscriber drone2_sub, drone0_sub, 
                                     YOLOPose0_sub, YOLOPose2_sub,
                                     eskf_init_1_sub, eskf_init_2_sub;
    ros::Publisher face_eskf_pub, covTr_pub, face_odom_pub, face_init_pub, face_quat_pub;
    int queue_size = 1;

    // timing
    ros::Timer predictStateTimer;
    ros::Time startTimeEskf, faceLastCall;
    double t_smpl =0.05;
    
    // process
    bool rvizEnable     = false;
    bool StartOfTest    = false;                   // if SOT is false, then eskf would be re-initialized
    bool setSOT         = false;
    bool useQuatVec4d   = false;
    double t_reinit     = 5.0;
    double trace_reinit = 0.01;                 // test by exp, should exploded for ~10 sec loss

    /*=================[param]====================*/
    // shooting drone
    std::string droneName[3] = {"/tello_0", "/tello_1", "/tello_2"};

    // keep the drones state
    Eigen::Quaterniond quat_drones[3], quat_init_drones[2];         // init: Ii --> I0
    Eigen::Vector3d position_drones[3], position_init_drones[2];    // init: I0 --> Ii in I0

    /*==================[function]==================*/
    /*
    @telloSOTCallback
        synchronized start
        set the SOT flag to true
    */
    // void telloSOTCallback(std_msgs::Empty const& msg);
    /*
    @drone0CB
        get drone 0's state
        update the drone 0's true-state position and orientation
    */
    void drone0CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt);
    /*
    @drone1CB
        get drone 1's state
        update the drone 1's true-state position and orientation
    */
    void drone1CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt);
    /*
    @drone2CB
        get drone 2's state
        update the drone 2's true-state position and orientation
    */
    void drone2CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt);
    /*
    @drone1InitCB
        get initial state
    */
    void drone1InitCB(tello_pub::SE3Pose const& pose);
    /*
    @drone2InitCB
        get initial state
    */
    void drone2InitCB(tello_pub::SE3Pose const& pose);
    /*
    @YOLOPose0CB
        update the face odom with yolo x measurement
    */
    void YOLOPose0CB(geometry_msgs::PoseArray const& pose_arr);
    /*
    @YOLOPose2CB
        update the face odom with yolo x measurement
    */
    void YOLOPose2CB(geometry_msgs::PoseArray const& pose_arr);
    /*
    @FacePoseCB
        get cam1 to Face pose
        know when to initialize the face pose eskf
    */
    void FacePoseCB(geometry_msgs::PoseArray const& pose_arr);
    /*
    @predictTimerCB
        publish face odom state
        run the internal predict state
    */
    void predictTimerCB(ros::TimerEvent const& ev);
    /*
    @publishTrueState
    */
    void publishTrueState();
    /*
    @reinitialize
        input:
            double: time elapsed from last call of face pose
        output:
            true:   reinitialize the ESKF
            false:  o.w.
    */
    bool reinitialize(double timeElapsed);
public:
    /*
    @FaceOdom
        empty
    */
    FaceOdom(){}
    ~FaceOdom(){}
    /*
    @FaceOdom
        explicit
        set the flags
            useYOLO_
            usePostRvecJacobian
            enableRviz
    */
    explicit FaceOdom(
        ros::NodeHandle& nh_, 
        bool useYOLO_, bool usePostRvecJacob_,
        bool enableRviz, double t_smpl
    );
};


#endif