
# include "tello_pub/faceOdom.h"

/*
@FaceOdom
*/
FaceOdom::FaceOdom(
    ros::NodeHandle& nh_, 
    bool useYOLO_, bool usePostRvecJacob_,
    bool enableRviz, double t_smpl
) : FacePoseESKF(useYOLO_, usePostRvecJacob_, t_smpl)
{
    /*=============[ros-dep]======================*/
    std::string ros_nodeName = ros::this_node::getName();
    nh_.getParam(ros_nodeName + "/reinit_time",     t_reinit);
    nh_.getParam(ros_nodeName + "/reinit_trace",    trace_reinit);
    nh_.getParam(ros_nodeName + "/debug_set_SOT",   setSOT);
    nh_.getParam(ros_nodeName + "/useQuatVec4d",    useQuatVec4d);

    // debug
    std::cout << ros_nodeName << std::endl 
              << "time to reinit: " << t_reinit << std::endl
              << "trace to reinit: " << trace_reinit << std::endl
              << "debug_set_SOT: " << setSOT << std::endl;

    // sub / pub
    // SOT_sub = nh_.subscribe("/SOT", queue_size, &FaceOdom::telloSOTCallback, this);
    // face pose
    if (useQuatVec4d){
        face_eskf_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/face_eskf_q4d", queue_size
        );
        covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            "/face_cov_trace_q4d", queue_size
        );
        face_quat_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            "/face_quat_norm_q4d", queue_size
        );
        if (enableRviz){
            face_odom_pub = nh_.advertise<nav_msgs::Odometry>(
                "/face_odom_q4d", queue_size
            );
        }
    }
    else {
        face_eskf_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/face_eskf", queue_size
        );
        covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            "/face_cov_trace", queue_size
        );
        face_quat_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            "/face_quat_norm", queue_size
        );
        if (enableRviz){
            face_odom_pub = nh_.advertise<nav_msgs::Odometry>(
                "/face_odom", queue_size
            );
        }
    }
    // face_init_pub = nh_.advertise<geometry_msgs::Pose>(
    //     "/face_init", queue_size
    // );
    face_init_pub = nh_.advertise<tello_pub::SE3Pose>(
        "/face_init", queue_size
    );
    drone1_sub = nh_.subscribe(droneName[1] + "/eskf_state_full", queue_size,
        &FaceOdom::drone1CB, this);
    facePose_sub = nh_.subscribe(droneName[1] + "/face_pose_arr", queue_size,
        &FaceOdom::FacePoseCB, this);
    // check if use YOLO
    if (useYOLO_) {
        drone0_sub = nh_.subscribe(
            droneName[0] + "/eskf_state_full", queue_size,
            &FaceOdom::drone0CB, this
        );
        drone2_sub = nh_.subscribe(
            droneName[2] + "/eskf_state_full", queue_size,
            &FaceOdom::drone2CB, this
        );
        YOLOPose0_sub = nh_.subscribe(
            droneName[0] + "/human_pose_arr", queue_size,
            &FaceOdom::YOLOPose0CB, this
        );
        YOLOPose2_sub = nh_.subscribe(
            droneName[2] + "/human_pose_arr", queue_size,
            &FaceOdom::YOLOPose2CB, this
        );
        eskf_init_1_sub = nh_.subscribe(
            droneName[1] + "/eskf_init", queue_size,
            &FaceOdom::drone1InitCB, this
        );
        eskf_init_2_sub = nh_.subscribe(
            droneName[2] + "/eskf_init", queue_size,
            &FaceOdom::drone2InitCB, this
        );
    }
    // timer callback
    predictStateTimer = nh_.createTimer(
        ros::Duration(t_smpl),
        &FaceOdom::predictTimerCB, this, false, false
    );
    /*===============[param]=====================*/
    // set drone initial state
    for (int i=0; i<3; i++){
        quat_drones[i] = Eigen::Quaterniond::Identity();
        position_drones[i] = Eigen::Vector3d::Zero();
    }
    for (int i=0; i<2; i++){
        quat_init_drones[i] = Eigen::Quaterniond::Identity();
        position_init_drones[i] = Eigen::Vector3d::Zero();
    }

    rvizEnable = enableRviz;
    // set the flag
    FacePoseESKF::setQuatVec4dFlag(useQuatVec4d);
}
/*
@drone1CB
    get drone 1's state
    update the drone 1's true-state position and orientation
*/
void FaceOdom::drone1CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt){
    
    // assign drone's position and velocity
    position_drones[1](0) = traj_pt.transforms[0].translation.x;
    position_drones[1](1) = traj_pt.transforms[0].translation.y;
    position_drones[1](2) = traj_pt.transforms[0].translation.z;

    tf::quaternionMsgToEigen(traj_pt.transforms[0].rotation, quat_drones[1]);
}
/*
@drone2CB
    get drone 2's state
    update the drone 2's true-state position and orientation
*/
void FaceOdom::drone2CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt){
    
    // assign drone's position and velocity
    position_drones[2](0) = traj_pt.transforms[0].translation.x;
    position_drones[2](1) = traj_pt.transforms[0].translation.y;
    position_drones[2](2) = traj_pt.transforms[0].translation.z;

    tf::quaternionMsgToEigen(traj_pt.transforms[0].rotation, quat_drones[2]);
}
/*
@drone0CB
    get drone 0's state
    update the drone 0's true-state position and orientation
*/
void FaceOdom::drone0CB(trajectory_msgs::MultiDOFJointTrajectoryPoint const& traj_pt){
    
    // assign drone's position and velocity
    position_drones[0](0) = traj_pt.transforms[0].translation.x;
    position_drones[0](1) = traj_pt.transforms[0].translation.y;
    position_drones[0](2) = traj_pt.transforms[0].translation.z;

    tf::quaternionMsgToEigen(traj_pt.transforms[0].rotation, quat_drones[0]);
}
/*
@drone1InitCB
    get init states
    update for drone 0 and drone 2's YOLO measurement
*/
void FaceOdom::drone1InitCB(tello_pub::SE3Pose const& pose){
    position_init_drones[0](0) = pose.tvec.x;
    position_init_drones[0](1) = pose.tvec.y;
    position_init_drones[0](2) = pose.tvec.z;
    Eigen::Matrix3d tmp;
    std::memcpy(
        tmp.data(), &pose.rmat, 9 * sizeof(double)
    );
    quat_init_drones[0] = tmp;
    // debug
    std::cout << "========[drone 1 init]========\n"
              << "drone 1 init position\n" << pose.tvec << std::endl
              << "init orientation\n" << tmp << std::endl;
}
/*
@drone2InitCB
    get init states
    update for drone 2's YOLO measurement
*/
void FaceOdom::drone2InitCB(tello_pub::SE3Pose const& pose){
    position_init_drones[1](0) = pose.tvec.x;
    position_init_drones[1](1) = pose.tvec.y;
    position_init_drones[1](2) = pose.tvec.z;
    Eigen::Matrix3d tmp;
    std::memcpy(
        tmp.data(), &pose.rmat, 9 * sizeof(double)
    );
    quat_init_drones[1] = tmp;
    // debug
    std::cout << "========[drone 2 init]========\n"
              << "drone 2 init position\n" << pose.tvec << std::endl
              << "init orientation\n" << tmp << std::endl;
}
/*
@YOLOPose0CB
    no need to care the face pose re-init or not
*/
void FaceOdom::YOLOPose0CB(geometry_msgs::PoseArray const& pose_arr){
    Eigen::Quaterniond quat_yolo;       // yolo to cam0
    Eigen::Vector3d position_yolo;      // cam0 to yolo in cam0
    position_yolo << pose_arr.poses[0].position.x,
                     pose_arr.poses[0].position.y,
                     pose_arr.poses[0].position.z;
    tf::quaternionMsgToEigen(pose_arr.poses[0].orientation, quat_yolo);
    
    // pass to ESKF
    FacePoseESKF::YOLOPose0CB(
        position_yolo,
        quat_yolo,
        position_init_drones[0],
        quat_init_drones[0],
        position_drones[0],
        quat_drones[0]
    );
}
/*
@YOLOPose2CB
    no need to care the face pose re-init or not
*/
void FaceOdom::YOLOPose2CB(geometry_msgs::PoseArray const& pose_arr){
    Eigen::Quaterniond quat_yolo;       // yolo to cam2
    Eigen::Vector3d position_yolo;      // cam2 to yolo in cam2
    position_yolo << pose_arr.poses[0].position.x,
                     pose_arr.poses[0].position.y,
                     pose_arr.poses[0].position.z;
    tf::quaternionMsgToEigen(pose_arr.poses[0].orientation, quat_yolo);
    
    // pass to ESKF
    FacePoseESKF::YOLOPose2CB(
        position_yolo,
        quat_yolo,
        position_init_drones[0],
        position_init_drones[1],
        quat_init_drones[0],
        quat_init_drones[1],
        position_drones[2],
        quat_drones[2]
    );
}
/*
@FacePoseCB
    get cam1 to Face pose
    passing drone1's state to face pose eskf
    know when to initialize the face pose eskf
*/
void FaceOdom::FacePoseCB(geometry_msgs::PoseArray const& pose_arr){

    // convert to eigen
    Eigen::Quaterniond quatCamToFace;
    Eigen::Vector3d positionCamToFace;
    positionCamToFace << pose_arr.poses[0].position.x, 
                         pose_arr.poses[0].position.y, 
                         pose_arr.poses[0].position.z;
    tf::quaternionMsgToEigen(pose_arr.poses[0].orientation, quatCamToFace);
    
    // passing to the eskf
    // if not started, let the eskf record the initial state!
    FacePoseESKF::FacePoseCB(
        positionCamToFace, quatCamToFace, quat_drones[1], position_drones[1]
    );
    // get current time
    if (!predictStateTimer.hasStarted()) {
        predictStateTimer.start();
        startTimeEskf = ros::Time::now();
    }
    faceLastCall = ros::Time::now(); 
    // start to the ESKF
    StartOfTest = true;
    FacePoseESKF::setStartFlag(StartOfTest);
}
/*
@predictTimerCB
    publish face odom state
    run the internal predict state
    measure the time to last face call and re-initialize the filter
*/
void FaceOdom::predictTimerCB(ros::TimerEvent const& ev){

    // debug
    // std::cout << "predict Timer callback" << std::endl;
    
    if (StartOfTest){
        // check if re-initialized
        if (!reinitialize((ros::Time::now() - faceLastCall).toSec())){
            publishTrueState();
            FacePoseESKF::predictState();
        }
        else {
            // set the flag to false and disable the filter
            StartOfTest = false;
            FacePoseESKF::setStartFlag(StartOfTest);
            // let the eskf reset
            FacePoseESKF::predictState();
            // reset the last call time to avoid useless call of re-init
            // faceLastCall = ros::Time::now();
            // debug
            std::cout << "face pose eskf re-initialized!" << std::endl;
        }
    }
}
/*
@reinitialize
    judge whether to re-initialize the eskf or not
    first by the time elapsed
    then check the covarance trace
    both satisfy then reinitialize

    input:
        double: time elapsed from last call of face pose
    output:
        true:   reinitialize the ESKF
        false:  o.w.
*/
bool FaceOdom::reinitialize(double timeElapsed){
    
    // check time
    if (timeElapsed > t_reinit){
        // check trace
        Eigen::MatrixXd errCov;
        FacePoseESKF::getErrCov(errCov);
        // debug
        std::cout << "========[face pose eskf]========\n"
                  << "time elapsed from last callback: " << timeElapsed << std::endl
                  << "covariance trace: " << errCov.trace() << std::endl;
        
        if (errCov.trace() > trace_reinit) { return true; }
        else { return false; }
    }
    else { return false; }
}
/*
    @publishTrueState
*/
void FaceOdom::publishTrueState(){
    // get eskf state
    Eigen::VectorXd nominalPosVel_;
    Eigen::Vector3d nominalAngVel_, initPos_;   // initPos_:    I1 --> IF in I1
    // Eigen::Quaterniond nominalQuat_, initQuat_; // initQuat_:   IF --> I1
    Eigen::Quaterniond nominalQuat_; 
    Eigen::Matrix3d initRMat_;
    FacePoseESKF::getNominalState(
        nominalPosVel_, nominalQuat_, nominalAngVel_
    );
    FacePoseESKF::getInitState(
        initPos_, initRMat_
    );
    // check number of subscribers
    if (face_eskf_pub.getNumSubscribers() > 0){
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
        trajPt.time_from_start = ros::Time::now() - startTimeEskf;
        // pose
        geometry_msgs::Transform geoTF_msg;
        geoTF_msg.translation.x = nominalPosVel_(0);
        geoTF_msg.translation.y = nominalPosVel_(1);
        geoTF_msg.translation.z = nominalPosVel_(2);
        tf::quaternionEigenToMsg(nominalQuat_, geoTF_msg.rotation);
        // twist
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = nominalPosVel_(3);
        twist_msg.linear.y = nominalPosVel_(4);
        twist_msg.linear.z = nominalPosVel_(5);
        twist_msg.angular.x = nominalAngVel_(0);
        twist_msg.angular.y = nominalAngVel_(1);
        twist_msg.angular.z = nominalAngVel_(2);
        // set traj
        trajPt.velocities.emplace_back(twist_msg);
        trajPt.transforms.emplace_back(geoTF_msg);
        face_eskf_pub.publish(trajPt);
    }
    if (covTr_pub.getNumSubscribers() > 0){
        // get cov
        Eigen::MatrixXd errCov;
        FacePoseESKF::getErrCov(errCov);
        // set msg
        std_msgs::Float64MultiArray arr_msg;
        arr_msg.data.emplace_back(errCov.trace());
        covTr_pub.publish(arr_msg);
    }
    if (face_init_pub.getNumSubscribers() > 0){
        tello_pub::SE3Pose SE3Pose_msg;
        SE3Pose_msg.header.stamp = ros::Time::now();
        SE3Pose_msg.tvec.x = initPos_(0);
        SE3Pose_msg.tvec.y = initPos_(1);
        SE3Pose_msg.tvec.z = initPos_(2);
        std::memcpy(
            &SE3Pose_msg.rmat, initRMat_.data(), 9 * sizeof(double)
        );
        face_init_pub.publish(SE3Pose_msg);

        // debug
        std::cout << "========[face init]========\n"
                  << "IF to I1 rotation:\n" << initRMat_ << std::endl
                  << "I1 to IF in I1 translation:\n" << initPos_ << std::endl;
    }
    if (rvizEnable){
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "eskf_odom";
        odom_msg.pose.pose.position.x = nominalPosVel_(0);
        odom_msg.pose.pose.position.y = nominalPosVel_(1);
        odom_msg.pose.pose.position.z = nominalPosVel_(2);
        tf::quaternionEigenToMsg(nominalQuat_, odom_msg.pose.pose.orientation);

        odom_msg.twist.twist.linear.x = nominalPosVel_(3);
        odom_msg.twist.twist.linear.y = nominalPosVel_(4);
        odom_msg.twist.twist.linear.z = nominalPosVel_(5);
        odom_msg.twist.twist.angular.x = nominalAngVel_(0);
        odom_msg.twist.twist.angular.y = nominalAngVel_(1);
        odom_msg.twist.twist.angular.z = nominalAngVel_(2);
        
        face_odom_pub.publish(odom_msg);
    }
    if (face_quat_pub.getNumSubscribers() > 0){
        std_msgs::Float64MultiArray norm_arr;
        norm_arr.data.emplace_back(nominalQuat_.norm());
        face_quat_pub.publish(norm_arr);
    }
}

/*
@main
    test
*/
int main(int argc, char **argv){
    
    // for testing
    ros::init(argc, argv, "face_odom", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    bool useYOLO = false;
    double t_smpl = 0.05;
    nh.getParam(ros::this_node::getName()  + "/smpl_time", t_smpl);
    nh.getParam(ros::this_node::getName() + "/useYOLO", useYOLO);
    std::cout << "use YOLO: " << useYOLO << std::endl;

    // FaceOdom fo(nh, useYOLO, true, true);
    // recall that we are using prior error-state in Jacobians of posterior covariance propagation
    FaceOdom fo(nh, useYOLO, false, true, t_smpl);
    ros::spin();
    
    return 0;
}