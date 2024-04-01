
# include "tello_pub/dronesVIO.h"


/*
@Drone0InerOdom
    constructor w. nodehandler
*/
Drone0InerOdom::Drone0InerOdom(ros::NodeHandle& nh_, int drone_id, bool enableRViz, double smpl_time) 
    : FullQuatESKF(drone_id, smpl_time)
{
    t_smpl = smpl_time;
    
    bool addPerturb = false;
    double angPerturb = 0.0;
    /*==========================[ros-dep]====================*/
    // get nodename
    std::string ros_nodeName = ros::this_node::getName();
    // get param
    nh_.getParam(ros_nodeName + "/useImuQuat", useImuQuat);
    nh_.getParam(ros_nodeName + "/useQuatVec4d", useQuatVec4d);
    // nh_.getParam(ros_nodeName + "/smpl_time", t_smpl);
    nh_.getParam(ros_nodeName + "/cmd_vel_thr", cmdVelThr);
    nh_.getParam(ros_nodeName + "/add_perturb", addPerturb);
    nh_.getParam(ros_nodeName + "/ang_perturb", angPerturb);
    std::cout << ros_nodeName << std::endl 
              << useQuatVec4d << std::endl
              << useImuQuat << std::endl;
            //   << t_smpl << std::endl;


    // sub/pub
    SOT_sub = nh_.subscribe("/SOT", queue_size, &Drone0InerOdom::telloSOTCallback, this);
    imu_sub = nh_.subscribe(droneName + "/imu", queue_size, 
        &Drone0InerOdom::imuCB, this);
    odom_sub = nh_.subscribe(droneName + "/odom", queue_size,
        &Drone0InerOdom::odomCB, this);
    // predictStateTimer = nh_.createTimer(
    //     ros::Duration(FullQuatESKF::getTs()), 
    //     &Drone0InerOdom::predictTimerCB, this, false
    // );
    predictStateTimer = nh_.createTimer(
        ros::Duration(t_smpl), 
        &Drone0InerOdom::predictTimerCB, this, false
    );
    if (useImuQuat){
        eskf_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            droneName + "/eskf_state_full", queue_size
        );
        // debug publisher
        covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            droneName + "/cov_trace", queue_size
        );
        eskf_bias_pub = nh_.advertise<geometry_msgs::Accel>(
            droneName + "/bias", queue_size
        );
        // check the norm w. or w.o. normalization
        quat_norm_pub = nh_.advertise<std_msgs::Float64MultiArray>(
            droneName + "/quat_norm", queue_size
        );
        // visualization
        eskf_odom_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName + "/eskf_odom", queue_size
        );
    }
    else {
        if (useQuatVec4d){
            eskf_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
                droneName + "/eskf_state_q4d", queue_size
            );
            // debug publisher
            covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
                droneName + "/cov_trace_q4d", queue_size
            );
            eskf_bias_pub = nh_.advertise<geometry_msgs::Accel>(
                droneName + "/bias_q4d", queue_size
            );
            // check the norm w. or w.o. normalization
            quat_norm_pub = nh_.advertise<std_msgs::Float64MultiArray>(
                droneName + "/quat_norm_q4d", queue_size
            );
            // visualization
            eskf_odom_pub = nh_.advertise<nav_msgs::Odometry>(
                droneName + "/eskf_odom_q4d", queue_size
            );
        }
        else {
            eskf_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
                droneName + "/eskf_state_origin", queue_size
            );
            // debug publisher
            covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
                droneName + "/cov_trace_origin", queue_size
            );
            eskf_bias_pub = nh_.advertise<geometry_msgs::Accel>(
                droneName + "/bias_origin", queue_size
            );
            // check the norm w. or w.o. normalization
            quat_norm_pub = nh_.advertise<std_msgs::Float64MultiArray>(
                droneName + "/quat_norm_origin", queue_size
            );
            // visualization
            eskf_odom_pub = nh_.advertise<nav_msgs::Odometry>(
                droneName + "/eskf_odom_origin", queue_size
            );
        }
    }
    // publish initial states
    /*============================[ESKF setting]========================*/
    // initialize the imu vectors
    imuAcc = Eigen::Vector3d::Zero();
    imuAngVel = Eigen::Vector3d::Zero();
    
    // set the internal flag
    setStartFlag(StartOfTest);
    if (!useImuQuat) { setQuatVec4dFlag(useQuatVec4d); }
    // set the perturb flag
    setPerturbFlag(addPerturb, angPerturb);

    rvizEnable = enableRViz;
}
/*
@telloSOTCallback
    trigger the initialization of timer and start flag
*/
void Drone0InerOdom::telloSOTCallback(std_msgs::Empty const& msg){
    if (!StartOfTest) {
        predictStateTimer.start();
        startTimeEskf = ros::Time::now();
        StartOfTest = true;
        setStartFlag(StartOfTest);
        // debug
        ROS_INFO("========[drone 0]========\nreceived SOT\n");
    }
}
/*
@imuCB
    input
    imu input (acc, ang vel) to Eigen::Vector3d
    
    measurement
    correction and injection
    imu orientation to Eigen::Quaterniond
*/
void Drone0InerOdom::imuCB(sensor_msgs::Imu const& imu){
    // assign input 
    // Eigen::Vector3d imuAcc, imuAngVel;
    Eigen::Quaterniond imuQuat;
    imuAcc << imu.linear_acceleration.x,
              imu.linear_acceleration.y,
              imu.linear_acceleration.z;
    imuAngVel << imu.angular_velocity.x,
                 imu.angular_velocity.y,
                 imu.angular_velocity.z;
    tf::quaternionMsgToEigen(imu.orientation, imuQuat);
    
    // debug
    // std::cout << "========[eskf imu]========\ncurrent time stamp: " 
    //           << (ros::Time::now() - startTimeEskf).toSec() << std::endl;
    
    // call internal imu cb for input update and ESKF operation 
    if (useImuQuat){
        FullQuatESKF::imuCB(imuAcc, imuAngVel, imuQuat);
    }
    else {
        FullQuatESKF::imuInputCB(imuAcc, imuAngVel, imuQuat);
    }
    
}
/*
@odomCB
    measurement
    correction and injection
    reset...?
    odom velocity to Eigen::Vector3d
*/
void Drone0InerOdom::odomCB(nav_msgs::Odometry const& odom){
    // assign 
    Eigen::Vector3d odomVel;
    odomVel << odom.twist.twist.linear.x,
               odom.twist.twist.linear.y,
               odom.twist.twist.linear.z;
    // call internal odom cb
    // FullQuatESKF::odomCB(odomVel);
    // call inertial odom cb
    FullQuatESKF::odomInertialCB(odomVel);
    // set the flag
    if (odomVel.norm() > cmdVelThr || std::pow(odom.twist.twist.angular.x, 2) + 
        std::pow(odom.twist.twist.angular.y, 2) + 
        std::pow(odom.twist.twist.angular.z, 2) > std::pow(cmdVelThr, 2)){
            blockAruco = true;
        }
    else { blockAruco = false; }
}   
/*
@predictTimerCB
        publish droneState 
        run the internal predictState
*/
void Drone0InerOdom::predictTimerCB(ros::TimerEvent const& ev){
    // check SOT
    if (StartOfTest) {
        publishTrueState();
        FullQuatESKF::predictState();
    }
}
/*
@publishTrueState
    publish true state estimate
*/
void Drone0InerOdom::publishTrueState(){
    
    // get eskf state
    Eigen::VectorXd nominalPosVel_, nominalBias_;
    Eigen::Quaterniond nominalQuat_;
    FullQuatESKF::getNominalState(
        nominalPosVel_, nominalQuat_, nominalBias_
    );
    // check number of subscribers
    if (eskf_pub.getNumSubscribers() < 1) {
        // ROS_INFO_STREAM("========[drone0 eskf]========\nNO DRONE 0 ESKF OBS!");
    }
    else {
        // set msg
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
        trajPt.time_from_start = ros::Time::now() - startTimeEskf;
        // pose
        geometry_msgs::Transform geoTF_msg;
        geoTF_msg.translation.x = nominalPosVel_(0);
        geoTF_msg.translation.y = nominalPosVel_(1);
        geoTF_msg.translation.z = nominalPosVel_(2);
        tf::quaternionEigenToMsg(nominalQuat_, geoTF_msg.rotation);
        // twist
        geometry_msgs::Twist twist_msg, acc_msg;
        twist_msg.linear.x = nominalPosVel_(3);
        twist_msg.linear.y = nominalPosVel_(4);
        twist_msg.linear.z = nominalPosVel_(5);
        twist_msg.angular.x = imuAngVel(0);
        twist_msg.angular.y = imuAngVel(1);
        twist_msg.angular.z = imuAngVel(2);
        // acc_msg.linear.x = imuAcc(0);
        // acc_msg.linear.y = imuAcc(1);
        // acc_msg.linear.z = imuAcc(2);
        acc_msg.linear.x = G0*imuAcc(0);
        acc_msg.linear.y = G0*imuAcc(1);
        acc_msg.linear.z = G0*imuAcc(2);
        // set traj
        trajPt.accelerations.emplace_back(acc_msg);
        trajPt.velocities.emplace_back(twist_msg);
        trajPt.transforms.emplace_back(geoTF_msg);
        // publish
        eskf_pub.publish(trajPt);
    }
    // for the covariance trace
    if (covTr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========[drone0 eskf]========\nNO COV_TR SUBs!!");
    }
    else {
        // get err cov
        Eigen::MatrixXd errCov;
        FullQuatESKF::getErrCov(errCov);
        // set msg
        std_msgs::Float64MultiArray arr_msg;
        arr_msg.data.emplace_back(errCov.trace());
        // publish
        covTr_pub.publish(arr_msg);
    }
    // for the bias
    if (eskf_bias_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========[drone0 eskf]========\nNO BIAS SUBs!!");
    }
    else {
        geometry_msgs::Accel bias_msg;
        bias_msg.linear.x = nominalBias_(0);
        bias_msg.linear.y = nominalBias_(1);
        bias_msg.linear.z = nominalBias_(2);
        bias_msg.angular.x = nominalBias_(3);
        bias_msg.angular.y = nominalBias_(4);
        bias_msg.angular.z = nominalBias_(5);
        eskf_bias_pub.publish(bias_msg);
    }
    if (quat_norm_pub.getNumSubscribers() > 0){
        std_msgs::Float64MultiArray norm_arr;
        norm_arr.data.emplace_back(
            nominalQuat_.norm()
        );
        quat_norm_pub.publish(norm_arr);
    }
    // publish odom for RViz
    if (rvizEnable) {
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
        odom_msg.twist.twist.angular.x = imuAngVel(0);
        odom_msg.twist.twist.angular.y = imuAngVel(1);
        odom_msg.twist.twist.angular.z = imuAngVel(2);

        eskf_odom_pub.publish(odom_msg);
    }
}

/*==================[DronesVIO]==============================================*/
/*
@DronesVIO
    take node handler
    construct drone0, drone1, drone2 object in the same time
*/
DronesVIO::DronesVIO(ros::NodeHandle& nh_, double t_smpl) : 
    drone0_odom(nh_, 0, true, t_smpl), drone1_eskf(1, t_smpl), drone2_eskf(2, t_smpl)
{
    /*=========================[ros-dep]==============================*/
    // subs
    imu_1_sub = nh_.subscribe(droneName[1] + "/imu", queue_size, 
        &DronesVIO::drone1_imuCB, this);
    imu_2_sub = nh_.subscribe(droneName[2] + "/imu", queue_size, 
        &DronesVIO::drone2_imuCB, this);

    odom_1_sub = nh_.subscribe(droneName[1] + "/odom", queue_size, 
        &DronesVIO::drone1_odomCB, this);
    odom_2_sub = nh_.subscribe(droneName[2] + "/odom", queue_size, 
        &DronesVIO::drone2_odomCB, this);

    drone0_marker_sub = nh_.subscribe(droneName[0] + "/aruco_pose_arr", queue_size,
        &DronesVIO::ArUcoCB, this);
    marker_1_sub = nh_.subscribe(droneName[1] + "/aruco_pose_arr", queue_size,
        &DronesVIO::drone1_ArUcoCB, this);
    marker_2_sub = nh_.subscribe(droneName[2] + "/aruco_pose_arr", queue_size,
        &DronesVIO::drone2_ArUcoCB, this);

    // cmd_vel_0_sub = nh_.subscribe(droneName[0] + "/cmd_vel", queue_size,
    //     &DronesVIO::drone0_velCB, this);
    // cmd_vel_1_sub = nh_.subscribe(droneName[1] + "/cmd_vel", queue_size,
    //     &DronesVIO::drone1_velCB, this);
    // cmd_vel_2_sub = nh_.subscribe(droneName[2] + "/cmd_vel", queue_size,
    //     &DronesVIO::drone2_velCB, this);

    // takeoff_sub = nh_.subscribe("/takeoff", queue_size,
    //     &DronesVIO::takeoffCB, this);
    SOT_sub = nh_.subscribe("/SOT", queue_size, 
        &DronesVIO::telloSOTCallback, this);
    // EOT_sub = nh_.subscribe("/EOT", queue_size,
    //     &DronesVIO::telloEOTCallback, this);

    // pubs
    eskf_1_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        droneName[1] + "/eskf_state_full", queue_size
    );
    eskf_2_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        droneName[2] + "/eskf_state_full", queue_size
    );
    eskf_init_1_pub = nh_.advertise<tello_pub::SE3Pose>(
        droneName[1] + "/eskf_init", queue_size
    );
    eskf_init_2_pub = nh_.advertise<tello_pub::SE3Pose>(
        droneName[2] + "/eskf_init", queue_size
    );
    // debug pubs
    eskf_bias_1_pub = nh_.advertise<geometry_msgs::Accel>(
        droneName[1] + "/bias", queue_size
    );
    eskf_bias_2_pub = nh_.advertise<geometry_msgs::Accel>(
        droneName[2] + "/bias", queue_size
    );
    covTr_pub = nh_.advertise<std_msgs::Float64MultiArray>(
        "/two_drone_cov_tr", queue_size
    );
    // visualization
    eskf_odom_1_pub = nh_.advertise<nav_msgs::Odometry>(
        droneName[1] + "/eskf_odom", queue_size
    );
    eskf_odom_2_pub = nh_.advertise<nav_msgs::Odometry>(
        droneName[2] + "/eskf_odom", queue_size
    );
    // timers
    predictStateTimer_1 = nh_.createTimer(
        ros::Duration(drone0_odom.getTs()), 
        &DronesVIO::drone1_predictTimerCB, this, false
    );
    predictStateTimer_2 = nh_.createTimer(
        ros::Duration(drone0_odom.getTs()), 
        &DronesVIO::drone2_predictTimerCB, this, false
    );
    /*=========================[parameter]============================*/
    // setting the vectors
    imuAcc[0] = Eigen::Vector3d::Zero();
    imuAcc[1] = Eigen::Vector3d::Zero();
    imuAngVel[0] = Eigen::Vector3d::Zero();
    imuAngVel[1] = Eigen::Vector3d::Zero();

    // setting the internal flags for drone1 and drone2
    drone1_eskf.setStartFlag(drone1_SOT);
    drone2_eskf.setStartFlag(drone2_SOT);

    // setting the visualization flag
    rvizEnable[0] = true;
    rvizEnable[1] = true;

    // get nodename
    std::string ros_nodeName = ros::this_node::getName();
    // get param
    nh_.getParam(ros_nodeName + "/cmd_vel_thr", cmdVelThr);

    // debug
    std::cout << "drone 1 and 2 timer duration: " << drone0_odom.getTs()
              << std::endl;
}
/*
@telloSOTCallback
    initialization and set flag
*/
void DronesVIO::telloSOTCallback(std_msgs::Empty const& msg){
    if (!drone1_SOT){
        predictStateTimer_1.start();
        startTime_1 = ros::Time::now();
        drone1_SOT = true;
        drone1_eskf.setStartFlag(drone1_SOT);
        // debug
        ROS_INFO("========[drone 1]========\nreceived SOT\n");
    }
    if (!drone2_SOT){
        predictStateTimer_2.start();
        startTime_2 = ros::Time::now();
        drone2_SOT = true;
        drone2_eskf.setStartFlag(drone2_SOT);
        // debug
        ROS_INFO("========[drone 2]========\nreceived SOT\n");
    }
}
/*
@telloEOTCallback
    set aruco flag
*/
// void DronesVIO::telloEOTCallback(std_msgs::Empty const& msg){
//     for (int i=0; i<3; i++) { blockAruco[i] = true; }
//     ROS_INFO("========[drones vio]========\nreceived EOT\n");
// }
/*
@takeoffCB
    set aruco flag
*/
// void DronesVIO::takeoffCB(std_msgs::Empty const& msg) { for (int i=0; i<3; i++) { blockAruco[i] = true; } }
/*
@imuCB
    input:          imu acc, gyro angvel to Eigen::Vector3d
    measurement:    orientation
*/
void DronesVIO::drone1_imuCB(sensor_msgs::Imu const& imu){
    
    // debug
    // std::cout << "drone1 imuCB" << std::endl;

    Eigen::Quaterniond imuQaut;
    imuAcc[0] << imu.linear_acceleration.x,
                 imu.linear_acceleration.y,
                 imu.linear_acceleration.z;
    imuAngVel[0] << imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z;
    tf::quaternionMsgToEigen(imu.orientation, imuQaut);
    drone1_eskf.imuCB(imuAcc[0], imuAngVel[0], imuQaut);
}
void DronesVIO::drone2_imuCB(sensor_msgs::Imu const& imu){

    // debug
    // std::cout << "drone2 imu CB" << std::endl;

    Eigen::Quaterniond imuQaut;
    imuAcc[1] << imu.linear_acceleration.x,
                 imu.linear_acceleration.y,
                 imu.linear_acceleration.z;
    imuAngVel[1] << imu.angular_velocity.x,
                    imu.angular_velocity.y,
                    imu.angular_velocity.z;
    tf::quaternionMsgToEigen(imu.orientation, imuQaut);
    drone2_eskf.imuCB(imuAcc[1], imuAngVel[1], imuQaut);
}
/*
@odomCB
*/
void DronesVIO::drone1_odomCB(nav_msgs::Odometry const& odom){
    Eigen::Vector3d odomVel;
    odomVel << odom.twist.twist.linear.x,
               odom.twist.twist.linear.y,
               odom.twist.twist.linear.z;
    drone1_eskf.odomInertialCB(odomVel);
    // set the flag
    if (odomVel.norm() > cmdVelThr || (
            std::pow(odom.twist.twist.angular.x, 2) +
            std::pow(odom.twist.twist.angular.y, 2) + 
            std::pow(odom.twist.twist.angular.z, 2)
        ) > std::pow(cmdVelThr, 2)) { blockAruco[1] = true; }
    else { blockAruco[1] = false; }
}
void DronesVIO::drone2_odomCB(nav_msgs::Odometry const& odom){
    Eigen::Vector3d odomVel;
    odomVel << odom.twist.twist.linear.x,
               odom.twist.twist.linear.y,
               odom.twist.twist.linear.z;
    drone2_eskf.odomInertialCB(odomVel);
    // set the flag
    if (odomVel.norm() > cmdVelThr || (
            std::pow(odom.twist.twist.angular.x, 2) +
            std::pow(odom.twist.twist.angular.y, 2) + 
            std::pow(odom.twist.twist.angular.z, 2)
        ) > std::pow(cmdVelThr, 2)) { blockAruco[2] = true; }
    else { blockAruco[2] = false; }
}
/*
@drone1_velCB
    set the flag according to velocity threshold
*/
// void DronesVIO::drone0_velCB(geometry_msgs::Twist const& twist){
//     // debug
//     std::cout << "========[multi-drone VIO]========\n"
//               << "drone 0 twist:\n" << twist << std::endl;

//     // check the thr
//     if (
//         (std::abs(twist.angular.z) > cmdVelThr) || (std::sqrt(
//             std::pow(twist.linear.x, 2) + std::pow(twist.linear.y, 2) + std::pow(twist.linear.z, 2)
//         ) > cmdVelThr)
//     ){ blockAruco[0] = true; }
// }
// void DronesVIO::drone1_velCB(geometry_msgs::Twist const& twist){
//     // debug
//     std::cout << "========[multi-drone VIO]========\n"
//               << "drone 1 twist:\n" << twist << std::endl;

//     // check the thr
//     if (
//         (std::abs(twist.angular.z) > cmdVelThr) || (std::sqrt(
//             std::pow(twist.linear.x, 2) + std::pow(twist.linear.y, 2) + std::pow(twist.linear.z, 2)
//         ) > cmdVelThr)
//     ){ blockAruco[1] = true; }
// }
// void DronesVIO::drone2_velCB(geometry_msgs::Twist const& twist){
//     // debug
//     std::cout << "========[multi-drone VIO]========\n"
//               << "drone 2 twist:\n" << twist << std::endl;

//     // check the thr
//     if (
//         (std::abs(twist.angular.z) > cmdVelThr) || (std::sqrt(
//             std::pow(twist.linear.x, 2) + std::pow(twist.linear.y, 2) + std::pow(twist.linear.z, 2)
//         ) > cmdVelThr)
//     ){ blockAruco[2] = true; }
// }
/*
@ArUcoCB
    measurement: 
        relative pose array in geometry_msgs/Pose
        marker ids in int32[]
        sizes in float64[]
    find the desired marker pose by desiredArUcoID
    set as 
        measured position: Eigen::Vector3d
        measured orientation: Eigen::Quaterniond
    also need
        drone0's nominal orientation in Eigen::Quaterniond
        drone0's current position in Eigen::Vector3d
*/
void DronesVIO::ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr){

    // assign the measurement
    Eigen::Vector3d p1_aruco, p2_aruco;
    Eigen::Quaterniond q1_aruco, q2_aruco;

    bool idFound[2] = {false, false};
    int idCount = 0;
    for (int i=0; i<arucoPoseArr.ids.size(); i++) {
        // debug
        // std::cout << "========[eskf aruco]========\naruco id: " 
        //             << arucoPoseArr.ids[i] << std::endl;
        // assign relative position for the two drones
        if (arucoPoseArr.ids[i] == desiredArUcoID[0]){
            p1_aruco << arucoPoseArr.poses[i].position.x,
                        arucoPoseArr.poses[i].position.y,
                        arucoPoseArr.poses[i].position.z;
            tf::quaternionMsgToEigen(arucoPoseArr.poses[i].orientation, q1_aruco);
            idFound[0] = true;
            
            // debug
            // ROS_INFO_STREAM("========[eskf aruco]========\nMATCHED ID as " << desiredArUcoID[0]);
            std::cout << "MATCHED ID: " << desiredArUcoID[0] << std::endl;
        }
        if (arucoPoseArr.ids[i] == desiredArUcoID[1]){
            p2_aruco << arucoPoseArr.poses[i].position.x,
                        arucoPoseArr.poses[i].position.y,
                        arucoPoseArr.poses[i].position.z;
            tf::quaternionMsgToEigen(arucoPoseArr.poses[i].orientation, q2_aruco);
            idFound[1] = true;
            
            // debug
            // ROS_INFO_STREAM("========[eskf aruco]========\nMATCHED ID as " << desiredArUcoID[1]);
            std::cout << "MATCHED ID as " << desiredArUcoID[1] << std::endl;

        }
        if (idFound[0] && idFound[1]) { break; }
    }
    // check if blocked
    blockAruco[0] = drone0_odom.blockArUco();
    if (blockAruco[0]) {
        std::cout << "========[multi-drone VIO]========\ndrone 0 moving, block all aruco!" << std::endl;
    }
    else if (blockAruco[1] && blockAruco[2]) {
        std::cout << "========[multi-drone VIO]========\ndrone 1 and 2 both moving, block all aruco!" << std::endl;
    }
    else {
        // find the drone0's state
        Eigen::VectorXd drone0PosVel, drone0Bias;
        Eigen::Quaterniond drone0Quat;
        drone0_odom.getNominalState(drone0PosVel, drone0Quat, drone0Bias);

        // not block and id found
        if ((!blockAruco[1]) && idFound[0]) {
            // call the callback
            // note that the callback is developed based on Mi --> C0 in C0
            // but the aruco position is calculated as C0 --> Mi in C0
            // so add the - sign on position
            drone1_eskf.ArUcoPositionCB(
                -p1_aruco, q1_aruco, drone0Quat, drone0PosVel.head(3)
            );
        }
        if ((!blockAruco[2]) && idFound[1]) {
            // call the callback
            // note that the callback is developed based on Mi --> C0 in C0
            // but the aruco position is calculated as C0 --> Mi in C0
            // so add the - sign on position
            drone2_eskf.ArUcoPositionCB(
                -p2_aruco, q2_aruco, drone0Quat, drone0PosVel.head(3)
            );
        }
    }
}
/*
@drone1/2_ArUcoCB
    according to drone1/2's mission
    not involved in ESKF!
*/
void DronesVIO::drone1_ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr){
    // TODO
}
void DronesVIO::drone2_ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr){
    // TODO
}
/*
@predictTimerCB
*/
void DronesVIO::drone1_predictTimerCB(ros::TimerEvent const& ev){
    if (drone1_SOT){
        publishTrueState(drone1_eskf);
        drone1_eskf.predictState();
    }
}
void DronesVIO::drone2_predictTimerCB(ros::TimerEvent const& ev){
    if (drone2_SOT){
        publishTrueState(drone2_eskf);
        drone2_eskf.predictState();
    }
}
/*
@publishTrueState
    publish true state of drone1/2 eskf according to the input object
*/
void DronesVIO::publishTrueState(FullQuatESKF &eskfObj){
    // get the true state estimate from the obj
    Eigen::VectorXd nominalPosVel_, nominalBias_;
    Eigen::Quaterniond nominalQuat_;
    
    // get the initial states 
    Eigen::Vector3d initPos_;       // I0 --> Ii in I0 frame
    Eigen::Matrix3d initRot_;       // Ii --> I0
    // Eigen::Quaterniond initQuat_;   // Ii --> I0

    eskfObj.getNominalState(
        nominalPosVel_, nominalQuat_, nominalBias_
    );
    eskfObj.getInitState(
        initPos_, initRot_
    );
    // check which topic to publish
    switch (eskfObj.getDroneID()){
        case 1:
            // traj pub
            if (eskf_1_pub.getNumSubscribers() > 0){
                // debug
                // std::cout << "drone id: " << eskfObj.getDroneID() << std::endl
                //           << "publish state traj." << std::endl;

                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - startTime_1;
                // pose
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = nominalPosVel_(0);
                geoTF_msg.translation.y = nominalPosVel_(1);
                geoTF_msg.translation.z = nominalPosVel_(2);
                tf::quaternionEigenToMsg(nominalQuat_, geoTF_msg.rotation);
                // twist
                geometry_msgs::Twist twist_msg, acc_msg;
                twist_msg.linear.x = nominalPosVel_(3);
                twist_msg.linear.y = nominalPosVel_(4);
                twist_msg.linear.z = nominalPosVel_(5);

                // assign to the motion axis
                twist_msg.angular.x = -imuAngVel[0](0);
                twist_msg.angular.y = -imuAngVel[0](1);
                twist_msg.angular.z = -imuAngVel[0](2);
                acc_msg.linear.x = -G0 * imuAcc[0](0);
                acc_msg.linear.y = -G0 * imuAcc[0](1);
                acc_msg.linear.z = -G0 * imuAcc[0](2);
                // set traj
                trajPt.accelerations.emplace_back(acc_msg);
                trajPt.velocities.emplace_back(twist_msg);
                trajPt.transforms.emplace_back(geoTF_msg);
                // publish
                eskf_1_pub.publish(trajPt); 
            }
            // bias pub
            if (eskf_bias_1_pub.getNumSubscribers() > 0){
                geometry_msgs::Accel bias_msg;
                bias_msg.linear.x = nominalBias_(0);
                bias_msg.linear.y = nominalBias_(1);
                bias_msg.linear.z = nominalBias_(2);
                bias_msg.angular.x = nominalBias_(3);
                bias_msg.angular.y = nominalBias_(4);
                bias_msg.angular.z = nominalBias_(5);
                eskf_bias_1_pub.publish(bias_msg);
            }
            // initial state
            if (eskf_init_1_pub.getNumSubscribers() > 0){
                tello_pub::SE3Pose SE3Pose_msg;
                SE3Pose_msg.header.stamp = ros::Time::now();
                SE3Pose_msg.tvec.x = initPos_(0);
                SE3Pose_msg.tvec.y = initPos_(1);
                SE3Pose_msg.tvec.z = initPos_(2);
                std::memcpy(
                    &SE3Pose_msg.rmat, initRot_.data(), 9 * sizeof(double)
                );
                eskf_init_1_pub.publish(SE3Pose_msg);
                // debug
                std::cout << "========[eskf init]========\nI1 to I0 rotation mat:\n" 
                          << initRot_ << std::endl
                          << "quaternion:\n" << Eigen::Quaterniond(initRot_).w() << std::endl
                          << Eigen::Quaterniond(initRot_).vec() << std::endl
                          << "I0 to I1 in I0 translation:\n" << initPos_ << std::endl;
            }
            // rviz visualization
            // publish odom for RViz
            if (rvizEnable[0]) {
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = "eskf_odom";
                odom_msg.pose.pose.position.x = nominalPosVel_(0);
                odom_msg.pose.pose.position.y = nominalPosVel_(1);
                odom_msg.pose.pose.position.z = nominalPosVel_(2);
                tf::quaternionEigenToMsg(
                    nominalQuat_, odom_msg.pose.pose.orientation
                );
                odom_msg.twist.twist.linear.x = nominalPosVel_(3);
                odom_msg.twist.twist.linear.y = nominalPosVel_(4);
                odom_msg.twist.twist.linear.z = nominalPosVel_(5);
                odom_msg.twist.twist.angular.x = -imuAngVel[0](0);
                odom_msg.twist.twist.angular.y = -imuAngVel[0](1);
                odom_msg.twist.twist.angular.z = -imuAngVel[0](2);

                eskf_odom_1_pub.publish(odom_msg);
            }
            break;

        case 2:
            // traj pub
            if (eskf_2_pub.getNumSubscribers() > 0){
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - startTime_2;
                // pose
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = nominalPosVel_(0);
                geoTF_msg.translation.y = nominalPosVel_(1);
                geoTF_msg.translation.z = nominalPosVel_(2);
                tf::quaternionEigenToMsg(nominalQuat_, geoTF_msg.rotation);
                // twist
                geometry_msgs::Twist twist_msg, acc_msg;
                twist_msg.linear.x = nominalPosVel_(3);
                twist_msg.linear.y = nominalPosVel_(4);
                twist_msg.linear.z = nominalPosVel_(5);

                // assign motion axis
                twist_msg.angular.x = -imuAngVel[1](0);
                twist_msg.angular.y = -imuAngVel[1](1);
                twist_msg.angular.z = -imuAngVel[1](2);
                acc_msg.linear.x = -G0 * imuAcc[1](0);
                acc_msg.linear.y = -G0 * imuAcc[1](1);
                acc_msg.linear.z = -G0 * imuAcc[1](2);
                // set traj
                trajPt.accelerations.emplace_back(acc_msg);
                trajPt.velocities.emplace_back(twist_msg);
                trajPt.transforms.emplace_back(geoTF_msg);
                // publish
                eskf_2_pub.publish(trajPt); 
            }
            // bias pub
            if (eskf_bias_2_pub.getNumSubscribers() > 0){
                geometry_msgs::Accel bias_msg;
                bias_msg.linear.x = nominalBias_(0);
                bias_msg.linear.y = nominalBias_(1);
                bias_msg.linear.z = nominalBias_(2);
                bias_msg.angular.x = nominalBias_(3);
                bias_msg.angular.y = nominalBias_(4);
                bias_msg.angular.z = nominalBias_(5);
                eskf_bias_2_pub.publish(bias_msg);
            }
            // initial state
            if (eskf_init_2_pub.getNumSubscribers() > 0){
                tello_pub::SE3Pose SE3Pose_msg;
                SE3Pose_msg.header.stamp = ros::Time::now();
                SE3Pose_msg.tvec.x = initPos_(0);
                SE3Pose_msg.tvec.y = initPos_(1);
                SE3Pose_msg.tvec.z = initPos_(2);
                std::memcpy(
                    &SE3Pose_msg.rmat, initRot_.data(), 9 * sizeof(double)
                );
                eskf_init_2_pub.publish(SE3Pose_msg);
                
                // debug
                std::cout << "========[eskf init]========\nI2 to I0 rotation mat:\n" 
                          << initRot_ << std::endl
                          << "quaternion:\n" << Eigen::Quaterniond(initRot_).w() << std::endl
                          << Eigen::Quaterniond(initRot_).vec() << std::endl
                          << "I0 to I2 in I0 translation:\n" << initPos_ << std::endl;
            }
            // rviz visualization
            // publish odom for RViz
            if (rvizEnable[1]) {
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = "eskf_odom";
                odom_msg.pose.pose.position.x = nominalPosVel_(0);
                odom_msg.pose.pose.position.y = nominalPosVel_(1);
                odom_msg.pose.pose.position.z = nominalPosVel_(2);
                tf::quaternionEigenToMsg(
                    nominalQuat_, odom_msg.pose.pose.orientation
                );
                odom_msg.twist.twist.linear.x = nominalPosVel_(3);
                odom_msg.twist.twist.linear.y = nominalPosVel_(4);
                odom_msg.twist.twist.linear.z = nominalPosVel_(5);
                odom_msg.twist.twist.angular.x = -imuAngVel[1](0);
                odom_msg.twist.twist.angular.y = -imuAngVel[1](1);
                odom_msg.twist.twist.angular.z = -imuAngVel[1](2);

                eskf_odom_2_pub.publish(odom_msg);
            }
            break;

        default:
            // debug
            ROS_INFO_STREAM("========[eskf]========\nNO DRONE ID MATCH: " 
                            << eskfObj.getDroneID() << "!");
    }
}

/*
@main
    test
*/
int main(int argc, char **argv){
    
    // for testing
    ros::init(argc, argv, "multi_drone_odom", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    
    bool testMultiDrone = false;
    
    // get nodename
    std::string ros_nodeName = ros::this_node::getName();
    
    // get param
    double t_smpl = 0.01;
    nh.getParam(ros_nodeName + "/testMultiDrone", testMultiDrone);
    nh.getParam(ros_nodeName + "/smpl_time", t_smpl);
    std::cout << "t_smpl: " << t_smpl << std::endl;

    // testing case
    if (!testMultiDrone){
        // drone 0 odom
        Drone0InerOdom test_drone0(nh, 0, true, t_smpl);
        ros::spin();
    }
    else {
        // three-drone VIO
        DronesVIO test_drones(nh, t_smpl);
        ros::spin();
        
        // use three threads
        //      wrong innovation signal data
        //      need to assign correct callback queues
        // ros::AsyncSpinner spinner(3);
        // spinner.start();
        // ros::waitForShutdown();
    }

    // Drone0InerOdom test_drone0(nh, 0, true);
    // DronesVIO test_drones(nh);

    // ros::spin();

    // return 0;

    // for testing --> single drone odom is okay
    // ros::init(argc, argv, "multi_drone_odom", ros::init_options::AnonymousName);
    // ros::NodeHandle nh;
    // Drone0InerOdom test(nh, 0, true);
    // ros::spin();

    return 0;
}