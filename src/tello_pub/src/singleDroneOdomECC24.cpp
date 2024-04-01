
# include "tello_pub/dronesVIO_ECC24.h"


/*
@Drone0InerOdom
    constructor w. nodehandler
*/
Drone0InerOdom::Drone0InerOdom(ros::NodeHandle& nh_, int drone_id, bool enableRViz) 
    : FullQuatESKF_ECC24(drone_id)
{
    /*==========================[ros-dep]====================*/
    // get nodename
    std::string ros_nodeName = ros::this_node::getName();
    // get param
    nh_.getParam(ros_nodeName + "/useImuQuat", useImuQuat);
    nh_.getParam(ros_nodeName + "/smpl_time", t_smpl);
    std::cout << ros_nodeName << std::endl << useImuQuat << std::endl
              << t_smpl << std::endl;


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
        // visualization
        eskf_odom_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName + "/eskf_odom", queue_size
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
        // visualization
        eskf_odom_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName + "/eskf_odom_origin", queue_size
        );
    }
    /*============================[ESKF setting]========================*/
    // initialize the imu vectors
    imuAcc = Eigen::Vector3d::Zero();
    imuAngVel = Eigen::Vector3d::Zero();
    
    // set the internal flag
    setStartFlag(StartOfTest);
    rvizEnable = enableRViz;
}
/*
@telloSOTCallback
    trigger the initialization of timer and start flag
*/
void Drone0InerOdom::telloSOTCallback(std_msgs::Empty const& msg){
    
    // std::cout << "AAAAA" << std::endl;
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
        FullQuatESKF_ECC24::imuCB(imuAcc, imuAngVel, imuQuat);
    }
    else {
        FullQuatESKF_ECC24::imuInputCB(imuAcc, imuAngVel, imuQuat);
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
    FullQuatESKF_ECC24::odomInertialCB(odomVel);
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
        FullQuatESKF_ECC24::predictState();
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
    FullQuatESKF_ECC24::getNominalState(
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
        acc_msg.linear.x = imuAcc(0);
        acc_msg.linear.y = imuAcc(1);
        acc_msg.linear.z = imuAcc(2);
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
        FullQuatESKF_ECC24::getErrCov(errCov);
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

/*
@main
    test
*/
int main(int argc, char **argv){

    // for testing --> single drone odom is okay
    // fixed version for ECC24
    ros::init(argc, argv, "single_drone_odom", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    Drone0InerOdom test(nh, 0, true);
    ros::spin();

    return 0;
}