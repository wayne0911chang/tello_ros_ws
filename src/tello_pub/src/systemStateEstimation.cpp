// for self-defined header
#include "tello_pub/systemStateEstimation.h"

/*================================ main content ============================*/
// @todo
//      Drone state as linear KF
//          available measurement
//              drone position and yaw
//      Target state as linear KF
//          output velocity x,y in local frame
//      Target odometry as ESKF (not yet)
//          output translation from world frame origin and rotation from world frame to current

/*
@LinearKalmanFilter
    state: 
        position, velocity, acceleration
        orientation, angular velocity
    setup the process equation, the measurement equation
    
    the predict process follows the timer event
    the measurement update follows the subscriber
*/
LinearKalmanFilter::LinearKalmanFilter(int targetType, bool useJoseph, ros::NodeHandle& nh_){

    // setup the timer horizon
    t_pred = 0.05;

    // setup the queue_size
    int queue_size = 1;

    // setting the wall and board ids
    wall_ids.push_back(11);     // wall marker in MD5F
    wall_ids.push_back(17);     // wall marker for testing in MD601
    // setting the human id     
    board_ids.push_back(0);     // setting as face ground truth
    board_ids.push_back(22);
    board_ids.push_back(25);

    // use Joseph form for numerically stability
    JosephForm = useJoseph;

    // threshold of quaternions ==> smaller than 10 deg
    quatThr = (1 - std::cos(M_PI/18)) * 0.5;

    // exp process
    StartOfTest = false;
    SOT_sub = nh_.subscribe(
        "/SOT", queue_size, &LinearKalmanFilter::telloSOTCallback, this
    );

    // debug
    std::cout << "target type: " << targetType << std::endl;

    // check the topic
    myType = targetType;
    /*
    @todo
        transform the sensors to the correct frame!!!
    */
    // LKF for the drone0 is the odometry of drone 0
    // under the initial frame of drone0's odom vel. frame
    if (targetType == DRONE_STATE_0){

        // constant acceleration
        // required topics
        //      velocity: odom
        //      acceleration: imu
        //      yaw and yaw rate: imu
        // publish topics
        //      all state: trajectory_msgs::MultiDOFJointTrajectoryPoint

        stateSize = 11;         // p v a psi wz
        measureSize = 3;        // p / v / a
        measureSizeAng = 2;     // psi wz

        // setting noise var
        nVelVar = 0.01;
        nAccVar = 0.01;
        nAngVelVar = 0.01;

        // setting state size
        state = Eigen::VectorXd::Zero(stateSize);

        // setting state matrices
        // F_(stateSize, stateSize);
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2) * 0.5;
        
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        
        F_.block<3, 3>(6, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 3) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3);

        F_.block<2, 2>(9, 9) << 1.0, t_pred, 0, 1;
        
        F_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        F_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);

        // setting the process noise
        // Q_(stateSize, stateSize);
        Q_ = Eigen::MatrixXd(stateSize, stateSize);

        Q_.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4) * nAccVar;
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3) * nAccVar;
        Q_.block<3, 3>(0, 6) = Q_.block<3, 3>(6, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 2) * nAccVar;
        
        Q_.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2) * nAccVar;
        Q_.block<3, 3>(3, 6) = Q_.block<3, 3>(6, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * t_pred * nAccVar;
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3) * nAccVar;

        Q_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        Q_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);
        Q_.block<2, 2>(9, 9) << std::pow(t_pred, 2), t_pred, t_pred, 1;
        Q_.block<2, 2>(9, 9) *= nAngVelVar;

        // setting the error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);

        // setting the measurement matrix
        // odom measure the velocity 
        H_v = Eigen::MatrixXd(measureSize, stateSize);
        H_v.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(3, 3);
        H_v.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3);
        H_v.block<3, 5>(0, 6) = Eigen::MatrixXd::Zero(3, 5);

        // H_imu(measureSize + measureSizeAng, stateSize);
        H_imu = Eigen::MatrixXd(measureSize + measureSizeAng, stateSize);
        H_imu.block<5, 6>(0, 0) = Eigen::MatrixXd::Zero(5, 6);
        H_imu.block<5, 5>(0, 6) = Eigen::MatrixXd::Identity(5, 5);

        // setup the measurement noise
        R_v = Eigen::Matrix3d::Identity(measureSize, measureSize) * nVelVar;
        R_a = Eigen::MatrixXd::Identity(
            measureSize + measureSizeAng, measureSize + measureSizeAng
        );
        R_a.block<3, 3>(0, 0) *= nAccVar;
        R_a.block<2, 2>(3, 3) *= nAngVelVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize + measureSizeAng);

        // initialize the orientation as all zero
        q_initImu.w = q_prevImu.w = 0;
        q_initImu.x = q_prevImu.x = 0;
        q_initImu.y = q_prevImu.y = 0;
        q_initImu.z = q_prevImu.z = 0;

        // initialize the angular velocity hat operator
        omegaHat = Eigen::Matrix3d::Zero(3, 3);
        
        // fixed the imu to camera transformation
        R_imu_to_cam << 0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0,
                        1.0, 0.0, 0.0;

        // setup the ros subscriber, publisher, timer
        odom_sub = nh_.subscribe(
            "/tello_0/odom", queue_size,
            &LinearKalmanFilter::odomCB, this
        );
        imu_sub = nh_.subscribe(
            "/tello_0/imu", queue_size,
            &LinearKalmanFilter::imuCB, this
        );
        state_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/tello_0_KF_state", queue_size
        );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );      
    }
    else if (targetType == DRONE_STATE_1){

        // constant acceleration
        // required topics
        //      position and yaw angle: aruco_pose_arr
        //      velocity: odom
        //      acceleration: imu
        // publish topics
        //      all state: trajectory_msgs::MultiDOFJointTrajectoryPoint

        stateSize = 11;         // p v a psi wz
        measureSize = 3;        // p / v / a
        measureSizeAng = 2;     // psi wz

        // setting noise var
        nPositionVar = 0.01;
        nVelVar = 0.01;
        nAccVar = 0.01;
        nAngVelVar = 0.01;

        // desired ids
        desiredArUcoID = 25;

        // setting state size
        state = Eigen::VectorXd::Zero(stateSize);

        // setting state matrices
        // F_(stateSize, stateSize);
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2) * 0.5;
        
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        
        F_.block<3, 3>(6, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 3) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3);

        F_.block<2, 2>(9, 9) << 1.0, t_pred, 0, 1;
        
        F_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        F_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);

        // setting the process noise
        // Q_(stateSize, stateSize);
        Q_ = Eigen::MatrixXd(stateSize, stateSize);

        Q_.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4) * nAccVar;
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3) * nAccVar;
        Q_.block<3, 3>(0, 6) = Q_.block<3, 3>(6, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 2) * nAccVar;
        
        Q_.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2) * nAccVar;
        Q_.block<3, 3>(3, 6) = Q_.block<3, 3>(6, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * t_pred * nAccVar;
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3) * nAccVar;

        Q_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        Q_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);
        Q_.block<2, 2>(9, 9) << std::pow(t_pred, 2), t_pred, t_pred, 1;
        Q_.block<2, 2>(9, 9) *= nAngVelVar;

        // setting the error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);

        // setting the measurement matrix
        // H_p(measureSize, stateSize);
        H_p = Eigen::MatrixXd(measureSize, stateSize);
        H_p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        H_p.block<3, 8>(0, 3) = Eigen::MatrixXd::Zero(3, 8);

        // odom measure the velocity 
        // H_v(measureSize, stateSize);
        H_v = Eigen::MatrixXd(measureSize, stateSize);
        H_v.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(3, 3);
        H_v.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3);
        H_v.block<3, 5>(0, 6) = Eigen::MatrixXd::Zero(3, 5);

        // H_imu(measureSize + measureSizeAng, stateSize);
        H_imu = Eigen::MatrixXd(measureSize + measureSizeAng, stateSize);
        H_imu.block<5, 6>(0, 0) = Eigen::MatrixXd::Zero(5, 6);
        H_imu.block<5, 5>(0, 6) = Eigen::MatrixXd::Identity(5, 5);

        // setup the measurement noise
        R_p = Eigen::MatrixXd::Identity(measureSize, measureSize) * nPositionVar;
        R_v = Eigen::Matrix3d::Identity(measureSize, measureSize) * nVelVar;
        R_a = Eigen::MatrixXd::Identity(
            measureSize + measureSizeAng, measureSize + measureSizeAng
        );
        R_a.block<3, 3>(0, 0) *= nAccVar;
        R_a.block<2, 2>(3, 3) *= nAngVelVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize + measureSizeAng);

        // initialize the orientation as all zero
        q_initImu.w = q_prevImu.w = 0;
        q_initImu.x = q_prevImu.x = 0;
        q_initImu.y = q_prevImu.y = 0;
        q_initImu.z = q_prevImu.z = 0;

        // initialize the angular velocity hat operator
        omegaHat = Eigen::Matrix3d::Zero(3, 3);
        
        // fixed the imu to camera transformation
        R_imu_to_cam << 0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0,
                        1.0, 0.0, 0.0;

        // setup the ros subscriber, publisher, timer
        pose_sub = nh_.subscribe(
            "/tello_0/aruco_pose_arr", queue_size,
            &LinearKalmanFilter::ArUcoPoseArrCB, this
        );
        odom_sub = nh_.subscribe(
            "/tello_1/odom", queue_size,
            &LinearKalmanFilter::odomCB, this
        );
        imu_sub = nh_.subscribe(
            "/tello_1/imu", queue_size,
            &LinearKalmanFilter::imuCB, this
        );
        state_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/tello_1_KF_state", queue_size
        );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );      
    }
    else if (targetType == DRONE_STATE_2){

        // const acceleration
        // required topics
        //      position and yaw angle: aruco_pose_arr
        //      velocity: odom
        //      acceleration: imu

        stateSize = 11;
        measureSize = 3;
        measureSizeAng = 2;

        // setting noise var
        // setting noise var
        nPositionVar = 0.01;
        nVelVar = 0.01;
        nAccVar = 0.01;
        nAngVelVar = 0.01;

        desiredArUcoID = 22;
        
        // setting state
        state = Eigen::VectorXd::Zero(stateSize);

        // setting state matrices
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2) * 0.5;
        
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        
        F_.block<3, 3>(6, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 3) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3);

        F_.block<2, 2>(9, 9) << 1.0, t_pred, 0, 1;
        
        F_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        F_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);

        // setting the process noise
        Q_ = Eigen::MatrixXd(stateSize, stateSize);
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4);
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3);
        Q_.block<3, 3>(0, 6) = Q_.block<3, 3>(6, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 2);
        
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2);
        Q_.block<3, 3>(3, 6) = Q_.block<3, 3>(6, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * t_pred;
        Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity(3, 3);

        Q_.block<9, 2>(0, 9) = Eigen::MatrixXd::Zero(9, 2);
        Q_.block<2, 9>(9, 0) = Eigen::MatrixXd::Zero(2, 9);
        Q_.block<2, 2>(9, 9) << std::pow(t_pred, 2), t_pred, t_pred, 1;
        Q_.block<2, 2>(9, 9) *= nAngVelVar;

        // setting error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);

        // setting the measurement matrix
        H_p = Eigen::MatrixXd(measureSize, stateSize);
        H_p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        H_p.block<3, 8>(0, 3) = Eigen::MatrixXd::Zero(3, 8);

        // odom measure the velocity 
        H_v = Eigen::MatrixXd(measureSize, stateSize);
        H_v.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero(3, 3);
        H_v.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3);
        H_v.block<3, 5>(0, 6) = Eigen::MatrixXd::Zero(3, 5);

        // imu measure the acceleration and yaw angle and yaw rate
        H_imu = Eigen::MatrixXd(measureSize + measureSizeAng, stateSize);
        H_imu.block<5, 6>(0, 0) = Eigen::MatrixXd::Zero(5, 6);
        H_imu.block<5, 5>(0, 6) = Eigen::MatrixXd::Identity(5, 5);

        // setup the measurement noise
        R_p = Eigen::MatrixXd::Identity(measureSize, measureSize) * nPositionVar;
        R_v = Eigen::Matrix3d::Identity(measureSize, measureSize) * nVelVar;
        R_a = Eigen::MatrixXd::Identity(
            measureSize + measureSizeAng, measureSize + measureSizeAng
        );
        R_a.block<3, 3>(0, 0) *= nAccVar;
        R_a.block<2, 2>(3, 3) *= nAngVelVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize + measureSizeAng);

        // initialize the orientation as all zero
        q_initImu.w = q_prevImu.w = 0;
        q_initImu.x = q_prevImu.x = 0;
        q_initImu.y = q_prevImu.y = 0;
        q_initImu.z = q_prevImu.z = 0;

        // initialize the angular velocity hat operator
        omegaHat = Eigen::Matrix3d::Zero(3, 3);

        // fixed the imu to camera transformation
        R_imu_to_cam << 0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0,
                        1.0, 0.0, 0.0;

        // setup the ros subscriber, publisher, timer
        pose_sub = nh_.subscribe(
            "/tello_0/aruco_pose_arr", queue_size,
            &LinearKalmanFilter::ArUcoPoseArrCB, this
        );
        odom_sub = nh_.subscribe(
            "/tello_2/odom", queue_size,
            &LinearKalmanFilter::odomCB, this
        );
        imu_sub = nh_.subscribe(
            "/tello_2/imu", queue_size,
            &LinearKalmanFilter::imuCB, this
        );
        state_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/tello_2_KF_state", queue_size
        );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );
    }
    else if (targetType == TARGET_FACE){

        // const velocity is suitable
        // required topics
        //      position and yaw angle: face_pose_arr
        stateSize = 6;
        measureSize = 3;

        // setting noise variance
        nPositionVar = 0.01;
        nAccVar = 0.01;
        
        // setting state
        state = Eigen::VectorXd::Zero(stateSize);

        // setting the state transition matrix
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);

        // setting the process noise
        Q_ = Eigen::MatrixXd(stateSize, stateSize);
        Q_.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4);
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) =
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3);
        Q_.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2);
        Q_ *= nAccVar;

        // setting the initial error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);
        
        // setup the measurement matrix
        H_p = Eigen::MatrixXd(measureSize, stateSize);
        H_p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        H_p.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(3, 3);

            // setup the measurement noise
        R_p = Eigen::MatrixXd::Identity(measureSize, measureSize) * nPositionVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize);

        // setup the ros subscriber, publisher, timer
        pose_sub = nh_.subscribe(
            "/tello_1/face_pose_arr", queue_size,
            &LinearKalmanFilter::FacePoseArrCB, this
        );
        state_pub = nh_.advertise<geometry_msgs::PoseStamped>(
            "/face_KF_state", queue_size
        );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );
    }
    else if (targetType == TARGET_STATE_0){

        // const velocity is suitable
        // required topics
        //      position and rotation: /tello_0/human_pose_arr
        stateSize = 6;
        measureSize = 3;

        // setting noise variance
        nPositionVar = 0.01;
        nAccVar = 0.01;
        
        // setting state
        state = Eigen::VectorXd::Zero(stateSize);

        // setting the state transition matrix
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);

        // setting the process noise
        Q_ = Eigen::MatrixXd(stateSize, stateSize);
        Q_.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4);
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) =
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3);
        Q_.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2);
        Q_ *= nAccVar;

        // setting the initial error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);
        
        // setup the measurement matrix
        H_p = Eigen::MatrixXd(measureSize, stateSize);
        H_p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        H_p.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(3, 3);

            // setup the measurement noise
        R_p = Eigen::MatrixXd::Identity(measureSize, measureSize) * nPositionVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize);

        // setup the ros subscriber, publisher, timer
        pose_sub = nh_.subscribe(
            "/tello_0/human_pose_arr", queue_size,
            &LinearKalmanFilter::YOLOPoseArrCB, this
        );
        // state_pub = nh_.advertise<geometry_msgs::PoseStamped>(
        //     "/face_KF_state", queue_size
        // );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );
    }
    else if (targetType == TARGET_STATE_2){

        // const velocity is suitable
        // required topics
        //      position and yaw angle: face_pose_arr
        stateSize = 6;
        measureSize = 3;

        // setting noise variance
        nPositionVar = 0.01;
        nAccVar = 0.01;
        
        // setting state
        state = Eigen::VectorXd::Zero(stateSize);

        // setting the state transition matrix
        F_ = Eigen::MatrixXd(stateSize, stateSize);
        F_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        F_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        F_.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero(3, 3);
        F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity(3, 3);

        // setting the process noise
        Q_ = Eigen::MatrixXd(stateSize, stateSize);
        Q_.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4);
        Q_.block<3, 3>(0, 3) = Q_.block<3, 3>(3, 0) =
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3);
        Q_.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2);
        Q_ *= nAccVar;

        // setting the initial error covariance
        P_ = Eigen::MatrixXd::Identity(stateSize, stateSize);
        
        // setup the measurement matrix
        H_p = Eigen::MatrixXd(measureSize, stateSize);
        H_p.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        H_p.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero(3, 3);

            // setup the measurement noise
        R_p = Eigen::MatrixXd::Identity(measureSize, measureSize) * nPositionVar;

        // initialize the kalman gain
        kalmanGain = Eigen::MatrixXd::Zero(stateSize, measureSize);

        // initialize the quaternions
        q_initYOLO.w = q_prevYOLO.w = 0;
        q_initYOLO.x = q_prevYOLO.x = 0;
        q_initYOLO.y = q_prevYOLO.y = 0;
        q_initYOLO.z = q_prevYOLO.z = 0;

        // setup the ros subscriber, publisher, timer
        pose_sub = nh_.subscribe(
            "/tello_2/human_pose_arr", queue_size,
            &LinearKalmanFilter::YOLOPoseArrCB, this
        );
        // state_pub = nh_.advertise<geometry_msgs::PoseStamped>(
        //     "/face_KF_state", queue_size
        // );
        predictStateTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearKalmanFilter::predictState, this
        );
        // try stop the thread first to avoid run before initialization
        // predictStateTimer.stop();
    }
    else {
        ROS_INFO_STREAM(
            "========[Linear KF]========\nWRONG TYPE OF TARGET: " << targetType
        );
        return;
    }
    // debug
    std::cout << "state transition matrix: " << std::endl << F_ << std::endl
                << "noise covariance matrix: " << std::endl << Q_ << std::endl
                << "state variable: " << std::endl << state << std::endl
                << "predict state timer started or not: " << predictStateTimer.hasStarted() << std::endl;
}
/*
@telloSOTCallback
    trigger the initialization of timer and start flag
    only when SOT is false
*/
void LinearKalmanFilter::telloSOTCallback(std_msgs::Empty const& msg){

    if (!StartOfTest) {
        predictStateTimer.start();
        startTimeKF = ros::Time::now();
        StartOfTest = true;
    }
}
/*
@publishPosteriorState
    publish the posterior estimated state
*/
void LinearKalmanFilter::publishPosteriorState(){
    
    // check the subscribers
    if (state_pub.getNumSubscribers() < 1) {
        ROS_INFO_STREAM("========[linear kf]========\nNO KF SUBs!");
    }
    else {
        // publish posterior states
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajPoint;
        trajPoint.time_from_start = ros::Time::now() - startTimeKF;
        
        // debug
        ROS_INFO("========[linear kf]========\nsampled timestamp: %f", 
                    trajPoint.time_from_start.toSec());
        
        // debug
        std::cout << "state: " << std::endl << state << std::endl;

        geometry_msgs::Transform geometryTF_msg;
        geometryTF_msg.translation.x = state(0);
        geometryTF_msg.translation.y = state(1);
        geometryTF_msg.translation.z = state(2);

        // naive method
        // need to ensure the rotation error
        // geometryTF_msg.rotation = q_measured;
        tf2::Quaternion q_tf2;
        q_tf2.setRPY(0, 0, state(9));
        tf2::convert(geometryTF_msg.rotation, q_tf2);

        geometry_msgs::Twist acc_msg, vel_msg;
        vel_msg.linear.x = state(3);
        vel_msg.linear.y = state(4);
        vel_msg.linear.z = state(5); 
        vel_msg.angular.z = state(10);

        acc_msg.linear.x = state(6);
        acc_msg.linear.y = state(7);
        acc_msg.linear.z = state(8);

        // collect the states
        trajPoint.accelerations.emplace_back(acc_msg);
        trajPoint.transforms.emplace_back(geometryTF_msg);
        trajPoint.velocities.emplace_back(vel_msg);

        state_pub.publish(trajPoint);
    }
}
/*
@publishPosteriorStateFace
    publish the posterior estimated state for face pose
*/
void LinearKalmanFilter::publishPosteriorStateFace(){
    
    // check the subscribers
    if (state_pub.getNumSubscribers() < 1) {
        ROS_INFO_STREAM("========[linear kf face]========\nNO KF FACE SUBs!");
    }
    else {
        // publish posterior states
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = state(0);
        pose_msg.pose.position.y = state(1);
        pose_msg.pose.position.z = state(2);

        // assign the face orientation with prevFace
        pose_msg.pose.orientation = q_prevFace;

        // debug
        ROS_INFO("========[linear kf face]========\nsampled timestamp: %f", 
                    pose_msg.header.stamp.toSec());
        
        // debug
        std::cout << "state: " << std::endl << state << std::endl;

        state_pub.publish(pose_msg);
    }
}
/*
@ArUcoPoseArrCB
    update the aurco pose with received pose array
    and then publish the posterior estimation w.r.t. pose as
        trajectory_msgs::MultiDOFJointTrajectoryPoint 
*/
void LinearKalmanFilter::ArUcoPoseArrCB(tello_sub::ArUcoPoseArr const& arucoPoseArr){
    
    // debug
    ROS_INFO_STREAM("========[aruco pose kf]========");

    // assign values
    Eigen::Vector3d p_measured;
    geometry_msgs::Quaternion q_measured;

    // check the ids are desired drone marker pose
    for (int i=0; i<arucoPoseArr.poses.size(); i++){
        
        // debug
        std::cout << "aruco id: " << arucoPoseArr.ids[i] << std::endl;

        if (arucoPoseArr.ids[i] == desiredArUcoID) {
            p_measured << arucoPoseArr.poses[i].position.x,
                            arucoPoseArr.poses[i].position.y,
                            arucoPoseArr.poses[i].position.z;
            q_measured = arucoPoseArr.poses[i].orientation;
            break;
        }
        else {
            ROS_INFO_STREAM(
                "========[aruco pose kf]========\nNO MATCHED IDs WITH " << desiredArUcoID << "!"
            );
        }
    }
    // debug
    std::cout << "p_measure" << std::endl << p_measured << std::endl
                << "innov corr inv: " << std::endl << (R_p + H_p * P_ * H_p.transpose()).inverse() << std::endl
                << "prior err cov: " << std::endl << P_ << std::endl; 

    // compute the kalman gain iteration
    kalmanGain = P_ * H_p.transpose() * (R_p + H_p * P_ * H_p.transpose()).inverse();
    state = state + kalmanGain * (p_measured - H_p * state);

    // update the error cov
    if (JosephForm) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(stateSize, stateSize);
        temp = temp - kalmanGain * H_p;
        P_ = temp * P_ * temp.transpose() + kalmanGain * R_p * kalmanGain.transpose();
    }
    else {
        P_ = P_ - kalmanGain * H_p * P_;
    }
    // debug
    std::cout << "kalman gain: " << std::endl << kalmanGain << std::endl
                << "posterior state: " << std::endl << state << std::endl
                << "posterior err cov: " << std::endl << P_ << std::endl;

    // publish posterior states
    // publishPosteriorState();

    // update the transformation
    // check SOT or not
    if (~StartOfTest){
        q_prevMarker = q_measured;
    }
    else {
        double cosDist = findQuatCosDist(q_measured, q_prevMarker);
        // debug
        std::cout << "cosine distance (1-cos)/2 of previous and current quaternion:" << cosDist << std::endl;

        if (cosDist < quatThr){
            q_prevMarker = q_measured;
        }
        else {
            ROS_INFO_STREAM("========[aruco pose kf]========\nWRONG ORIENTATION!");
        }
    }
}
/*
@getCam0toMarker
    Give the rotation matrix only
    for the drone pose 1 => desiredArUcoID = 25
    for the drone pose 2 => desiredArUcoID = 22
*/
void LinearKalmanFilter::getCam0toMarker(Eigen::Matrix3d& R_m_to_c0){
    
    // check SOT or not 
    // if (~StartOfTest) {
    //     ROS_INFO_STREAM("========[yolo pose kf]========\nSOT NOT YET!");
    // }
    // else {
    // debug
    std::cout << "getting rotation from marker " << desiredArUcoID
                << " to camera0" << std::endl;

    Eigen::Quaterniond q_temp(
        q_prevMarker.w,
        q_prevMarker.x,
        q_prevMarker.y,
        q_prevMarker.z
    );
    q_temp.normalize();

    // debug
    std::cout << "q in geometry_msgs: " << std::endl << q_prevMarker << std::endl
                << "q in Eigen: " << q_temp.w() << ", " << q_temp.x() << ", " << q_temp.y() << ", " << q_temp.z() << std::endl;
    R_m_to_c0 = q_temp.toRotationMatrix();
    // }
}
/*
@getOrientationToInit
    for drone 2 and drone 1
    get the filtered orientation relative to initial condition 
*/
void LinearKalmanFilter::getOrientationToCamInit(Eigen::Matrix3d& R_c_to_init){
    // check initialized or not
    if (
        (q_initImu.w == 0) && (q_initImu.x == 0) && (q_initImu.y == 0) && (q_initImu.z == 0)
    ){
        ROS_INFO_STREAM("========[linear kf]========\nORIENTATION NOT YET UPDATED!! SET AS IDENTITY!!");
        // set as identity   
        R_c_to_init = Eigen::Matrix3d::Identity(3, 3);
    }
    else {
        Eigen::Quaterniond q_0(
            q_initImu.w, q_initImu.x, q_initImu.y, q_initImu.z
        ), q_c(
            q_prevImu.w, q_prevImu.x, q_prevImu.y, q_prevImu.z
        );
        // get euler angle
        Eigen::Vector3d eulerAngle_c = q_c.toRotationMatrix().eulerAngles(0, 1, 2);

        // current camera pose 
        // use the filtered yaw angle of imu
        tf::Quaternion qc_tf;
        qc_tf.setRPY(eulerAngle_c(0), eulerAngle_c(1), state(9));
        
        Eigen::Matrix3d R_c;
        tf::matrixTFToEigen(tf::Matrix3x3(qc_tf), R_c);

        // Find the transformation from cam current to cam 0
        R_c_to_init = 
            R_imu_to_cam * q_0.toRotationMatrix() * R_c.transpose() * R_imu_to_cam.transpose(); 
    }
}
/*
@FacePoseArrCB
    update the face pose 
    and publish the posterior state w.r.t. face pose measurement as
        geometry_msgs::Pose
*/
void LinearKalmanFilter::FacePoseArrCB(geometry_msgs::PoseArray const& poseArr){

    // debug
    ROS_INFO_STREAM("========[face pose kf]========");

    Eigen::Vector3d p_measured;
    geometry_msgs::Quaternion q_measured;
    for (int i=0; i<poseArr.poses.size(); i++){

        // assume the pose received is the correct face
        p_measured << poseArr.poses[i].position.x,
                        poseArr.poses[i].position.y,
                        poseArr.poses[i].position.z;
        q_measured = poseArr.poses[i].orientation;
    }
    // debug
    std::cout << "p_measure: " << std::endl << p_measured << std::endl
                << "kalman gain: " << std::endl << kalmanGain << std::endl
                << "Innov Corr: " << std::endl << (R_p + H_p * P_ * H_p.transpose()) << std::endl
                << "Innov Corr inv: " << std::endl << (R_p + H_p * P_ * H_p.transpose()).inverse() << std::endl
                << "prior err cov: " << std::endl << P_ << std::endl;

    // compute the kalman gain corresponding to the iteration
    kalmanGain = P_ * H_p.transpose() * (R_p + H_p * P_ * H_p.transpose()).inverse();
    state = state + kalmanGain * (p_measured - H_p * state);

    // update the error covariance
    if (JosephForm) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(stateSize, stateSize);
        temp = temp - kalmanGain * H_p;
        P_ = temp * P_ * temp.transpose() + kalmanGain * R_p * kalmanGain.transpose();
    }
    else {
        P_ = P_ - kalmanGain * H_p * P_;
    }
    // debug
    std::cout << "posterior state estimation: " << std::endl << state << std::endl
                << "posterior err cov: " << std::endl << P_ << std::endl;

    // publish posterior states 
    // publishPosteriorStateFace();

    if (~StartOfTest){
        q_prevFace = q_measured;
    }
    else {
        // update the transformation
        double cosDist = findQuatCosDist(q_measured, q_prevFace);
        // debug
        std::cout << "cosine distance (1-cos)/2 of previous and current quaternion:" << cosDist << std::endl;

        if (cosDist < quatThr){
            q_prevFace = q_measured;
        }
        else {
            ROS_INFO_STREAM("========[face pose kf]========\nWRONG ORIENTATION!");
        }
    }
}
/*
@YOLOPoseArrCB
    update the human pose from yolo 
    no need to publish the state
*/
void LinearKalmanFilter::YOLOPoseArrCB(geometry_msgs::PoseArray const& poseArr){

    // debug
    ROS_INFO_STREAM("========[yolo pose kf]========");

    Eigen::Vector3d p_measured;
    geometry_msgs::Quaternion q_measured;
    for (int i=0; i<poseArr.poses.size(); i++){

        // assume the pose received is the correct face
        p_measured << poseArr.poses[i].position.x,
                        poseArr.poses[i].position.y,
                        poseArr.poses[i].position.z;
        q_measured = poseArr.poses[i].orientation;
    }
    // debug
    std::cout << "p_measure: " << std::endl << p_measured << std::endl
                << "kalman gain: " << std::endl << kalmanGain << std::endl
                << "Innov Corr: " << std::endl << (R_p + H_p * P_ * H_p.transpose()) << std::endl
                << "Innov Corr inv: " << std::endl << (R_p + H_p * P_ * H_p.transpose()).inverse() << std::endl
                << "prior err cov: " << std::endl << P_ << std::endl;

    // compute the kalman gain corresponding to the iteration
    kalmanGain = P_ * H_p.transpose() * (R_p + H_p * P_ * H_p.transpose()).inverse();
    state = state + kalmanGain * (p_measured - H_p * state);

    // update the error covariance
    if (JosephForm) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(stateSize, stateSize);
        temp = temp - kalmanGain * H_p;
        P_ = temp * P_ * temp.transpose() + kalmanGain * R_p * kalmanGain.transpose();
    }
    else {
        P_ = P_ - kalmanGain * H_p * P_;
    }
    // debug
    std::cout << "posterior state estimation: " << std::endl << state << std::endl
                << "posterior err cov: " << std::endl << P_ << std::endl;

    // publish posterior states 
    // publishPosteriorStateFace();

    if (~StartOfTest){
        q_initYOLO = q_prevYOLO = q_measured;
    }
    else {
        // update the transformation
        double cosDist = findQuatCosDist(q_measured, q_prevYOLO);
        // debug
        std::cout << "cosine distance (1-cos)/2 of previous and current quaternion:" << cosDist << std::endl;

        if (cosDist < quatThr){
            q_prevYOLO = q_measured;
        }
        else {
            ROS_INFO_STREAM("========[yolo pose kf]========\nWRONG ORIENTATION!");
        }
    }
}
/*
@getCamInitToHumanInit
    get the camera0 initial frame to the leader's frame
    translation is get from the initial state.head(3)
*/
void LinearKalmanFilter::getCamInitToHumanInit(
    Eigen::Matrix3d& R_c_init_to_L_init
){
    if (
        (q_initYOLO.w == 0) && (q_initYOLO.x == 0) && (q_initYOLO.y == 0) && (q_initYOLO.z == 0)
    ){
        ROS_INFO_STREAM("========[linear kf]========\nORIENTATION NOT YET UPDATED!! SET AS IDENTITY!!");
        // set as default
        R_c_init_to_L_init << 1, 0, 0,
                              0, 0, 1,
                              0, -1, 0;
    }
    else {
        Eigen::Quaterniond q_0(
            q_initYOLO.w, q_initYOLO.x, q_initYOLO.y, q_initYOLO.z
        );
        R_c_init_to_L_init = q_0.toRotationMatrix();
    }
        
}
/*
@odomCB
    update the vel with received pose array
*/
void LinearKalmanFilter::odomCB(nav_msgs::Odometry const& odom){

    // debug
    ROS_INFO_STREAM("========[odom kf]========");

    // assign the measurement vector 
    Eigen::Vector3d odom_measured;
    odom_measured << odom.twist.twist.linear.x,
                        odom.twist.twist.linear.y,
                        odom.twist.twist.linear.z;
    // debug
    std::cout << "odom_measured: " << std::endl << odom_measured << std::endl
                << "innov corr inv: " << std::endl << (R_v + H_v * P_ * H_v.transpose()) << std::endl
                << "prior err cov: " << std::endl << P_ << std::endl;

    // compute the kalman gain
    kalmanGain = P_ * H_v.transpose() * (R_v + H_v * P_ * H_v.transpose()).inverse();
    state = state + kalmanGain * (odom_measured - H_v * state);
    
    // update the error cov
    if (JosephForm) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(stateSize, stateSize);
        temp = temp - kalmanGain * H_v;
        P_ = temp * P_ * temp.transpose() + kalmanGain * R_v * kalmanGain.transpose();
    }
    else {
        P_ = P_ - kalmanGain * H_v * P_;
    }
    // debug
    std::cout << "kalman gain: " << std::endl << kalmanGain << std::endl
                << "posterior state: " << std::endl << state << std::endl
                << "posterior err cov: " << std::endl << P_ << std::endl;
}
/*
@imuCB
    update the acc, yaw, yaw rate with received pose array
    drone0 --> all state update on the cmd vel frame
    drone1, 2 --> all state update on the marker frame
*/
void LinearKalmanFilter::imuCB(sensor_msgs::Imu const& imu){
    
    double roll, pitch, yaw, yawRate;
    geometry_msgs::Quaternion q_measured = imu.orientation;

    // assign yaw
    tf::Quaternion q_tf;
    tf::quaternionMsgToTF(q_measured, q_tf);
    tf::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);

    // debug
    ROS_INFO_STREAM("========[imu kf]========\nRPY from tf: "
        << roll << ", " << pitch << ", " << yaw);

    // assign yaw rate
    yawRate = imu.angular_velocity.z;

    // assign the measurement vector 
    Eigen::VectorXd imu_measured(5);
    if (myType == DRONE_STATE_0) {
        imu_measured << -imu.linear_acceleration.z,
                        -imu.linear_acceleration.y,
                        imu.linear_acceleration.x,
                        yaw,        // camera y axis
                        yawRate;    // camera y axis
    }
    else if (myType == DRONE_STATE_1) {
        // from imu acc frame to M1 frame
        imu_measured << imu.linear_acceleration.y,
                        imu.linear_acceleration.z,
                        imu.linear_acceleration.x,
                        yaw,        // camera y axis
                        yawRate;    // camera y axis
    }
    else if (myType == DRONE_STATE_2) {
        // from imu acc frame to M2 frame
        imu_measured << imu.linear_acceleration.x,
                        imu.linear_acceleration.z,
                        -imu.linear_acceleration.y,
                        yaw,
                        yawRate;
    }
    // debug
    std::cout << "imu_measured: "<< std::endl << imu_measured << std::endl;

    // compute the kalman gain
    kalmanGain = P_ * H_imu.transpose() * (R_a + H_imu * P_ * H_imu.transpose()).inverse();
    state = state + kalmanGain * (imu_measured - H_imu * state);
    
    // debug
    std::cout << "q_measured: " << std::endl << q_measured << std::endl
                << "innov corr inv: " << std::endl << (R_a + H_imu * P_ * H_imu.transpose()) << std::endl
                << "prior err cov: " << std::endl << P_ << std::endl;
    
    // update the error cov
    if (JosephForm) {
        Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(stateSize, stateSize);
        temp = temp - kalmanGain * H_imu;
        P_ = temp * P_ * temp.transpose() + kalmanGain * R_a * kalmanGain.transpose();
    }
    else {
        P_ = P_ - kalmanGain * H_imu * P_;
    }
    // debug
    std::cout << "kalman gain: " << std::endl << kalmanGain << std::endl
                << "posterior state: " << std::endl << state << std::endl
                << "posterior err cov: " << std::endl << P_ << std::endl;

    // publish posterior state
    // publishPosteriorState();

    if (~StartOfTest) {
        q_initImu = q_prevImu = q_measured;
    }
    else {
        // check the orientation is not deviate largely
        double cosDist = findQuatCosDist(q_measured, q_prevImu);
        // debug
        std::cout << "cosine distance (1-cos)/2 of previous and current quaternion: "
                    << cosDist << std::endl;
        if (cosDist < quatThr){
            q_prevImu = q_measured;

            // update the hat operator
            // yaw from imu is downward
            // camera rotating in the downward z = camera y          
            omegaHat = Eigen::Matrix3d::Zero(3, 3);
            omegaHat(0, 2) = yawRate;
            omegaHat(2, 0) = -yawRate;
        }
        else {
            ROS_INFO_STREAM("========[imu pose kf]========\nWRONG ORIENTATION!");
        }
    }
}
/*
@predictState
    publish the last posterior state update
    compute the prior state and prior error cov. only
*/
void LinearKalmanFilter::predictState(ros::TimerEvent const& ev){
    
    // check the stateSize and publish state
    if (stateSize == 11) { publishPosteriorState(); }
    else if (stateSize == 6) { publishPosteriorStateFace(); }
    else {
        ROS_INFO_STREAM("========[linear KF]========\nUNKNOWN STATE SIZE!!");
    }
    // update the state
    state = F_ * state;
    P_ = F_ * P_ * F_.transpose() + Q_;
}
/*lkf_t0(TARGET_STATE_0, true, nh_), 
        lkf_t2(TARGET_STATE_2, true, nh_),
        lkf_d2(DRONE_STATE_2, true, nh_),
        lkf_d0(DRONE_STATE_0, true, nh_)
    {ateDroneOdom(){

}


/*
@LinearMinVarFusion
    the cross correlation of the two KF is
    Fusion the two KF result in the sense of minimum variance
        x* = A1*x1 + A2*x2
    contain two drone position KF inside
    take the transformation of marker 2 to cam 0

    produce constant velocity KF for double input from drone0 and drone2
        Target state => ros_ns + "/human_pose_arr"
*/
class LinearMinVarFusion
{
    // ros-dep
    // ros::NodeHandle nh_;
    ros::Subscriber SOT_sub;
    ros::Publisher odom_pub, world_odom_pub, world_odom_pred_pub;
    ros::Timer fusionTimer;
    ros::Time startTimeFusion;
    int queue_size;
    double t_pred;
    bool StartOfTest;
    double quatThr;

    // KFs
    LinearKalmanFilter lkf_t0, lkf_t2, lkf_d2, lkf_d0;

    // fusion
    int fusionSize, stateSize;
    Eigen::MatrixXd A_;         // weighting A = [A_0; A_2]
    Eigen::MatrixXd Sigma_;     // cov Sigma = [P_00, P_02; P_20, P_22]
    Eigen::MatrixXd F_fusion, Q_fusion, P_fusion;
    
    Eigen::Matrix3d R_m2_to_c2, R_c0_init_to_L_init;
    Eigen::Vector3d t_m2_to_c2;             // in the m2 frame
    Eigen::Vector3d t_c0_init_to_L_init;    // in the C0 init frame

    Eigen::VectorXd stateFused, stateFusedWorld;

public:
    /*
    @LinearMinVarFusion
        get the number of fused kf
        initialize the cross cov and cov by the fused kf
    */
    explicit LinearMinVarFusion(ros::NodeHandle& nh_) : 
        lkf_t0(TARGET_STATE_0, true, nh_), 
        lkf_t2(TARGET_STATE_2, true, nh_),
        lkf_d2(DRONE_STATE_2, true, nh_),
        lkf_d0(DRONE_STATE_0, true, nh_)
    {
        // debug
        std::cout << "========[LMV fusion]========" << std::endl;

        // setting parameters
        t_pred = 0.05;
        queue_size = 1;
        const int dim_0 = lkf_t0.getStateSize();
        const int dim_2 = lkf_t2.getStateSize();
        fusionSize = dim_0 + dim_2;
        stateSize = dim_0;

        // check the dimension
        if ((dim_0 != dim_2) || (dim_0 != 6) || (dim_2 != 6)){
            
            // debug
            std::cout << "========[LMV fusion]========" << std::endl
                      << "unmatched stateSize!" << std::endl
                      << "dim 0 = " << dim_0 << ", dim 2 = " << dim_2 << std::endl;
            return;
        }
        // initialize the cov and cross cov
        // size engrossed for the kf stateSize
        Sigma_ = Eigen::MatrixXd(fusionSize, fusionSize);
        Sigma_.block<6, 6>(0, 0) = Sigma_.block<6, 6>(6, 6) = 
            Eigen::MatrixXd::Identity(6, 6);
        Sigma_.block<6, 6>(0, 6) = Sigma_.block<6, 6>(6, 0) = 
            Eigen::MatrixXd::Identity(6, 6);

        // setting the fused state cov initial as identity
        P_fusion = Eigen::MatrixXd::Identity(stateSize, stateSize);

        // initialize the weighting matrix as averaging
        A_ = Eigen::MatrixXd(fusionSize, 6);
        A_.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6) * 0.5;
        A_.block<6, 6>(6, 0) = Eigen::MatrixXd::Identity(6, 6) * 0.5;

        // setting the state transit matrix same as the kfs'
        F_fusion = Eigen::MatrixXd::Identity(stateSize, stateSize);
        F_fusion.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
        
        // Q_ = lkf_d2.getStateTransit();
        double nAccVar = 0.01;
        Q_fusion = Eigen::MatrixXd(stateSize, stateSize);
        Q_fusion.block<3, 3>(0, 0) = 
            Eigen::Matrix3d::Identity(3, 3) * 0.25 * std::pow(t_pred, 4);
        Q_fusion.block<3, 3>(0, 3) = Q_fusion.block<3, 3>(3, 0) =
            Eigen::Matrix3d::Identity(3, 3) * 0.5 * std::pow(t_pred, 3);
        Q_fusion.block<3, 3>(3, 3) = 
            Eigen::Matrix3d::Identity(3, 3) * std::pow(t_pred, 2);
        Q_fusion *= nAccVar;

        // fix the cam2 to marker2 rotation and translation
        R_m2_to_c2 << 0, 0, 1, 
                      0, -1, 0, 
                      1, 0, 0;
        t_m2_to_c2 << 31, -100, -2;
        t_m2_to_c2 *= 0.001;         // in meter

        // world to Leader init
        R_c0_init_to_L_init << 1,  0, 0,
                               0,  0, 1,
                               0, -1, 0;

        // initialization
        stateFused = stateFusedWorld = Eigen::VectorXd::Zero(stateSize);

        // threshold of quaternions ==> smaller than 10 deg
        quatThr = (1 - std::cos(M_PI/18)) * 0.5;

        // ros-dep
        StartOfTest = false;
        SOT_sub = nh_.subscribe(
            "/SOT", queue_size, &LinearMinVarFusion::telloSOTcallback, this
        );
        odom_pub = nh_.advertise<nav_msgs::Odometry>(
            "/human_state_fused", queue_size
        );
        world_odom_pub = nh_.advertise<nav_msgs::Odometry>(
            "/human_state_fused_world", queue_size
        );
        // world_odom_pred_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        //     "/predicted_"
        // );
        fusionTimer = nh_.createTimer(
            ros::Duration(t_pred), &LinearMinVarFusion::estimateFusion, this
        );
        // stop first to avoid running before initialization
        // fusionTimer.stop();

        // debug
        std::cout << "========[LMV fusion]========" << std::endl
                  << "total cov: " << std::endl << Sigma_ << std::endl
                  << "weighting matrix: " << std::endl << A_ << std::endl
                  << "state transition matrix: " << std::endl << F_fusion << std::endl
                  << "process noise cov: " << std::endl << Q_fusion << std::endl
                  << "fused err cov: " << std::endl << P_fusion << std::endl
                  << "initial state fused: " << std::endl << stateFused << std::endl
                  << "fusion timer state: " << fusionTimer.hasStarted() << std::endl;
    }
    ~LinearMinVarFusion(){}
    /*
    @telloSOTcallback
        trigger the initialization of the timer and start flag
        only when SOT is false
    */
    void telloSOTcallback(std_msgs::Empty const& msg){

        if (!StartOfTest){
            fusionTimer.start();
            startTimeFusion = ros::Time::now();

            // update the init frame information
            lkf_d0.getCamInitToHumanInit(R_c0_init_to_L_init);
            t_c0_init_to_L_init = lkf_t0.state.head(3);

            StartOfTest = true;
        }
    }
    /*
    @publishLMVFusionState
        publish the stateFusion as odom msg
    */
    void publishLMVFusionState(){

        // local fused state
        if (odom_pub.getNumSubscribers() < 1) {
            ROS_INFO_STREAM("========[LMV fusion]========\nNO FUSION SUBS!");
        }
        else {
            nav_msgs::Odometry odom_msg;
            odom_msg.pose.pose.position.x = stateFused(0);
            odom_msg.pose.pose.position.y = stateFused(1);
            odom_msg.pose.pose.position.z = stateFused(2);
            // odom_msg.pose.pose.orientation = lkf_d0.

            // directly copy from memory
            Eigen::MatrixXd P_00 = P_fusion.block<6, 6>(0, 0);
            std::copy_n(P_00.data(), 36, &odom_msg.pose.covariance[0]);
            
            odom_msg.twist.twist.linear.x = stateFused(3);
            odom_msg.twist.twist.linear.y = stateFused(4);
            odom_msg.twist.twist.linear.z = stateFused(5);
            
            // another copy by memory method
            Eigen::MatrixXd P_22 = P_fusion.bottomRightCorner(6, 6);
            std::copy_n(P_22.data(), 36, &odom_msg.twist.covariance[0]);

            odom_pub.publish(odom_msg);
        }
        // global fused state
        if (world_odom_pub.getNumSubscribers() < 1) {
            ROS_INFO_STREAM("========[LMV fusion]========\nNO WORLD FUSION SUBS!");
        }
        else {
            nav_msgs::Odometry odom_msg;
            odom_msg.pose.pose.position.x = stateFusedWorld(0);
            odom_msg.pose.pose.position.y = stateFusedWorld(1);
            odom_msg.pose.pose.position.z = stateFusedWorld(2);
            // odom_msg.pose.pose.orientation = lkf_d0.

            // set as zero temporarily!
            // Eigen::MatrixXd P_00 = P_fusion.block<6, 6>(0, 0);
            Eigen::MatrixXd P_p = Eigen::MatrixXd::Zero(6, 6);
            std::copy_n(P_p.data(), 36, &odom_msg.pose.covariance[0]);
            
            odom_msg.twist.twist.linear.x = stateFusedWorld(3);
            odom_msg.twist.twist.linear.y = stateFusedWorld(4);
            odom_msg.twist.twist.linear.z = stateFusedWorld(5);
            
            // set as zero temporarily
            // Eigen::MatrixXd P_22 = P_fusion.bottomRightCorner(6, 6);
            Eigen::MatrixXd P_v = Eigen::MatrixXd::Zero(6, 6);
            std::copy_n(P_v.data(), 36, &odom_msg.twist.covariance[0]);

            odom_pub.publish(odom_msg);
        }
    }
    /*
    @estimateFusion
        estimation in inertial frame --> initial C0
        update the error cov Sigma_ and publish the fusion result 
        according to the timer 
    */
    void estimateFusion(ros::TimerEvent const& ev){

        // update if SOT is true
        if (!StartOfTest) {
            ROS_INFO_STREAM("========[LMV fusion]========\nSOT NOT YET!");
        }
        else {
            // update the Sigma_ via iteration
            // err cov update
            Sigma_.block<6, 6>(0, 0) = lkf_t0.P_.eval();
            Sigma_.block<6, 6>(6, 6) = lkf_t2.P_.eval();
            
            // calculate the error cross cov
            Eigen::MatrixXd temp0, temp2;
            temp0 = Eigen::MatrixXd::Identity(6, 6) - lkf_t0.kalmanGain * lkf_t0.H_p;
            temp2 = Eigen::MatrixXd::Identity(6, 6) - lkf_t2.kalmanGain * lkf_t2.H_p;
            
            Sigma_.block<6, 6>(0, 6) = 
                temp0 * (F_fusion * Sigma_.block<6, 6>(0, 6) * F_fusion.transpose()) * temp2.transpose();
            Sigma_.block<6, 6>(6, 0) = 
                temp2 * (F_fusion * Sigma_.block<6, 6>(6, 0) * F_fusion.transpose()) * temp0.transpose();
            // calculate the optimal weighting
            Eigen::MatrixXd sigmaInv = Sigma_.inverse();
            
            // special form from two sensor case
            P_fusion = (
                sigmaInv.block<6, 6>(0, 0) + sigmaInv.block<6, 6>(0, 6) +
                sigmaInv.block<6, 6>(6, 0) + sigmaInv.block<6, 6>(6, 6)
            ).inverse();

            // decomposed weighting matrix by the sigmaInv
            A_.block<6, 6>(0, 0) = 
                (sigmaInv.block<6, 6>(0, 0) + sigmaInv.block<6, 6>(0, 6)) * P_fusion;
            A_.block<6, 6>(6, 0) = 
                (sigmaInv.block<6, 6>(6, 0) + sigmaInv.block<6, 6>(6, 6)) * P_fusion;

            // calculate the optimal state estimate
            Eigen::Matrix3d R_m2_to_c0, R_c0_to_init, R_c2_to_init;
            
            // transform the cam2 state to cam0
            lkf_d2.getCam0toMarker(R_m2_to_c0);
            
            // find the cam2 to init
            lkf_d2.getOrientationToCamInit(R_c2_to_init);
            
            // find the cam0 to init
            lkf_d0.getOrientationToCamInit(R_c0_to_init);

            Eigen::VectorXd tempVecD2(6), tempVecD0(6);

            // position: t^{W}_{C0 \to L} estimated
            tempVecD2.head(3) = 
                R_c2_to_init * (lkf_t2.state.head(3) + R_m2_to_c2 * t_m2_to_c2) + 
                R_c0_to_init * lkf_d2.state.head(3);
            // velocity: 
            tempVecD2.tail(3) = 
                R_c0_to_init * (lkf_d0.omegaHat * lkf_d2.state.head(3) + lkf_d2.state.segment(3, 3)) +
                R_c2_to_init * (
                    lkf_d2.omegaHat * (lkf_t2.state.head(3) + R_m2_to_c2 * t_m2_to_c2) + 
                    lkf_t2.state.tail(3)
                );
            // position from drone 0 in world frame
            tempVecD0.head(3) = R_c0_to_init * lkf_t0.state.head(3);
            // velocity from drone 0 in world frame
            tempVecD0.tail(3) = R_c0_to_init * (lkf_d0.omegaHat * lkf_t0.state.head(3) + lkf_t0.state.tail(3));

            stateFused = 
                A_.block<6, 6>(0, 0) * tempVecD0 + A_.block<6, 6>(6, 0) * tempVecD2;

            // calculate the state in target's frame L0
            // stateFusedWorld.head(3) = 
            //     R_c0_init_to_L_init * (stateFused.head(3) - t_c0_init_to_L_init);
            // stateFusedWorld.tail(3) = R_c0_init_to_L_init * stateFused.tail(3);

            // publish state
            publishLMVFusionState();
        }
    }
    /*
    @predictFusedState
        predict Fused state for a fixed number of waypoints
            waypoints at least distanced by 1m
        recall that the stateFused is MMSE in current time stamp
    */
    void predictFusedState(double& horizon_){

        if (world_odom_pred_pub.getNumSubscribers() < 1){
            ROS_INFO_STREAM("========[LMV Fusion predict]========\nNO SUBs!");
        }
        else {
            trajectory_msgs::MultiDOFJointTrajectoryPoint drone1_wps;

            // the power drop to the scaling of upper-right block
            Eigen::MatrixXd F_pred = Eigen::MatrixXd::Identity(stateSize, stateSize);
            F_pred.block<3, 3>(0, 3) = 
                horizon_ * F_fusion.block<3, 3>(0, 3);
            Eigen::VectorXd statePredWorld = F_pred * stateFusedWorld;
            
            // publish the fused state as 
        }
    }
    /*
    @publishNStepPrediction
        publish the predicted N-step forward state 
            for waypoint generation
    */
};



/*
@LinearMinVarFusion::estimateFusionState
    input: Eigen::MatrixXd P_00, Eigen::MatrixXd P_22;

*/
// void LinearMinVarFusion::estimateFusionState(
//     Eigen::MatrixXd& P_00, Eigen::MatrixXd& P_22
// ){
//     ROS_INFO_STREAM("P_00: \n" << P_00 << "\nP_22:\n" << P_22);
// }

//main function
int main(int argc, char **argv)
{
    //ros::init()
    ros::init(argc, argv, "system_state_estimation", ros::init_options::AnonymousName);

    ros::NodeHandle nh;

    // drone states
    // LinearKalmanFilter drone1_kf(DRONE_STATE_1, true), face_kf(TARGET_FACE, true);
    // LinearKalmanFilter drone2_kf(DRONE_STATE_2, true);

    // target states
    // LinearKalmanFilter target_d0(TARGET_STATE_0, true, nh), target_d2(TARGET_STATE_2, true, nh);
    // LinearMinVarFusion lmv_fusion;

    LinearMinVarFusion target_kf(nh);
    // target_kf.fusionTimerPublic = nh.createTimer(
    //     ros::Duration(0.05), boost::bind(
    //         LinearMinVarFusion::estimateFusionState, target_d0.getErrCov(), target_d2.getErrCov()
    //     )
    // );

    // ErrorStateKalmanFilter target_odom(TARGET_ODOM);

    ros::spin();

    return 0;
}