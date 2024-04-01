# include "tello_pub/formation_ctrl.h"

/*
@GeometricFormCtrl

*/
GeometricFormCtrl::GeometricFormCtrl(ros::NodeHandle& nh_)
{
    // default camera positioning param
    double distXYMax = 3.5;                         // (m), conservative choice
    double desireDistXY = 1.5;                      // (m), suitable for drone 1 and 2 only
    double desireRoll = -rollCamInit;               // (rad), determine the drone height relative to face
    double desireTheta = M_PI / 2;                  // (rad), determine drone 2's relative pose to LoA
    double desireFaceTheta = 0.0;                   // (rad), determine drone 1's relative pose to face

    // set default twist gain
    for (int i=0; i<3; i++){
        twistGain[i] = -Eigen::MatrixXd::Identity(6, 6);
    }
    /*========[ros-dep]========*/
    // get nodename
    ros_Nodename = ros::this_node::getName();    
    nh_.getParam(ros_Nodename + "/smpl_time", t_smpl);
    
    // camera positioning param
    nh_.getParam(ros_Nodename + "/des_dist_xy", desireDistXY);
    nh_.getParam(ros_Nodename + "/dist_xy_max", distXYMax);
    nh_.getParam(ros_Nodename + "/des_roll", desireRoll);
    nh_.getParam(ros_Nodename + "/des_theta", desireTheta);
    nh_.getParam(ros_Nodename + "/des_face_theta", desireFaceTheta);
    nh_.getParam(ros_Nodename + "/cmd_disable", disableCmd);
    nh_.getParam(ros_Nodename + "/no_roll_pitch", noRollPitch);
    nh_.getParam(ros_Nodename + "/height_offset", height_offset);
    nh_.getParam(ros_Nodename + "/angvel_mode", angVelMode);
    nh_.getParam(ros_Nodename + "/alt_rel_pose", useAltPose);
    nh_.getParam(ros_Nodename + "/pub_des_pose", pubDesPose);
    nh_.getParam(ros_Nodename + "/smooth_human_vel", smoothHumanVel);
    nh_.getParam(ros_Nodename + "/human_vel_wndw", wndw_size);

    // twist gain, negative definite
    try {
        // path to config
        std::string configPath = "/tello_pub/cfg/twistGain.yaml";
        nh_.getParam(ros_Nodename + "/gain_matrix", configPath);
        YAML::Node config = YAML::LoadFile(configPath);
        // assign to matrix
        YAML::Node gain = config["twistGain"];
        int ind = 0;
        for (YAML::const_iterator it = gain.begin(); it != gain.end(); ++it){
            twistGain[ind] = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(
                it->second.as<std::vector<double>>().data()
            );
            // debug
            std::cout << "twist gain " << ind << std::endl
                      << twistGain[ind] << std::endl;
            ind += 1;
        }
    }
    catch (...){
        std::cout << "========[formation ctrl]========\nUSE DEFAULT GAIN" << std::endl;
    }
    // sub and pub
    // process control
    exp_mode_sub = nh_.subscribe("/exp_mode", queue_size, &GeometricFormCtrl::exp_mode_callback, this);
    // EOT_pub = nh_.advertise<std_msgs::Empty>("/EOT", queue_size);

    // multi-drone VIO
    eskf_0_sub = nh_.subscribe(
        droneName[0] + "/eskf_state_full", queue_size,
        &GeometricFormCtrl::drone0ESKFCb, this
    );
    eskf_1_sub = nh_.subscribe(
        droneName[1] + "/eskf_state_full", queue_size,
        &GeometricFormCtrl::drone1ESKFCb, this
    );
    eskf_2_sub = nh_.subscribe(
        droneName[2] + "/eskf_state_full", queue_size,
        &GeometricFormCtrl::drone2ESKFCb, this
    );
    eskf_init_1_sub = nh_.subscribe(
        droneName[1] + "/eskf_init", queue_size,
        &GeometricFormCtrl::drone1InitCb, this
    );
    eskf_init_2_sub = nh_.subscribe(
        droneName[2] + "/eskf_init", queue_size,
        &GeometricFormCtrl::drone2InitCb, this
    );
    // face odom
    eskf_face_sub = nh_.subscribe(
        "/face_eskf", queue_size,
        &GeometricFormCtrl::faceESKFCb, this
    );
    eskf_init_face_sub = nh_.subscribe(
        "/face_init", queue_size,
        &GeometricFormCtrl::faceInitCb, this
    );
    // YOLO pose
    // YOLO_pose_0_sub = nh_.subscribe(
    //     droneName[0] + "/human_pose_arr", queue_size,
    //     &GeometricFormCtrl::drone0YOLOCb, this
    // );
    // YOLO_pose_2_sub = nh_.subscribe(
    //     droneName[2] + "/human_pose_arr", queue_size,
    //     &GeometricFormCtrl::drone2YOLOCb, this
    // );
    // drone cmd
    cmd_0_pub = nh_.advertise<geometry_msgs::Twist>(droneName[0] + "/cmd_vel", queue_size);
    cmd_1_pub = nh_.advertise<geometry_msgs::Twist>(droneName[1] + "/cmd_vel", queue_size);
    cmd_2_pub = nh_.advertise<geometry_msgs::Twist>(droneName[2] + "/cmd_vel", queue_size);
    // publish desired pose for visualization
    if (pubDesPose){
        des_pose_0_pub = nh_.advertise<tello_pub::SE3Pose>(
            droneName[0] + "/des_pose", queue_size
        );
        des_pose_1_pub = nh_.advertise<tello_pub::SE3Pose>(
            droneName[1] + "/des_pose", queue_size
        );
        des_pose_2_pub = nh_.advertise<tello_pub::SE3Pose>(
            droneName[2] + "/des_pose", queue_size
        );
    }
    // timer
    formationCtrlTimer = nh_.createTimer(
        ros::Duration(t_smpl), &GeometricFormCtrl::ctrlTimerCb, this, false, false
    );
    // debug
    ctrller_debug_pub = nh_.advertise<tello_pub::GeometricCtrlDebug>(
        "/cost_func_and_grad", queue_size
    );
    
    /*==============[param]=====================*/
    // constant transformation

    // simulation use this coordinate!!
    // rotCamTiltToBody << 0.0, std::sin(rollCamInit), -std::cos(rollCamInit),
    //                     1.0, 0.0, 0.0,
    //                     0.0, -std::cos(rollCamInit), -std::sin(rollCamInit);
    // rotCamIdealToBody << 0.0, 0.0, -1.0,
    //                     1.0, 0.0, 0.0,
    //                     0.0, -1.0, 0.0;
    // rotBodyToCmd << 0.0, 1.0, 0.0,
    //                 -1.0, 0.0, 0.0,
    //                 0.0, 0.0, 1.0;
    
    // use body frame as the previous sensing frame
    rotCamTiltToBody << 0.0, -std::sin(rollCamInit), std::cos(rollCamInit),
                        1.0, 0.0, 0.0,
                        0.0, std::cos(rollCamInit), std::sin(rollCamInit);
    rotCamIdealToBody << 0.0, 0.0, 1.0,
                         1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0;
    rotBodyToCmd << 0.0, 1.0, 0.0,
                    1.0, 0.0, 0.0,
                    0.0, 0.0, -1.0;
    rotCamTilt1ToM1 << -1.0, 0.0, 0.0,
                      0.0, -std::cos(rollCamInit), -std::sin(rollCamInit),
                      0.0, -std::sin(rollCamInit), std::cos(rollCamInit);
    // debug
    std::cout << "constant transformations:\n RMat C to B:\n" << rotCamTiltToBody << std::endl
              << "RMat C ideal to B:\n" << rotCamIdealToBody << std::endl
              << "RMat B to cmd:\n" << rotBodyToCmd << std::endl
              << "RMat C1 to M1:\n" << rotCamTilt1ToM1 << std::endl;

    // drones state
    for (int i=0; i<3; i++){
        // received state
        dronesPos[i] = Eigen::Vector3d::Zero();
        dronesVel[i] = Eigen::Vector3d::Zero();
        dronesAngVel[i] = Eigen::Vector3d::Zero();
        dronesQuat[i] = Eigen::Quaterniond::Identity();

        // relative pose and twist
        droneRelPose[i] = Eigen::Matrix4d::Identity();
        droneRelTwist[i] = Eigen::VectorXd::Zero(6);
    }
    // drones and face initial condition
    posIner0ToIneri[0] = posIner0ToIneri[1] = posIner1ToInitFace = Eigen::Vector3d::Zero();
    rotIneriToIner0[0] = rotIneriToIner0[1] = rotInitFaceToIner1 = Eigen::Matrix3d::Identity();

    // face states
    facePos = faceVel = faceAngVel = Eigen::Vector3d::Zero();
    faceQuat = Eigen::Quaterniond::Identity();


    // LoA rotation
    rotInitFaceToLoA = Eigen::Matrix3d::Identity();
    rotFaceToLoA = Eigen::Matrix3d::Identity();

    // sensor
    rotFaceToCamTilt = rotM1ToCamTilt = Eigen::Matrix3d::Identity();
    posCamTiltToFace = posCamTiltToM1 = Eigen::Vector3d::Zero();

    // setting the desired states
    setDesireRelPose(
        distXYMax, desireDistXY, desireRoll, desireTheta, desireFaceTheta
    );    
}
/*
@exp_mode_callback
    receive SOT
        start the timer and set the start time to now
        sot flag set true
*/
void GeometricFormCtrl::exp_mode_callback(std_msgs::Empty const& msg){
    if (!StartOfTest){
        formationCtrlTimer.start();
        startTimeCtrller = ros::Time::now();
        StartOfTest = true;
        // debug
        ROS_INFO_STREAM("========[formation control]========\nreceived exp mode!\n");
    }
}
/*
@drone0ESKFCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::drone0ESKFCb(
    trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt
){
    // update the states
    dronesPos[0](0) = trajPt.transforms[0].translation.x;
    dronesPos[0](1) = trajPt.transforms[0].translation.y;
    dronesPos[0](2) = trajPt.transforms[0].translation.z + height_offset;
    dronesVel[0](0) = trajPt.velocities[0].linear.x;
    dronesVel[0](1) = trajPt.velocities[0].linear.y;
    dronesVel[0](2) = trajPt.velocities[0].linear.z;
    dronesAngVel[0](0) = trajPt.velocities[0].angular.x;
    dronesAngVel[0](1) = trajPt.velocities[0].angular.y;
    dronesAngVel[0](2) = trajPt.velocities[0].angular.z;
    tf::quaternionMsgToEigen(trajPt.transforms[0].rotation, dronesQuat[0]);
}
/*
@drone1ESKFCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::drone1ESKFCb(
    trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt
){
    // update the states
    dronesPos[1](0) = trajPt.transforms[0].translation.x;
    dronesPos[1](1) = trajPt.transforms[0].translation.y + height_offset;
    dronesPos[1](2) = trajPt.transforms[0].translation.z;
    dronesVel[1](0) = trajPt.velocities[0].linear.x;
    dronesVel[1](1) = trajPt.velocities[0].linear.y;
    dronesVel[1](2) = trajPt.velocities[0].linear.z;
    dronesAngVel[1](0) = trajPt.velocities[0].angular.x;
    dronesAngVel[1](1) = trajPt.velocities[0].angular.y;
    dronesAngVel[1](2) = trajPt.velocities[0].angular.z;
    tf::quaternionMsgToEigen(trajPt.transforms[0].rotation, dronesQuat[1]);
}
/*
@drone2ESKFCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::drone2ESKFCb(
    trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt
){
    // update the states
    dronesPos[2](0) = trajPt.transforms[0].translation.x;
    dronesPos[2](1) = trajPt.transforms[0].translation.y + height_offset;
    dronesPos[2](2) = trajPt.transforms[0].translation.z;
    dronesVel[2](0) = trajPt.velocities[0].linear.x;
    dronesVel[2](1) = trajPt.velocities[0].linear.y;
    dronesVel[2](2) = trajPt.velocities[0].linear.z;
    dronesAngVel[2](0) = trajPt.velocities[0].angular.x;
    dronesAngVel[2](1) = trajPt.velocities[0].angular.y;
    dronesAngVel[2](2) = trajPt.velocities[0].angular.z;
    tf::quaternionMsgToEigen(trajPt.transforms[0].rotation, dronesQuat[2]);
}
/*
@drone1InitCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::drone1InitCb(tello_pub::SE3Pose const& pose){
    // update the initial condition
    posIner0ToIneri[0](0) = pose.tvec.x;
    posIner0ToIneri[0](1) = pose.tvec.y;
    posIner0ToIneri[0](2) = pose.tvec.z;
    std::memcpy(
        rotIneriToIner0[0].data(), &pose.rmat, 9 * sizeof(double)
    );
    // debug
    std::cout << "========[formation ctrl]========\nI1 to I0 rotation:\n"
              << rotIneriToIner0[0] << std::endl;
}
/*
@drone2InitCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::drone2InitCb(tello_pub::SE3Pose const& pose){
    // update the initial condition
    posIner0ToIneri[1](0) = pose.tvec.x;
    posIner0ToIneri[1](1) = pose.tvec.y;
    posIner0ToIneri[1](2) = pose.tvec.z;
    std::memcpy(
        rotIneriToIner0[1].data(), &pose.rmat, 9 * sizeof(double)
    );
    // debug
    std::cout << "========[formation ctrl]========\nI2 to I0 rotation:\n"
              << rotIneriToIner0[1] << std::endl;
}
/*
@faceESKFCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::faceESKFCb(
    trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt
){
    // update the state
    facePos(0) = trajPt.transforms[0].translation.x;
    facePos(1) = trajPt.transforms[0].translation.y;
    facePos(2) = trajPt.transforms[0].translation.z;
    faceVel(0) = trajPt.velocities[0].linear.x;
    faceVel(1) = trajPt.velocities[0].linear.y;
    faceVel(2) = trajPt.velocities[0].linear.z;
    faceAngVel(0) = trajPt.velocities[0].angular.x;
    faceAngVel(1) = trajPt.velocities[0].angular.y;
    faceAngVel(2) = trajPt.velocities[0].angular.z;
    tf::quaternionMsgToEigen(trajPt.transforms[0].rotation, faceQuat);

    if (smoothHumanVel){
        // store the face vel in the wndw
        if (faceVelWndw.size() < wndw_size) {faceVelWndw.emplace_back(faceVel);}
        else {
            // pop front and emplace back
            assert(!faceVelWndw.empty());
            faceVelWndw.erase(faceVelWndw.begin());
            faceVelWndw.emplace_back(faceVel);
        }
        // smooth the vec
        Eigen::Vector3d v_tmp = Eigen::Vector3d::Zero();
        for (int i=0; i<faceVelWndw.size(); i++){
            v_tmp += faceVelWndw[i];
        }
        faceVel = v_tmp / faceVelWndw.size();
    }
    // compute IF = IL --> LoA by the vz, vx (pitch angle for IF)
    // note that Y axis is upward
    Eigen::Matrix3d RMat;
    if (faceVel.norm() < VEL_THR){
        // set as previous
        RMat = rotInitFaceToLoA;
        // rotInitFaceToLoA = Eigen::Matrix3d::Identity();
    }
    else {
        double cosTh = faceVel(2) / std::sqrt(std::pow(faceVel(0), 2) + std::pow(faceVel(2), 2));
        double sinTh = faceVel(0) / std::sqrt(std::pow(faceVel(0), 2) + std::pow(faceVel(2), 2));
        // RMat << cosTh,  0, sinTh,
        //         0,      1, 0,
        //         -sinTh, 0, cosTh;
        RMat << cosTh,  0, -sinTh,
                0,      1, 0,
                sinTh, 0, cosTh;
        rotInitFaceToLoA = RMat;
        // rotInitFaceToLoA << cosTh,  0, sinTh,
        //                     0,      1, 0,
        //                     -sinTh, 0, cosTh;
    }
    // to find the rotation between F and L,
    // find rotation F --> IF and IL --> L
    // the (3, 3)-entry of F --> L is the inner product of two z-axis
    rotFaceToLoA = rotInitFaceToLoA * faceQuat.toRotationMatrix();

    // debug
    std::cout << "========[formation ctrl]========\nFace Odom CB:\nIF --> L rotation:\n"
              << rotInitFaceToLoA << std::endl
              << "F --> L rotation:\n" << rotFaceToLoA << std::endl
              << "face linear velocity in IF=IL:\n" << faceVel << std::endl;
}
/*
@faceInitCb
    receive state regardless of SOT flag
*/
void GeometricFormCtrl::faceInitCb(tello_pub::SE3Pose const& facePose){
    // update the initial condition
    posIner1ToInitFace(0) = facePose.tvec.x;
    posIner1ToInitFace(1) = facePose.tvec.y;
    posIner1ToInitFace(2) = facePose.tvec.z;
    std::memcpy(
        rotInitFaceToIner1.data(), &facePose.rmat, 9 * sizeof(double)
    );
    // debug
    std::cout << "IF --> I1 rotation:\n" << rotInitFaceToIner1 << std::endl;
}
/*
@ctrlTimerCb
    calculate relative pose based on the estimates
*/
void GeometricFormCtrl::ctrlTimerCb(ros::TimerEvent const& ev){
    // check start or not
    if (!StartOfTest){
        // do something...?
        // reset the velocity
        geometry_msgs::Twist zero_twist;
        cmd_0_pub.publish(zero_twist);
        cmd_1_pub.publish(zero_twist);
        cmd_2_pub.publish(zero_twist);
    }
    else {
        /*============[relative pose]===================*/
        // I1 --> F=L in I1
        Eigen::Vector3d posIner1ToFace = 
            posIner1ToInitFace + rotInitFaceToIner1 * facePos; 
        // L --> I1
        Eigen::Matrix3d rotLoAToIner1 = 
            rotInitFaceToIner1 * rotInitFaceToLoA.transpose();
            
        // debug 
        std::cout << "========[formation ctrl]========\nI1 --> F=L position in I1:\n"
                  << posIner1ToFace << std::endl
                  << "L --> I1 rotation:\n" 
                  << rotLoAToIner1 << std::endl;
        // update relative pose
        if (useAltPose) {
            // drone 1
            // B1 --> F
            droneRelPose[1].block<3, 3>(0, 0) = 
                faceQuat.conjugate().toRotationMatrix() * 
                rotInitFaceToIner1.transpose() * dronesQuat[1].toRotationMatrix();
            // F --> B1 in F
            droneRelPose[1].block<3, 1>(0, 3) = faceQuat.conjugate().toRotationMatrix() * (
                -facePos + rotInitFaceToIner1.transpose() * (
                    -posIner1ToInitFace + dronesPos[1]
                )
            );
            // debug
            std::cout << "========[drone 1 new relative pose]========\n"
                      << "relative pose:\n" << droneRelPose[1] << std::endl
                      << "rot B1 to I1:\n" << dronesQuat[1].toRotationMatrix() << std::endl
                      << "rot I1 to IF:\n" << rotInitFaceToIner1.transpose() << std::endl
                      << "rot IF to F:\n" << faceQuat.conjugate().toRotationMatrix() << std::endl
                      << "trans face:\n" << facePos << std::endl
                      << "trans I1 to IF in I1:\n" << posIner1ToInitFace << std::endl
                      << "trans drone 1:\n" << dronesPos[1] << std::endl;
            // drone 0
            // B0 --> L
            droneRelPose[0].block<3, 3>(0, 0) = 
                (rotIneriToIner0[0] * rotLoAToIner1).transpose() * 
                dronesQuat[0].toRotationMatrix();
            // L --> B0 in L
            droneRelPose[0].block<3, 1>(0, 3) = rotInitFaceToLoA * (
                -facePos + rotInitFaceToIner1.transpose() * (
                    -posIner1ToInitFace + rotIneriToIner0[0].transpose() * (
                        -posIner0ToIneri[0] + dronesPos[0]
                    )
                )
            );
            // debug
            std::cout << "========[drone 0 new relative pose]========\n" 
                      << "relative pose:\n" << droneRelPose[0] << std::endl
                      << "rot B0 to I0:\n" << dronesQuat[0].toRotationMatrix() << std::endl
                      << "rot I0 to I1:\n" << rotIneriToIner0[0].transpose() << std::endl
                      << "rot I1 to L:\n" << rotLoAToIner1.transpose() << std::endl
                      << "rot IF to L:\n" << rotInitFaceToLoA << std::endl
                      << "trans I0 to I1 in I0:\n" << posIner0ToIneri[0] << std::endl
                      << "trans drone 0:\n" << dronesPos[0] << std::endl;
            // drone 2
            // B2 --> L
            droneRelPose[2].block<3, 3>(0, 0) = 
                (rotIneriToIner0[0] * rotLoAToIner1).transpose() * 
                rotIneriToIner0[1] * dronesQuat[2].toRotationMatrix();
            // L --> B2 in L
            droneRelPose[2].block<3, 1>(0, 3) = rotInitFaceToLoA * (
                -facePos + rotInitFaceToIner1.transpose() * (
                    -posIner1ToInitFace + rotIneriToIner0[0].transpose() * (
                        -posIner0ToIneri[0] + posIner0ToIneri[1] + 
                        rotIneriToIner0[1] * dronesPos[2]
                    )
                )
            );
            // debug
            std::cout << "========[drone 2 new relative pose]========\n"
                      << "relative pose:\n" << droneRelPose[2] << std::endl
                      << "rot B2 to I2:\n" << dronesQuat[2].toRotationMatrix() << std::endl
                      << "rot I2 to I0:\n" << rotIneriToIner0[1] << std::endl
                      << "trans I0 to I2 in I0:\n" << posIner0ToIneri[1] << std::endl
                      << "trans drone 2:\n" << dronesPos[2] << std::endl;
        }
        else {
            // drone 1
            droneRelPose[1].block<3, 1>(0, 3) = dronesQuat[1].conjugate().toRotationMatrix() * (
                -dronesPos[1] + posIner1ToFace
            );
            // droneRelPose[1].block<3, 3>(0, 0) = 
            //     dronesQuat[1].conjugate().toRotationMatrix() * 
            //     rotInitFaceToIner1 * faceQuat.conjugate().toRotationMatrix();
            droneRelPose[1].block<3, 3>(0, 0) = 
                dronesQuat[1].conjugate().toRotationMatrix() * 
                rotInitFaceToIner1 * faceQuat.toRotationMatrix();
            // debug
            std::cout << "========[Drone 1 relative pose]========:\n"
                    << droneRelPose[1] << std::endl
                    << "I1 to B1 Rotation:\n" << dronesQuat[1].conjugate().toRotationMatrix() << std::endl
                    << "IF to I1 Rotation:\n" << rotInitFaceToIner1 << std::endl
                    << "F to IF Rotation:\n" << faceQuat.toRotationMatrix() << std::endl;

            // drone 0
            droneRelPose[0].block<3, 1>(0, 3) = dronesQuat[0].conjugate().toRotationMatrix() * (
                -dronesPos[0] + posIner0ToIneri[0] + rotIneriToIner0[0] * posIner1ToFace
            );
            droneRelPose[0].block<3, 3>(0, 0) = 
                dronesQuat[0].conjugate().toRotationMatrix() * rotIneriToIner0[0] * rotLoAToIner1;
            // debug
            std::cout << "========[Drone 0 relative pose]========:\n"
                    << droneRelPose[0] << std::endl
                    << "I0 to B0 Rotation:\n" << dronesQuat[0].conjugate().toRotationMatrix() << std::endl
                    << "I1 to I0 Rotation:\n" << rotIneriToIner0[0] << std::endl
                    << "IF to I1 Rotation:\n" << rotInitFaceToIner1 << std::endl
                    << "L to IF Rotation:\n" << rotInitFaceToLoA.transpose() << std::endl;

            // drone 2
            droneRelPose[2].block<3, 1>(0, 3) = dronesQuat[2].conjugate().toRotationMatrix() * (
                -dronesPos[2] +  rotIneriToIner0[1].transpose() * (
                    -posIner0ToIneri[1] + posIner0ToIneri[0] + rotIneriToIner0[0] * posIner1ToFace
                )
            );  
            droneRelPose[2].block<3, 3>(0, 0) = 
                dronesQuat[2].conjugate().toRotationMatrix() * 
                rotIneriToIner0[1].transpose() * rotIneriToIner0[0] * rotLoAToIner1;
            // debug
            std::cout << "========[Drone 2 relative pose]========\n"
                    << droneRelPose[2] << std::endl
                    << "I2 to B2 Rotation:\n" << dronesQuat[2].conjugate().toRotationMatrix() << std::endl
                    << "I0 to I2 Rotation:\n" << rotIneriToIner0[1].transpose() << std::endl
                    << "I1 to I0 Rotation:\n" << rotIneriToIner0[0] << std::endl
                    << "L to I1 Rotation:\n" << rotLoAToIner1 << std::endl;
        }
        /*======================[config error and twist cmd]===========================*/
        /*==================[old]==========================*/
        // SE(3) components
        Eigen::Matrix3d errConfigRmat[3];
        Eigen::Vector3d errConfigTvec[3];        // in face desired frame
        // TSE(3) components
        Eigen::Matrix4d gradientOld[3];
        // error function
        double errFuncOld[3];

        /*=============[new]===================*/
        // SE(3) components
        // right-invariant form
        Eigen::Matrix3d rotBodyDesToBody[3];
        Eigen::Vector3d posBodyToBodyDes[3];
        // TSE(3) components
        // Eigen::Matrix4d gradientNew[3];
        // error function
        // double errFuncNew[3];

        // drone twist command
        Eigen::VectorXd twistCmd[3];

        for (int i=0; i<3; i++){
            errConfigRmat[i] = 
                desRelPose[i].block<3, 3>(0, 0).transpose() * droneRelPose[i].block<3, 3>(0, 0);
            errConfigTvec[i] = desRelPose[i].block<3, 3>(0, 0).transpose() * (
                droneRelPose[i].block<3, 1>(0, 3) - desRelPose[i].block<3, 1>(0, 3)
            );
            rotBodyDesToBody[i] = 
                droneRelPose[i].block<3, 3>(0, 0) * desRelPose[i].block<3, 3>(0, 0).transpose();
            
            // wrong formula
            // posBodyToBodyDes[i] = desRelPose[i].block<3, 3>(0, 0) * (
            //     -droneRelPose[i].block<3, 3>(0, 0).transpose() * droneRelPose[i].block<3, 1>(0, 3) + 
            //     desRelPose[i].block<3, 3>(0, 0).transpose() * desRelPose[i].block<3, 1>(0, 3)
            // );
            posBodyToBodyDes[i] = 
                -droneRelPose[i].block<3, 3>(0, 0) * desRelPose[i].block<3, 3>(0, 0).transpose() * 
                desRelPose[i].block<3, 1>(0, 3) + droneRelPose[i].block<3, 1>(0, 3); 
            // debug
            if (useAltPose) {
                std::cout << "========[err config]========\nBody to Body desired " << i << std::endl
                          << "rot Body to Body desired:\n" << errConfigRmat[i] << std::endl
                          << "trans Body desired to Body in Body desired:\n" << errConfigTvec[i] << std::endl;
            }
            else {
                std::cout << "========[err config]========\nFace to Body " << i << std::endl
                          << droneRelPose[i] << std::endl
                          << "desired Face to Body " << i << std::endl
                          << desRelPose[i] << std::endl
                          << "F/L to F/L desired rotation:\n" << errConfigRmat[i] << std::endl
                          << "F/L desired to F/L translation:\n" << errConfigTvec[i] << std::endl
                          << "B desired to B rotation:\n" << rotBodyDesToBody[i] << std::endl
                          << "B to B desired translation:\n" << posBodyToBodyDes[i] << std::endl;
            }
            // compute TSE(3)
            // check error rotation is identity or not!!
            Eigen::AngleAxisd errRvec;
            if (errConfigRmat[i].isIdentity()){
                // debug
                std::cout << "========[formation ctrl]========\nROTATION ERROR IS IDENTITY!\n"
                          << errConfigRmat[i] << std::endl;

                // take rotation axis from current R
                errRvec = droneRelPose[i].block<3, 3>(0, 0);
                // set angle to 0
                errRvec.angle() = 0.0;

                gradientOld[i].block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
                gradientOld[i].block<3, 1>(0, 3) = errConfigRmat[i] * errConfigTvec[i];

                // assign values
                droneRelTwist[i] = Eigen::VectorXd::Zero(6);
                droneRelTwist[i].tail(3) = errConfigTvec[i];
            }
            else {
                // assign the rotation vector
                errRvec = errConfigRmat[i];

                gradientOld[i].block<3, 3>(0, 0) = errConfigRmat[i] * vector3dToSkeySym(
                    errRvec.angle() * errRvec.axis()
                );
                gradientOld[i].block<3, 1>(0, 3) = errConfigRmat[i] * leftJacobInvFromRvec(
                    errRvec.axis(), errRvec.angle()
                ) * errConfigTvec[i];

                // assign values
                droneRelTwist[i] = Eigen::VectorXd::Zero(6);
                droneRelTwist[i].head(3) = errRvec.angle() * errRvec.axis();
                droneRelTwist[i].tail(3) = leftJacobInvFromRvec(
                    errRvec.axis(), errRvec.angle()
                ) * errConfigTvec[i];
            }
            // calculate error function with default se3 inner product
            errFuncOld[i] = droneRelTwist[i].norm();

            // debug
            std::cout << "========[relative twist cmd]========\n"
                      << "se3 relative pose:\n" << droneRelTwist[i] << std::endl;

            // adjoint operator
            // skip the zero terms from the desired twist
            // droneRelTwist[i] = twistGain[i] * droneRelTwist[i] + adjointActionMatSE3(
            //     errConfigRmat[i].transpose(), -errConfigRmat[i].transpose() * errConfigTvec[i]
            // ) * desRelTwist[i];
            // multiply by gain
            droneRelTwist[i] = twistGain[i] * droneRelTwist[i];
            /*
            ======================================================================================================
            @note
                current droneRelTwist is under the body frame
                to construct Face frame or LoA frame linear velocity
                we need to add a rotation from body to Face or LoA on linear velocity
            ======================================================================================================
            */

            // debug
            std::cout << "inv left jacob:\n" << leftJacobInvFromRvec(errRvec.axis(), errRvec.angle()) << std::endl
                      << "rvec axis:\n" << errRvec.axis() << std::endl
                      << "rvec angle:\n" << errRvec.angle() << std::endl
                      << "drone " << i << " relative twist command:\n" 
                      << droneRelTwist[i] << std::endl;

            // double tmp;
            // tmp = droneRelTwist[i](3);
            // droneRelTwist[i](3) = droneRelTwist[i](4);
            // droneRelTwist[i](4) = tmp;
            // std::cout  << "work around: x -> y, y -> x\n" << droneRelTwist[i] << std::endl;

            // initialize drone twist to zero
            twistCmd[i] = Eigen::VectorXd::Zero(6);
        }
        /*============[calculate the drone twist cmd]====================*/
        if (useAltPose) {
            // drone 1
            // yaw rate need to be mapped to cmd yaw direction
            // twistCmd[1](2) = Eigen::Vector3d(0, 0, -1).transpose() * (
            //     droneRelTwist[1].head(3) + 
            //     droneRelPose[1].block<3, 3>(0, 0).transpose() * faceAngVel
            // );
            // set the angular velocity cmd frame to be the opposite of cmd frame
            // twistCmd[1].head(3) = -rotBodyToCmd * (
            //     droneRelTwist[1].head(3) + 
            //     droneRelPose[1].block<3, 3>(0, 0).transpose() * faceAngVel
            // );

            // set the angular velocity cmd frame to be the body frame
            twistCmd[1].head(3) = droneRelTwist[1].head(3) + 
                droneRelPose[1].block<3, 3>(0, 0).transpose() * faceAngVel;
            twistCmd[1].tail(3) = rotBodyToCmd * (
                droneRelTwist[1].tail(3) + 
                droneRelPose[1].block<3, 3>(0, 0).transpose() * (
                    vector3dToSkeySym(faceAngVel) * droneRelPose[1].block<3, 1>(0, 3)
                ) + dronesQuat[1].conjugate().toRotationMatrix() * rotInitFaceToIner1 * faceVel - 
                vector3dToSkeySym(dronesAngVel[1]) * dronesQuat[1].conjugate().toRotationMatrix() * dronesPos[1]
            );
            // drone 0
            // yaw rate need to be mapped to cmd yaw direction
            // assume no angular velocity in LoA frame
            // twistCmd[0](2) = Eigen::Vector3d(0, 0, -1).transpose() * droneRelTwist[0].head(3);
            // set the angular velocity cmd frame to be the opposite of cmd frame
            // twistCmd[0].head(3) = -rotBodyToCmd * droneRelTwist[0].head(3);

            // set the angular velocity cmd frame to be the body frame
            twistCmd[0].head(3) = droneRelTwist[0].head(3);
            twistCmd[0].tail(3) = rotBodyToCmd * (
                droneRelTwist[0].tail(3) +
                dronesQuat[0].conjugate().toRotationMatrix() * rotIneriToIner0[0] * rotInitFaceToIner1 * faceVel - 
                vector3dToSkeySym(dronesAngVel[0]) * dronesQuat[0].conjugate().toRotationMatrix() * dronesPos[0]
            );
            // drone 2
            // yaw rate need to be mapped to cmd yaw direction
            // assume no angular velocity in LoA frame
            // twistCmd[2](2) = Eigen::Vector3d(0, 0, -1).transpose() * droneRelTwist[2].head(3);
            // set the ang vel cmd frame to be the opposite of cmd frame
            // twistCmd[2].head(3) = -rotBodyToCmd * droneRelTwist[2].head(3);
            
            // set the ang vel cmd frame to be the body frame
            twistCmd[2].head(3) = droneRelTwist[2].head(3);
            twistCmd[2].tail(3) = rotBodyToCmd * (
                droneRelTwist[2].tail(3) + 
                dronesQuat[2].conjugate().toRotationMatrix() * rotIneriToIner0[1].transpose() * 
                rotIneriToIner0[0] * rotInitFaceToIner1 * faceVel - 
                vector3dToSkeySym(dronesAngVel[2]) * dronesQuat[2].conjugate().toRotationMatrix() * dronesPos[2]
            );
        }
        else {
            // drone 1
            // yaw rate need to be mapped to cmd yaw direction
            twistCmd[1](2) = Eigen::Vector3d(0, 0, -1).transpose() * 
                droneRelPose[1].block<3, 3>(0, 0) * (droneRelTwist[1].head(3) - faceAngVel);
            twistCmd[1].tail(3) = rotBodyToCmd * (
                vector3dToSkeySym(-droneRelPose[1].block<3, 1>(0, 3)) * dronesAngVel[1] + 
                dronesQuat[1].conjugate().toRotationMatrix() * rotInitFaceToIner1 * faceVel - 
                droneRelTwist[1].tail(3)
            );
            // drone 0
            // yaw rate to cmd yaw direction
            twistCmd[0](2) = Eigen::Vector3d(0, 0, -1).transpose() * 
                droneRelPose[0].block<3, 3>(0, 0) * droneRelTwist[0].head(3);
            twistCmd[0].tail(3) = rotBodyToCmd * (
                vector3dToSkeySym(-droneRelPose[0].block<3, 1>(0, 3)) * dronesAngVel[0] +
                dronesQuat[0].conjugate().toRotationMatrix() * rotIneriToIner0[0] * rotInitFaceToIner1 * faceVel - 
                droneRelTwist[0].tail(3)
            );
            // drone 2
            // yaw rate to cmd yaw direction
            twistCmd[2](2) = Eigen::Vector3d(0, 0, -1).transpose() * 
                droneRelPose[2].block<3, 3>(0, 0) * droneRelTwist[2].head(3);
            twistCmd[2].tail(3) = rotBodyToCmd * (
                vector3dToSkeySym(-droneRelPose[2].block<3, 1>(0, 3)) * dronesAngVel[2] + 
                dronesQuat[2].conjugate().toRotationMatrix() * 
                    rotIneriToIner0[1].transpose() * rotIneriToIner0[0] * rotInitFaceToIner1 * faceVel - 
                droneRelTwist[2].tail(3)
            );
        }
        // debug
        std::cout << "========[formation ctrl]========\n";
        for (int i=0; i<3; i++){
            std::cout << "drone " << i << " twist cmd:\n" << twistCmd[i] << std::endl;
        }
        std::cout << "velocity in F and L:\n" << faceVel << std::endl
                  << "ang vel in F:\n" << faceAngVel << std::endl;
        /*===============[publish cmd]==========================*/
        if (angVelMode) {
            publishAngCmd(gradientOld, twistCmd, errFuncOld);
        }
        else {
            publishCmd(gradientOld, twistCmd, errFuncOld);
        }
        if (pubDesPose) {publishDesPose();}

    }
}
/*
@drone0YOLOCb
    receive state regardless of SOT flag
*/
// void GeometricFormCtrl::drone0YOLOCb(geometry_msgs::PoseArray const& poseArr){
//     // TODO:
//     //      use state-feedback or fuse in face odom
// }
/*
@drone2YOLOCb
    receive state regardless of SOT flag
*/
// void GeometricFormCtrl::drone2YOLOCb(geometry_msgs::PoseArray const& poseArr){
//     // TODO:
//     //      use state-feedback or fuse in face odom
// }
/*
@publishCmd
    pass array as pointer to the beginning
    by default the length is 3
*/
void GeometricFormCtrl::publishCmd(
    Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
){
    if (!disableCmd){
        if (!noRollPitch){
            // drone twist control
            if (cmd_0_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.x = twist_cmd[0](0);
                twist_msg.angular.y = twist_cmd[0](1);
                twist_msg.angular.z = twist_cmd[0](2);
                twist_msg.linear.x = twist_cmd[0](3);
                twist_msg.linear.y = twist_cmd[0](4);
                twist_msg.linear.z = twist_cmd[0](5);
                cmd_0_pub.publish(twist_msg);
            }
            if (cmd_1_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.x = twist_cmd[1](0);
                twist_msg.angular.y = twist_cmd[1](1);
                twist_msg.angular.z = twist_cmd[1](2);
                twist_msg.linear.x = twist_cmd[1](3);
                twist_msg.linear.y = twist_cmd[1](4);
                twist_msg.linear.z = twist_cmd[1](5);
                cmd_1_pub.publish(twist_msg);
            }
            if (cmd_2_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.x = twist_cmd[2](0);
                twist_msg.angular.y = twist_cmd[2](1);
                twist_msg.angular.z = twist_cmd[2](2);
                twist_msg.linear.x = twist_cmd[2](3);
                twist_msg.linear.y = twist_cmd[2](4);
                twist_msg.linear.z = twist_cmd[2](5);
                cmd_2_pub.publish(twist_msg);
            }
        }
        else {
            // drone twist control
            if (cmd_0_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.z = twist_cmd[0](2);
                twist_msg.linear.x = twist_cmd[0](3);
                twist_msg.linear.y = twist_cmd[0](4);
                twist_msg.linear.z = twist_cmd[0](5);
                cmd_0_pub.publish(twist_msg);
            }
            if (cmd_1_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.z = twist_cmd[1](2);
                twist_msg.linear.x = twist_cmd[1](3);
                twist_msg.linear.y = twist_cmd[1](4);
                twist_msg.linear.z = twist_cmd[1](5);
                cmd_1_pub.publish(twist_msg);
            }
            if (cmd_2_pub.getNumSubscribers() > 0){
                geometry_msgs::Twist twist_msg;
                twist_msg.angular.z = twist_cmd[2](2);
                twist_msg.linear.x = twist_cmd[2](3);
                twist_msg.linear.y = twist_cmd[2](4);
                twist_msg.linear.z = twist_cmd[2](5);
                cmd_2_pub.publish(twist_msg);
            }
        }
    }
    // debugger
    if (ctrller_debug_pub.getNumSubscribers() > 0){
        tello_pub::GeometricCtrlDebug debug_msg;
        debug_msg.header.stamp = ros::Time::now();
        
        debug_msg.cost_funcs[0] = err_func[0];
        debug_msg.cost_funcs[1] = err_func[1];
        debug_msg.cost_funcs[2] = err_func[2];
        // assign by array
        std::memcpy(
            &debug_msg.grad_0, grad[0].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        std::memcpy(
            &debug_msg.grad_1, grad[1].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        std::memcpy(
            &debug_msg.grad_2, grad[2].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        ctrller_debug_pub.publish(debug_msg);
    }
}
/*
@publishCmd
    debug mode
    only publish angular velocity
*/
void GeometricFormCtrl::publishAngCmd(
    Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
){
    if (!disableCmd){
        // drone twist control
        if (cmd_0_pub.getNumSubscribers() > 0){
            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = twist_cmd[0](2);
            // twist_msg.linear.x = twist_cmd[0](3);
            // twist_msg.linear.y = twist_cmd[0](4);
            // twist_msg.linear.z = twist_cmd[0](5);
            cmd_0_pub.publish(twist_msg);
        }
        if (cmd_1_pub.getNumSubscribers() > 0){
            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = twist_cmd[1](2);
            // twist_msg.linear.x = twist_cmd[1](3);
            // twist_msg.linear.y = twist_cmd[1](4);
            // twist_msg.linear.z = twist_cmd[1](5);
            cmd_1_pub.publish(twist_msg);
        }
        if (cmd_2_pub.getNumSubscribers() > 0){
            geometry_msgs::Twist twist_msg;
            twist_msg.angular.z = twist_cmd[2](2);
            // twist_msg.linear.x = twist_cmd[2](3);
            // twist_msg.linear.y = twist_cmd[2](4);
            // twist_msg.linear.z = twist_cmd[2](5);
            cmd_2_pub.publish(twist_msg);
        }
    }
    // debugger
    if (ctrller_debug_pub.getNumSubscribers() > 0){
        tello_pub::GeometricCtrlDebug debug_msg;
        debug_msg.header.stamp = ros::Time::now();
        
        debug_msg.cost_funcs[0] = err_func[0];
        debug_msg.cost_funcs[1] = err_func[1];
        debug_msg.cost_funcs[2] = err_func[2];
        // assign by array
        std::memcpy(
            &debug_msg.grad_0, grad[0].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        std::memcpy(
            &debug_msg.grad_1, grad[1].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        std::memcpy(
            &debug_msg.grad_2, grad[2].block<3, 4>(0, 0).data(), 12 * sizeof(double)
        );
        ctrller_debug_pub.publish(debug_msg);
    }
}
/*
@publishDesPose
    publish desired pose for each drone in I0
    give error of relative pose
*/
void GeometricFormCtrl::publishDesPose(){
    if (des_pose_1_pub.getNumSubscribers() > 0){
        tello_pub::SE3Pose SE3Pose;
        SE3Pose.header.stamp = ros::Time::now();
        // drone 1's desired position in I0
        // Eigen::Vector3d tvecDes = 
        //     rotIneriToIner0[0] * rotInitFaceToIner1 * (
        //         facePos + 
        //         faceQuat.toRotationMatrix() * desRelPose[1].block<3, 1>(0, 3)
        //     );
        Eigen::Vector3d tvecDes = rotIneriToIner0[0] * (
            rotInitFaceToIner1 * (
                facePos + 
                faceQuat.toRotationMatrix() * desRelPose[1].block<3, 1>(0, 3)
            ) + posIner1ToInitFace
        ) + posIner0ToIneri[0];
        // B1 --> F --> I0
        Eigen::Matrix3d rmatDes = 
            rotIneriToIner0[0] * rotInitFaceToIner1 * 
            faceQuat.toRotationMatrix() * desRelPose[1].block<3, 3>(0, 0);
        SE3Pose.tvec.x = tvecDes(0);
        SE3Pose.tvec.y = tvecDes(1);
        SE3Pose.tvec.z = tvecDes(2);
        std::memcpy(
            &SE3Pose.rmat, rmatDes.data(), 9 * sizeof(double)
        );
        des_pose_1_pub.publish(SE3Pose);
    }
    if (des_pose_0_pub.getNumSubscribers() > 0){
        tello_pub::SE3Pose SE3Pose;
        SE3Pose.header.stamp = ros::Time::now();
        // drone 0's desired position in I0
        // Eigen::Vector3d tvecDes = 
        //     rotIneriToIner0[0] * rotInitFaceToIner1 * (
        //         facePos + 
        //         rotInitFaceToLoA.transpose() * desRelPose[0].block<3, 1>(0, 3)
        //     );
        Eigen::Vector3d tvecDes = rotIneriToIner0[0] * (
            rotInitFaceToIner1 * (
                facePos + 
                rotInitFaceToLoA.transpose() * desRelPose[0].block<3, 1>(0, 3)
            ) + posIner1ToInitFace
        ) + posIner0ToIneri[0];
        // B0 --> L --> I0
        Eigen::Matrix3d rmatDes = 
            rotIneriToIner0[0] * rotInitFaceToIner1 * 
            rotInitFaceToLoA.transpose() * desRelPose[0].block<3, 3>(0, 0);
        SE3Pose.tvec.x = tvecDes(0);
        SE3Pose.tvec.y = tvecDes(1);
        SE3Pose.tvec.z = tvecDes(2);
        std::memcpy(
            &SE3Pose.rmat, rmatDes.data(), 9 * sizeof(double)
        );
        des_pose_0_pub.publish(SE3Pose);
    }
    if (des_pose_2_pub.getNumSubscribers() > 0){
        tello_pub::SE3Pose SE3Pose;
        SE3Pose.header.stamp = ros::Time::now();
        // drone 2's desired position in I0
        // Eigen::Vector3d tvecDes = 
        //     rotIneriToIner0[0] * rotInitFaceToIner1 * (
        //         facePos + 
        //         rotInitFaceToLoA.transpose() * desRelPose[2].block<3, 1>(0, 3)
        //     );
        Eigen::Vector3d tvecDes = rotIneriToIner0[0] * (
            rotInitFaceToIner1 * (
                facePos + 
                rotInitFaceToLoA.transpose() * desRelPose[2].block<3, 1>(0, 3)
            ) + posIner1ToInitFace
        ) + posIner0ToIneri[0];        
        // B2 --> L --> I0
        Eigen::Matrix3d rmatDes = 
            rotIneriToIner0[0] * rotInitFaceToIner1 * 
            rotInitFaceToLoA.transpose() * desRelPose[2].block<3, 3>(0, 0);
        SE3Pose.tvec.x = tvecDes(0);
        SE3Pose.tvec.y = tvecDes(1);
        SE3Pose.tvec.z = tvecDes(2);
        std::memcpy(
            &SE3Pose.rmat, rmatDes.data(), 9 * sizeof(double)
        );
        des_pose_2_pub.publish(SE3Pose);
    }
}
/*
@setDesireRelPose
    set the desired relative pose from cam. positinging param.
    camera positioning parameters
        default:
            missions assigned for drone 0, 1, 2
            drone 0's parameters are determined by drone 1 and 2
        shared parameters for drone 1 and 2:
            desired roll
            desired dist xy
        drone 1
            desired face theta
        drone 2
            desired theta
*/
void GeometricFormCtrl::setDesireRelPose(
    double& distXYMax, double &desDistXY, double &desRoll, double &desTheta, double &desFaceTheta
){
    // debug
    std::cout << "camera positioning params:\n"
              << "r_d = " << desDistXY << std::endl
              << "d_max = " << distXYMax << std::endl
              << "phi_d = " << desRoll << std::endl
              << "theta_L,d = " << desTheta << std::endl
              << "theta_F,d = " << desFaceTheta << std::endl;
              
    // clip the value
    desDistXY = clip(desDistXY, DIST_XY_MIN, distXYMax);
    desFaceTheta = clip(desFaceTheta, -FACE_THETA_MAX, FACE_THETA_MAX);

    // set parameters
    // calculate the rotated angle from L to F by trigonometric identities on cosine
    // in case rotFaceToLoA is a rotation matrix, the sqrt would not have imaginary values
    double desDistXY0 = distXYMax - desDistXY * (
        rotFaceToLoA(2, 2) * std::cos(desFaceTheta) - 
        std::sqrt(1-std::pow(rotFaceToLoA(2, 2), 2)) * std::sin(desFaceTheta)
    );
    // clip the values
    desDistXY0 = clip(desDistXY0, DIST_XY_MIN, distXYMax);
    
    double distCamY = desDistXY * std::tan(desRoll);
    double distCamY0 = desDistXY0 * std::tan(desRoll);

    // debug
    std::cout << "clipped value of r_d0 = " << desDistXY0 << std::endl
              << "desire height for drone 0 = " << distCamY0 << std::endl
              << "desire height for drone 1, 2 = " << distCamY << std::endl;

    // clip the values
    distCamY0 = clip(distCamY0, DIST_Z_MIN, DIST_Z_MAX);
    distCamY = clip(distCamY, DIST_Z_MIN, DIST_Z_MAX);
    desTheta = clip(desTheta, FACE_THETA_MAX, THETA_MAX);

    // debug
    std::cout << "clipped values:\nr_d = " << desDistXY << std::endl
              << "theta_F,d = " << desFaceTheta << std::endl
              << "theta_L,d = " << desTheta << std::endl
              << "desire height for drone 0 = " << distCamY0 << std::endl
              << "desire height for drone 1, 2 = " << distCamY << std::endl;
    // set desired relative pose
    if (useAltPose){
        // rotation:    body to L/F
        // translation: L/F to body

        // drone 0
        // B0 --> L
        desRelPose[0] = Eigen::Matrix4d::Identity();
        // desRelPose[0].block<3, 3>(0, 0) << 0.0, -1.0, 0.0,
        //                                    0.0, 0.0, 1.0,
        //                                    -1.0, 0.0, 0.0;
        // use sensing frame as body frame
        desRelPose[0].block<3, 3>(0, 0) << 0.0, -1.0, 0.0,
                                           0.0, 0.0, -1.0,
                                           1.0, 0.0, 0.0;
        // L --> B0 in L
        desRelPose[0].block<3, 1>(0, 3) = Eigen::Vector3d(
            0.0, distCamY0, -desDistXY0
        );
        // drone 1
        desRelPose[1] = Eigen::Matrix4d::Identity();
        // F' --> F
        Eigen::Matrix3d desFaceRot = Eigen::Matrix3d::Identity();
        desFaceRot << std::cos(desFaceTheta), 0.0, std::sin(desFaceTheta),
                      0.0, 1.0, 0.0,
                      -std::sin(desFaceTheta), 0.0, std::cos(desFaceTheta);
        // B1 --> F'
        // desRelPose[1].block<3, 3>(0, 0) << 0.0, 1.0, 0.0,
        //                                      0.0, 0.0, 1.0,
        //                                      1.0, 0.0, 0.0;
        // use sensing frame as body frame
        desRelPose[1].block<3, 3>(0, 0) << 0.0, 1.0, 0.0,
                                           0.0, 0.0, -1.0,
                                           -1.0, 0.0, 0.0;
        // B1 --> F
        desRelPose[1].block<3, 3>(0, 0) = 
            desFaceRot * desRelPose[1].block<3, 3>(0, 0);
        // F' --> B1 in F'
        desRelPose[1].block<3, 1>(0, 3) = Eigen::Vector3d(
            0.0, distCamY, desDistXY
        );
        // F -- > B1 in F
        desRelPose[1].block<3, 1>(0, 3) = 
            desFaceRot * desRelPose[1].block<3, 1>(0, 3);
        // drone 2
        desRelPose[2] = Eigen::Matrix4d::Identity();
        // L' --> L
        Eigen::Matrix3d desLoARot = Eigen::Matrix3d::Identity();
        desLoARot << std::cos(desTheta), 0.0, std::sin(desTheta),
                     0.0, 1.0, 0.0,
                     -std::sin(desTheta), 0.0, std::cos(desTheta);
        // B2 --> L'
        // desRelPose[2].block<3, 3>(0, 0) << 0.0, 1.0, 0.0,
        //                                    0.0, 0.0, 1.0,
        //                                    1.0, 0.0, 0.0;
        // use sensing frame as body frame
        desRelPose[2].block<3, 3>(0, 0) << 0.0, 1.0, 0.0,
                                           0.0, 0.0, -1.0,
                                           -1.0, 0.0, 0.0;
        // B2 --> L
        desRelPose[2].block<3, 3>(0, 0) = 
            desLoARot * desRelPose[2].block<3, 3>(0, 0);
        // L' --> B2 in L'
        desRelPose[2].block<3, 1>(0, 3) = Eigen::Vector3d(
            0.0, distCamY, desDistXY
        );
        // L --> B2 in L
        desRelPose[2].block<3, 1>(0, 3) = 
            desLoARot * desRelPose[2].block<3, 1>(0, 3);
        
        // set desired twist
        // 0 means no desired change of relative pose
        for (int i=0; i<3; i++) {
            desRelTwist[i] = Eigen::VectorXd::Zero(6);
        }
    }
    else {
        // rotation:    L/F to body
        // translation: body to L/F
        // drone 0
        desRelPose[0] = Eigen::Matrix4d::Identity();
        // desRotMat ==> L to B_0  
        Eigen::Matrix3d desRotMat;
        desRotMat << 0.0, 0.0, -1.0,
                    -1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0;
        // only initially, need to be update in ctrl callback
        desRelPose[0].block<3, 3>(0, 0) = desRotMat;
        // translation
        desRelPose[0].block<3, 1>(0, 3) = Eigen::Vector3d(
            -desDistXY0, 0.0, -distCamY0 
        );
        // drone 1
        desRelPose[1] = Eigen::Matrix4d::Identity();
        // F' --> B1
        desRotMat(0, 2) = desRotMat(1, 0) = 1;
        // F --> F'
        Eigen::Matrix3d desFaceRot = Eigen::Matrix3d::Identity();
        desFaceRot << std::cos(desFaceTheta), 0.0, -std::sin(desFaceTheta),
                    0.0, 1.0, 0.0,
                    std::sin(desFaceTheta), 0.0, std::cos(desFaceTheta);
        // desRelPose[1].block<3, 3>(0, 0) = rotCamTiltToBody * desRotMat * desFaceRot;
        // F --> B1
        desRelPose[1].block<3, 3>(0, 0) = desRotMat * desFaceRot;
        // translation
        desRelPose[1].block<3, 1>(0, 3) = Eigen::Vector3d(
            -desDistXY, 0.0, -distCamY 
        );
        // drone 2
        desRelPose[2] = Eigen::Matrix4d::Identity();
        // L -- > L'
        Eigen::Matrix3d desLoARot = Eigen::Matrix3d::Identity();
        desLoARot << std::cos(desTheta), 0.0, -std::sin(desTheta),
                    0.0, 1.0, 0.0,
                    std::sin(desTheta), 0.0, std::cos(desTheta);
        // only initially, need to be update in ctrl callback
        // L --> B2
        desRelPose[2].block<3, 3>(0, 0) = desRotMat * desLoARot;
        // translation
        desRelPose[2].block<3, 1>(0, 3) = Eigen::Vector3d(
            -desDistXY, 0.0, -distCamY
        );
        // set desired twist
        // 0 means no desired change of relative pose
        for (int i=0; i<3; i++) {
            desRelTwist[i] = Eigen::VectorXd::Zero(6);
        }
    }
    // debug
    std::cout << "desired relative pose:\n";
    for (int i=0; i<3; i++){
        std::cout << "drone " << i << std::endl << desRelPose[i] << std::endl;
    }
}
/*
@test
*/
void test(){
    // Eigen array pointer
    Eigen::Matrix3d mat;
    mat << 1,2,3,4,5,6,7,8,9;
    std::cout << "initial mat = \n" << mat << std::endl;

    double* matPtr = mat.data();
    std::cout << "pointer to data as array:\n[ ";
    for (int i=0; i<9; i++) {
        std::cout << matPtr[i] << " ";
    }
    std::cout << "]" << std::endl;

    double *matPtrNew = new double[mat.size()];
    // Eigen::MatrixXd mat2;
    Eigen::Map<Eigen::MatrixXd, Eigen::ColMajor>(matPtrNew, mat.rows(), mat.cols()) = mat;
    std::cout << "mapped mat = \n" << mat << std::endl
              << "pointer to data as array:\n[ ";
    for (int i=0; i<9; i++) {
        std::cout << matPtrNew[i] << " ";
    }
    std::cout << "]" << std::endl;

    // Eigen::Map<const Eigen::MatrixXd> matMap(&mat(0), mat.size());
    // std::cout << matMap << std::endl;
    double* p = &mat(0, 0);
    for (int i=0; i<mat.size(); i++){
        std::cout << *p << " ";
        p++;
    }
    std::cout << std::endl;

    double arr_cpy[9];
    std::copy(mat.data(), mat.data() + mat.size(), arr_cpy);
    for (int i=0; i<mat.size(); i++){
        std::cout << arr_cpy[i] << " ";
    }
    std::cout << std::endl << Eigen::Matrix3d(arr_cpy) << std::endl;

    double arr_memcpy[9];
    std::memcpy(arr_memcpy, mat.data(), mat.size() * sizeof(double));
    for (int i=0; i<mat.size(); i++){
        std::cout << arr_memcpy[i] << " ";
    }
    std::cout << std::endl << Eigen::Matrix3d(arr_memcpy) << std::endl;

    double arr[6];
    std::memcpy(&arr, mat.data(), mat.size() * sizeof(double));
    for (int i=0; i<mat.size(); i++){
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
    // Eigen::MatrixXd(arr, 2, 3);

    std::cout << "test the angle axis and matrix log" << std::endl;
    Eigen::Matrix3d rMat;
    rMat << 0.698131, 0.232551, -0.677151,
            -0.69328, -0.0166488, -0.720476,
            -0.178821,  0.972442, 0.1496;
    Eigen::AngleAxisd rvec;
    rvec = rMat;
    double theta = std::acos((rMat.trace() - 1)/2);
    Eigen::Vector3d LogRMat;
    so3SkewSymToVec3d(rMat - rMat.transpose(), LogRMat);
    Eigen::Quaterniond quat;
    quat = rMat;

    std::cout << "AngleAxisd\nangle = " << rvec.angle() << std::endl
              << "axis =\n" << rvec.axis() << std::endl;
    std::cout << "angle from acos = " << theta << std::endl
              << "axis from acos and vee map:\n" << LogRMat / (2*std::sin(theta)) << std::endl;
    std::cout << "Quaternion\nangle = " << 2 * std::atan2(quat.vec().norm(), quat.w()) << std::endl
              << "axis = \n" << quat.vec() / quat.vec().norm() << std::endl;
    
    // test the matrix conversion
    Eigen::Matrix3d testMat;
    testMat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << "testing mat:\n" << testMat << std::endl;
    double testArr[9];
    std::memcpy(&testArr, testMat.data(), sizeof(double) * 9);
    std::cout << "testing mat copy in arr:\n";
    for (int i=0; i<9; i++) {
        std::cout << testArr[i] << ", ";
    }
    std::cout << std::endl;
    Eigen::Matrix3d testMat2;
    std::memcpy(testMat2.data(), &testArr, sizeof(double) * 9);
    std::cout << "new testing mat:\n" << testMat2 << std::endl;
    
    // test the skew operation
    Eigen::Vector3d v1, v2;
    v1 << 1.0, 2.0, 3.0;
    v2 << 0.0, 1.0, 0.0;
    std::cout << "v1:\n" << v1 << std::endl
              << "v2:\n" << v2 << std::endl
              << "skew(v1):\n" << vector3dToSkeySym(v1) << std::endl
              << "skew(v2):\n" << vector3dToSkeySym(v2) << std::endl
              << "v1 x v2 =\n" << vector3dToSkeySym(v1) * v2 << std::endl
              << "v2 x v1 =\n" << vector3dToSkeySym(v2) * v1 << std::endl;
    // test the quaternion composition and rotation composition
    Eigen::Quaterniond q1, q2;
    Eigen::Matrix3d tempMat;
    // B1 --> I1
    tempMat << 0.0, -1.0, 0.0,
               0.0, 0.0, -1.0,
               1.0, 0.0, 0.0;
    q1 = tempMat;
    // B1 --> C1
    tempMat << 0.0, 1.0, 0.0,
               -std::sin(0.206), 0.0, std::cos(0.206),
               std::cos(0.206), 0.0, std::sin(0.206);
    q2 = tempMat.transpose();
    std::cout << "q1 in rot:\n" << q1.toRotationMatrix() << std::endl
              << "q2 in rot:\n" << q2.toRotationMatrix() << std::endl
              << "C1 --> I1 in quat:\n" << (q1 * q2).w() << std::endl << (q1 * q2).vec() << std::endl
              << "C1 --> I1 in rot:\n" << (q1 * q2).toRotationMatrix() << std::endl
              << "q2 * q1 in quat:\n" << (q2 * q1).w() << std::endl << (q2 * q1).vec() << std::endl
              << "q2 * q1 in rot:\n" << (q2 * q1).toRotationMatrix() << std::endl;

}

/*
@main
*/
int main(int argc, char **argv){
    
    ros::init(argc, argv, "geometric_formation_controller", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    GeometricFormCtrl gf_ctrl(nh);

    // test();

    ros::spin();


    return 0;
}