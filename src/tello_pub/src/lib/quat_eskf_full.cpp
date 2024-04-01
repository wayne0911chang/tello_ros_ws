# include "tello_pub/quat_eskf_full.h"

/*
@FullQuatESKF
    constructor
*/
FullQuatESKF::FullQuatESKF(int drone_id, double t_smpl){

    // configure the gyro to inertial frame rotation 
    Eigen::Matrix3d rotGyro2Iner;

    // get drone id
    droneID_ = drone_id;
    t_pred = t_smpl;
    // debug
    std::cout << "========[eskf]========\n prediction time: " << t_pred << std::endl;
    // setting the gyro to inertial frame
    switch (droneID_){
        case 0:
            // debug
            std::cout << "========[eskf]========\ndrone_0 eskf constructor" << std::endl;

            // setting the constant Jacobians        
            // Body frame is the sensing frame
            rotGyro2Iner << 0, 1, 0,
                            1, 0, 0,
                            0, 0, -1;
            // new frame transformation
            // rotGyro2Iner << 0, 1, 0,
            //                 -1, 0, 0,
            //                 0, 0, 1;
            // setting the inertial odom to inertial frame
            odomToIner = Eigen::Matrix3d::Identity(3, 3);
            break;
        case 1:
            // debug
            std::cout << "========[eskf]========\ndrone_1 eskf constructor" << std::endl;

            // setting the constant Jacobians        
            rotGyro2Iner << 0, -1, 0,
                            0, 0, -1,
                            1, 0, 0;
            // new frame transformation
            // rotGyro2Iner << 0, -1, 0,
            //                 0, 0, 1,
            //                 -1, 0, 0;
            // setting the inertial odom to inertial frame
            odomToIner << -1, 0, 0,
                          0, 0, 1,
                          0, 1, 0;
            break;
        case 2:
            // debug
            std::cout << "========[eskf]========\ndrone_2 eskf constructor" << std::endl;

            // setting the constant Jacobians        
            rotGyro2Iner << 1, 0, 0,
                            0, 0, -1,
                            0, 1, 0;
            // new frame transformation
            // rotGyro2Iner << -1, 0, 0,
            //                 0, 0, 1,
            //                 0, 1, 0;
            odomToIner << 0, 1, 0,
                          0, 0, 1,
                          1, 0, 0;
    }
    /*===========================[parameters]================================*/
    // initialization
    nominalQuat = Eigen::Quaterniond::Identity();
    nominalState = Eigen::VectorXd::Zero(stateSize - 4 - biasSize);
    nominalBias = Eigen::VectorXd::Zero(biasSize);
    imuInput = Eigen::VectorXd::Zero(inputSize);                        // body frame measurement
    errState = Eigen::VectorXd::Zero(errStateSize);

    // initialize the imu input in {imu,i} frame
    // imuInput(2) = -G1;
    // imuInput(2) = -G0;
    // new frame transformation
    // imuInput(2) = G0;
    // use body frame with negative gravitational acceleration offset
    imuInput(2) = -G0;


    // setting the error state cov
    errCov_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);

    // setting the process noise impulse covariance
    nProcCov_ = Eigen::MatrixXd::Identity(stateSize - 4, stateSize - 4);
    nProcCov_.block<3, 3>(0, 0) *= nAccVar * std::pow(t_pred, 2);
    nProcCov_.block<3, 3>(3, 3) *= nAngVelVar * std::pow(t_pred, 2);
    nProcCov_.block<3, 3>(6, 6) *= nAccBiasVar * t_pred;
    nProcCov_.block<3 ,3>(9, 9) *= nAngVelBiasVar * t_pred;

    // setting the measurement noise impulse covariance
    // no need of square of sampling time!!
    nGyroCov_ = nAngVelVar * Eigen::Matrix3d::Identity(3, 3);
    nGyroCov4d_ = nAngVelVar * Eigen::Matrix4d::Identity(4, 4);
    nOdomCov_ = nVelVar * Eigen::Matrix3d::Identity(3, 3);
    nArUcoCov_ = nPositionVar * Eigen::Matrix3d::Identity(3, 3);

    // setting the frame transformation
    quatGyro2Iner = rotGyro2Iner;
    // gyroToOdom << 0, -1, 0,
    //               -1, 0, 0,
    //               0, 0, -1;
    // new frame transformation
    // gyroToOdom << 0, -1, 0,
    //               1, 0, 0,
    //               0, 0, 1;
    // gyroToOdom << 0, 1, 0,
    //               -1, 0, 0,
    //               0, 0, 1;
    gyroToOdom << 0, 1, 0,
                  1, 0, 0,
                  0, 0, -1;            
            
    // gyroToTiltCam << 0, 1, 0,
    //              0, 0, 1,
    //              1, 0, 0;
    // new frame transformation
    // gyroToTiltCam << 0, 1, 0,
    //              0, 0, -1,
    //              -1, 0, 0;
    // consider the camera roll rotation
    // gyroToTiltCam << 0, 1, 0,
    //              std::sin(rollCamInit), 0, -std::cos(rollCamInit),
    //              -std::cos(rollCamInit), 0, -std::sin(rollCamInit);
    gyroToTiltCam << 0, 1, 0,
                 -std::sin(rollCamInit), 0, std::cos(rollCamInit),
                 std::cos(rollCamInit), 0, std::sin(rollCamInit);
    
    // imuToGyro << 1, 0, 0,
    //              0, -1, -1,
    //              0, 0, -1;
    // imuToGyro << 1, 0, 0,
    //              0, -1, 0,
    //              0, 0, -1;
    imuToGyro = Eigen::Matrix3d::Identity();

    // camToCmd << 1, 0, 0,
    //             0, 0, 1,
    //             0, -1, 0;
    // consider the camera roll rotation
    // camToCmd << 1, 0, 0,
    //             0, -std::sin(rollCamInit), std::cos(rollCamInit),
    //             0, -std::cos(rollCamInit), -std::sin(rollCamInit);
    // compute tilt camera to ideal camera
    tiltCamToIdeal << 1.0, 0.0, 0.0,
                      0.0, std::cos(rollCamInit), std::sin(rollCamInit),
                      0.0, -std::sin(rollCamInit), std::cos(rollCamInit);
    
    // setting the constant selection matrix
    // jacobErrStateRvec = Eigen::MatrixXd::Zero(3, errStateSize);
    // jacobErrStateRvec.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity(3, 3);

    // constant selection matrix for the innovation signal of the correct quaternion product order
    jacobErrStateRvec = Eigen::MatrixXd::Zero(3, errStateSize);
    jacobErrStateRvec.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3);

    // setting the constant noise state transit Jacob
    nF_ = Eigen::MatrixXd::Zero(errStateSize, inputSize + biasSize);
    nF_.block<12, 12>(3, 0) = Eigen::MatrixXd::Identity(12, 12);

    // setting the measurement initial
    markerPositionInit = Eigen::Vector3d::Zero();
    rotIneriToIner0 = Eigen::Matrix3d::Identity();
    imuQuatPrev = Eigen::Quaterniond::Identity();
}
/*===================[ESKF functions]========================*/
/*
@stateTransitJacob
    find the state transition Jacobian for error states
*/
void FullQuatESKF::stateTransitJacob(Eigen::MatrixXd &errF_){

    errF_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);

    // setup components
    Eigen::Matrix3d accHatImu;
    vector3dToSkeySym(
        imuInput.head(3) - nominalBias.head(3),
        accHatImu
    );
    // setting the jacobian
    errF_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * t_pred;
    errF_.block<3, 3>(3, 6) = 
        -t_pred * nominalQuat.toRotationMatrix() * imuToGyro * accHatImu;
    errF_.block<3, 3>(3, 9) = 
        -t_pred * nominalQuat.toRotationMatrix() * imuToGyro;
    errF_.block<3, 3>(6, 6) = vector3dToQuaterniond(
        t_pred * (imuInput.tail(3) - nominalBias.tail(3))
    ).toRotationMatrix().transpose();
    errF_.block<3, 3>(6, 12) = -t_pred * Eigen::Matrix3d::Identity(3, 3);
}
/*
@true2errJacob
    find the true-state to error-state Jacobian
*/
void FullQuatESKF::true2errJacob(Eigen::MatrixXd &Jacob){
    
    // quaternion composition to error rvec
    Eigen::MatrixXd quatCompToRvec(4, 3);
    quatCompToRvec.block<1, 3>(0, 0) = -0.5 * nominalQuat.vec().transpose();
    quatCompToRvec.block<3, 3>(1, 0) = 0.5 * (
        nominalQuat.w() * Eigen::Matrix3d::Identity(3, 3) + 
        vector3dToSkeySym(nominalQuat.vec())
    );
    // full jacobian
    Jacob = Eigen::MatrixXd::Identity(stateSize, errStateSize);
    Jacob.block<4, 3>(6, 6) = quatCompToRvec;
}
/*
@predictState
    predict the nominal state w. DT kinematics
    predict the error state w. DT kinematics (zero mean)
    propagate the error cov. of error state
*/
void FullQuatESKF::predictState(){
    // check if started
    if (eskfStart){
        // nominal state prediction
        // acceleration from {imu,i} to {gyro,i}
        // gravitational acceleration in {I_i}, different for each drone
        // imuInput have been mapped to the imu motion axis
        // imuInput on body frame
        Eigen::Vector3d acc_gyro(3), gVec(3);
        acc_gyro << (imuInput(0) - nominalBias(0)), 
                    (imuInput(1) - nominalBias(1)),
                    (imuInput(2) - nominalBias(2));
        // acc_gyro *= G0;
        // gVec << 0.0, 0.0, G1;
        switch (droneID_)
        {
            case 0:
                // gVec << 0.0, 0.0, G1;

                // follow previous code
                // gVec << 0.0, 0.0, G0;
                
                // under {I_0}
                gVec << 0.0, 0.0, -G0;

                break;
            case 1:
                // gVec << 0.0, G1, 0.0;
                gVec << 0.0, -G0, 0.0;
                break;
            case 2:
                // gVec << 0.0, G1, 0.0;
                gVec << 0.0, -G0, 0.0;
                break;
            
            default:
                break;
        }
        Eigen::Quaterniond quat_angvel = 
            vector3dToQuaterniond((imuInput.tail(3) - nominalBias.tail(3)) * t_pred);

        // predict nominal state
        nominalState.head(3) += nominalState.tail(3) * t_pred + 
            0.5 * std::pow(t_pred, 2) * (nominalQuat.toRotationMatrix() * acc_gyro + gVec);
        nominalState.tail(3) += (nominalQuat.toRotationMatrix() * acc_gyro + gVec) * t_pred;
        // nominalState.head(3) += nominalState.tail(3) * t_pred + 
        //     0.5 * std::pow(t_pred, 2) * G0 * (nominalQuat.toRotationMatrix() * acc_gyro + gVec);
        // nominalState.tail(3) += G0 * (nominalQuat.toRotationMatrix() * acc_gyro + gVec) * t_pred;
        nominalQuat = nominalQuat * quat_angvel;
        // nominalBias = nominalBias;

        // error state mean prediction --> zero mean, skipped
        // Find Jacobian
        Eigen::MatrixXd errF_;
        stateTransitJacob(errF_);
        errCov_ = errF_ * errCov_ * errF_.transpose() + nF_ * nProcCov_ * nF_.transpose();

        // debug
        // std::cout << "gyro to inertial:\n" << nominalQuat.toRotationMatrix() << std::endl;
    }
}
/*
@injectNominal
    adopt the group composition for the state
*/
void FullQuatESKF::injectNominal(){
    // follow the right oplus operator
    nominalState += errState.head(6);
    nominalQuat = nominalQuat * vector3dToQuaterniond(errState.segment(6, 3));
    nominalBias += errState.tail(6);

    // ensure the unit quaternion
    nominalQuat.normalize();
}
/*
@resetErrState
    reset error state to 0
    calculate the Jacobian
*/
void FullQuatESKF::resetErrState(){
    // reset operation corresponds to the right inverse operator
    Eigen::MatrixXd resetJacob = Eigen::MatrixXd::Identity(errStateSize, errStateSize);
    resetJacob.block<3, 3>(6, 6) -= vector3dToSkeySym(0.5 * errState.segment(6, 3));

    // reset and propagate the state
    errState = Eigen::VectorXd::Zero(errStateSize);
    errCov_ = resetJacob * errCov_ * resetJacob.transpose();
}
/*
@odomCB
    *deprecated*
    take the linear velocity as measurment
    if not started, set the initial condition
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF::odomCB(const Eigen::Vector3d &odomVel){
    // check if started
    if (!eskfStart){
        // direct assign the measured velocity
        nominalState.tail(3) = 
            nominalQuat.toRotationMatrix() * gyroToOdom.transpose() * odomVel;
        
        // debug
        std::cout << "odom vel: \n" << odomVel << std::endl
                  << "nominal vel:\n" << nominalState.tail(3) << std::endl;
    }
    else {
        /*=================[ESKF correction====================]*/
        // innovation signal
        Eigen::Vector3d innovVel = odomVel - 
            gyroToOdom * nominalQuat.conjugate().toRotationMatrix() * nominalState.tail(3);
        
        // measurement Jacobian
        // measurement to true state
        Eigen::MatrixXd odomToTrueState(3, stateSize), jacobRot(3, 4);
        // assign the jacobian
        jacobVector3dRotToQuatd(
            nominalQuat.conjugate(), nominalState.tail(3), jacobRot
        );
        odomToTrueState = Eigen::MatrixXd::Zero(3, stateSize);
        odomToTrueState.block<3, 3>(0, 3) = gyroToOdom * nominalQuat.conjugate().toRotationMatrix();
        odomToTrueState.block<3, 4>(0, 6) = gyroToOdom * jacobRot;
        // true state to error state
        Eigen::MatrixXd H_odom, trueToErrJacob;
        true2errJacob(trueToErrJacob);
        H_odom = odomToTrueState * trueToErrJacob;

        // kalman gain
        kalmanGain = errCov_ * H_odom.transpose() * (
            H_odom * errCov_ * H_odom.transpose() + nOdomCov_
        ).inverse();

        // find new error-state mean
        errState = kalmanGain * innovVel;

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_odom;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nOdomCov_ * kalmanGain.transpose();
        
        /*======================[ESKF injection]========================*/
        injectNominal();
        /*======================[ESKF reset]============================*/
        resetErrState();
    }
}
/*
@odomInertialCB
    take the linear velocity in inertial odometry frame as measurement
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF::odomInertialCB(const Eigen::Vector3d &odomInerVel){
    // check if started
    if (!eskfStart){
        // direct assign the measured velocity
        nominalState.tail(3) = odomToIner * odomInerVel;
    }
    else {
        /*=================[ESKF correction====================]*/
        // innovation signal
        // Eigen::Vector3d innovVel = odomInerVel - odomToIner * nominalState.tail(3);
        Eigen::Vector3d innovVel = odomInerVel - odomToIner.transpose() * nominalState.tail(3);
        
        // measurement Jacobian
        // measurement to true state
        Eigen::MatrixXd odomToTrueState = Eigen::MatrixXd::Zero(3, stateSize);
        // odomToTrueState.block<3, 3>(0, 3) = odomToIner;
        odomToTrueState.block<3, 3>(0, 3) = odomToIner.transpose();
        // true state to error state
        Eigen::MatrixXd H_odom, trueToErrJacob;
        true2errJacob(trueToErrJacob);
        H_odom = odomToTrueState * trueToErrJacob;

        // kalman gain
        kalmanGain = errCov_ * H_odom.transpose() * (
            H_odom * errCov_ * H_odom.transpose() + nOdomCov_
        ).inverse();

        // find new error-state mean
        errState = kalmanGain * innovVel;

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_odom;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nOdomCov_ * kalmanGain.transpose();

        // debug
        // std::cout << "========[drone " << droneID_ << "]========\n"
        //           << "innov. vel.\n" << innovVel << std::endl
        //           << "odom vel:\n" << odomInerVel << std::endl
        //           << "nominal vel.:\n" << nominalState.tail(3) << std::endl
        //           << "nominal position:\n" << nominalState.head(3) << std::endl
        //           << "Iner to Odom:\n" << odomToIner.transpose() << std::endl
        //           << "true 2 err Jacob:\n" << trueToErrJacob << std::endl
        //           << "odom 2 true state:\n" << odomToTrueState << std::endl
        //           << "kalmanGain:\n" << kalmanGain << std::endl
        //           << "error state from odom:\n" << errState << std::endl;
        
        /*======================[ESKF injection]========================*/
        injectNominal();
        /*======================[ESKF reset]============================*/
        resetErrState();
    }
}
/*
@imuInputCB
    take the lin acc and ang vel as input
    do not take orientation measurement
*/
void FullQuatESKF::imuInputCB(
    const Eigen::Vector3d &imuAcc, const Eigen::Vector3d &imuAngVel,
    const Eigen::Quaterniond &imuQuat
){
    // assign imu input
    // imuInput.head(3) = imuAcc;
    // imuInput.tail(3) = imuAngVel;

    // assign in motion axis
    // imuInput.head(3) = -G0 * imuAcc;
    // imuInput.tail(3) = -imuAngVel;
    
    // assign in body frame 
    imuInput.head(3) = G0 * imuAcc;
    imuInput.tail(3) = imuAngVel;

    // initialize orientation if not started
    if (!eskfStart){
        // nominalQuat = imuQuat.conjugate() * quatGyro2Iner;
        // assign in motion axis
        // nominalQuat = imuQuat * quatGyro2Iner;
        nominalQuat = quatGyro2Iner * imuQuat;
        imuQuatPrev = imuQuat;

        // debug
        std::cout << "setting initial quaternion:\n" << nominalQuat.w() << std::endl
                  << nominalQuat.vec() << std::endl;
    }
    else if (useQuatVec4d){
        // pre-process the imu measurement
        // measure the discountinuity
        Eigen::Vector3d imuQuatDiff = quaterniondToRvec(imuQuat * imuQuatPrev.conjugate());
        
        if (imuQuatDiff.norm() > 5 * M_PI / 180){
            invertIMUQuat = !invertIMUQuat;
            // debug
            std::cout << "invert the measured quaternion!" << std::endl
                      << "inverting quaternion:\n" << imuQuat.w() << std::endl
                      << imuQuat.vec() << std::endl;
        }
        Eigen::Quaterniond q_imu = imuQuatAdjust(imuQuat);
        
        // update the quaternion as a R^4 vector
        Eigen::Vector4d innovRvec;
        Eigen::MatrixXd H_q4d;
        if (addPerturb){
            Eigen::Quaterniond q_perturb = rvecToQuaterniond(
                quatGyro2Iner.vec() / quatGyro2Iner.vec().norm(),
                angPerturb
            );
            innovRvec = quaterniondToVec4d(q_imu) - 
                quatdToMultiplyMat((quatGyro2Iner * q_perturb).conjugate(), LEFT_MULTIPLY) * quaterniondToVec4d(nominalQuat);
            // jacobian
            Eigen::MatrixXd trueToErrJacob;
            true2errJacob(trueToErrJacob);
            Eigen::MatrixXd quat4dToTrueState = Eigen::MatrixXd::Zero(4, stateSize);
            quat4dToTrueState.block<4, 4>(0, 6) = quatdToMultiplyMat((quatGyro2Iner * q_perturb).conjugate(), LEFT_MULTIPLY);
            // debug
            // std::cout << "q4d to true-state:\n" << quat4dToTrueState << std::endl;
            H_q4d = quat4dToTrueState * trueToErrJacob;
            // debug
            // std::cout << "Jacobian to err-state:\n" << H_q4d << std::endl;
        }
        else {    
            innovRvec = quaterniondToVec4d(q_imu) - 
                quatdToMultiplyMat(quatGyro2Iner.conjugate(), LEFT_MULTIPLY) * quaterniondToVec4d(nominalQuat);
            // jacobian
            Eigen::MatrixXd trueToErrJacob;
            true2errJacob(trueToErrJacob);
            Eigen::MatrixXd quat4dToTrueState = Eigen::MatrixXd::Zero(4, stateSize);
            quat4dToTrueState.block<4, 4>(0, 6) = quatdToMultiplyMat(quatGyro2Iner.conjugate(), LEFT_MULTIPLY);
            // debug
            // std::cout << "q4d to true-state:\n" << quat4dToTrueState << std::endl;
            H_q4d = quat4dToTrueState * trueToErrJacob;
            // debug
            // std::cout << "Jacobian to err-state:\n" << H_q4d << std::endl;
        }
        // kalman gain
        kalmanGain = errCov_ * H_q4d.transpose() * (
            H_q4d * errCov_ * H_q4d.transpose() + nGyroCov4d_
        ).inverse();
        // estimate
        errState = kalmanGain * innovRvec;
        // Riccati recursion
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - 
            kalmanGain * H_q4d;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nGyroCov4d_ * kalmanGain.transpose();
        // update previous imu
        imuQuatPrev = imuQuat;
        // injection
        injectNominal();
        // reset
        resetErrState();
    }
}
/*
@imuCB
    take the linear acceleration and angular velocity as input
    take the orientation quaternion as measurement
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF::imuCB(
    const Eigen::Vector3d &imuAcc,
    const Eigen::Vector3d &imuAngVel, 
    const Eigen::Quaterniond &imuQuat
){
    // assign imu input
    imuInput.head(3) = G0 * imuAcc;
    imuInput.tail(3) = imuAngVel;

    // assign in motion axis
    // imuInput.head(3) = -G0 * imuAcc;
    // imuInput.tail(3) = -imuAngVel;

    // check if started
    if (!eskfStart){
        // initialize the nominal quaternion as gyro,i --> Ii
        // imuQuat: gyro.init --> gyro,i
        // nominalQuat = imuQuat.conjugate() * quatGyro2Iner;
        // imuQuatPrev = imuQuat;

        // assign in motion axis
        // nominalQuat = imuQuat * quatGyro2Iner;
        // imuQuatPrev = imuQuat.conjugate();
        
        // use correct quaternion product order
        nominalQuat = quatGyro2Iner * imuQuat;
        imuQuatPrev = imuQuat;

        // debug
        std::cout << "========[drone " << droneID_ << "]========\n"
                  << "setting initial quaternion:\n" << nominalQuat.w() << std::endl
                  << nominalQuat.vec() << std::endl
                  << "initial rotation:\n" << nominalQuat.toRotationMatrix() << std::endl;
    }
    else {
        // debug
        // std::cout << "determine to invert IMU or not..." << std::endl;

        // pre-process the imu measurement
        // measure the discountinuity
        Eigen::Vector3d imuQuatDiff = quaterniondToRvec(imuQuat * imuQuatPrev.conjugate());
        
        if (imuQuatDiff.norm() > 5 * M_PI / 180){
            invertIMUQuat = !invertIMUQuat;
            // debug
            std::cout << "invert the measured quaternion!" << std::endl
                      << "inverting quaternion:\n" << imuQuat.w() << std::endl
                      << imuQuat.vec() << std::endl;
        }
        Eigen::Quaterniond q_imu = imuQuatAdjust(imuQuat);

        /*==================[ESKF orientation quaternion correction]======*/
        // debug
        // std::cout << "find the innovation rvec" << std::endl;
        
        // Eigen::Vector3d innovRvec = quaterniondToRvec(
        //     nominalQuat * quatGyro2Iner.conjugate() * q_imu
        // );

        Eigen::Vector3d innovRvec;
        if (addPerturb){
            Eigen::Quaterniond q_perturb = rvecToQuaterniond(
                quatGyro2Iner.vec() / quatGyro2Iner.vec().norm(),
                angPerturb
            );
            innovRvec = quaterniondToRvec(
                nominalQuat.conjugate() * quatGyro2Iner * q_perturb * q_imu
            );
        }
        else{
            innovRvec = quaterniondToRvec(
                nominalQuat.conjugate() * quatGyro2Iner * q_imu
            );
        }
        // use correct quaternion product order
        // Eigen::Vector3d innovRvec = quaterniondToRvec(
        //     nominalQuat.conjugate() * quatGyro2Iner * q_imu
        // );
        // measurement Jacobian
        // Eigen::MatrixXd H_gyro = nominalQuat.toRotationMatrix() * jacobErrStateRvec;

        // 2nd order term of the inverse of the right Jacobian
        // Eigen::Matrix3d invRightJacob = Eigen::Matrix3d::Identity(3, 3) - 
        //     0.5 * vector3dToSkeySym(
        //         nominalQuat.toRotationMatrix() * errState.segment(6, 3)
        //     );
        // since the prior estimate is 0
        // Eigen::Matrix3d invRightJacob = Eigen::Matrix3d::Identity(3, 3);

        // kalman gain
        // debug
        // std::cout << "compute kalman gain" << std::endl;
        
        // kalmanGain = errCov_ * H_gyro.transpose() * (
        //     H_gyro * errCov_ * H_gyro.transpose() + 
        //     invRightJacob * nGyroCov_ * invRightJacob.transpose()
        // ).inverse();
        
        // using correct quaternion product order
        kalmanGain = errCov_ * jacobErrStateRvec.transpose() * (
            jacobErrStateRvec * errCov_ * jacobErrStateRvec.transpose() + nGyroCov_
        ).inverse();

        // innovation signal for estimate
        // debug
        // std::cout << "compute the posterior estimate of error-state" << std::endl;
        errState = kalmanGain * innovRvec;
        
        // Riccati recursion in Joseph form
        // debug
        // std::cout << "Riccati recursion in Joseph form" << std::endl;

        // Eigen::MatrixXd temp = 
        //     Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_gyro;
        // errCov_ = temp * errCov_ * temp.transpose() + 
        //     (kalmanGain * invRightJacob) * nGyroCov_ * (kalmanGain * invRightJacob).transpose();

        // use the correct quaternion product order
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - 
            kalmanGain * jacobErrStateRvec;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nGyroCov_ * kalmanGain.transpose();

        // update the previous measured quaternion
        imuQuatPrev = imuQuat;

        // debug
        // std::cout << "nominal quat:\n" << nominalQuat.w() << std::endl << nominalQuat.vec() << std::endl
        //           << "rotation matrix from nominal quat:\n" << nominalQuat.toRotationMatrix() << std::endl
        //           << "measured quat:\n" << imuQuat.w() << std::endl << imuQuat.vec() << std::endl
        //           << "q_imu:\n" << q_imu.w() << std::endl << q_imu.vec() << std::endl
        //           << "innovation vector:\n" << innovRvec << std::endl;
        // std::cout << "========[drone " << droneID_ << "]========\n" 
        //           << "innovation vector:\n" << innovRvec << std::endl;

        /*=====================[ESKF injection]==========================*/
        injectNominal();
        /*=====================[ESKF reset]==============================*/
        resetErrState();                        
    }
}
/*
@ArUcoPositionCB
    for drone1 and drone2 position
    measured from drone0, update at drone1/2

    input required
        measurement model: 
            drone0 nominalQuat, 
            drone0 nominal position,
            drone0 to dronei initial relative pose --> initial measurement
    
    take the relative position from ArUco marker in tilted camera frame
    
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF::ArUcoPositionCB(
    const Eigen::Vector3d &p_measured,
    const Eigen::Quaterniond &q_measured,
    const Eigen::Quaterniond &q_drone0,
    const Eigen::Vector3d &p_drone0
){
    // check if started
    if (!eskfStart){
        // setting initial relative pose
        
        // q_measured gives Mi --> C0 rotation
        // we want Ii --> I0
        // so compute Mi(Ii) --> C0 and C0 --> I0

        // rotIneriToIner0 = camToCmd * q_measured.toRotationMatrix();
        // rotIneriToIner0 = camToCmd * q_measured.conjugate().toRotationMatrix();
        // rotIneriToIner0 = nominalQuat.toRotationMatrix() * gyroToTiltCam.transpose() * q_measured.toRotationMatrix();
        
        // B0 --> I0, C0 --> B0, Mi(Ii) --> C0      
        // note that it is captured in tilted camera frame
        rotIneriToIner0 = q_drone0.toRotationMatrix() * gyroToTiltCam.transpose() * q_measured.toRotationMatrix();
        
        // p_measured should be Mi --> C0 in C0 frame
        // but we want I0 --> Ii in I0
        // add the - sign on the p_measured
        // markerPositionInit = camToCmd * -p_measured;
        // markerPositionInit = 
        //     nominalQuat.toRotationMatrix() * gyroToTiltCam.transpose() * -p_measured + 
        //     p_drone0 - rotIneriToIner0 * nominalState.head(3);
        markerPositionInit = 
            q_drone0.toRotationMatrix() * gyroToTiltCam.transpose() * -p_measured + 
            p_drone0 - rotIneriToIner0 * nominalState.head(3);
        // debug
        std::cout << "========[drone " << droneID_ << " aruco]========\nI0 to I" << droneID_ << " translation:\n"
                  << markerPositionInit << std::endl
                  << "I" << droneID_ << " to I0 rotation:\n" << rotIneriToIner0 << std::endl
                  << "marker to C0 translation in C0:\n" << -p_measured << std::endl
                  << "marker to C'0 translation in C'0:\n" << -tiltCamToIdeal*p_measured << std::endl
                  << "drone 0's position:\n" << p_drone0 << std::endl
                  << "drone 1's position:\n" << nominalState.head(3) << std::endl
                  << "C0 to I0 rotation:\n" << q_drone0.toRotationMatrix() * gyroToTiltCam.transpose() << std::endl;
    }
    else {
        /*=====================[ESKF relative position measurement]========*/
        Eigen::Vector3d innovPosition = p_measured - 
            gyroToTiltCam * q_drone0.conjugate().toRotationMatrix() * (
                p_drone0 - markerPositionInit - rotIneriToIner0 * nominalState.head(3)
        );
        // measurement Jacobian
        Eigen::MatrixXd ArUcoToTrueState = Eigen::MatrixXd::Zero(3, stateSize);
        Eigen::MatrixXd trueToErrJacob;
        ArUcoToTrueState.block<3, 3>(0, 0) = 
            -gyroToTiltCam * q_drone0.conjugate().toRotationMatrix() * rotIneriToIner0;
        true2errJacob(trueToErrJacob);
        Eigen::MatrixXd H_ArUco = ArUcoToTrueState * trueToErrJacob;

        // kalman gain
        kalmanGain = errCov_ * H_ArUco.transpose() * (
            H_ArUco * errCov_ * H_ArUco.transpose() + nArUcoCov_
        ).inverse();
        // error state
        errState = kalmanGain * innovPosition;

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_ArUco;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nArUcoCov_ * kalmanGain.transpose();

        // debug
        std::cout << "========[drone " << droneID_ << "]========\n"
                  << "aruco position innovation vector:\n" << innovPosition << std::endl;

        /*======================[ESKF injection]==========================*/
        injectNominal();
        /*======================[ESKF reset]==============================*/
        resetErrState();
    }
}
/*
@imuQuatAdjust
    adjust the imu according to the flag
    avoid the discountinuity of quaternions
*/
Eigen::Quaterniond FullQuatESKF::imuQuatAdjust(const Eigen::Quaterniond &imuQuat){
    Eigen::Quaterniond q = imuQuat;
    if (invertIMUQuat){
        q.w() *= -1;
        q.vec() *= -1;
    }
    return q;
}


// int main(int argc, char **argv) {
//     // test
//     FullQuatESKF test(0);
// }