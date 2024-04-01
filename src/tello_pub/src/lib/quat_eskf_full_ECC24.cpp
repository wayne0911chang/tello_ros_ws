# include "tello_pub/quat_eskf_full_ECC24.h"

/*
@FullQuatESKF
    constructor
*/
FullQuatESKF_ECC24::FullQuatESKF_ECC24(int drone_id){

    // configure the gyro to inertial frame rotation 
    Eigen::Matrix3d rotGyro2Iner;

    // get drone id
    droneID_ = drone_id;

    // setting the gyro to inertial frame
    switch (droneID_){
        case 0:
            // debug
            std::cout << "========[eskf]========\ndrone_0 eskf constructor" << std::endl;

            // setting the constant Jacobians        
            rotGyro2Iner << 0, 1, 0,
                            1, 0, 0,
                            0, 0, -1;
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
            odomToIner << 0, 1, 0,
                          0, 0, 1,
                          1, 0, 0;
    }
    /*===========================[parameters]================================*/
    // initialization
    nominalQuat = Eigen::Quaterniond::Identity();
    nominalState = Eigen::VectorXd::Zero(stateSize - 4 - biasSize);
    nominalBias = Eigen::VectorXd::Zero(biasSize);
    imuInput = Eigen::VectorXd::Zero(inputSize);
    errState = Eigen::VectorXd::Zero(errStateSize);

    // initialize the imu input
    imuInput(2) = -G1;
    
    // setting the error state cov
    errCov_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);

    // setting the process noise impulse covariance
    nProcCov_ = Eigen::MatrixXd::Identity(stateSize - 4, stateSize - 4);
    nProcCov_.block<3, 3>(0, 0) *= nAccVar * std::pow(t_pred, 2);
    nProcCov_.block<3, 3>(3, 3) *= nAngVelVar * std::pow(t_pred, 2);
    nProcCov_.block<3, 3>(6, 6) *= nAccBiasVar * t_pred;
    nProcCov_.block<3 ,3>(9, 9) *= nAngVelBianVar * t_pred;

    // setting the measurement noise impulse covariance
    // no need of square of sampling time!!
    nGyroCov_ = nAngVelVar * Eigen::Matrix3d::Identity(3, 3);
    nOdomCov_ = nVelVar * Eigen::Matrix3d::Identity(3, 3);
    nArUcoCov_ = nPositionVar * Eigen::Matrix3d::Identity(3, 3);

    // setting the frame transformation
    quatGyro2Iner = rotGyro2Iner;
    gyroToOdom << 0, -1, 0,
                  -1, 0, 0,
                  0, 0, -1;
            
    gyroToCam << 0, 1, 0,
                 0, 0, 1,
                 1, 0, 0;

    imuToGyro << 1, 0, 0,
                  0, -1, -1,
                  0, 0, -1;

    camToCmd << 1, 0, 0,
                0, 0, 1,
                0, -1, 0;

    // setting the constant selection matrix
    jacobErrStateRvec = Eigen::MatrixXd::Zero(3, errStateSize);
    jacobErrStateRvec.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity(3, 3);

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
void FullQuatESKF_ECC24::stateTransitJacob(Eigen::MatrixXd &errF_){

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
void FullQuatESKF_ECC24::true2errJacob(Eigen::MatrixXd &Jacob){
    
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
void FullQuatESKF_ECC24::predictState(){
    // check if started
    if (eskfStart){
        // nominal state prediction
        // acceleration from {imu,i} to {gyro,i}
        // gravitational acceleration in {I_i}
        Eigen::Vector3d acc_gyro(3), gVec(3);
        acc_gyro << (imuInput(0) - nominalBias(0)), 
                    -(imuInput(1) - nominalBias(1)),
                    -(imuInput(2) - nominalBias(2));
        gVec << 0.0, 0.0, G1;

        Eigen::Quaterniond quat_angvel = 
            vector3dToQuaterniond((imuInput.tail(3) - nominalBias.tail(3)) * t_pred);

        // predict nominal state
        nominalState.head(3) += nominalState.tail(3) * t_pred + 
            0.5 * std::pow(t_pred, 2) * (nominalQuat.toRotationMatrix() * acc_gyro + gVec);
        nominalState.tail(3) += (nominalQuat.toRotationMatrix() * acc_gyro + gVec) * t_pred;
        nominalQuat = nominalQuat * quat_angvel;
        // nominalBias = nominalBias;

        // error state mean prediction --> zero mean, skipped
        // Find Jacobian
        Eigen::MatrixXd errF_;
        stateTransitJacob(errF_);
        errCov_ = errF_ * errCov_ * errF_.transpose() + nF_ * nProcCov_ * nF_.transpose();
    }
}
/*
@injectNominal
    adopt the group composition for the state
*/
void FullQuatESKF_ECC24::injectNominal(){
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
void FullQuatESKF_ECC24::resetErrState(){
    // reset operation corresponds to the right inverse operator
    Eigen::MatrixXd resetJacob = Eigen::MatrixXd::Identity(errStateSize, errStateSize);
    resetJacob.block<3, 3>(6, 6) -= vector3dToSkeySym(0.5 * errState.segment(6, 3));

    // reset and propagate the state
    errState = Eigen::VectorXd::Zero(errStateSize);
    errCov_ = resetJacob * errCov_ * resetJacob.transpose();
}
/*
@odomCB
    take the linear velocity as measurment
    if not started, set the initial condition
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF_ECC24::odomCB(const Eigen::Vector3d &odomVel){
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
void FullQuatESKF_ECC24::odomInertialCB(const Eigen::Vector3d &odomInerVel){
    // check if started
    if (!eskfStart){
        // direct assign the measured velocity
        nominalState.tail(3) = odomToIner * odomInerVel;
    }
    else {
        /*=================[ESKF correction====================]*/
        // innovation signal
        Eigen::Vector3d innovVel = odomInerVel - odomToIner * nominalState.tail(3);
        
        // measurement Jacobian
        // measurement to true state
        Eigen::MatrixXd odomToTrueState = Eigen::MatrixXd::Zero(3, stateSize);
        odomToTrueState.block<3, 3>(0, 3) = odomToIner;
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
@imuInputCB
    take the lin acc and ang vel as input
    do not take orientation measurement
*/
void FullQuatESKF_ECC24::imuInputCB(
    const Eigen::Vector3d &imuAcc, const Eigen::Vector3d &imuAngVel,
    const Eigen::Quaterniond &imuQuat
){
    // assign imu input
    imuInput.head(3) = imuAcc;
    imuInput.tail(3) = imuAngVel;

    // initialize orientation if not started
    if (!eskfStart){
        nominalQuat = imuQuat.conjugate() * quatGyro2Iner;
        // debug
        std::cout << "setting initial quaternion:\n" << nominalQuat.w() << std::endl
                  << nominalQuat.vec() << std::endl;
    }
}
/*
@imuCB
    take the linear acceleration and angular velocity as input
    take the orientation quaternion as measurement
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF_ECC24::imuCB(
    const Eigen::Vector3d &imuAcc,
    const Eigen::Vector3d &imuAngVel, 
    const Eigen::Quaterniond &imuQuat
){
    // assign imu input
    imuInput.head(3) = imuAcc;
    imuInput.tail(3) = imuAngVel;

    // check if started
    if (!eskfStart){
        // initialize the nominal quaternion as gyro,i --> Ii
        // imuQuat: gyro.init --> gyro,i
        nominalQuat = imuQuat.conjugate() * quatGyro2Iner;
        imuQuatPrev = imuQuat;

        // debug
        std::cout << "setting initial quaternion:\n" << nominalQuat.w() << std::endl
                  << nominalQuat.vec() << std::endl
                  << "initial rotation:\n" << nominalQuat.toRotationMatrix() << std::endl;
    }
    else {
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
        Eigen::Vector3d innovRvec = quaterniondToRvec(
            nominalQuat * quatGyro2Iner.conjugate() * q_imu
        );
        // measurement Jacobian
        Eigen::MatrixXd H_gyro = nominalQuat.toRotationMatrix() * jacobErrStateRvec;

        // 2nd order term of the inverse of the right Jacobian
        // Eigen::Matrix3d invRightJacob = Eigen::Matrix3d::Identity(3, 3) - 
        //     0.5 * vector3dToSkeySym(
        //         nominalQuat.toRotationMatrix() * errState.segment(6, 3)
        //     );
        // since the prior estimate is 0
        Eigen::Matrix3d invRightJacob = Eigen::Matrix3d::Identity(3, 3);

        // kalman gain
        kalmanGain = errCov_ * H_gyro.transpose() * (
            H_gyro * errCov_ * H_gyro.transpose() + 
            invRightJacob * nGyroCov_ * invRightJacob.transpose()
        ).inverse();
        
        // innovation signal
        errState = kalmanGain * innovRvec;
        
        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_gyro;
        errCov_ = temp * errCov_ * temp.transpose() + 
            (kalmanGain * invRightJacob) * nGyroCov_ * (kalmanGain * invRightJacob).transpose();

        // update the previous measured quaternion
        imuQuatPrev = imuQuat;

        // debug
        // std::cout << "nominal quat:\n" << nominalQuat.w() << std::endl << nominalQuat.vec() << std::endl
        //           << "rotation matrix from nominal quat:\n" << nominalQuat.toRotationMatrix() << std::endl
        //           << "measured quat:\n" << imuQuat.w() << std::endl << imuQuat.vec() << std::endl
        //           << "q_imu:\n" << q_imu.w() << std::endl << q_imu.vec() << std::endl
        //           << "innovation vector:\n" << innovRvec << std::endl;
        std::cout << "innovation vector:\n" << innovRvec << std::endl;

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
    take the relative position from ArUco marker
    conduct ESKF correction
    call ESKF injection and reset
*/
void FullQuatESKF_ECC24::ArUcoPositionCB(
    const Eigen::Vector3d &p_measured,
    const Eigen::Quaterniond &q_measured,
    const Eigen::Quaterniond &q_drone0,
    const Eigen::Vector3d &p_drone0
){
    // check if started
    if (!eskfStart){
        // setting initial relative pose
        // p_measured should be Mi --> C0 in C0 frame
        // but we want I0 --> Ii in I0
        // add the - sign on the p_measured
        markerPositionInit = camToCmd * -p_measured;
        
        // q_measured gives Mi --> C0 rotation
        // we want Ii --> I0
        // so compute Mi(Ii) --> C0 and C0 --> I0
        rotIneriToIner0 = camToCmd * q_measured.toRotationMatrix();
        // rotIneriToIner0 = camToCmd * q_measured.conjugate().toRotationMatrix();
    }
    else {
        /*=====================[ESKF relative position measurement]========*/
        Eigen::Vector3d innovPosition = p_measured - 
            gyroToCam * q_drone0.conjugate().toRotationMatrix() * (
                p_drone0 - markerPositionInit - rotIneriToIner0 * nominalState.head(3)
        );
        // measurement Jacobian
        Eigen::MatrixXd ArUcoToTrueState = Eigen::MatrixXd::Zero(3, stateSize);
        Eigen::MatrixXd trueToErrJacob;
        ArUcoToTrueState.block<3, 3>(0, 0) = 
            -gyroToCam * q_drone0.conjugate().toRotationMatrix() * rotIneriToIner0;
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
Eigen::Quaterniond FullQuatESKF_ECC24::imuQuatAdjust(const Eigen::Quaterniond &imuQuat){
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