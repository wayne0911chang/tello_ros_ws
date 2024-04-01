#ifndef QUAT_ESKF_FULL_ECC24_H
#define QUAT_ESKF_FULL_ECC24_H

// std-lib
# include <iostream>
# include <string>
# include <vector> 

// utils and parameters
# include "tello_pub/math_utils.h"

/*=========================[global parameters]================================*/
# define T_SMPL             double(0.01);

/*
@FullQuatESKF
    serve as the class to be inherited by drone's odom in dronesVIO.h
    independent of ROS system
*/
class FullQuatESKF_ECC24
{

    double t_pred       = T_SMPL;
    bool eskfStart      = false;
    bool invertIMUQuat  = false;            // for imu discountinuity
    int droneID_;
    /*========================[parameters]======================================*/
    
    
    int stateSize       = 16;
    int errStateSize    = 15;
    int inputSize       = 6;
    int biasSize        = 6;

    double nAngVelVar       = 1e-6;    // rad^2 s^-2
    double nAccVar          = 1e-6;    // g^2 --> 100 m^2 s^-4
    double nAccBiasVar      = 1e-8;    // m^2 s^-2.5
    double nAngVelBianVar   = 1e-8;    // rad^2 s^-2.5
    double nVelVar          = 1e-6;    // m^2 s^-2
    double nPositionVar     = 1e-4;    // m^2 

    // nominal state
    Eigen::VectorXd nominalState, nominalBias, imuInput;     // nominal state = [p, v], nominal bias = [a_b, omega_b]
    Eigen::Quaterniond nominalQuat;             // gyro to inertial

    // error state 
    Eigen::VectorXd errState;                   // error state mean = [dp, dv, dtheta, da, domega]
    Eigen::MatrixXd errCov_, nProcCov_;
    Eigen::Matrix3d nGyroCov_;                  // rvec measurement noise cov
    Eigen::Matrix3d nOdomCov_;
    Eigen::Matrix3d nArUcoCov_;                 // position covariance

    // measurement 
    Eigen::Quaterniond imuQuatPrev;  // previous measurement

    // constant Jacobians
    Eigen::Quaterniond quatGyro2Iner;           // the transformation quat
    Eigen::MatrixXd jacobErrStateRvec;          // errRvec w.r.t. errState
    Eigen::MatrixXd nF_;                        // noise state transit Jacob

    // frame transform
    Eigen::Matrix3d gyroToOdom, imuToGyro, camToCmd, gyroToCam, odomToIner;              

    // kalman gain
    Eigen::MatrixXd kalmanGain;

    // position measurement
    Eigen::Matrix3d rotIneriToIner0;            // Ii --> I0 rotation
    Eigen::Vector3d markerPositionInit;         // I0 --> Ii in I0 frame

public:
    /*
    @FullQuatESKF
        empty constructor and destructor
    */
    FullQuatESKF_ECC24(){};
    ~FullQuatESKF_ECC24(){};
    /*
    @FullQuatESKF
        constructor w.o. nodehandles
    */
    explicit FullQuatESKF_ECC24(int drone_id);

    /*========================[ESKF functions=========================]*/
    /*
    @predictState
        predict the nominal state w. DT kinematics
        predict the error state w. DT kinematics (zero mean)
        propagate the error cov. of error state
    */
    void predictState();
    /*
    @stateTransitJacob
        find the state transition Jacobian for error states
    */
    void stateTransitJacob(Eigen::MatrixXd& errF_);
    /*
    @true2errJacob
        find the true-state to error-state Jacobian
    */
    void true2errJacob(Eigen::MatrixXd& Jacob);
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

    /*===========[callbacks for ESKF correction]====================*/
    /*
    @imuCB
        take the linear acceleration and angular velocity as input
        take the orientation quaternion as measurement
        conduct ESKF correction
        call ESKF injection and reset
    */
    void imuCB(
        const Eigen::Vector3d &imuAcc, 
        const Eigen::Vector3d &imuAngVel, 
        const Eigen::Quaterniond &imuQuat
    );
    /*
    @imuInputCB
        take the lin acc and ang vel as input
        do not take orientation measurement
    */
    void imuInputCB(
        const Eigen::Vector3d &imuAcc,
        const Eigen::Vector3d &imuAngVel,
        const Eigen::Quaterniond &imuQuat
    );
    /*
    @odomCB
        take the linear velocity as measurment
        conduct ESKF correction
        call ESKF injection and reset
    */
    void odomCB(const Eigen::Vector3d &odomVel);
    /*
    @odomInertialCB
        take the linear velocity in inertial odometry frame as measurement
        conduct ESKF correction
        call ESKF injection and reset
    */
    void odomInertialCB(const Eigen::Vector3d &odomInerVel);
    /*
    @ArUcoPositionCB
        for drone1 and drone2
        input required
            measurement model: 
                drone0 nominalQuat, 
                drone0 nominal position,
                drone0 to dronei initial relative pose --> initial measurement
        take the relative position from ArUco marker
        conduct ESKF correction
        call ESKF injection and reset
    */
    void ArUcoPositionCB(
        const Eigen::Vector3d &p_measured,
        const Eigen::Quaterniond &q_measured,
        const Eigen::Quaterniond &q_drone0,
        const Eigen::Vector3d &p_drone0
    );

    /*=============[getters]===============================================*/
    /*
    @getNominalState
        pass by reference
    */
    void getNominalState(
        Eigen::VectorXd &nominalPosVel_, 
        Eigen::Quaterniond &nominalQuat_,
        Eigen::VectorXd &nominalBias_
    ) { nominalPosVel_ = nominalState; nominalQuat_ = nominalQuat; nominalBias_ = nominalBias; };
    /*
    @getErrState
        pass by ref
    */
    void getErrState(Eigen::VectorXd &error) { error = errState; };
    /*
    @geterrCov
        pass by ref
    */
    void getErrCov(Eigen::MatrixXd &errCov) { errCov = errCov_; };
    /*
    @getTs
        return t_pred
    */
    double getTs() { return t_pred; }
    /*
    @getDroneID
        return droneID_
    */
    int getDroneID() { return droneID_; }

    /*=====================[setter]===========================*/
    /*
    @setStartFlag
        set the eskfStart flag
    */
    void setStartFlag(const bool flag) { eskfStart = flag; }

    /*====================[other functions]======================*/
    /*
    @imuQuatAdjust
        invert the imu according to the flag
    */
    Eigen::Quaterniond imuQuatAdjust(const Eigen::Quaterniond &imuQuat);
};







#endif