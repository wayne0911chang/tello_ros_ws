#ifndef QUAT_ESKF_FULL_H
#define QUAT_ESKF_FULL_H

// std-lib
# include <iostream>
# include <string>
# include <vector> 

// utils and parameters
# include "tello_pub/math_utils.h"

/*=========================[global parameters]================================*/
# define T_SMPL             double(0.01);
# define T_SMPL_F           double(0.05);

/*
@FullQuatESKF
    serve as the class to be inherited by drone's odom in dronesVIO.h
    independent of ROS system
*/
class FullQuatESKF
{
    double t_pred       = T_SMPL;
    bool eskfStart      = false;
    bool invertIMUQuat  = false;            // for imu discountinuity
    bool useQuatVec4d   = false;
    bool addPerturb     = false;
    int droneID_;

    /*========================[parameters]======================================*/    
    int stateSize       = 16;
    int errStateSize    = 15;
    int inputSize       = 6;
    int biasSize        = 6;

    double nAngVelVar       = 1e-6;    // rad^2 s^-2
    // double nAccVar          = 1e-6;    // g^2 --> 100 m^2 s^-4
    double nAccVar          = 1e-4;    // m^2 s^-4
    double nAccBiasVar      = 1e-8;    // m^2 s^-2.5
    double nAngVelBiasVar   = 1e-8;    // rad^2 s^-2.5
    double nVelVar          = 1e-6;    // m^2 s^-2
    double nPositionVar     = 1e-4;    // m^2 

    // double rollCamInit      = -0.206;  // rad, avg of two experimental roll angle
    double rollCamInit      = 0.206;  // rad, abs value of the avg of two experimental roll angle
    double angPerturb       = 0.0;

    // nominal state
    // nominal state = [p, v], nominal bias = [a_b, omega_b]
    // imuInput represent in {imu,i} motion axis
    Eigen::VectorXd nominalState, nominalBias, imuInput;     
    Eigen::Quaterniond nominalQuat;             // gyro to inertial

    // error state 
    Eigen::VectorXd errState;                   // error state mean = [dp, dv, dtheta, da, domega]
    Eigen::MatrixXd errCov_, nProcCov_;
    Eigen::Matrix3d nGyroCov_;                  // rvec measurement noise cov
    Eigen::Matrix4d nGyroCov4d_;                // rvec measurement noise cov
    Eigen::Matrix3d nOdomCov_;
    Eigen::Matrix3d nArUcoCov_;                 // position covariance

    // measurement 
    Eigen::Quaterniond imuQuatPrev;  // previous measurement

    // constant Jacobians
    Eigen::Quaterniond quatGyro2Iner;           // the transformation quat
    Eigen::MatrixXd jacobErrStateRvec;          // errRvec w.r.t. errState
    Eigen::MatrixXd nF_;                        // noise state transit Jacob

    // frame transform
    Eigen::Matrix3d gyroToOdom, imuToGyro, camToCmd, gyroToTiltCam, odomToIner, tiltCamToIdeal;              

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
    FullQuatESKF(){};
    ~FullQuatESKF(){};
    /*
    @FullQuatESKF
        constructor w.o. nodehandles
    */
    explicit FullQuatESKF(int drone_id, double t_smpl);

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
    @getInitState
        inertial 0 to inertial i
    */
    void getInitState(
        Eigen::Vector3d &initPos_,
        Eigen::Matrix3d &initRot_
    ) { initPos_ = markerPositionInit; initRot_ = rotIneriToIner0; };
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
    /*
    @setQuatVec4dFlag
    */
    void setQuatVec4dFlag(const bool flag) { useQuatVec4d = flag; }
    /*
    @setQuatVec4dFlag
    */
    void setPerturbFlag(const bool flag, const double angle) { addPerturb = flag; angPerturb = angle; }
    /*====================[other functions]======================*/
    /*
    @imuQuatAdjust
        invert the imu according to the flag
    */
    Eigen::Quaterniond imuQuatAdjust(const Eigen::Quaterniond &imuQuat);
};

/*
@FacePoseESKF
    Estimate the face odometry from drone1's view
    assume constant velocity and constant angular velocity
    noisy angular acceleration and linear acceleration

    required:
        callback for update the drone1's state
        if useYOLO:
            then callback for update drone2 and drone0 state + YOLO state

    independent of ROS system
*/
class FacePoseESKF
{
    double t_pred           = T_SMPL_F;
    bool eskfStart          = false;
    bool useYOLO            = false;
    bool usePostRvecJacob   = true;
    bool invertFaceQuat     = false;
    bool useQuatVec4d       = false;
    bool addPerturb         = false;

    /*========================[parameters]======================================*/    
    int stateSize       = 13;
    int errStateSize    = 12;
    int noiseSize       = 6;

    // double rollCamInit      = -0.206;  // rad, avg of two experimental roll angle
    double rollCamInit      = 0.206;  // rad, abs value of the avg of two experimental roll angle
    double angPerturb       = 0.0;

    // stochastic integration
    double nAngVelVar       = 1e-6;    // rad^2 s^-3
    double nAccVar          = 1e-6;    // m^2 s^-5
    double nRvecVar         = 1e-4;    // rad^2
    double nPositionVar     = 1e-4;    // m^2 

    // nominal state
    Eigen::VectorXd nominalState;
    Eigen::Vector3d nominalAngVel;     
    Eigen::Quaterniond nominalQuat;             // face to face init

    // error state 
    Eigen::VectorXd errState;                   // error state mean = [dp, dv, dtheta, domega]
    Eigen::MatrixXd errCov_, nProcCov_;
    Eigen::Matrix3d nRvecCov_;                  // rvec aruco noise cov
    Eigen::Matrix4d nRvec4dCov_;                // rvec4d aruco noise cov
    Eigen::Matrix3d nPosCov_;                   // position covariance

    // measurement previous
    Eigen::Quaterniond faceQuatPrev;            // previous measurement
    Eigen::Vector3d facePositionPrev;

    // constant transformation
    Eigen::Quaterniond quatTiltCamToGryo, quatTiltCamToIdeal;         // drone 1 frame transform

    // constant Jacobians
    Eigen::MatrixXd jacobErrStateRvec;          // errRvec w.r.t. errState
    Eigen::MatrixXd nF_;                        // noise state transit Jacob

    // kalman gain
    Eigen::MatrixXd kalmanGain;

    // position measurement
    Eigen::Matrix3d rotFaceInitToIner1;         // F,init --> I1 rotation
    Eigen::Quaterniond quatFaceInitToIner1;     // quaternion version
    Eigen::Vector3d facePositionInit;           // I1--> F,init in I1 frame

public:
    /*
    @FacePoseESKF
        empty constructor and destructor
    */
    FacePoseESKF(){};
    ~FacePoseESKF(){};
    /*
    @FullQuatESKF
        constructor w.o. nodehandles
    */
    explicit FacePoseESKF(bool useYOLO_, bool usePostRvecJacob_, double t_smpl);

    /*========================[ESKF functions=========================]*/
    /*
    @predictState
        predict the nominal state w. DT kinematics
        predict the error state w. DT kinematics (zero mean)
        propagate the error cov. of error state
    */
    void predictState();
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
    @FacePoseCB
        for drone1 
        input required
            measurement model: 
                drone1 nominalQuat, 
                drone1 nominal position,
                drone1 to face initial relative pose --> initial measurement
        take the relative pose from face landmark
        conduct ESKF correction
        call ESKF injection and reset
    */
    void FacePoseCB(
        const Eigen::Vector3d &positionCamToFace,
        const Eigen::Quaterniond &quatCamToFace,
        const Eigen::Quaterniond &q_drone1,
        const Eigen::Vector3d &p_drone1
    );
    /*
    @YOLOPose0CB
        input drone 1's state and initial states
    */
    void YOLOPose0CB(
        const Eigen::Vector3d &positionCamYolo,
        const Eigen::Quaterniond &quatCamYolo,
        const Eigen::Vector3d &posIner0ToIner1,
        const Eigen::Quaterniond &quatIner1ToIner0,
        const Eigen::Vector3d &p_drone0,
        const Eigen::Quaterniond &q_drone0
    );
    /*
    @YOLOPose2CB
        input drone 2's state and initial states
        input drone 1's initial states
    */
    void YOLOPose2CB(
        const Eigen::Vector3d &positionCamYolo,
        const Eigen::Quaterniond &quatCamYolo,
        const Eigen::Vector3d &posIner0ToIner1,
        const Eigen::Vector3d &posIner0ToIner2,
        const Eigen::Quaterniond &quatIner1ToIner0,
        const Eigen::Quaterniond &quatIner2ToIner0,
        const Eigen::Vector3d &p_drone2,
        const Eigen::Quaterniond &q_drone2
    );
    /*=============[getters]===============================================*/
    /*
    @getNominalState
        pass by reference
    */
    void getNominalState(
        Eigen::VectorXd &nominalPosVel_, 
        Eigen::Quaterniond &nominalQuat_,
        Eigen::Vector3d &nominalAngVel_
    ) { nominalPosVel_ = nominalState; nominalQuat_ = nominalQuat; nominalAngVel_ = nominalAngVel; };
    /*
    @getInitState
        IF --> I1
    */
    void getInitState(
        Eigen::Vector3d &initPos_,
        Eigen::Matrix3d &initRMat_
    ) { initPos_ = facePositionInit; initRMat_ = rotFaceInitToIner1; };
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

    /*=====================[setter]===========================*/
    /*
    @setStartFlag
        set the eskfStart flag
            true: start testing
            false: re-initialize
    */
    void setStartFlag(const bool flag) { eskfStart = flag; }
    /*
    @setQuatVec4dFlag
        set the eskfStart flag
    */
    void setQuatVec4dFlag(const bool flag) { useQuatVec4d = flag; }
    /*
    @setQuatVec4dFlag
    */
    void setPerturbFlag(const bool flag, const double angle) { addPerturb = flag; angPerturb = angle; }
    /*====================[other functions]======================*/
    /*
    @faceQuatAdjust
        invert the imu according to the flag
    */
    Eigen::Quaterniond faceQuatAdjust(const Eigen::Quaterniond &faceQuat);
};







#endif