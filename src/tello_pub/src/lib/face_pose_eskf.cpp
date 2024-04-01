# include "tello_pub/quat_eskf_full.h"

/*
@FacePoseESKF
    constructor
*/
FacePoseESKF::FacePoseESKF(bool useYOLO_, bool usePostRvecJacob_, double t_smpl){
    /*===========================[parameters]================================*/
    t_pred = t_smpl;
    
    // initialization
    nominalQuat = Eigen::Quaterniond::Identity();
    nominalState = Eigen::VectorXd::Zero(stateSize-7);
    nominalAngVel = Eigen::Vector3d::Zero();
    errState = Eigen::VectorXd::Zero(errStateSize);

    // setting the error state cov
    errCov_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);

    // setting the process noise impulse covariance
    nProcCov_ = Eigen::MatrixXd::Identity(noiseSize, noiseSize);
    nProcCov_.block<3, 3>(0, 0) *= nAccVar * t_pred;
    nProcCov_.block<3, 3>(3, 3) *= nAngVelVar * t_pred;

    // setting the measurement noise impulse covariance
    // no need of square of sampling time!!
    nRvecCov_ = nRvecVar * Eigen::Matrix3d::Identity(3, 3);
    nRvec4dCov_ = nRvecVar * Eigen::Matrix4d::Identity(4, 4);
    nPosCov_ = nPositionVar * Eigen::Matrix3d::Identity(3, 3);

    // setting the constant noise state transit Jacob
    nF_ = Eigen::MatrixXd::Zero(errStateSize, noiseSize);
    nF_.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
    nF_.block<3, 3>(9, 3) = Eigen::Matrix3d::Identity();

    // setting the constant transformation
    Eigen::Matrix3d gyroToTiltCam;
    // gyroToTiltCam << 0, 1, 0,
    //              std::sin(rollCamInit), 0, -std::cos(rollCamInit),
    //              -std::cos(rollCamInit), 0, -std::sin(rollCamInit);
    // use body frame as the sensing frame
    gyroToTiltCam << 0, 1, 0,
                 -std::sin(rollCamInit), 0, std::cos(rollCamInit),
                 std::cos(rollCamInit), 0, std::sin(rollCamInit);
    quatTiltCamToGryo = gyroToTiltCam.transpose();
    // compute tilt camera to ideal camera
    Eigen::Matrix3d tiltCamToIdeal;
    // tiltCamToIdeal << 1.0, 0.0, 0.0,
    //                   0.0, std::cos(rollCamInit), -std::sin(rollCamInit),
    //                   0.0, std::sin(rollCamInit), std::cos(rollCamInit);
    // use body frame as the sensing frame
    tiltCamToIdeal << 1.0, 0.0, 0.0,
                      0.0, std::cos(rollCamInit), std::sin(rollCamInit),
                      0.0, -std::sin(rollCamInit), std::cos(rollCamInit);
    quatTiltCamToIdeal = tiltCamToIdeal;

    // setting the constant selection matrix
    jacobErrStateRvec = Eigen::MatrixXd::Zero(3, errStateSize);
    // jacobErrStateRvec.block<3, 3>(0, 6) = -Eigen::Matrix3d::Identity(3, 3);
    // using correct quaternion product order
    jacobErrStateRvec.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity(3, 3);


    // setting the measurement initial
    facePositionInit = Eigen::Vector3d::Zero();
    rotFaceInitToIner1 = Eigen::Matrix3d::Identity();
    quatFaceInitToIner1 = Eigen::Quaterniond::Identity();
    faceQuatPrev = Eigen::Quaterniond::Identity();

    // setting the flag
    useYOLO = useYOLO_;
    usePostRvecJacob = usePostRvecJacob_;
}
/*===================[ESKF functions]========================*/
/*
@true2errJacob
    find the true-state to error-state Jacobian
*/
void FacePoseESKF::true2errJacob(Eigen::MatrixXd &Jacob){
    
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
void FacePoseESKF::predictState(){
    // check if started
    if (!eskfStart) {
        // not started --> initialize the states
        nominalQuat = Eigen::Quaterniond::Identity();
        nominalState = Eigen::VectorXd::Zero(stateSize-7);
        nominalAngVel = Eigen::Vector3d::Zero();
        errState = Eigen::VectorXd::Zero(errStateSize);

        errCov_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);
    }
    else {
        // started --> predict
        // nominal state prediction
        // acceleration from noise in F,init
        Eigen::Quaterniond quat_angvel = 
            vector3dToQuaterniond(nominalAngVel * t_pred);

        // predict nominal state
        nominalState.head(3) += nominalState.tail(3) * t_pred;
        // nominalState.tail(3) = nominalState.tail(3);
        nominalQuat = nominalQuat * quat_angvel;
        // nominalAngVel = nominalAngVel;

        // error state mean prediction --> zero mean, skipped
        // Find Jacobian
        Eigen::MatrixXd errF_ = Eigen::MatrixXd::Identity(errStateSize, errStateSize);
        errF_.block<3, 3>(0, 3) = errF_.block<3, 3>(6, 9) = Eigen::Matrix3d::Identity() * t_pred;
        errF_.block<3, 3>(6, 6) = quat_angvel.toRotationMatrix().transpose();
        errCov_ = errF_ * errCov_ * errF_.transpose() + nF_ * nProcCov_ * nF_.transpose();

        // debug
        std::cout << "nominal position and velocity:\n" 
                  << nominalState << std::endl
                  << "nominal quat:\n" << nominalQuat.w() << std::endl
                  << nominalQuat.vec() << std::endl
                  << "nominal angvel:\n" << nominalAngVel << std::endl
                  << "error state covariance trace: " << errCov_.trace() << std::endl;
    }
}
/*
@injectNominal
    adopt the group composition for the state
*/
void FacePoseESKF::injectNominal(){
    // follow the right oplus operator
    nominalState += errState.head(6);
    nominalQuat = nominalQuat * vector3dToQuaterniond(errState.segment(6, 3));
    nominalAngVel += errState.tail(3);

    // ensure the unit quaternion
    // nominalQuat.normalize();
}
/*
@resetErrState
    reset error state to 0
    calculate the Jacobian
*/
void FacePoseESKF::resetErrState(){
    // reset operation corresponds to the right inverse operator
    Eigen::MatrixXd resetJacob = Eigen::MatrixXd::Identity(errStateSize, errStateSize);
    resetJacob.block<3, 3>(6, 6) -= vector3dToSkeySym(0.5 * errState.segment(6, 3));

    // reset and propagate the state
    errState = Eigen::VectorXd::Zero(errStateSize);
    errCov_ = resetJacob * errCov_ * resetJacob.transpose();
}
/*
@FacePoseCB
    for drone1 and drone2
    input required
        measurement model: 
            drone1 nominalQuat, 
            drone1 nominal position,
            drone1 to face initial relative pose --> initial measurement
    take the relative pose from face landmark
    conduct ESKF correction
        position
        rotation vector
    call ESKF injection and reset
*/
void FacePoseESKF::FacePoseCB(
    const Eigen::Vector3d &positionCamTiltToFace,
    const Eigen::Quaterniond &quatFaceToCamTilt,
    const Eigen::Quaterniond &q_drone1,
    const Eigen::Vector3d &p_drone1
){
    // map the tilted measure to ideal
    // quatFaceToCamTilt = quatTiltCamToIdeal.conjugate() * quatFaceToCamTilt;
    // positionCamTiltToFace = quatTiltCamToIdeal.toRotationMatrix() * positionCamTiltToFace;

    // check if started
    if (!eskfStart){    
        // setting initial relative pose
        // rotFaceInitToIner1 = quatFaceToCamTilt.conjugate() * quatTiltCamToGryo * q_drone1;
        // quatFaceInitToIner1 = quatFaceToCamTilt * quatTiltCamToGryo * q_drone1;
        // use correct quaternion product order
        quatFaceInitToIner1 = q_drone1 * quatTiltCamToGryo * quatFaceToCamTilt;


        // rotFaceInitToIner1 = quatFaceInitToIner1;        WRONG!
        rotFaceInitToIner1 = q_drone1.toRotationMatrix() * 
            quatTiltCamToGryo.toRotationMatrix() * quatFaceToCamTilt.toRotationMatrix();

        // wrong transform!!
        // facePositionInit = (quatTiltCamToGryo * q_drone1).toRotationMatrix() * 
        //     positionCamTiltToFace + p_drone1;

        // test --> correct!
        facePositionInit = (q_drone1 * quatTiltCamToGryo).toRotationMatrix() * 
            positionCamTiltToFace + p_drone1;

        // correct
        // facePositionInit = q_drone1.toRotationMatrix() * 
        //     quatTiltCamToGryo.toRotationMatrix() * 
        //     positionCamTiltToFace + p_drone1;

        // initialize the face pose prev
        faceQuatPrev = quatFaceToCamTilt.conjugate();
        facePositionPrev = positionCamTiltToFace;

        // no need to update position here, definitely 0!
        // no nned to update orientation here, definitely identity!
        
        // debug
        std::cout << "========[face pose eskf]========\n"
                  << "B1 to I1 rotation:\n" << q_drone1.toRotationMatrix() << std::endl
                  << "IF to I1 Rotation: \n" << rotFaceInitToIner1 << std::endl
                  << "IF to I1 Rotation from quat: \n" << quatFaceInitToIner1.toRotationMatrix() << std::endl
                  << "I1 to IF in I1 translation:\n" << facePositionInit << std::endl
                  << "R{q{B1 to I1} * q{C1 to B1}}=\n" << (q_drone1 * quatTiltCamToGryo).toRotationMatrix() << std::endl
                  << "R{q{C1 to B1} * q{B1 to I1}}=\n" << (quatTiltCamToGryo * q_drone1).toRotationMatrix() << std::endl
                  << "R{q{B1 to I1}} * R{q{C1 to B1}}=\n" << q_drone1.toRotationMatrix() * quatTiltCamToGryo.toRotationMatrix() << std::endl;
                //   << "IF to I1 previous quat.:\n" << faceQuatPrev.w() << std::endl << faceQuatPrev.vec() << std::endl
                //   << "drone 1 position:\n" << p_drone1 << std::endl << "B1 to I1:\n" << q_drone1.toRotationMatrix() << std::endl
                //   << "Cam to Body rotation:\n" << quatTiltCamToGryo.toRotationMatrix() << std::endl
                //   << "Face to Cam rotation:\n" << quatFaceToCamTilt.toRotationMatrix() << std::endl
                //   << "Cam to Face in Cam translation:\n" << positionCamTiltToFace << std::endl;
    }
    else {
        // pre-process the face pose measurement
        // check the discontinuity
        Eigen::Vector3d faceQuatDiff = quaterniondToRvec(
            quatFaceToCamTilt * faceQuatPrev
        );
        // debug
        std::cout << "========[face pose eskf]========\ndifference to previous face quat:\n" << faceQuatDiff << std::endl;
        
        // face pose could change abruptly due to either fast face motion or packet loss
        // separate the invert condition and directly assign condition
        // invert due to almost 180 deg rotation
        if (std::abs(faceQuatDiff.norm() - M_PI) < 5 * M_PI / 180){
            invertFaceQuat = !invertFaceQuat;
            // debug
            std::cout << "invert the measured quaternion!" << std::endl
                      << "inverting quaternion:\n" << quatFaceToCamTilt.w() << std::endl
                      << quatFaceToCamTilt.vec() << std::endl;
        }
        // assign due to too large rotation difference but not 180 deg
        else if (faceQuatDiff.norm() > 10 * M_PI / 180){
            // nominalQuat = quatFaceToCamTilt * quatTiltCamToGryo * q_drone1 * quatFaceInitToIner1.conjugate();
            // use correct quaternion product order!
            nominalQuat = quatFaceInitToIner1.conjugate() * q_drone1 * quatTiltCamToGryo * quatFaceToCamTilt;

            // debug
            std::cout << "too large orientation diff!\ndirectly assign new face quat F --> IF:\n"
                      << nominalQuat.w() << std::endl << nominalQuat.vec() << std::endl;
        }
        else if (useQuatVec4d) {
            Eigen::Vector4d innovRvec = 
                quaterniondToVec4d(faceQuatAdjust(quatFaceToCamTilt)) - 
                quatdToMultiplyMat(
                    quatTiltCamToGryo.conjugate() * q_drone1.conjugate() * quatFaceInitToIner1,
                    LEFT_MULTIPLY
                ) * quaterniondToVec4d(nominalQuat);
            // jacobian
            Eigen::MatrixXd H_q4d, trueToErrJacob;
            true2errJacob(trueToErrJacob);
            Eigen::MatrixXd quat4dToTrueState = Eigen::MatrixXd::Zero(4, stateSize);
            quat4dToTrueState.block<4, 4>(0, 6) = quatdToMultiplyMat(
                    quatTiltCamToGryo.conjugate() * q_drone1.conjugate() * quatFaceInitToIner1,
                    LEFT_MULTIPLY
                );
            H_q4d = quat4dToTrueState * trueToErrJacob;
            // kalman gain
            kalmanGain = errCov_ * H_q4d.transpose() * (
                H_q4d * errCov_ * H_q4d.transpose() + nRvec4dCov_
            ).inverse();
            // the estimate
            errState = kalmanGain * innovRvec;
            // Riccati recursion
            Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(errStateSize, errStateSize) - 
                kalmanGain * H_q4d;
            errCov_ = temp * errCov_ * temp.transpose() + 
                kalmanGain * nRvec4dCov_ * kalmanGain.transpose();
            // debug
            std::cout << "========[face pose]========\n"
                      << "face orientation q4d innovation vector:\n" << innovRvec << std::endl;
             /*=====================[ESKF injection]==========================*/
            injectNominal();
            /*=====================[ESKF reset]==============================*/
            resetErrState();
        }
        else {
            /*==================[ESKF orientation quaternion correction]======*/
            // Eigen::Vector3d innovRvec = quaterniondToRvec(
            //     nominalQuat * quatFaceInitToIner1 * q_drone1.conjugate() * quatTiltCamToGryo.conjugate() * 
            //     faceQuatAdjust(quatFaceToCamTilt.conjugate())
            // );
            // using correct quaternion product order and alternative measurement model
            // i.e. measurement model take rotation from F --> Camera tilted
            Eigen::Vector3d innovRvec = quaterniondToRvec(
                nominalQuat.conjugate() * quatFaceInitToIner1.conjugate() * q_drone1 * quatTiltCamToGryo * 
                faceQuatAdjust(quatFaceToCamTilt)
            );
            
            // measurement Jacobian
            // Eigen::MatrixXd H_faceOrient = nominalQuat.toRotationMatrix() * jacobErrStateRvec;

            // kalman gain depends on the prior estimate of error-state
            // thus inverse of right jacobian is identity
            // kalmanGain = errCov_ * H_faceOrient.transpose() * (
            //     H_faceOrient * errCov_ * H_faceOrient.transpose() + nRvecCov_).inverse();
            
            // using correct quaternion product order and alternative measurement model
            kalmanGain = errCov_ * jacobErrStateRvec.transpose() * (
                jacobErrStateRvec * errCov_ * jacobErrStateRvec.transpose() + nRvecCov_).inverse();

            // innovation signal
            errState = kalmanGain * innovRvec;

            // Riccati recursion in Joseph form
            // Eigen::MatrixXd tempPos = 
            //     Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_faceOrient;
            // use correct quaternion product order and alternative measurement model
            Eigen::MatrixXd tempPos = 
                Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * jacobErrStateRvec;
            
            // check the jacob flag
            if (usePostRvecJacob){
                Eigen::Matrix3d rvecHatmap = 0.5 * vector3dToSkeySym(
                    nominalQuat.toRotationMatrix() * errState.segment(6, 3)
                );
                errCov_ = tempPos * errCov_ * tempPos.transpose() + 
                    kalmanGain * (Eigen::Matrix3d::Identity() - rvecHatmap) * nRvecCov_ * 
                    (Eigen::Matrix3d::Identity() + rvecHatmap) * kalmanGain.transpose();
            }
            else {
                errCov_ = tempPos * errCov_ * tempPos.transpose() + 
                            kalmanGain * nRvecCov_ * kalmanGain.transpose();
            }    
            // debug
            std::cout << "========[face pose]========\n"
                      << "face orientation innovation vector:\n" << innovRvec << std::endl;
 
            /*=====================[ESKF injection]==========================*/
            injectNominal();
            /*=====================[ESKF reset]==============================*/
            resetErrState();
        }
        // update the previous measured quaternion
        faceQuatPrev = quatFaceToCamTilt.conjugate();

        /*=====================[ESKF relative position measurement]========*/
        // Eigen::Vector3d innovPosition = positionCamTiltToFace - 
        //     (q_drone1.conjugate() * quatTiltCamToGryo.conjugate()).toRotationMatrix() * (
        //         -p_drone1 + facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
        //     );
        // debug
        std::cout << "position measure diff:\n" << positionCamTiltToFace - facePositionPrev << std::endl;

        Eigen::Vector3d innovPosition = positionCamTiltToFace - 
            quatTiltCamToGryo.conjugate().toRotationMatrix() * q_drone1.conjugate().toRotationMatrix() * (
                -p_drone1 + facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
            );
        // measurement Jacobian
        Eigen::MatrixXd FacePositionJacob = Eigen::MatrixXd::Zero(3, stateSize);
        FacePositionJacob.block<3, 3>(0, 0) = 
            quatTiltCamToGryo.conjugate().toRotationMatrix() * 
            q_drone1.conjugate().toRotationMatrix() * rotFaceInitToIner1;
        Eigen::MatrixXd trueToErrJacob;
        true2errJacob(trueToErrJacob);
        Eigen::MatrixXd H_facePosition = FacePositionJacob * trueToErrJacob;

        // kalman gain
        kalmanGain = errCov_ * H_facePosition.transpose() * (
            H_facePosition * errCov_ * H_facePosition.transpose() + nPosCov_
        ).inverse();
        
        // error state
        errState = kalmanGain * innovPosition;

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_facePosition;
        errCov_ = temp * errCov_ * temp.transpose() + 
            kalmanGain * nPosCov_ * kalmanGain.transpose();

        // debug
        std::cout << "========[face pose]========\n"
                  << "C1 to F in C1 translation:\n" << positionCamTiltToFace << std::endl
                  << "I1 to IF in I1 translation:\n" << facePositionInit << std::endl
                  << "drone 1 position:\n" << p_drone1 << std::endl
                  << "B1 to I1 rotation:\n" << q_drone1.toRotationMatrix() << std::endl
                  << "I1 to C1 rotation:\n" << quatTiltCamToGryo.conjugate().toRotationMatrix() * q_drone1.conjugate().toRotationMatrix() << std::endl
                  << "IF to I1 rotation:\n" << rotFaceInitToIner1 << std::endl
                  << "face position:\n" << nominalState.head(3) << std::endl
                  << "face position innovation vector:\n" << innovPosition << std::endl;

        /*======================[ESKF injection]==========================*/
        injectNominal();
        /*======================[ESKF reset]==============================*/
        resetErrState();
        // update the last one 
        facePositionPrev = positionCamTiltToFace;
    }
}
/*
@YOLOPose0CB
    input cam to Yolo in cam position, Yolo to cam rotation
    input drone 0's state and drone 1's initial state
*/
void FacePoseESKF::YOLOPose0CB(
    const Eigen::Vector3d &positionCamTiltYolo,
    const Eigen::Quaterniond &quatCamTiltYolo,
    const Eigen::Vector3d &posIner0ToIner1,
    const Eigen::Quaterniond &quatIner1ToIner0,
    const Eigen::Vector3d &p_drone0,
    const Eigen::Quaterniond &q_drone0
){
    // positionCamTiltYolo = quatTiltCamToIdeal.toRotationMatrix() * positionCamTiltYolo;
    if (!eskfStart){
        // do something
    }
    else {
        // innovation signal in vector
        // Eigen::Vector3d innovPosition = positionCamTiltYolo - 
        //     (q_drone0.conjugate() * quatTiltCamToGryo.conjugate()).toRotationMatrix() * (
        //         -p_drone0 + posIner0ToIner1 + quatIner1ToIner0.toRotationMatrix() * (
        //             facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
        //         )
        //     );
        Eigen::Vector3d innovPosition = positionCamTiltYolo - 
            quatTiltCamToGryo.conjugate().toRotationMatrix() * q_drone0.conjugate().toRotationMatrix() * (
                -p_drone0 + posIner0ToIner1 + quatIner1ToIner0.toRotationMatrix() * (
                    facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
                )
            );
        // measurement Jacobian
        Eigen::MatrixXd YOLOPositionJacob = Eigen::MatrixXd::Zero(1, stateSize);
        // select the first row
        // YOLOPositionJacob.block<1, 3>(0, 0) = (
        //     (q_drone0.conjugate() * quatTiltCamToGryo.conjugate()).toRotationMatrix() * 
        //     quatIner1ToIner0.toRotationMatrix() * rotFaceInitToIner1
        // ).block<1, 3>(0, 0);
        YOLOPositionJacob.block<1, 3>(0, 0) = (
            quatTiltCamToGryo.conjugate().toRotationMatrix() * 
            q_drone0.conjugate().toRotationMatrix() * 
            quatIner1ToIner0.toRotationMatrix() * rotFaceInitToIner1
        ).block<1, 3>(0, 0);
        Eigen::MatrixXd trueToErrorJacob;
        true2errJacob(trueToErrorJacob);
        Eigen::MatrixXd H_YOLOPosition = YOLOPositionJacob * trueToErrorJacob;

        // kalman gain as vector
        kalmanGain = errCov_ * H_YOLOPosition.transpose() / 
            ((H_YOLOPosition * errCov_ * H_YOLOPosition.transpose())(0, 0) + nPositionVar);
        // error state
        errState = kalmanGain * innovPosition.head(0);

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_YOLOPosition;
        errCov_ = temp * errCov_ * temp.transpose() + 
            nPositionVar * kalmanGain * kalmanGain.transpose();

        // debug
        std::cout << "========[face YOLO 0]========\nYOLO innov position:\n" << innovPosition << std::endl
                  << "innovation covariance:\n"
                  << (H_YOLOPosition * errCov_ * H_YOLOPosition.transpose())(0, 0) + nPositionVar << std::endl;

        /*======================[ESKF injection]==========================*/
        injectNominal();
        /*======================[ESKF reset]==============================*/
        resetErrState();
    }
}
/*
@YOLOPose2CB
    input cam to Yolo in cam position, Yolo to cam rotation
    input drone 2's state and drone 1 & 2's initial state
*/
void FacePoseESKF::YOLOPose2CB(
    const Eigen::Vector3d &positionCamTiltYolo,
    const Eigen::Quaterniond &quatCamTiltYolo,
    const Eigen::Vector3d &posIner0ToIner1,
    const Eigen::Vector3d &posIner0ToIner2,
    const Eigen::Quaterniond &quatIner1ToIner0,
    const Eigen::Quaterniond &quatIner2ToIner0,
    const Eigen::Vector3d &p_drone2,
    const Eigen::Quaterniond &q_drone2
){
    if (!eskfStart){
        // do something
    }
    else {
        // innovation signal in vector
        // Eigen::Vector3d innovPosition = positionCamTiltYolo - 
        //     (q_drone2.conjugate() * quatTiltCamToGryo.conjugate()).toRotationMatrix() * (
        //         -p_drone2 + quatIner2ToIner0.conjugate().toRotationMatrix() * (
        //             -posIner0ToIner2 + posIner0ToIner1 + quatIner1ToIner0.toRotationMatrix() * (
        //                 facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
        //             )
        //         )
        //     );
        Eigen::Vector3d innovPosition = positionCamTiltYolo - 
            quatTiltCamToGryo.conjugate().toRotationMatrix() * q_drone2.conjugate().toRotationMatrix() * (
                -p_drone2 + quatIner2ToIner0.conjugate().toRotationMatrix() * (
                    -posIner0ToIner2 + posIner0ToIner1 + quatIner1ToIner0.toRotationMatrix() * (
                        facePositionInit + rotFaceInitToIner1 * nominalState.head(3)
                    )
                )
            );
        // measurement Jacobian
        Eigen::MatrixXd YOLOPositionJacob = Eigen::MatrixXd::Zero(1, stateSize);
        // select the first row
        // YOLOPositionJacob.block<1, 3>(0, 0) = (
        //     (q_drone2.conjugate() * quatTiltCamToGryo.conjugate()).toRotationMatrix() * 
        //     quatIner2ToIner0.conjugate().toRotationMatrix() * 
        //     quatIner1ToIner0.toRotationMatrix() * rotFaceInitToIner1
        // ).block<1, 3>(0, 0);
        YOLOPositionJacob.block<1, 3>(0, 0) = (
            quatTiltCamToGryo.conjugate().toRotationMatrix() * 
            q_drone2.conjugate().toRotationMatrix() * 
            quatIner2ToIner0.conjugate().toRotationMatrix() * 
            quatIner1ToIner0.toRotationMatrix() * rotFaceInitToIner1
        ).block<1, 3>(0, 0);
        Eigen::MatrixXd trueToErrorJacob;
        true2errJacob(trueToErrorJacob);
        Eigen::MatrixXd H_YOLOPosition = YOLOPositionJacob * trueToErrorJacob;

        // kalman gain as vector
        kalmanGain = errCov_ * H_YOLOPosition.transpose() / 
            ((H_YOLOPosition * errCov_ * H_YOLOPosition.transpose())(0, 0) + nPositionVar);
        
        // error state by selecting x-component only
        errState = kalmanGain * innovPosition.head(0);

        // Riccati recursion in Joseph form
        Eigen::MatrixXd temp = 
            Eigen::MatrixXd::Identity(errStateSize, errStateSize) - kalmanGain * H_YOLOPosition;
        errCov_ = temp * errCov_ * temp.transpose() + 
            nPositionVar * kalmanGain * kalmanGain.transpose();

        // debug
        std::cout << "========[face YOLO 2]========\nYOLO innov position:\n" << innovPosition << std::endl
                  << "innova. cov.\n"
                  << (H_YOLOPosition * errCov_ * H_YOLOPosition.transpose())(0, 0) + nPositionVar << std::endl;

        /*======================[ESKF injection]==========================*/
        injectNominal();
        /*======================[ESKF reset]==============================*/
        resetErrState();
    }
}
/*
@faceQuatAdjust
    adjust the imu according to the flag
    avoid the discountinuity of quaternions
*/
Eigen::Quaterniond FacePoseESKF::faceQuatAdjust(const Eigen::Quaterniond &faceQuat){
    Eigen::Quaterniond q = faceQuat;
    if (invertFaceQuat){
        q.w() *= -1;
        q.vec() *= -1;
    }
    return q;
}
