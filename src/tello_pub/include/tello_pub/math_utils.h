#ifndef MATH_UTILS_H
#define MATH_UTILS_H

# include <Eigen/Core>
# include <Eigen/Geometry>


const double G0 = 9.81;         // gravity in ms^-2
const double G1 = 1.00;         // gravity in g
const double angleThresh = 1e-10;  // threshold
const double normThresh = 1e-15;

enum quatMultiply {
    LEFT_MULTIPLY,
    RIGHT_MULTIPLY
};

/*=========================== define helper functions ==================*/
/*
@findInVec
    find a value in the given vector by brute force
*/
template <class T>
bool findInVec(T Tval, std::vector<T> &Tvec){
    for (auto t : Tvec){
        if (t == Tval) {return true;}
    }
    return false;
}
/*
@clip
    clip the value by lower and upper bound
*/
template<typename T>
T clip(const T& num, const T& lower, const T& upper){
    return std::max(lower, std::min(num, upper));
}
/*================== ros-dep functions ====================================*/
/*
@findQuatDist
    check two input quaternion are normalized
    ~~find the distance on SO(3)?~~
    using cosine property of the quaternion
*/
// double findQuatCosDist(
//     geometry_msgs::Quaternion& q1,
//     geometry_msgs::Quaternion& q2
// ){
//     return 1-std::pow((
//         q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
//     ), 2);
// }
/*=================== Eigen-based functions ================================*/
/*
@vector3dToSkewSym
    hat operator for Eigen::Vector3d
    R^3 to so(3), Lie algebra of SO(3)
*/
static void vector3dToSkeySym(const Eigen::Vector3d &rvec, Eigen::Matrix3d &so3Mat){
    so3Mat << 0.0, -rvec(2), rvec(1),
              rvec(2), 0.0, -rvec(0),
              -rvec(1), rvec(0), 0.0;
}
/*
@vector3dToSkewSym
    overload for return Eigen::Matrix3d
    hat operator for Eigen::Vector3d
    R^3 to so(3), Lie algebra of SO(3)
*/
static Eigen::Matrix3d vector3dToSkeySym(const Eigen::Vector3d &rvec){
    Eigen::Matrix3d so3Mat;
    so3Mat << 0.0, -rvec(2), rvec(1),
              rvec(2), 0.0, -rvec(0),
              -rvec(1), rvec(0), 0.0;
    return so3Mat;
}
/*
@so3SkewSymToVec3d
    convert so3 to Eigen::Vector3d, a rvec incorporate the sin theta
    the vee operator map 
*/
static void so3SkewSymToVec3d(const Eigen::Matrix3d &skewSymMat, Eigen::Vector3d &rvecScaled){
    rvecScaled << skewSymMat(2, 1), skewSymMat(0, 2), skewSymMat(1, 0);
}

/*
@leftJacobFromRvec
    input:
        double angle
        Eigen::Vector3d axis
    output:
        Eigen::Matrix3d left Jacobian matrix 
*/
static Eigen::Matrix3d leftJacobFromRvec(
    const Eigen::Vector3d &axis,
    double theta
){
    if (theta < angleThresh) {
        return Eigen::Matrix3d::Identity();
    }
    else {
        Eigen::Matrix3d so3Mat;
        vector3dToSkeySym(axis, so3Mat);
        return Eigen::Matrix3d::Identity() + 
            (1 - std::cos(theta)) / theta * so3Mat + 
            ((theta - std::sin(theta)) / std::pow(theta, 3)) * so3Mat * so3Mat;
    }
}
/*
@leftJacobInvFromRvec
    input:
        double angle
        Eigen::Vector3d axis
    output:
        Eigen::Matrix3d left inv Jacobian matrix 
*/
static Eigen::Matrix3d leftJacobInvFromRvec(
    const Eigen::Vector3d &axis,
    double theta
){
    if (theta < angleThresh) {
        return Eigen::Matrix3d::Identity();
    }
    else {
        Eigen::Matrix3d so3Mat;
        vector3dToSkeySym(axis, so3Mat);
        return Eigen::Matrix3d::Identity() - 1 / 2 * so3Mat + (
            1/std::pow(theta, 2) - (1 + std::cos(theta))/(2 * theta * std::sin(theta))
        ) * so3Mat * so3Mat;
    }
}
/*
@adjointActionMatSE3
    input:
        Eigen::Matrix3d rotation matrix
        Eigen::Vector3d translation vector
    output:
        Eigen::Matrix6d adjoint action matrix
*/
static Eigen::MatrixXd adjointActionMatSE3(
    const Eigen::Matrix3d &rotMat,
    const Eigen::Vector3d &posVec
){
    Eigen::MatrixXd AdM = Eigen::MatrixXd::Zero(6, 6);
    AdM.block<3, 3>(0, 0) = AdM.block<3, 3>(3, 3) = rotMat;
    AdM.block<3, 3>(3, 0) = vector3dToSkeySym(posVec) * rotMat;
    return AdM;
}
/*
@rvecToQuaterniond
    convert Eigen::Vector3d with rotation angle to unit quaternion
*/
static Eigen::Quaterniond rvecToQuaterniond(const Eigen::Vector3d &axis, double theta){
    if (theta < angleThresh){
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Quaterniond q;
    q.w() = std::cos(theta / 2.0f);
    q.vec() = std::sin(theta / 2.0f) * axis;
    
    return q;
}
/*
@vector3dToQuaterniond
    convert Eigen::Vector3d to unit quaternion
*/
static Eigen::Quaterniond vector3dToQuaterniond(const Eigen::Vector3d &rvec){
    double theta = rvec.norm();
    if (theta < angleThresh){
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Vector3d axis = rvec / theta;
    return rvecToQuaterniond(axis, theta);
}
/*
@quaterniondToRvec
    convert the Eigen::Quaterniond to rotation vector in Eigen::Vector3d with std::atant2 function
    one could use truncated atan Taylor series either to avoid the small norm singularity problem.
*/
static Eigen::Vector3d quaterniondToRvec(const Eigen::Quaterniond &q){
    if (q.vec().norm() < normThresh) { return Eigen::Vector3d(2 * std::atan2(q.vec().norm(), q.w()) * q.vec() / normThresh); }
    else { return Eigen::Vector3d(2 * std::atan2(q.vec().norm(), q.w()) * q.vec() / q.vec().norm()); }
    
}
/*
@quaterniondToVec4d
    convert the quaternion into R^4 in double
    in the [w(), vec()]^T manner
*/
static Eigen::Vector4d quaterniondToVec4d(const Eigen::Quaterniond &q){
    Eigen::Vector4d q4d;
    q4d(0) = q.w();
    q4d.tail(3) = q.vec();
    return q4d;
}
/*
@quatMultiplyMat
    return the right/left multiply matrix for Eigen::Quaterniond type
    int multiplyType:
        0 --> LEFT
        1 --> RIGHT
*/
static Eigen::Matrix4d quatdToMultiplyMat(
    const Eigen::Quaterniond &quat, int multiplyType
){
    if (multiplyType == LEFT_MULTIPLY) {
        Eigen::Matrix4d qL = quat.w() * Eigen::Matrix4d::Identity(4, 4);
        qL.block<3, 3>(1, 1) += vector3dToSkeySym(quat.vec());
        qL.block<3, 1>(1, 0) += quat.vec();
        qL.block<1, 3>(0, 1) += -quat.vec().transpose();
        return qL;
    }
    else if (multiplyType == RIGHT_MULTIPLY) {
        Eigen::Matrix4d qR = quat.w() * Eigen::Matrix4d::Identity(4, 4);
        qR.block<3, 3>(1, 1) += -vector3dToSkeySym(quat.vec());
        qR.block<3, 1>(1, 0) += quat.vec();
        qR.block<1, 3>(0, 1) += -quat.vec().transpose();
        return qR;
    }
    else {
        std::cout << "WRONG TYPE: " << multiplyType << std::endl
                  << "Return identity" << std::endl;
        return Eigen::Matrix4d::Identity(4, 4);
    }
}
/*
@jacobVector3dRotToQuatd
    Return the Jacobian given the quaternion to rotate and vector 
    \partial q \otimes v \otimes q*
    -------------------------------
    \partial q
*/
static void jacobVector3dRotToQuatd(
    const Eigen::Quaterniond &quat,
    const Eigen::Vector3d &vec,
    Eigen::MatrixXd &jacob
){
    if ((jacob.cols() != 4) && (jacob.rows() != 3)){
        std::cout << "incorret size of Jacobian!" << std::endl;
    }
    else {
        double scalar = quat.vec().transpose() * vec;
        jacob.block<3, 1>(0, 0) = 2 *quat.w() * vec + 2 * vector3dToSkeySym(quat.vec()) * vec;
        jacob.block<3, 3>(0, 1) = 
            2 * scalar * Eigen::Matrix3d::Identity(3, 3) + 
            2 * quat.vec() * vec.transpose() - vec * quat.vec().transpose() - 
            2 * quat.w() * vector3dToSkeySym(vec);
    }
}
/*=======================[ROS-Eigen depends]=================================*/



# endif