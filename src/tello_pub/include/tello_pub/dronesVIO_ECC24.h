#ifndef DRONESVIO_H
#define DRONESVIO_H

// ros-dep
# include <ros/ros.h>
# include <tf/tf.h>
# include <std_msgs/Empty.h>
# include <std_msgs/Float64MultiArray.h>
# include <geometry_msgs/PoseArray.h>
# include <geometry_msgs/Accel.h>
# include <nav_msgs/Odometry.h>
# include <sensor_msgs/Imu.h>
# include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
# include <tf/transform_datatypes.h>

// Eigen
# include <eigen3/Eigen/Dense>
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

// full eskf depends
# include "tello_pub/quat_eskf_full_ECC24.h"

/*================== define global variables ============================*/


/*
@Drone0InerOdom
    provide the odometry for drone0
    constructor inherit from ErrorStateKalmanFilter class
*/
class Drone0InerOdom : public FullQuatESKF_ECC24
{
    /*======================================[ros-dep]====================*/
    // sub and pub
    ros::Subscriber imu_sub, odom_sub, SOT_sub;
    ros::Publisher eskf_pub, eskf_bias_pub, covTr_pub, eskf_odom_pub;
    int queue_size = 1;

    // timing
    ros::Timer predictStateTimer;
    ros::Time startTimeEskf;
    double t_smpl = 0.05;           // default publish state in 20Hz

    // process
    bool StartOfTest = false;
    bool rvizEnable = false;

    /*=============================[parameters]===========================*/
    // drone name
    std::string droneName = "/tello_0";

    // record input and pass to trajectory msg
    Eigen::Vector3d imuAcc, imuAngVel;

    // compare with [Joan Sola 2017]
    bool useImuQuat = true;
    
    /*=============================[functions]==================================*/
    /*
    @telloSOTCallback
        trigger the initialization of timer and start flag
    */
    void telloSOTCallback(std_msgs::Empty const& msg);
    /*
    @imuCB
        input
        imu input (acc, ang vel) to Eigen::Vector3d
        
        measurement
        correction and injection
        imu orientation to Eigen::Quaterniond
    */
    void imuCB(sensor_msgs::Imu const& imu);
    /*
    @odomCB
        measurement
        correction and injection
        reset...?
        odom velocity to Eigen::Vector3d
    */
   void odomCB(nav_msgs::Odometry const& odom);
    /*
    @predictTimerCB
            publish droneState 
            run the internal predictState
    */
    void predictTimerCB(ros::TimerEvent const& ev);
    /*
    @publishTrueState
        publish true state estimate
    */
    void publishTrueState();
    
public:
    /*
    @getTs
        return the t_smpl
    */
    double getTs(){ return t_smpl; }

public:
    /*
    @Drone0InerOdom
        empty constructor and destructors
    */
    Drone0InerOdom(){}
    ~Drone0InerOdom(){}
    /*
    @Drone0InerOdom
        explicit constructor
    */
    explicit Drone0InerOdom(ros::NodeHandle& nh_, int drone_id, bool enableRViz);
};

#endif