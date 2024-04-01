#ifndef DRONESVIO_H
#define DRONESVIO_H

// ros-dep and eigen-dep
# include "tello_pub/ros_eigen_dep.h"

// self-defined
# include "tello_sub/ArUcoPoseArr.h"
# include "tello_pub/SE3Pose.h"

// full eskf depends
# include "tello_pub/quat_eskf_full.h"

/*================== define global variables ============================*/


/*
@Drone0InerOdom
    provide the odometry for drone0
    constructor inherit from ErrorStateKalmanFilter class
*/
class Drone0InerOdom : public FullQuatESKF
{
    /*======================================[ros-dep]====================*/
    // sub and pub
    ros::Subscriber imu_sub, odom_sub, SOT_sub;
    ros::Publisher eskf_pub, eskf_bias_pub, covTr_pub, eskf_odom_pub, quat_norm_pub;
    int queue_size = 1;

    // timing
    ros::Timer predictStateTimer;
    ros::Time startTimeEskf;
    double t_smpl = 0.05;           // default publish state in 20Hz

    double cmdVelThr = 0.1;

    // process
    bool StartOfTest = false;
    bool rvizEnable = false;
    bool blockAruco = false;

    /*=============================[parameters]===========================*/
    // drone name
    std::string droneName = "/tello_0";

    // record input and pass to trajectory msg
    Eigen::Vector3d imuAcc, imuAngVel;

    // compare with [Joan Sola 2017]
    bool useImuQuat     = true;
    bool useQuatVec4d   = false;
    
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
    @Drone0InerOdom
        empty constructor and destructors
    */
    Drone0InerOdom(){}
    ~Drone0InerOdom(){}
    /*
    @Drone0InerOdom
        explicit constructor
    */
    explicit Drone0InerOdom(ros::NodeHandle& nh_, int drone_id, bool enableRViz, double t_smpl);
    /*
    @getTs
        return the t_smpl
    */
    double getTs(){ return t_smpl; }
    /*
    @blockArUco
        return the flag
    */
    bool blockArUco() {return blockAruco; }
};




/*
@DronesVIO
    provide the odometry for drone1 and drone2
    require drone 0's odometry --> rotation and translation
    create object of drone0's odom have same life time as the class
    yet ESKF nodes need to be rewritten......?
*/
class DronesVIO 
{
    /*===================[ros-dep]===================================*/
    // create two sets of subs and pubs
    ros::Subscriber imu_1_sub, imu_2_sub, 
                    odom_1_sub, odom_2_sub,
                    marker_1_sub, marker_2_sub,
                    drone0_marker_sub,
                    SOT_sub;
    // for avoid the dynamic distortion of camera view
    ros::Subscriber cmd_vel_0_sub, cmd_vel_1_sub, cmd_vel_2_sub,
                    takeoff_sub, EOT_sub;

    ros::Publisher eskf_1_pub, eskf_2_pub,
                   eskf_bias_1_pub, eskf_bias_2_pub,
                   eskf_odom_1_pub, eskf_odom_2_pub,
                   eskf_init_1_pub, eskf_init_2_pub,
                   covTr_pub;

    ros::Timer predictStateTimer_1, predictStateTimer_2;
    ros::Time startTime_1, startTime_2; 

    /*=================[objects]==================================*/
    FullQuatESKF drone1_eskf, drone2_eskf;
    Drone0InerOdom drone0_odom;

    /*======================[parameters]===============================*/
    int queue_size = 1;
    std::string droneName[3] = {"/tello_0", "/tello_1", "/tello_2"};
    int desiredArUcoID[2] = {25, 22};

    // flags
    bool drone1_SOT = false;
    bool drone2_SOT = false;
    bool rvizEnable[2] = {false, false};
    bool blockAruco[3] = {false, false, false};     // due to fast motion

    double cmdVelThr = 0.1;

    // record imu and gyro
    Eigen::Vector3d imuAcc[2], imuAngVel[2];
    
    /*==============================[functions]===========================*/
    /*
    @drone1_velCB
        setting the flag of stop aruco callback
    */
    // void drone0_velCB(geometry_msgs::Twist const& twist);
    // void drone1_velCB(geometry_msgs::Twist const& twist);
    // void drone2_velCB(geometry_msgs::Twist const& twist);
    /*
    @drone1_imuCB
        wrapping drone1 and drone2's imu eskf
    */
    void drone1_imuCB(sensor_msgs::Imu const& imu);
    void drone2_imuCB(sensor_msgs::Imu const& imu);
    /*
    @drone1/2_odomCB
        wrapping drone1 and drone2's odom measurement
    */
    void drone1_odomCB(nav_msgs::Odometry const& odom);
    void drone2_odomCB(nav_msgs::Odometry const& odom);
    /*
    @ArUcoCB
        from drone0's point of view
        wrapping drone1 and drone2's relative position measurement
    */
    void ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr);
    /*
    @drone1/2_ArUcoCB
        possible markers appeared in drone1 and drone2's view
        TBD
    */
    void drone1_ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr);
    void drone2_ArUcoCB(tello_sub::ArUcoPoseArr const& arucoPoseArr);
    /*
    @telloSOTCB
        initialization and start flag
    */
    void telloSOTCallback(std_msgs::Empty const& msg);
    /*
    @telloEOTCallback
        disable the aruco callback by a flag
    */
    // void telloEOTCallback(std_msgs::Empty const& msg);
    /*
    @takeoffCB
        disable the aruco callback by a flag
    */
    // void takeoffCB(std_msgs::Empty const& msg);
    /*
    @drone1/2_predictTimerCB
        timer callback for drone1/2
    */
    void drone1_predictTimerCB(ros::TimerEvent const& ev);
    void drone2_predictTimerCB(ros::TimerEvent const& ev);
    /*
    @publishTrueState
        publish true state of drone1/2 eskf according to the input object
    */
    void publishTrueState(FullQuatESKF &eskfObj);


public:
    /*
    @DronesVIO
        take node handler
        construct drone0, drone1, drone2 object in the same time
    */
    DronesVIO(){}
    ~DronesVIO(){}
    explicit DronesVIO(ros::NodeHandle& nh_, double t_smpl);
};


#endif