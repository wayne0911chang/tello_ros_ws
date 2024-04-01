#ifndef FORMATION_CTRL_H
#define FORMATION_CTRL_H

// ros-dep and Eigen-dep
# include "tello_pub/ros_eigen_dep.h"
# include "tello_pub/GeometricCtrlDebug.h"
# include "tello_pub/SE3Pose.h"
# include "tello_sub/ArUcoPoseArr.h"


// Eigen geometry and core and self-built algorithm
# include "tello_pub/math_utils.h"

// parsing the matrix
# include "yaml-cpp/yaml.h"

/*======================[define global var. here]==================*/
# define DIST_XY_MIN                double(0.50)        // min. dist to human
# define DIST_Z_MAX                 double(1.20)        // max. height
# define DIST_Z_MIN                 double(-0.30)       // min. height
# define FACE_THETA_MAX             double(0.50)        // max. allowable yaw to detect the face
# define THETA_MAX                  double(M_PI*0.75)   // max. allowable yaw for drone 2
# define VEL_THR                    double(0.01)        // min. L vel. (m)

enum Config {
    cfg0,
    cfg1
};

/*
@GeometricFormCtrl
    mission assignment is given as 
        drone 0 oversee drone 1, drone 2, and human actor, tracking 
*/
class GeometricFormCtrl
{
    /*===============[ros-dep]=============================*/
    // sub and pub
    // multi-drone VIO
    ros::Subscriber eskf_0_sub, eskf_1_sub, eskf_2_sub,
                    eskf_init_1_sub, eskf_init_2_sub;
    // face odom
    ros::Subscriber eskf_face_sub, eskf_init_face_sub;
    // YOLO pose
    // ros::Subscriber YOLO_pose_0_sub, YOLO_pose_2_sub;
    // process control
    ros::Subscriber exp_mode_sub;

    // desired leader pose generation
    // ros::Subscriber face_arr_sub, aruco_sub;

    // ros::Publisher EOT_pub;
    // drone
    ros::Publisher cmd_0_pub, cmd_1_pub, cmd_2_pub;

    // debug
    // publish the gradients and cost functions
    ros::Publisher ctrller_debug_pub;

    // publish the desired pose in I0 for each drone
    ros::Publisher des_pose_0_pub, des_pose_1_pub, des_pose_2_pub;

    // timer
    ros::Timer formationCtrlTimer;
    ros::Time startTimeCtrller;

    /*===========[param]========================================*/
    // ros-dep
    int queue_size = 1;
    double t_smpl = 0.05;

    bool StartOfTest = false;
    bool disableCmd = false;
    bool angVelMode = false;
    bool noRollPitch = false;
    bool useAltPose = true;             // define rel pose in L/F --> body
    bool pubDesPose = false;            // publish drones' desired pose
    bool smoothHumanVel = false;        // use a window to smooth the human velocity

    std::string droneName[3] = {"/tello_0", "/tello_1", "/tello_2"};
    std::string ros_Nodename;


    double rollCamInit      = 0.206;  // rad, abs value of the avg of two experimental roll angle
    double height_offset    = 0.5;
    // double rollCamInit = -0.206;    // rad
    // double loaTheta = 0.0;          // rotation from IF = IL to L
    // double thetaLoAToFace = 0.0;    // rotation from L to F

    // Eigen
    // constant transformation
    Eigen::Matrix3d rotCamTiltToBody;           // tilt camera to body frame
    Eigen::Matrix3d rotCamIdealToBody;          // ideal camera to body
    Eigen::Matrix3d rotBodyToCmd;           // body to cmd frame
    Eigen::Matrix3d rotCamTilt1ToM1;
    // drone states
    Eigen::Vector3d dronesPos[3], dronesVel[3], dronesAngVel[3];
    Eigen::Quaterniond dronesQuat[3]; 
    
    // face and LoA
    /*
    face frame and LoA frame
        SAME initial frame
        SAME position in initial frame
        DIFFERENT velocity
        DIFFERENT rotation
    */
    Eigen::Vector3d facePos, faceVel, faceAngVel;   // face pos vel is loa pos vel
    Eigen::Quaterniond faceQuat;                    // F --> IF=IL
    Eigen::Matrix3d rotInitFaceToLoA;               // IF=IL --> L
    Eigen::Matrix3d rotFaceToLoA;                   // F --> L
    std::vector<Eigen::Vector3d> faceVelWndw;       
    int wndw_size = 10;

    // sensor
    Eigen::Matrix3d rotFaceToCamTilt, rotM1ToCamTilt;
    Eigen::Vector3d posCamTiltToFace, posCamTiltToM1;


    // initial conditinos
    Eigen::Vector3d posIner0ToIneri[2];     // I0 --> Ii in I0 frame
    Eigen::Vector3d posIner1ToInitFace;     // I1 --> IF = IL in I1 frame
    Eigen::Matrix3d rotIneriToIner0[2];     // Ii --> I0
    Eigen::Matrix3d rotInitFaceToIner1;     // IF = IL --> I1

    // Relative pose SE(3)
    // order follows the drone ids
    Eigen::Matrix4d droneRelPose[3];   // L --> B0, F --> B1, L --> B2
    Eigen::VectorXd droneRelTwist[3];  // L --> B0, F --> B1, L --> B2
    // desired
    Eigen::Matrix4d desRelPose[3];
    Eigen::VectorXd desRelTwist[3]; 
    // gain matrix
    Eigen::MatrixXd twistGain[3];       // 6 by 6 gain matrix, ND or Hurwitz



    /*=============[functions]==================================*/
    /*
    @telloSOTCb
    */
    void exp_mode_callback(std_msgs::Empty const& msg);
    /*
    @ctrlTimerCb

    */
    void ctrlTimerCb(ros::TimerEvent const& ev);
    /*
    @drone0ESKFCb

    */
    void drone0ESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt);
    /*
    @drone1ESKFCb

    */
    void drone1ESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt);
    /*
    @drone2ESKFCb

    */
    void drone2ESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt);
    /*
    @drone1InitCb

    */
    void drone1InitCb(tello_pub::SE3Pose const& pose);
    /*
    @drone2InitCb

    */
    void drone2InitCb(tello_pub::SE3Pose const& pose);
    /*
    @faceESKFCb

    */
    void faceESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& faceTrajPt);
    /*
    @faceInitCb

    */
    // void faceInitCb(geometry_msgs::Pose const& facePose);
    void faceInitCb(tello_pub::SE3Pose const& facePose);
    /*
    @facePoseArrCb
    */
    // void facePoseArrCb(geometry_msgs::PoseArray const& poseArr);
    /*
    @arucoPoseArrCb
    */
    // void d0ArucoCb(tello_sub::ArUcoPoseArr const& poseArr);
    /*
    @drone0YOLOCb

    */
    // void drone0YOLOCb(geometry_msgs::PoseArray const& poseArr);
    /*
    @drone2YOLOCb

    */
    // void drone2YOLOCb(geometry_msgs::PoseArray const& poseArr);
    /*
    @publishCmd
        publish drone command and debug msg
    */
    void publishCmd(
        Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
    );
    /*
    @publishAngCmd
        publish drone command and debug msg
    */
    void publishAngCmd(
        Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
    );
    /*
    @publishDesPose
        publish desired pose for each drone in I0
    */
    void publishDesPose();
    /*========================[setter and getter]========================*/
    /*
    @setDesireRelPose
        set the desired relative pose from camera positioning parameters
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
    void setDesireRelPose(
        double& distXYMax, double &desDistXY, double &desRoll, double &desTheta, double &desFaceTheta
    );

public:
    /*
    @GeometricFormCtrl
        empty constructor and destructor
    */
    GeometricFormCtrl(){}
    ~GeometricFormCtrl(){}
    /*
    @GeometricFormCtrl
        TODO
    */
    explicit GeometricFormCtrl(ros::NodeHandle& nh_);


};
/*
@GeometricFormCtrl
    mission assignment is given as 
        drone 0 oversee drone 1, drone 2, and human actor, tracking 
*/
class GeometricFormCtrlTwoDrone
{
    /*===============[ros-dep]=============================*/
    // sub and pub
    // multi-drone VIO
    ros::Subscriber eskf_0_sub, eskf_1_sub, eskf_init_1_sub;
    // face odom
    ros::Subscriber eskf_face_sub, eskf_init_face_sub;
    // YOLO pose
    // ros::Subscriber YOLO_pose_0_sub, YOLO_pose_2_sub;
    // process control
    ros::Subscriber exp_mode_sub;

    // desired leader pose generation
    // ros::Subscriber face_arr_sub, aruco_sub;

    // ros::Publisher EOT_pub;
    // drone
    ros::Publisher cmd_0_pub, cmd_1_pub;

    // debug
    // publish the gradients and cost functions
    ros::Publisher ctrller_debug_pub;

    // publish the desired pose in I0 for each drone
    ros::Publisher des_pose_0_pub, des_pose_1_pub;

    // timer
    ros::Timer formationCtrlTimer;
    ros::Time startTimeCtrller;

    /*===========[param]========================================*/
    // ros-dep
    int queue_size = 1;
    double t_smpl = 0.05;

    bool StartOfTest = false;
    bool disableCmd = false;
    bool angVelMode = false;
    bool noRollPitch = false;
    bool useAltPose = true;             // define rel pose in L/F --> body
    bool pubDesPose = false;            // publish drones' desired pose
    bool smoothHumanVel = false;        // use a window to smooth the human velocity
    int desFormConfig = Config::cfg1;    

    std::string droneName[2] = {"/tello_0", "/tello_1"};
    std::string ros_Nodename;


    double rollCamInit      = 0.206;  // rad, abs value of the avg of two experimental roll angle
    double height_offset    = 0.5;
    // double rollCamInit = -0.206;    // rad
    // double loaTheta = 0.0;          // rotation from IF = IL to L
    // double thetaLoAToFace = 0.0;    // rotation from L to F

    // Eigen
    // constant transformation
    Eigen::Matrix3d rotCamTiltToBody;           // tilt camera to body frame
    Eigen::Matrix3d rotCamIdealToBody;          // ideal camera to body
    Eigen::Matrix3d rotBodyToCmd;           // body to cmd frame
    Eigen::Matrix3d rotCamTilt1ToM1;
    // drone states
    Eigen::Vector3d dronesPos[2], dronesVel[2], dronesAngVel[2];
    Eigen::Quaterniond dronesQuat[2]; 
    
    // face and LoA
    /*
    face frame and LoA frame
        SAME initial frame
        SAME position in initial frame
        DIFFERENT velocity
        DIFFERENT rotation
    */
    Eigen::Vector3d facePos, faceVel, faceAngVel;   // face pos vel is loa pos vel
    Eigen::Quaterniond faceQuat;                    // F --> IF=IL
    Eigen::Matrix3d rotInitFaceToLoA;               // IF=IL --> L
    Eigen::Matrix3d rotFaceToLoA;                   // F --> L
    std::vector<Eigen::Vector3d> faceVelWndw;       
    int wndw_size = 10;

    // sensor
    Eigen::Matrix3d rotFaceToCamTilt, rotM1ToCamTilt;
    Eigen::Vector3d posCamTiltToFace, posCamTiltToM1;


    // initial conditinos
    Eigen::Vector3d posIner0ToIneri[1];     // I0 --> Ii in I0 frame
    Eigen::Vector3d posIner1ToInitFace;     // I1 --> IF = IL in I1 frame
    Eigen::Matrix3d rotIneriToIner0[1];     // Ii --> I0
    Eigen::Matrix3d rotInitFaceToIner1;     // IF = IL --> I1

    // Relative pose SE(3)
    // order follows the drone ids
    Eigen::Matrix4d droneRelPose[2];   // L --> B0, F --> B1, L --> B2
    Eigen::VectorXd droneRelTwist[2];  // L --> B0, F --> B1, L --> B2
    // desired
    Eigen::Matrix4d desRelPose[2];
    Eigen::VectorXd desRelTwist[2]; 
    // gain matrix
    Eigen::MatrixXd twistGain[3];       // 6 by 6 gain matrix, ND or Hurwitz



    /*=============[functions]==================================*/
    /*
    @telloSOTCb
    */
    void exp_mode_callback(std_msgs::Empty const& msg);
    /*
    @ctrlTimerCb

    */
    void ctrlTimerCb(ros::TimerEvent const& ev);
    /*
    @drone0ESKFCb

    */
    void drone0ESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt);
    /*
    @drone1ESKFCb

    */
    void drone1ESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& trajPt);
    /*
    @drone1InitCb
    */
    void drone1InitCb(tello_pub::SE3Pose const& pose);
    /*
    @faceESKFCb

    */
    void faceESKFCb(trajectory_msgs::MultiDOFJointTrajectoryPoint const& faceTrajPt);
    /*
    @faceInitCb

    */
    // void faceInitCb(geometry_msgs::Pose const& facePose);
    void faceInitCb(tello_pub::SE3Pose const& facePose);
    /*
    @publishCmd
        publish drone command and debug msg
    */
    void publishCmd(
        Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
    );
    /*
    @publishAngCmd
        publish drone command and debug msg
    */
    void publishAngCmd(
        Eigen::Matrix4d* grad, Eigen::VectorXd* twist_cmd, double* err_func
    );
    /*
    @publishDesPose
        publish desired pose for each drone in I0
    */
    void publishDesPose();
    /*========================[setter and getter]========================*/
    /*
    @setDesireRelPose
        set the desired relative pose from camera positioning parameters
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
    void setDesireRelPose(
        double& distXYMax, double &desDistXY, double &desRoll, double &desTheta, double &desFaceTheta
    );

public:
    /*
    @GeometricFormCtrl
        empty constructor and destructor
    */
    GeometricFormCtrlTwoDrone(){}
    ~GeometricFormCtrlTwoDrone(){}
    /*
    @GeometricFormCtrl
        TODO
    */
    explicit GeometricFormCtrlTwoDrone(ros::NodeHandle& nh_);


};









#endif