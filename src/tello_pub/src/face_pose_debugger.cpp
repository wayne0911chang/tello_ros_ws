// ros-dep and eigen-dep
# include "tello_pub/ros_eigen_dep.h"
// parsing the matrix
# include "yaml-cpp/yaml.h"

class FacePosePub{
    // ros-dep
    ros::Subscriber SOT_sub;
    ros::Publisher face_pose_pub;
    ros::Timer faceTimer;
    ros::Time startTime;

    // param
    int queue_size = 1;
    std::string ros_Nodename;
    std::string debug_mode = "static";
    double t_face = 0.1;

    // flags
    bool StartOfTest = false;

    // Eigen
    Eigen::Vector3d positionCamTiltToFace;
    Eigen::Matrix3d rotFaceToCamTilt;

    // callbacks
    void telloSOTCb(std_msgs::Empty const& msg){
        if (!StartOfTest){
            StartOfTest = true;
            startTime = ros::Time::now();
            faceTimer.start();
            // debug
            std::cout << "========[face pose dbg]========\nreceived SOT!" << std::endl;
        }
    }
    // timer callback
    void faceTimerCb(ros::TimerEvent const& ev){
        if (StartOfTest){
            // publish the face pose w.r.t. given mode
            if (debug_mode == "static"){
                geometry_msgs::Pose pose_msg;
                pose_msg.position.x = positionCamTiltToFace(0);
                pose_msg.position.y = positionCamTiltToFace(1);
                pose_msg.position.z = positionCamTiltToFace(2);
                tf::quaternionEigenToMsg(
                    Eigen::Quaterniond(rotFaceToCamTilt),
                    pose_msg.orientation
                );
                geometry_msgs::PoseArray pose_arr_msg;
                pose_arr_msg.header.stamp = ros::Time::now();
                pose_arr_msg.poses.emplace_back(pose_msg);
                face_pose_pub.publish(pose_arr_msg);
            }
        }
    }

public:
    explicit FacePosePub(ros::NodeHandle& nh_){
        // get param
        ros_Nodename = ros::this_node::getName();
        nh_.getParam(ros_Nodename + "/debug_mode", debug_mode);
        // debug
        std::cout << "debug mode: " << debug_mode << std::endl;
        // get pose from yaml file 
        try {
            std::string configPath;
            nh_.getParam(ros_Nodename + "/face_cfg", configPath);
            YAML::Node config = YAML::LoadFile(configPath);
            YAML::Node facePose = config["face"];
            facePose = facePose[debug_mode];
            // set the pose
            positionCamTiltToFace = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                facePose["position"].as<std::vector<double>>().data()
            );
            rotFaceToCamTilt = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                facePose["orientation"].as<std::vector<double>>().data()
            );
        }
        catch (...){
            // default values retrieved from face odom exp
            positionCamTiltToFace << -0.11, -0.36, 1.85;
            rotFaceToCamTilt = Eigen::Matrix3d::Identity();
            // debug
            std::cout << "========[face pose debug]========\nUse default face pose!" << std::endl;
        }
        // ros-dep
        SOT_sub = nh_.subscribe("/SOT", queue_size, &FacePosePub::telloSOTCb, this);
        face_pose_pub = nh_.advertise<geometry_msgs::PoseArray>(
            ros::this_node::getNamespace() + "/face_pose_arr", queue_size
        );
        faceTimer = nh_.createTimer(
            ros::Duration(t_face), &FacePosePub::faceTimerCb,
            this, false, false
        );
        // debug
        std::cout << "C1 --> F in C1 translation:\n" << positionCamTiltToFace << std::endl
                  << "F --> C1 rotation:\n" << rotFaceToCamTilt << std::endl;
        
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "face_pose_debugger", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    FacePosePub facePosePub(nh);
    ros::spin();
    
    return 0;
}