// ros and Eigen
# include "tello_pub/ros_eigen_dep.h"
// math tools
# include "tello_pub/math_utils.h"
/*
@Repub

*/
class Repub
{
    ros::Subscriber arucoPoseArr_sub, SOT_sub;
    ros::Publisher odom_pub;
    int queue_size = 1;
    ros::Time startTime;

    // parameters
    bool StartOfTest = false;
    bool useCamInit = false;

    Eigen::Vector3d posInerToCam;                   // in iner
    Eigen::Matrix3d rotRSCamToIner, rotBodyToMarker, 
                    rotMarkerToIner;
    Eigen::Quaterniond quatArUcoPrev;               // for compare the error

    // callback
    /*
    @SOTCb
    */
    void SOTCb(std_msgs::Empty const& msg){
        if (!StartOfTest){
            startTime = ros::Time::now();
            StartOfTest = true;
            std::cout << "========[repub realsense]========\nreceived SOT!" << std::endl;
        }
    }
    /*
    @arucoPoseArrCb
        check the id
        set the orientation and position
    */
    void arucoPoseArrCb(geometry_msgs::PoseArray const& pose_arr){
        Eigen::Vector3d posCamToMarker;         // in camera frame
        Eigen::Quaterniond quatMarkerToCam;
        for (int i=0; i<pose_arr.poses.size(); i++){
            posCamToMarker << pose_arr.poses[i].position.x,
                              pose_arr.poses[i].position.y,
                              pose_arr.poses[i].position.z;
            tf::quaternionMsgToEigen(
                pose_arr.poses[i].orientation, quatMarkerToCam
            );
        }
        if (!StartOfTest){
            quatArUcoPrev = quatMarkerToCam.conjugate();
            // debug
            std::cout << "========[repub realsense]========\nset initial quat conj\n"
                      << quatArUcoPrev.w() << quatArUcoPrev.vec() << std::endl;
            // update camera initial pose
            if (useCamInit){
                rotRSCamToIner = rotMarkerToIner * 
                    quatMarkerToCam.conjugate().toRotationMatrix();
                posInerToCam = -rotRSCamToIner * posCamToMarker;
                // debug
                std::cout << "========[repub realsense]========\ninitialize the camera with measure\n"
                          << "rot C --> I0:\n" << rotRSCamToIner << std::endl
                          << "pos I0 --> C in I0:\n" << posInerToCam << std::endl;
            }
        }
        else {
            Eigen::Vector3d rvecDiff = quaterniondToRvec(
                quatArUcoPrev * quatMarkerToCam
            );
            // if (std::abs(rvecDiff.norm() - M_PI) < 5*M_PI/180){
            //     quatMarkerToCam.w() *= -1;
            //     quatMarkerToCam.vec() *= -1;
            //     // debug
            //     std::cout << "========[repub realsense]========\ninvert the quaternion!" << std::endl
            //               << quatMarkerToCam.w() << std::endl
            //               << quatMarkerToCam.vec() << std::endl;
            // }
            if (rvecDiff.norm() > 5*M_PI/180){
                Eigen::Quaterniond tmp = quatMarkerToCam;
                tmp.w() *= -1;
                tmp.vec() *= -1;
                // set previous quaternion
                // if the inverted quaternion give smaller norm
                // then use the inverted quaternion
                if (quaterniondToRvec(quatArUcoPrev * tmp).norm() < rvecDiff.norm()){
                    // debug
                    std::cout << "========[repub realsense]========\nangle = " << rvecDiff.norm() << std::endl
                              << "invert angle = " << quaterniondToRvec(quatArUcoPrev * tmp).norm() << std::endl;
                    // quatArUcoPrev = quatMarkerToCam.conjugate();
                    quatMarkerToCam = tmp;
                    std::cout << "invert the quaternion!" << std::endl
                              << quatMarkerToCam.w() << std::endl
                              << quatMarkerToCam.vec() << std::endl;    
                }
                else {
                    // quatArUcoPrev = quatMarkerToCam.conjugate();
                }
            }
            // set previous
            quatArUcoPrev = quatMarkerToCam.conjugate();

            nav_msgs::Odometry odom;
            odom.header.frame_id = "eskf_odom";
            odom.header.stamp = ros::Time::now();
            Eigen::Quaterniond quatBodyToIner = Eigen::Quaterniond(rotRSCamToIner) * 
                quatMarkerToCam * Eigen::Quaterniond(rotBodyToMarker);
            tf::quaternionEigenToMsg(
                quatBodyToIner,
                odom.pose.pose.orientation
            );
            Eigen::Vector3d posIner = posInerToCam + rotRSCamToIner * posCamToMarker;
            odom.pose.pose.position.x = posIner(0);
            odom.pose.pose.position.y = posIner(1);
            odom.pose.pose.position.z = posIner(2);
            
            // debug
            std::cout << "========[repub realsense]========\nI0 position:\n"
                      << posIner << std::endl
                      << "B0 --> I0 rot:\n" 
                      << quatBodyToIner.toRotationMatrix() << std::endl;
            odom_pub.publish(odom);
        }
    }

public:
    Repub(ros::NodeHandle& nh_){

        std::string nodeName = ros::this_node::getName();
        // get position in I0 frame
        nh_.getParam(nodeName + "/height", posInerToCam(2));
        nh_.getParam(nodeName + "/front", posInerToCam(1));
        nh_.getParam(nodeName + "/right", posInerToCam(0));
        nh_.getParam(nodeName + "/use_measured_init", useCamInit);
        
        odom_pub = nh_.advertise<nav_msgs::Odometry>(
            ros::this_node::getNamespace() + "/realsense_odom", queue_size
        );
        // under the camera ns
        arucoPoseArr_sub = nh_.subscribe("aruco_pose_arr", queue_size,
            &Repub::arucoPoseArrCb, this);
        SOT_sub = nh_.subscribe("/SOT", queue_size,
            &Repub::SOTCb, this);
        
        rotRSCamToIner << 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0,
                        0.0, -1.0, 0.0;
        // rotBodyToMarker << 0.0, 0.0, -1.0,
        //                    1.0, 0.0, 0.0,
        //                    0.0, -1.0, 0.0;
        rotBodyToMarker << 0.0, 1.0, 0.0,
                           0.0, 0.0, -1.0,
                           -1.0, 0.0, 0.0;
        rotMarkerToIner << 1.0, 0.0, 0.0,
                           0.0, 0.0, -1.0,
                           0.0, 1.0, 0.0;
        // debug
        std::cout << "position I0 --> C_rs in I0:\n" << posInerToCam << std::endl
                  << "rot C_rs --> I0:\n" << rotRSCamToIner << std::endl
                  << "rot B0 --> M0:\n" << rotBodyToMarker << std::endl
                  << "rot M0 --> I0:\n" << rotMarkerToIner << std::endl;
    }
};
/*
@main
*/
int main(int argc, char** argv){
    ros::init(argc, argv, "repub_realsense", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    Repub repub_realsense(nh);
    ros::spin();

    return 0;
}