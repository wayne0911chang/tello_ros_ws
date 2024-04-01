// self-made header
# include "tello_sub/sensor_utils.h"
/*
@ImageConverterRS

*/
ImageConverterRS::ImageConverterRS(ros::NodeHandle& nh_) : it_(nh_)
{
    // param
    nh_.getParam(ros::this_node::getName() + "/default_cam_info", default_cam_info);
    
    // ros-dep
    ros_ns = ros::this_node::getNamespace();
    image_sub_ = it_.subscribe("color/image_raw", buffer_size,
        &ImageConverterRS::imageCb, this);
    image_pub_ = it_.advertise(ros_ns + "/image_raw/aruco", buffer_size);
    
    // assign the camera matrix
    if (default_cam_info){
        // debug
        std::cout << "use the default camera matrix" << std::endl;
        cameraMatrix = (cv::Mat1f(3, 3) << 613.59753, 0.0, 327.74966,
                                           0.0, 613.98608, 240.27623,
                                           0.0, 0.0, 1.0);
        distCoeffs = (cv::Mat1f(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    else {
        cam_info_sub = nh_.subscribe("color/camera_info", buffer_size,
            &ImageConverterRS::camInfoCb, this);
    }

    // aruco
    arucoPoseArr_pub = nh_.advertise<geometry_msgs::PoseArray>(
        ros_ns + "/aruco_pose_arr", buffer_size
    );
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // opencv depends
    cv::namedWindow(ros_ns + OPENCV_WINDOW);
}
/*
@camInfoCb
    get camera info
*/
void ImageConverterRS::camInfoCb(sensor_msgs::CameraInfo const& camInfo){
    if (!default_cam_info){
        cameraMatrix = (cv::Mat1f(3, 3) << camInfo.K[0], camInfo.K[1], camInfo.K[2],
                                           camInfo.K[3], camInfo.K[4], camInfo.K[5],
                                           camInfo.K[6], camInfo.K[7], camInfo.K[8]);
        int distLen = int(sizeof(camInfo.D) / sizeof(camInfo.D[0]));
        // distCoeffs = cv::Mat(camInfo.D);
        distCoeffs = cv::Mat(1, camInfo.D.size(), CV_64F);
        std::memcpy(distCoeffs.data, camInfo.D.data(), camInfo.D.size() * sizeof(double));
        // debug
        // std::cout << "========[camera info]========\n"
        //           << "sizeof camInfo.D: " << sizeof(camInfo.D) << std::endl
        //           << "sizeof 1st element of camInfo.D: " << sizeof(camInfo.D[0]) << std::endl
        //           << "len of camInfo.D: " << distLen << std::endl
        //           << "len of camInfo.D in vector size: " << camInfo.D.size() << std::endl;
        // std::cout << "========[realsense]========\ndistortion model: " 
        //           << camInfo.distortion_model << std::endl;
        // std::cout << "distortion coeff from message: ";
        // for (int i=0; i < camInfo.D.size(); i++){
        //     std::cout << camInfo.D[i] << ", ";
        // }
        // std::cout << "\ncamera matrix:\n" << cameraMatrix << std::endl
        //           << "distortion coefficent:\n" << distCoeffs << std::endl;
    }
}
/*
@ArUcoPoseArrPnP
    input:
        cv::Mat &im
        std::vector< std::vector<cv::Vec3d> > &marker_rvecs;
        std::vector< std::vector<cv::Vec3d> > &marker_tvecs;
    compute pnp from the image 
*/
void ImageConverterRS::ArUcoPoseArrPnP(
    cv::Mat &im,
    std::vector<cv::Vec3d> &marker_rvecs,
    std::vector<cv::Vec3d> &marker_tvecs,
    std::vector<int> &marker_ids,
    std::vector< std::vector<cv::Point2f> > &marker_corners
){
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners; 

    // assign corner info and id info with detected markers
    cv::aruco::detectMarkers(im, dictionary, corners, ids);

    // display the corner info.
    std::cout << "======== corners size: " << corners.size() << " ========" << std::endl;

    // if marker detected
    if (ids.size() > 0){
        // debug
        std::cout << "========[aruco pose]========\nPnP called" << std::endl;

        // separate the wall ids and the drone ids
        for (int i=0; i<ids.size(); i++){
            // the drone
            if (findInVec(ids[i], drone_marker_ids)) {
                marker_ids.emplace_back(ids[i]);
                marker_corners.emplace_back(corners[i]);
                // debug
                std::cout << "======== drone marker id: " << ids[i]
                          << " ========\ncorners:" << std::endl;
                for (int j=0; j<corners[i].size(); j++){
                    std::cout << corners[i][j] << std::endl;
                }
            }
            else {
                ROS_INFO_STREAM("========[multi-aruco]========\nUnknown IDs!!");
            }
        }
        if (marker_ids.size() > 0){
            std::vector<cv::Vec3d> rvecs(marker_ids.size()), 
                                   tvecs(marker_ids.size());
            // debug
            std::cout << "======== draw markers ========\n"
                      << "======== ids = ";
            for (int d=0; d<marker_ids.size(); d++){
                std::cout << marker_ids[d] << ", ";
            }
            std::cout << "========" << std::endl;
            // daw the markers
            cv::aruco::drawDetectedMarkers(
                im, marker_corners, marker_ids
            );
            // estimate the marker pose
            // check if the camera info is called if the default params are not used
            if (!default_cam_info){
                ROS_INFO_STREAM("========[multi-aruco]========\nCamera info is not updated! Use default values!");
                cameraMatrix = (cv::Mat1f(3, 3) << 613.59753, 0.0, 327.74966,
                                                   0.0, 613.98608, 240.27623,
                                                   0.0, 0.0, 1.0);
                distCoeffs = (cv::Mat1f(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
            }
            // determine which size to use
            cv::Mat objPoints(4, 1, CV_32FC3);
            // drone marker corners 
            objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(
                -DRONE_MARKER_SIZE/2.f, DRONE_MARKER_SIZE/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(
                DRONE_MARKER_SIZE/2.f, DRONE_MARKER_SIZE/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(
                DRONE_MARKER_SIZE/2.f, -DRONE_MARKER_SIZE/2.f, 0);
            objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(
                -DRONE_MARKER_SIZE/2.f, -DRONE_MARKER_SIZE/2.f, 0);
            // debug
            // std::cout << "======== start solve pnp ========\ncamera matrix:" << std::endl
            //           << cameraMatrix << std::endl
            //           << "distortion coeff:\n" << distCoeffs << std::endl
            //           << "marker corners:" << std::endl;
            // find the rvec and tvec for each marker
            std::cout << "======== start solve pnp ========" << std::endl;
            for (int j=0; j<marker_corners.size(); j++){
                // debug
                for (int k=0; k<marker_corners[j].size(); k++){
                    std::cout << marker_corners[j][k] << std::endl;
                }
                cv::solvePnP(
                    objPoints, marker_corners[j],
                    cameraMatrix, distCoeffs,
                    rvecs.at(j), tvecs.at(j)
                );
            }
            // debug
            std::cout << "======== draw axis ========" << std::endl;
            // draw the axis
            for (int j=0; j<marker_ids.size(); j++){
                cv::aruco::drawAxis(
                    im, cameraMatrix, distCoeffs,
                    rvecs[j], tvecs[j], 0.05
                );
                // debug
                // cv::Mat RMat;
                // cv::Rodrigues(rvecs[j], RMat);
                // std::cout << "rvec: " << rvecs[j] << std::endl
                //           << "rmat: " << RMat << std::endl
                //           << "tvec: " << tvecs[j] << std::endl
                //           << "length of rvec and tvec: " << rvecs.size() 
                //           << std::endl; 
            }
            marker_rvecs = rvecs;
            marker_tvecs = tvecs;
        }
    }
}
/*
@publishArUcoMsg
    publish ArUcoPoseArr and PointArr
*/
void ImageConverterRS::publishArUcoMsg(
    std::vector<cv::Vec3d> &marker_rvecs,
    std::vector<cv::Vec3d> &marker_tvecs,
    std::vector<int> &marker_ids
){
    /*======================[aruco pose]==================================*/
    if (arucoPoseArr_pub.getNumSubscribers() > 0) 
    {
        // check if there are marker data
        if (marker_ids.size() > 0){
            
            tello_sub::ArUcoPoseArr ArUcoPoseArr_msg;
            // wall or drone
            for (int i=0; i < marker_ids.size(); i++){
                geometry_msgs::Pose pose_msg;
                pose_msg.position.x = marker_tvecs[i][0];
                pose_msg.position.y = marker_tvecs[i][1];
                pose_msg.position.z = marker_tvecs[i][2];
                // use tf methods
                const double theta = cv::norm(marker_rvecs[i]);
                tf::Vector3 rot_axis(
                    marker_rvecs[i][0],
                    marker_rvecs[i][1],
                    marker_rvecs[i][2]
                );
                rot_axis /= theta;
                tf::Quaternion q_msg;
                q_msg.setRotation(rot_axis, theta);
                pose_msg.orientation.x = q_msg.x();
                pose_msg.orientation.y = q_msg.y();
                pose_msg.orientation.z = q_msg.z();
                pose_msg.orientation.w = q_msg.w();
                ArUcoPoseArr_msg.poses.emplace_back(pose_msg);
                ArUcoPoseArr_msg.ids.emplace_back(marker_ids[i]);
                // determine the size
                if (findInVec(marker_ids[i], drone_marker_ids)){
                    ArUcoPoseArr_msg.sizes.emplace_back(
                        DRONE_MARKER_SIZE
                    );
                }
                // show info
                ROS_INFO_STREAM("position:\nx: " << pose_msg.position.x 
                    << "\ny: " << pose_msg.position.y 
                    << "\nz: " << pose_msg.position.z);
                ROS_INFO_STREAM("orientation:\nw: " << pose_msg.orientation.w
                    << "\nx: " << pose_msg.orientation.x
                    << "\ny: " << pose_msg.orientation.y
                    << "\nz: " << pose_msg.orientation.z);
            }
            // header 
            ArUcoPoseArr_msg.header.frame_id = "ArUco_markers";
            ArUcoPoseArr_msg.header.stamp = ros::Time::now();
            
            // publish
            arucoPoseArr_pub.publish(ArUcoPoseArr_msg);
        }
    }
}
/*
@imageCb
    handle the images
*/
void ImageConverterRS::imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // get the image size
    imgHeight = cv_ptr->image.size().height;
    imgWidth = cv_ptr->image.size().width;

    /*=============== aruco detection ====================*/
    // for aruco
    std::vector<cv::Vec3d > marker_tvecs, marker_rvecs;

    // markers for the drone
    std::vector<int> marker_ids;

    // corners for the drone
    std::vector< std::vector<cv::Point2f> > marker_corners; 

    // get the pose and ids and corners
    ArUcoPoseArrPnP(
        cv_ptr->image, 
        marker_rvecs, marker_tvecs, marker_ids, marker_corners
    );
    // publish the data if there are
    publishArUcoMsg(
        marker_rvecs, marker_tvecs, marker_ids
    );
    // ================= show the image
    // Update GUI Window
    cv::imshow(ros_ns + OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // publish the modified image
    image_pub_.publish(cv_ptr->toImageMsg());
}

//main function
int main(int argc, char **argv)
{
    //ros::init()
    ros::init(argc, argv, "aruco_pose_realsense", ros::init_options::AnonymousName);

    ros::NodeHandle nh_;
    ImageConverterRS ic_rs(nh_);

    ros::spin();

    return 0;
}