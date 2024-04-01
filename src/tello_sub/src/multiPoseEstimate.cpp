// self-made header
# include "tello_sub/sensor_utils.h"
/*
@ImageConverter
    explicit constructor
*/
ImageConverter::ImageConverter(ros::NodeHandle& nh_) : it_(nh_)
{
    // face detection threshold
    face_thr = 55.0;

    // face model 
    useNTHU = false;
    /*=====================[ros param]============================*/
    // get the namespace
    ros_ns = ros::this_node::getNamespace();
    ros_nodeName = ros::this_node::getName();
    // debug
    std::cout << "ns: " << ros_ns << std::endl
              << "node name: " << ros_nodeName << std::endl;

    // get the flags
    nh_.getParam(ros_nodeName + "/aruco_estimate", missionFlag[0]);
    nh_.getParam(ros_nodeName + "/face_estimate", missionFlag[1]);
    nh_.getParam(ros_nodeName + "/debug_face", debugFaceMarker);
    nh_.getParam(ros_nodeName + "/yolo_estimate", missionFlag[2]);
    nh_.getParam(ros_nodeName + "/face_thresh", face_thr);
    nh_.getParam(ros_nodeName + "/face_bbox_min", face_bbox_min);
    nh_.getParam(ros_nodeName + "/face_bbox_max", face_bbox_max);
    nh_.getParam(ros_nodeName + "/useNTHU", useNTHU);
    
    // nh_.getParam(ros_ns + "/aruco_estimate", missionFlag[0]);
    // nh_.getParam(ros_ns + "/face_estimate", missionFlag[1]);
    // nh_.getParam(ros_ns + "/yolo_estimate", missionFlag[2]);

    
    // debug
    std::cout << missionFlag[0] << std::endl
              << missionFlag[1] << std::endl
              << missionFlag[2] << std::endl;


    /*==========================[subs and pubs]===========================*/
    int buffer_size = 1;   

    // Subscibe to input video feed and publish output video feed
    // subscribe node would automatically add the namespace
    image_sub_ = it_.subscribe("image_raw/h264_trans", buffer_size,
        &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(ros_ns + "/image_raw/aruco", buffer_size);

    // camera matrix by hand
    float fx = 901.57301941;
    float cx = 477.52938592;
    float fy = 907.36572961;
    float cy = 355.00994502;
    cameraMatrix = (cv::Mat1f(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);

    float k1 = 0.02622212;
    float k2 = -0.55220608;
    float p1 = -0.0034407;
    float p2 = 0.00321558;
    float k3 = 1.89103285;
    distCoeffs = (cv::Mat1f(1, 5) << k1, k2, p1, p2, k3);

    // depend on cases
    if (missionFlag[0]){
        // aruco
        arucoPoseArr_pub = nh_.advertise<tello_sub::ArUcoPoseArr>(
            ros_ns + "/aruco_pose_arr", buffer_size);
        pointarr_pub = nh_.advertise<tello_sub::PointArr>(ros_ns + "/aruco_corners", buffer_size);
    
        // aruco marker dictionary
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    }
    if (missionFlag[1]){
        // for face pose estimation
        facePoseArr_pub = nh_.advertise<geometry_msgs::PoseArray>(
            ros_ns + "/face_pose_arr", buffer_size
        );
        faceBBox_pub = nh_.advertise<tello_sub::PointArr>(
            ros_ns + "/face_corners", buffer_size
        );
        // setting the face detection part
        if (!debugFaceMarker){
            faceDetector.load(cascadePath);
            facemark = cv::face::FacemarkLBF::create();
            facemark -> loadModel(facemarkPath);
        }
        else {
            // for the debug purpose
            pointarr_pub = nh_.advertise<tello_sub::PointArr>(ros_ns + "/aruco_corners", buffer_size);
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        }
        // faceDetector.load(cascadePath);
        // facemark = cv::face::FacemarkLBF::create();
        // facemark -> loadModel(facemarkPath);
    }
    if (missionFlag[2]){
        // yolo
        YOLOBBox_sub = nh_.subscribe(
            "/darknet_ros/bounding_boxes", buffer_size,
            &ImageConverter::BBoxesCb, this
        );
        YOLOPose_pub = nh_.advertise<geometry_msgs::PoseArray>(
            ros_ns + "/human_pose_arr", buffer_size
        );
    }
    /*=============================[openCV depend]=======================*/
    cv::namedWindow(ros_ns + OPENCV_WINDOW);
    // set up the record csv file
    // myFile.open("face_bbox.csv");
}
/*
@getFacingWithAxis
    compute the axis from the face landmarks
    draw the axis
*/
void ImageConverter::getFacingWithAxis(
    cv::Mat &frame, 
    std::vector<cv::Point2f> &landmarks, 
    cv::Mat &faceRvec, 
    cv::Mat &faceTvec, 
    bool useNTHU)
{
    
    // setting the 2d-3d coord of face landmark
    std::vector<cv::Point2f> faceImgPts;
    std::vector<cv::Point3f> faceCoordPts;
    // draw axis in different color
    std::vector<cv::Point3f> heading;
    std::vector<cv::Point2f> heading2d;
    // set the full coord
    heading.push_back(cv::Point3f(100.0, 0.0, 0.0));
    heading.push_back(cv::Point3f(0.0, 100.0, 0.0));
    heading.push_back(cv::Point3f(0.0, 0.0, 100.0));

    // assing point pairs
    // image plane
    faceImgPts.push_back(landmarks[33]);    // center of nose tip -> pronasale
    faceImgPts.push_back(landmarks[27]);    // upmost nose bridge -> sellion
    faceImgPts.push_back(landmarks[8]);     // chin -> menton
    faceImgPts.push_back(landmarks[39]);    // left eye inner corner -> left inner ectocanthus
    faceImgPts.push_back(landmarks[42]);    // right eye inner corner -> right inner ectocanthus
    faceImgPts.push_back(landmarks[36]);    // left eye outer corner -> left outer ectocanthus
    faceImgPts.push_back(landmarks[45]);    // right eye outer corner -> right outer ectocanthus
    faceImgPts.push_back(landmarks[48]);    // left mouth corner
    faceImgPts.push_back(landmarks[54]);    // right mouth corner

    if (useNTHU) {
        // Yang and Yu, 2000
        // change the coord.
        faceCoordPts.push_back(cv::Point3f(0.0, 0.0, 0.0));        // pronasale
        faceCoordPts.push_back(cv::Point3f(0.0, 36.9, -22.2));     // sellion
        faceCoordPts.push_back(cv::Point3f(0.0, -78.1, -22.2));    // menton
        faceCoordPts.push_back(cv::Point3f(-16.45, 33.5, -40.4));  // left inner ectocanthus
        faceCoordPts.push_back(cv::Point3f(16.45, 33.5, -40.4)); // right inner ectocanthus
        faceCoordPts.push_back(cv::Point3f(-54.05, 33.5, -40.4));  // left outer ectocanthus
        faceCoordPts.push_back(cv::Point3f(54.05, 33.5, -40.4)); // right outer ectocanthus
        faceCoordPts.push_back(cv::Point3f(-24.35, -37.9, -22.2));  // left mouth corner
        faceCoordPts.push_back(cv::Point3f(24.35, -37.9, -22.2)); // right mouth corner
        // find the transformation
        cv::Mat rvec, tvec;
        cv::solvePnP(faceCoordPts, faceImgPts, cameraMatrix, distCoeffs,
            rvec, tvec, cv::SOLVEPNP_ITERATIVE);

        // debug
        std::cout << "rotation: " << std::endl << rvec << std::endl
            << "translation: " << std::endl << tvec << std::endl
            << "t,z: " << tvec.at<double>(2) << std::endl;

        
        // add additional transformation
        if (tvec.at<double>(2) < 0){
            tvec = -tvec;
            cv::Mat faceRMat;
            cv::Rodrigues(rvec, faceRMat);
            faceRMat.at<double>(0, 0) *= -1;
            faceRMat.at<double>(1, 1) *= -1;
            cv::Rodrigues(faceRMat, rvec);
            // debug
            std::cout << "rotation changed: " << std::endl << rvec << std::endl
                      << "translation changed: " << std::endl << tvec << std::endl;
        }
        // assign the result
        faceRvec = rvec; faceTvec = tvec;

        cv::projectPoints(heading, rvec, tvec, cameraMatrix, distCoeffs,
            heading2d);
        // the heading from Yang and Yu 2000
        cv::line(frame, faceImgPts[0], heading2d[0], cv::Scalar(0, 0, 255), 2);
        cv::line(frame, faceImgPts[0], heading2d[1], cv::Scalar(0, 255, 0), 2);
        cv::line(frame, faceImgPts[0], heading2d[2], cv::Scalar(255, 0, 0), 2);
    }
    else {
        // 3d coord
        // self-made version
        // change the coordinate
        faceCoordPts.push_back(cv::Point3f(0.0, 0.0, 0.0));         // pronasale
        faceCoordPts.push_back(cv::Point3f(0.0, 60.0, -33.0));      // sellion (by UAFH)
        faceCoordPts.push_back(cv::Point3f(0.0, -74.97, -42.0));    // menton
        faceCoordPts.push_back(cv::Point3f(-16.5, 60.0, -48.0));     // left eye inner corner (by aesthetic data of taiawnese)
        faceCoordPts.push_back(cv::Point3f(16.5, 60.0, -48.0));    // right eye inner corner (by aesthetic data of taiawnese)
        faceCoordPts.push_back(cv::Point3f(-48.5, 60.0, -48.0));     // left eye outer corner (by aesthetic data of taiawnese)
        faceCoordPts.push_back(cv::Point3f(48.5, 60.0, -48.0));    // right eye outer corner (by aesthetic data of taiawnese)
        faceCoordPts.push_back(cv::Point3f(-30.0, -23.92, -33.0));   // left mouth corner (by aesthetic data of taiawnese, self-measure, and Chen et al. 2014)
        faceCoordPts.push_back(cv::Point3f(30.0, -23.92, -33.0));  // right mouth corner (by aesthetic data of taiawnese, self-measure, and Chen et al. 2014)
        
        // debug
        std::cout << cameraMatrix << std::endl
                  << distCoeffs << std::endl;

        // find the transformation
        cv::Mat rvec, tvec;
        cv::solvePnP(faceCoordPts, faceImgPts, cameraMatrix, distCoeffs,
            rvec, tvec, cv::SOLVEPNP_ITERATIVE);

        // debug
        std::cout << "rotation: " << std::endl << rvec << std::endl
            << "translation: " << std::endl << tvec << std::endl
            << "t,z: " << tvec.at<double>(2) << std::endl;
            
        // add additional transformation
        if (tvec.at<double>(2) < 0){
            tvec = -tvec;
            cv::Mat faceRMat;
            cv::Rodrigues(rvec, faceRMat);
            faceRMat.at<double>(0, 0) *= -1;
            faceRMat.at<double>(1, 1) *= -1;
            cv::Rodrigues(faceRMat, rvec);
            // debug
            std::cout << "rotation changed: " << std::endl << rvec << std::endl
                      << "translation changed: " << std::endl << tvec << std::endl;
        }
        // assign the result
        faceRvec = rvec; faceTvec = tvec;

        // visualization
        cv::projectPoints(heading, rvec, tvec, cameraMatrix, distCoeffs,
            heading2d);
        // the heading from self-measured method
        cv::line(frame, faceImgPts[0], heading2d[0], cv::Scalar(0, 0, 255), 2);
        cv::line(frame, faceImgPts[0], heading2d[1], cv::Scalar(0, 255, 0), 2);
        cv::line(frame, faceImgPts[0], heading2d[2], cv::Scalar(255, 0, 0), 2);
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
void ImageConverter::ArUcoPoseArrPnP(
    cv::Mat &im,
    std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
    std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
    std::vector< std::vector<int> > &marker_ids,
    std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
){
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners; 

    // assign corner info and id info with detected markers
    cv::aruco::detectMarkers(
        im, dictionary, corners, ids);

    // display the corner info.
    std::cout << "========corners size: " 
                << corners.size() 
                << "========" << std::endl;

    // if marker detected
    if (ids.size() > 0){

        // debug
        std::cout << "========[aruco pose]========\nPnP called" << std::endl;
        
        // assign the size
        marker_ids.emplace_back();
        marker_ids.emplace_back();
        marker_corners.emplace_back();
        marker_corners.emplace_back();

        // separate the wall ids and the drone ids
        for (int i=0; i<ids.size(); i++){
            
            // the wall
            if (findInVec(ids[i], wall_ids)) {
                marker_ids[0].push_back(ids[i]);
                marker_corners[0].push_back(corners[i]);
                // debug
                std::cout << "======== wall marker id: " << ids[i]
                        << "========" << std::endl;
            }
            // the drone
            else if (findInVec(ids[i], board_ids)) {
                marker_ids[1].push_back(ids[i]);
                marker_corners[1].push_back(corners[i]);
                // debug
                std::cout << "======== drone marker id: " << ids[i]
                        << "========" << std::endl;
            }
            else {
                ROS_INFO_STREAM("========[multi-aruco]========\nUnknown IDs!!");
            }
        }
        // predefined 2 dimension
        marker_rvecs.emplace_back();
        marker_rvecs.emplace_back();
        marker_tvecs.emplace_back();
        marker_tvecs.emplace_back();

        for (int i=0; i<marker_ids.size(); i++){
            if (marker_ids[i].size() > 0){

                // rotation and translation vector in this scope
                // std::vector<cv::Vec3d> rvecs(marker_ids[i].size()), tvecs(marker_ids.size());
                std::vector<cv::Vec3d> rvecs(marker_ids[i].size()), 
                                       tvecs(marker_ids[i].size());
                
                // debug
                std::cout << "======== draw markers ========\n"
                            << "======== ids = ";
                for (int d=0; d<marker_ids[i].size(); d++){
                    std::cout << marker_ids[i][d] << ", ";
                }
                std::cout << "========" << std::endl;

                // draw the markers
                cv::aruco::drawDetectedMarkers(
                    im, marker_corners[i], marker_ids[i]
                );
                // estimate the marker pose
                // determine which size to use
                cv::Mat objPoints(4, 1, CV_32FC3);
                if (i == 0){
                    // wall marker corners 
                    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(
                        -WALL_MARKER_SIZE/2.f, WALL_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(
                        WALL_MARKER_SIZE/2.f, WALL_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(
                        WALL_MARKER_SIZE/2.f, -WALL_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(
                        -WALL_MARKER_SIZE/2.f, -WALL_MARKER_SIZE/2.f, 0);
                }
                else if (i == 1){
                    // drone marker corners 
                    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(
                        -DRONE_MARKER_SIZE/2.f, DRONE_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(
                        DRONE_MARKER_SIZE/2.f, DRONE_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(
                        DRONE_MARKER_SIZE/2.f, -DRONE_MARKER_SIZE/2.f, 0);
                    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(
                        -DRONE_MARKER_SIZE/2.f, -DRONE_MARKER_SIZE/2.f, 0);
                }
                // debug
                std::cout << "======== start solve pnp ========" << std::endl;
                
                // find the rvec and tvec for each marker
                for (int j=0; j<marker_corners[i].size(); j++){
                    cv::solvePnP(
                        objPoints, marker_corners[i][j],
                        cameraMatrix, distCoeffs,
                        rvecs.at(j), tvecs.at(j)
                    );
                }
                // debug
                // std::cout << "======== start draw markers ========" << std::endl;

                // draw the axis for each marker
                for (int j=0; j<marker_ids[i].size(); j++){
                    cv::aruco::drawAxis(
                        im, cameraMatrix, distCoeffs,
                        rvecs[j], tvecs[j], 0.05
                    );
                    // debug
                    cv::Mat RMat;
                    cv::Rodrigues(rvecs[j], RMat);

                    std::cout << "rvec: " << rvecs[j] << std::endl
                              << "rmat: " << RMat << std::endl
                              << "tvec: " << tvecs[j] 
                              << std::endl; 
                    // std::cout << "access each elements: rvec: "
                    //           << rvecs[j][0] << ", "
                    //           << rvecs[j][1] << ", "
                    //           << rvecs[j][2] << std::endl;
                }
                // directly assign the vector
                marker_rvecs[i] = rvecs;
                marker_tvecs[i] = tvecs;

                // clear the rvec and tvec
                rvecs.clear(); tvecs.clear();
            }
        }
    }
}
/*
@publishArUcoMsg
    publish ArUcoPoseArr and PointArr
*/
void ImageConverter::publishArUcoMsg(
    std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
    std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
    std::vector< std::vector<int> > &marker_ids,
    std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
){
    /*======================[aruco pose]==================================*/
    if (arucoPoseArr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========[multi-aruco]========\nNO POSE SUBSCRIBERS");
    }
    else {
        // check if there are marker data
        if (marker_ids.size() > 0){
            
            tello_sub::ArUcoPoseArr ArUcoPoseArr_msg;
            // wall or drone
            for (int i=0; i < marker_ids.size(); i++){
                // length of vector as number of markers
                for (int j=0; j<marker_ids[i].size(); j++){
                    
                    geometry_msgs::Pose pose_msg;
                    pose_msg.position.x = marker_tvecs[i][j][0];
                    pose_msg.position.y = marker_tvecs[i][j][1];
                    pose_msg.position.z = marker_tvecs[i][j][2];

                    // use tf methods
                    const double theta = cv::norm(marker_rvecs[i][j]);
                    tf::Vector3 rot_axis(
                        marker_rvecs[i][j][0],
                        marker_rvecs[i][j][1],
                        marker_rvecs[i][j][2]
                    );
                    rot_axis /= theta;
                    tf::Quaternion q_msg;
                    q_msg.setRotation(rot_axis, theta);
                    pose_msg.orientation.x = q_msg.x();
                    pose_msg.orientation.y = q_msg.y();
                    pose_msg.orientation.z = q_msg.z();
                    pose_msg.orientation.w = q_msg.w();
                    
                    ArUcoPoseArr_msg.poses.push_back(pose_msg);
                    ArUcoPoseArr_msg.ids.push_back(marker_ids[i][j]);
                    
                    // determine the size
                    if (findInVec(marker_ids[i][j], wall_ids)){
                        ArUcoPoseArr_msg.sizes.push_back(
                            WALL_MARKER_SIZE
                        );
                    }
                    else if (findInVec(marker_ids[i][j], board_ids)){
                        ArUcoPoseArr_msg.sizes.push_back(
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
            }
            // header 
            ArUcoPoseArr_msg.header.frame_id = "ArUco_markers";
            ArUcoPoseArr_msg.header.stamp = ros::Time::now();
            
            // publish
            arucoPoseArr_pub.publish(ArUcoPoseArr_msg);
        }
    }
    /*========================[point array]===============================*/
    if (pointarr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========no corner subscriber========");
    }
    else {
        // check if there are marker data
        if (marker_corners.size() > 0){
            tello_sub::PointArr pointarr_msg;

            for (int i=0; i < marker_corners.size(); i++){
                for (int j=0; j<marker_corners[i].size(); j++){
                    
                    // add points
                    for (std::vector<cv::Point2f>::iterator it = marker_corners[i][j].begin();
                            it != marker_corners[i][j].end(); ++it){
                        
                        // on image plane
                        geometry_msgs::Point pt;
                        pt.x = (*it).x;
                        pt.y = (*it).y;
                        pt.z = 0.0;
                        
                        // show info
                        std::cout << "(x, y) = ("
                                    << pt.x << ", " << pt.y << ")"
                                    << std::endl;

                        pointarr_msg.points.push_back(pt);
                    }
                    pointarr_msg.ids.push_back(marker_ids[i][j]);
                }
            }
            // setting the pointarr size as ids size
            pointarr_msg.size = pointarr_msg.ids.size();

            // publish
            pointarr_msg.header.frame_id = "ArUco_markers";
            pointarr_msg.header.stamp = ros::Time::now();
            pointarr_pub.publish(pointarr_msg);
        }
    }
}
/*
@publishArUcoMsgAsFace
    publish ArUco Pose as the FacePose for debugging
*/
void ImageConverter::publishArUcoMsgAsFace(
    std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
    std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
    std::vector< std::vector<int> > &marker_ids,
    std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
){
    /*======================[aruco pose]==================================*/
    if (facePoseArr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========[multi-aruco]========\nNO POSE SUBSCRIBERS");
    }
    else {
        // check if there are marker data
        if (marker_ids.size() > 0){
            geometry_msgs::PoseArray facePoseArr_msg;
            // wall or drone
            for (int i=0; i < marker_ids.size(); i++){
                // length of vector as number of markers
                for (int j=0; j<marker_ids[i].size(); j++){
                    // check the ids
                    if (marker_ids[i][j] == 0){
                        geometry_msgs::Pose pose_msg;
                        pose_msg.position.x = marker_tvecs[i][j][0];
                        pose_msg.position.y = marker_tvecs[i][j][1];
                        pose_msg.position.z = marker_tvecs[i][j][2];

                        // use tf methods
                        const double theta = cv::norm(marker_rvecs[i][j]);
                        tf::Vector3 rot_axis(
                            marker_rvecs[i][j][0],
                            marker_rvecs[i][j][1],
                            marker_rvecs[i][j][2]
                        );
                        rot_axis /= theta;
                        tf::Quaternion q_msg;
                        q_msg.setRotation(rot_axis, theta);
                        pose_msg.orientation.x = q_msg.x();
                        pose_msg.orientation.y = q_msg.y();
                        pose_msg.orientation.z = q_msg.z();
                        pose_msg.orientation.w = q_msg.w();
                        
                        facePoseArr_msg.poses.push_back(pose_msg);
                        // show info
                        ROS_INFO_STREAM("========[face pose debug]========"
                            << "position:\nx: " << pose_msg.position.x 
                            << "\ny: " << pose_msg.position.y 
                            << "\nz: " << pose_msg.position.z 
                            << "\norientation:\nw: " << pose_msg.orientation.w
                            << "\nx: " << pose_msg.orientation.x
                            << "\ny: " << pose_msg.orientation.y
                            << "\nz: " << pose_msg.orientation.z);
                    }
                }
            }
            // header 
            facePoseArr_msg.header.frame_id = "Face_poses";
            facePoseArr_msg.header.stamp = ros::Time::now();
            // publish
            facePoseArr_pub.publish(facePoseArr_msg);
        }
    }
    /*========================[point array]===============================*/
    if (pointarr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========no corner subscriber========");
    }
    else {
        // check if there are marker data
        if (marker_corners.size() > 0){
            tello_sub::PointArr pointarr_msg;

            for (int i=0; i < marker_corners.size(); i++){
                for (int j=0; j<marker_corners[i].size(); j++){
                    
                    // add points
                    for (std::vector<cv::Point2f>::iterator it = marker_corners[i][j].begin();
                            it != marker_corners[i][j].end(); ++it){
                        
                        // on image plane
                        geometry_msgs::Point pt;
                        pt.x = (*it).x;
                        pt.y = (*it).y;
                        pt.z = 0.0;
                        
                        // show info
                        std::cout << "(x, y) = ("
                                    << pt.x << ", " << pt.y << ")"
                                    << std::endl;

                        pointarr_msg.points.push_back(pt);
                    }
                    pointarr_msg.ids.push_back(marker_ids[i][j]);
                }
            }
            // setting the pointarr size as ids size
            pointarr_msg.size = pointarr_msg.ids.size();

            // publish
            pointarr_msg.header.frame_id = "ArUco_markers";
            pointarr_msg.header.stamp = ros::Time::now();
            pointarr_pub.publish(pointarr_msg);
        }
    }
}
/*
@FacePoseArrPnP
    input
        std::vector<cv::Mat > &multiFaceRvec
        std::vector<cv::Mat> &multiFaceTvec
        std::vector<int > &face_sizes
        std::vector<std::vector<cv::Point2f> > &face_corners        
*/
void ImageConverter::FacePoseArrPnP(
    cv::Mat &im,
    std::vector<cv::Mat> &multiFaceRvec,
    std::vector<cv::Mat> &multiFaceTvec,
    std::vector<int > &face_sizes,
    std::vector<std::vector<cv::Point2f> > &face_corners
){
    /*===================[face pose]==================================*/
    // testing parameters
    std::vector<cv::Rect> faces;
    std::vector<int> numDetection, rejLevel;
    std::vector<double> levelWeight;
    int minNeighbor = 3;
    int flags = 0;
    double scaleFactor = 1.1;
    bool rejectFlag = true;

    // setting min and max size of the face bbox
    // cv::Size minSize(30, 30);
    cv::Size minSize(face_bbox_min, face_bbox_min);
    // cv::Size maxSize(150, 150);
    cv::Size maxSize(face_bbox_max, face_bbox_max);
    
    // convert to grayscale for the detector use
    cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);

    faceDetector.detectMultiScale(
        gray, faces, rejLevel, levelWeight, scaleFactor,
        minNeighbor, flags, minSize, maxSize, rejectFlag
    );
    // face landmark vector for more than one face
    std::vector<
        std::vector<cv::Point2f> > landmarks;

    // for single face
    // here we use cv::Mat for the rvecs and tvecs
    cv::Mat faceRvec, faceTvec;

    // fit the model
    bool success = facemark->fit(im, faces, landmarks);

    // draw the reference line for the camera coord.
    cv::line(
        im,
        cv::Point(0, imgHeight/2),
        cv::Point(imgWidth, imgHeight/2),
        cv::Scalar(0, 0, 255)
    );
    cv::line(
        im,
        cv::Point(imgWidth/2, 0),
        cv::Point(imgWidth/2, imgHeight),
        cv::Scalar(0, 255, 0)
    );
    if (!success){
        ROS_INFO_STREAM("========[face detection]========\nNOT SUCCESS!");
    }
    else {
        // debug
        std::cout << "======== number of faces: " << landmarks.size()
                    << "========\n======== record the face ========"
                    << std::endl;

        // render the images with landmarks and faces
        for (int i=0; i < landmarks.size(); i++){

            // show the level weight
            std::cout << "======== level weight: " << levelWeight[i]
                        << " ========" << std::endl;

            // reject the detections with level weight smaller than 55
            if (levelWeight[i] < face_thr) {
                std::cout << "========[face-detect]========\nTOO LOW LEVEL WEIGHT!" << std::endl;
            }
            else {
                // render face landmarks and draw axis
                renderFace(im, landmarks[i]);
                
                rectangle(
                    im, faces[i].tl(), faces[i].br(), 
                    cv::Scalar(255, 50, 50), 1);

                // debug
                // std::cout << "get rvec and tvec" << std::endl;

                // draw axis
                getFacingWithAxis(
                    im, landmarks[i], faceRvec, faceTvec,
                    useNTHU);

                // debug
                // std::cout << "turn into rmat" << std::endl;

                cv::Mat faceRMat;
                cv::Rodrigues(faceRvec, faceRMat);

                // debug
                // std::cout << "show pose info" << std::endl;

                // show the pose info
                displayPoseOnImg(
                    im, faceTvec, faceRMat, 
                    cv::Point(10, imgHeight-200),
                    imgWidth, imgHeight
                );
                // add the size of face box
                cv::putText(
                    im, std::to_string(faces[i].br().x - faces[i].tl().x),
                    cv::Point(faces[i].tl().x, faces[i].tl().y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(30, 255, 10), 2
                );          
                cv::putText(
                    im, std::to_string(levelWeight[i]),
                    cv::Point(faces[i].br().x, faces[i].tl().y - 10), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(30, 255, 10), 2
                );
                // debug
                // std::cout << "faceRvec: cols = " << faceRvec.cols 
                //         << ", rows = " << faceRvec.rows
                //         << "\nfaceTvec: cols = " << faceTvec.cols
                //         << ", rows = " << faceTvec.rows
                //         << std::endl;
                
                multiFaceRvec.push_back(faceRvec);
                multiFaceTvec.push_back(faceTvec);

                // saving current face bbox corner
                face_corners.push_back(
                    std::vector<cv::Point2f> {faces[i].tl(), faces[i].br()}
                );
                face_sizes.push_back(faces[i].br().x - faces[i].tl().x);
            }
        }
    }
}
/*
@publishFacePoseMsg
*/
void ImageConverter::publishFacePoseMsg(
    std::vector<cv::Mat> &multiFaceRvec,
    std::vector<cv::Mat> &multiFaceTvec,
    std::vector<int > &face_sizes,
    std::vector<std::vector<cv::Point2f> > &face_corners
){
    /*=============================[face pose array]======================*/
    if (facePoseArr_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM("========[multi-aruco]========\nNO FACE POSE SUBSCRIBERS");
    }
    else {
        // check if there is face pose arr
        if (multiFaceRvec.size() > 0) {
            geometry_msgs::PoseArray facePoseArr_msg;
        
            // the depth of faceTvec and faceRvec is 6 -> 64F
            for (int i=0; i<multiFaceRvec.size(); i++){
                // assign poses
                // ensure the correct size
                geometry_msgs::Pose facePose;
                facePose.position.x = multiFaceTvec[i].at<double>(i, 0) / 1000;
                facePose.position.y = multiFaceTvec[i].at<double>(i, 1) / 1000;
                facePose.position.z = multiFaceTvec[i].at<double>(i, 2) / 1000;

                // orientation
                const double face_theta = cv::norm(
                    cv::Point3d(
                        multiFaceRvec[i].at<double>(i, 0),
                        multiFaceRvec[i].at<double>(i, 1),
                        multiFaceRvec[i].at<double>(i, 2)
                    )  
                );
                tf::Vector3 face_rot_axis(
                    multiFaceRvec[i].at<double>(i, 0),
                    multiFaceRvec[i].at<double>(i, 1),
                    multiFaceRvec[i].at<double>(i, 2)
                );
                face_rot_axis /= face_theta;
                tf::Quaternion face_q;
                face_q.setRotation(face_rot_axis, face_theta);
                facePose.orientation.x = face_q.x();
                facePose.orientation.y = face_q.y();
                facePose.orientation.z = face_q.z();
                facePose.orientation.w = face_q.w();

                facePoseArr_msg.poses.push_back(facePose);

                // show info
                ROS_INFO_STREAM("face position:\nx: " << facePose.position.x 
                    << "\ny: " << facePose.position.y 
                    << "\nz: " << facePose.position.z);
                ROS_INFO_STREAM("face orientation:\nw: " << facePose.orientation.w
                    << "\nx: " << facePose.orientation.x
                    << "\ny: " << facePose.orientation.y
                    << "\nz: " << facePose.orientation.z);
            }
            facePoseArr_msg.header.frame_id = "Face_poses";
            facePoseArr_msg.header.stamp = ros::Time::now();
            facePoseArr_pub.publish(facePoseArr_msg);
        }
    }
    /*=====================[face bbox]=================================*/
    if (faceBBox_pub.getNumSubscribers() < 1)
    {
        // ROS_INFO_STREAM("========no face bbox subscriber========");
    }
    else
    {
        // if there are face pose data
        if (face_corners.size() > 0) {
            tello_sub::PointArr facebbox_msg;

            // corner points
            // setting the size as number of marker detected
            facebbox_msg.size = face_corners.size();

            for (int i=0; i<face_corners.size(); i++){
                
                // publish the face corners
                // for a face there is only tl and br
                for (std::vector<cv::Point2f>::iterator it = face_corners[i].begin();
                        it != face_corners[i].end();
                        ++it) {
                    geometry_msgs::Point pt;
                    pt.x = (*it).x;
                    pt.y = (*it).y;
                    pt.z = 0;
                    facebbox_msg.points.push_back(pt);
                }
                // size of a face as id
                facebbox_msg.ids.emplace_back(face_sizes[i]);
            }
            // publish
            facebbox_msg.header.frame_id = "face_bbox";
            facebbox_msg.header.stamp = ros::Time::now();
            faceBBox_pub.publish(facebbox_msg);
        }
    }
}
/*
@YOLOPoseArrPnP
    input:
        std::vector< std::vector<cv::Point2f> > &target_corners
        std::vector< cv::Vec3d> &target_rvecs
        std::vector< cv::Vec3d> &target_tvecs
    calculate the rvec and tvec according to the corners and the drone name
*/
void ImageConverter::YOLOPoseArrPnP(
    std::vector< std::vector<cv::Point2f> > &target_corners,
    std::vector< cv::Vec3d> &target_rvecs,
    std::vector< cv::Vec3d> &target_tvecs
){
    // human target as bbox is a coplanar object
    cv::Mat objPoints(4, 1, CV_32FC3);

    // check the drone type
    if (ros_ns == "/tello_2"){
        // only yl and zl
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(
            0, TARGET_WIDTH_SAG/2.f, TARGET_HEIGHT/2.f      // upper-left
        );
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(
            0, -TARGET_WIDTH_SAG/2.f, TARGET_HEIGHT/2.f     // upper-right
        );
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(
            0, TARGET_WIDTH_SAG/2.f, -TARGET_HEIGHT/2.f     // lower-left
        );
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(
            0, -TARGET_WIDTH_SAG/2.f, -TARGET_HEIGHT/2.f    // lower-right
        );
    }
    else if (ros_ns == "/tello_0"){
        // only xl and zl
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(
            -TARGET_WIDTH/2.f, 0, TARGET_HEIGHT/2.f      // upper-left
        );
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(
            TARGET_WIDTH/2.f, 0, TARGET_HEIGHT/2.f     // upper-right
        );
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(
            -TARGET_WIDTH/2.f, 0, -TARGET_HEIGHT/2.f     // lower-left
        );
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(
            TARGET_WIDTH/2.f, 0, -TARGET_HEIGHT/2.f    // lower-right
        );
    }
    // debug
    std::cout << "========[YOLO pose]========\nSolve PnP" << std::endl;

    // naive pose estimation
    cv::Vec3d rvec, tvec;
    for (int i=0; i<target_corners.size(); i++){

        cv::solvePnP(
            objPoints, target_corners.at(i),
            cameraMatrix, distCoeffs,
            rvec, tvec, false, cv::SOLVEPNP_ITERATIVE
        );
        // debug
        cv::Mat Rmat;
        cv::Rodrigues(rvec, Rmat);
        std::cout << "rvec" << std::endl << rvec << std::endl
                    << "tvec" << std::endl << tvec << std::endl
                    << "Rmat" << std::endl << Rmat << std::endl;
    }
    // save the vector
    target_rvecs.emplace_back(rvec);
    target_tvecs.emplace_back(tvec);
}
/*
@publishYOLOPoseMsg
    input:
        std::vector< cv::Vec3d> &target_rvecs
        std::vector< cv::Vec3d> &target_tvecs
*/
void ImageConverter::publishYOLOPoseMsg(
    std::vector< cv::Vec3d> &target_rvecs,
    std::vector< cv::Vec3d> &target_tvecs
){
    if (YOLOPose_pub.getNumSubscribers() < 1){
        // ROS_INFO_STREAM(
        //     "========[YOLO pose]========\nNO POSE SUBs!"
        // );
    }
    else {
        // check if there are data
        if (target_rvecs.size() > 0){
            geometry_msgs::PoseArray YOLOPoseArr_msg;
            for (int i=0; i<target_rvecs.size(); i++){
                geometry_msgs::Pose YOLOPose;
                YOLOPose.position.x = target_tvecs[i][0];
                YOLOPose.position.y = target_tvecs[i][1];
                YOLOPose.position.z = target_tvecs[i][2];
                
                // convert to quaternion with tf lib
                const double theta = cv::norm(target_rvecs[i]);
                tf::Vector3 rot_axis(
                    target_rvecs[i][0],
                    target_rvecs[i][1],
                    target_rvecs[i][2]
                );
                rot_axis /= theta;
                tf::Quaternion q;
                q.setRotation(rot_axis, theta);
                YOLOPose.orientation.x = q.x();
                YOLOPose.orientation.y = q.y();
                YOLOPose.orientation.z = q.z();
                YOLOPose.orientation.w = q.w();

                YOLOPoseArr_msg.poses.push_back(YOLOPose);
                
                // debug
                ROS_INFO_STREAM(
                    "position:\n" << YOLOPose.position << "\norientation:\n" << YOLOPose.orientation
                );
            }
            YOLOPoseArr_msg.header.frame_id = "YOLO_pose";
            YOLOPoseArr_msg.header.stamp = ros::Time::now();
            
            YOLOPose_pub.publish(YOLOPoseArr_msg);
        }
    }
}
/*
@imageCb
    sensor_msgs::Image callback
    detect aruco and human face
    render the result on the image
    publish the msg
*/
void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
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

    /*==========================[aruco detection]========================*/
    if (missionFlag[0]){
        // for aruco
        std::vector< std::vector<cv::Vec3d> > marker_tvecs, marker_rvecs;
        
        // marker_ids[0]: markers for the wall
        // marker_ids[1]: markers for the drone
        std::vector<std::vector<int> > marker_ids;

        // marker_cornres[0]: corners for the wall
        // marker_corners[1]: corners for the drone
        std::vector<
            std::vector<
                std::vector<cv::Point2f> > > marker_corners; 

        // get the pose and ids and corners
        ArUcoPoseArrPnP(
            cv_ptr->image, 
            marker_rvecs, marker_tvecs, marker_ids, marker_corners
        );
        // publish the data if there are
        publishArUcoMsg(
            marker_rvecs, marker_tvecs, marker_ids, marker_corners
        );
    }
    /*=====================[face detection]================================*/
    if (missionFlag[1]){
        // debug face mode
        if (debugFaceMarker){
            // for aruco
            std::vector< std::vector<cv::Vec3d> > marker_tvecs, marker_rvecs;
            
            // marker_ids[0]: markers for the wall
            // marker_ids[1]: markers for the drone
            std::vector<std::vector<int> > marker_ids;

            // marker_cornres[0]: corners for the wall
            // marker_corners[1]: corners for the drone
            std::vector<
                std::vector<
                    std::vector<cv::Point2f> > > marker_corners; 
            // get the pose and ids and corners
            ArUcoPoseArrPnP(
                cv_ptr->image, 
                marker_rvecs, marker_tvecs, marker_ids, marker_corners
            );
            // publish the data if there are
            publishArUcoMsgAsFace(
                marker_rvecs, marker_tvecs, marker_ids, marker_corners
            );
        }
        else {
            // for multiple face
            std::vector<cv::Mat> multiFaceRvec, multiFaceTvec;
            // face bbox collector
            std::vector<
                std::vector<cv::Point2f> > face_corners;
            std::vector<int > face_sizes;
            // get face poses and size and corners
            FacePoseArrPnP(
                cv_ptr->image, 
                multiFaceRvec, multiFaceTvec, face_sizes, face_corners
            );
            // publish the data if there are
            publishFacePoseMsg(
                multiFaceRvec, multiFaceTvec, face_sizes, face_corners
            );
        }
    }
    /*=====================[yolo]======================*/
    if (missionFlag[2]){
        // draw the bbox on the image
        std::vector<cv::Point2f> heading_2d;
        std::vector<cv::Point3f> heading;
        heading.push_back(cv::Point3f(100.0, 0.0, 0.0));
        heading.push_back(cv::Point3f(0.0, 100.0, 0.0));
        heading.push_back(cv::Point3f(0.0, 0.0, 100.0));
        
        for (int i=0; i<bbox_rvecs.size(); i++) {
            // project the axis
            cv::projectPoints(
                heading, bbox_rvecs[i], bbox_tvecs[i], cameraMatrix, distCoeffs,
                heading_2d
            );
            cv::line(
                cv_ptr->image, bbox_centers[i], heading_2d[0],
                cv::Scalar(0, 0, 255), 2
            );
            cv::line(
                cv_ptr->image, bbox_centers[i], heading_2d[1],
                cv::Scalar(0, 255, 0), 2
            );
            cv::line(
                cv_ptr->image, bbox_centers[i], heading_2d[2],
                cv::Scalar(255, 0, 0), 2
            );
        } 
    }
    // ================= show the image
    // Update GUI Window
    cv::imshow(ros_ns + OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // publish the modified image
    image_pub_.publish(cv_ptr->toImageMsg());
}
/*
@BBoxesCb
    get the image plane position of the human target
    calculate the position of human target under camera frame
*/
void ImageConverter::BBoxesCb(const darknet_ros_msgs::BoundingBoxes& msg){
    // record
    std::vector<std::vector<cv::Point2f> > target_corners;
    std::vector<cv::Point2f> target_centers;
    std::vector<double> target_prob;

    for (int i=0; i<msg.bounding_boxes.size(); i++){
        // debug
        std::cout << "========[yolo pose estimation]========" << std::endl
                  << "id: " << msg.bounding_boxes[i].id << std::endl
                  << "class: " << msg.bounding_boxes[i].Class << std::endl;
        
        // check the class and id
        if ((msg.bounding_boxes[i].id == 0) && 
            (msg.bounding_boxes[i].Class == "person"))
        {
            // assign the corner
            std::vector<cv::Point2f> corner_pts{
                cv::Point2f(msg.bounding_boxes[i].xmin, msg.bounding_boxes[i].ymin),
                cv::Point2f(msg.bounding_boxes[i].xmax, msg.bounding_boxes[i].ymin),
                cv::Point2f(msg.bounding_boxes[i].xmin, msg.bounding_boxes[i].ymax),
                cv::Point2f(msg.bounding_boxes[i].xmax, msg.bounding_boxes[i].ymax)     
            };
            target_corners.emplace_back(corner_pts);
            target_prob.emplace_back(msg.bounding_boxes[i].probability);
            target_centers.emplace_back(
                cv::Point2f(
                    (msg.bounding_boxes[i].xmin+msg.bounding_boxes[i].xmax)/2,
                    (msg.bounding_boxes[i].ymin+msg.bounding_boxes[i].ymax)/2 
                )
            );
        }
    }
    // debug
    std::cout << "number of target humans: " << target_corners.size() << std::endl;

    if (target_corners.size() > 0){
        // find the target pose
        std::vector<cv::Vec3d> target_rvecs, target_tvecs;
        YOLOPoseArrPnP(target_corners, target_rvecs, target_tvecs);

        // publish collected data
        publishYOLOPoseMsg(target_rvecs, target_tvecs);

        // assign the rvec and tvec for drawing on the image
        bbox_centers = target_centers;
        bbox_rvecs = target_rvecs;
        bbox_tvecs = target_tvecs;

        // clear the local stacks
        target_centers.clear();
        target_rvecs.clear();
        target_tvecs.clear();
    }
}

//main function
int main(int argc, char **argv)
{
    //ros::init()
    ros::init(argc, argv, "aruco_pose_estimator", ros::init_options::AnonymousName);

    ros::NodeHandle nh_;
    ImageConverter ic(nh_);

    ros::spin();

    return 0;
}