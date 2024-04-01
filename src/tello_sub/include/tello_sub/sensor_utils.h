#ifndef SENSOR_UTILS_H
#define SENSOR_UTILS_H

// std-lib
# include <iostream>
# include <string>
# include <vector> 
// ros-dep
# include <ros/ros.h>
# include <tf/tf.h>
// geometry msg
# include <geometry_msgs/PoseStamped.h>
# include <geometry_msgs/PoseArray.h>
# include <geometry_msgs/Pose.h>
# include <geometry_msgs/Point.h>

# include <image_transport/image_transport.h>

// opencv-depend
# include <cv_bridge/cv_bridge.h>
# include <opencv2/imgproc/imgproc.hpp>
# include <opencv2/highgui/highgui.hpp>
// for aruco marker example
# include <opencv2/aruco.hpp>
# include <opencv2/calib3d.hpp>
// opencv operation: norm
#include <opencv2/core.hpp>
// for opencv face detection
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/face.hpp>
// for pnp solver
#include <opencv2/opencv.hpp>

// YOLO msg
# include "darknet_ros_msgs/BoundingBoxes.h"
# include "darknet_ros_msgs/BoundingBox.h"

// for self-defined msg
#include "tello_sub/PointArr.h"
#include "tello_sub/ArUcoPoseArr.h"


// =================== define global variable here ==================
# define TARGET_HEIGHT          double(1.70)    // coronal height
# define TARGET_WIDTH_SAG       double(0.20)    // sagittal distance
# define TARGET_WIDTH           double(0.70)    // shoulder width
# define LINECOLOR           cv::Scalar(255, 200, 0)
# define WALL_MARKER_SIZE    double(0.175)
# define DRONE_MARKER_SIZE   double(0.09)

// opencv window
static const std::string OPENCV_WINDOW = "/Image window";

/*=========================[helper function]===============*/
// function template 
/*
@findInVec
    find a value in the given vector by brute force
*/
template <class T>
static bool findInVec(T Tval, std::vector<T> &Tvec){
    for (auto t : Tvec){
        if (t == Tval) {return true;}
    }
    return false;
}
// utils
/*
@Mat_type2str
    return the string of type of cv::Mat
    ref: https://stackoverflow.com/questions/10167534/how-to-find-out-what-type-of-a-mat-object-is-with-mattype-in-opencv
*/
static std::string Mat_type2str(int type){
    std::string r;
    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);
    switch (depth) {
        case CV_8U:     r = "8U"; break;
        case CV_8S:     r = "8S"; break;
        case CV_16U:    r = "16U"; break;
        case CV_16S:    r = "16S"; break;
        case CV_32S:    r = "32S"; break;
        case CV_32F:    r = "32F"; break;
        case CV_64F:    r = "64F"; break;
        default:        r = "User"; break;
    }
    r += "C";
    r += (chans + '0');
    return r;
}
/*==================[visualization]===============================*/
/*
@displayPoseOnImg
    display the given tvec and RMat on the frame
*/
static void displayPoseOnImg(
    cv::Mat &frame, cv::Mat &tvec_, cv::Mat &RMat_, cv::Point startPt,
    int &img_width, int &img_height
){
    // setup
    int fontSize = 1;
    
    std::string tvecText, rotMatText;
    tvecText = "translation: ";
    rotMatText = "rotation matrix: ";
    cv::putText(
        frame, tvecText,
        startPt, cv::FONT_HERSHEY_SIMPLEX,
        0.9, cv::Scalar(255, 255, 255), fontSize
    );
    cv::putText(
        frame, rotMatText,
        cv::Point(startPt.x, startPt.y+100), cv::FONT_HERSHEY_SIMPLEX,
        0.9, cv::Scalar(255, 255, 255), fontSize
    );
    // clean up previous content
    tvecText.clear();

    for (int i=0; i<3; i++){
        // clean up previous content
        rotMatText.clear();
        // add data and draw
        for (int j=0; j<3; j++){
            rotMatText += std::to_string(RMat_.at<double>(i, j)) + ", ";
        }
        cv::putText(
            frame, rotMatText,
            cv::Point(startPt.x, startPt.y+130+30*i), cv::FONT_HERSHEY_SIMPLEX,
            0.9, cv::Scalar(255, 255, 255), fontSize
        );
        tvecText += std::to_string(tvec_.at<double>(i)) + ", ";
    }
    // draw the translation
    cv::putText(
        frame, tvecText,
        cv::Point(startPt.x, startPt.y+30), cv::FONT_HERSHEY_SIMPLEX,
        0.9, cv::Scalar(255, 255, 255), fontSize
    );
}
/*
@drawPolyLine:
    draw the poly line by joining successive points between the start and end indices
*/
static void drawPolyLine(
    cv::Mat &im,
    const std::vector<cv::Point2f> &landmarks,
    const int start,
    const int end,
    bool isClosed = false
){
    // gather all points between the start and end indices
    std::vector<cv::Point> pts;
    for (int i = start; i <= end; i++){
        pts.push_back(cv::Point(landmarks[i].x, landmarks[i].y));
    }
    // draw polylines
    polylines(im, pts, isClosed, LINECOLOR, 0.8, 16);
}
/*
@renderFace
    plot the face landmarks
*/
static void renderFace(cv::Mat &im, std::vector<cv::Point2f> &landmarks){
    // Draw face for the 68-point model
    if (landmarks.size() == 68){
        drawPolyLine(im, landmarks, 0, 16);         // jaw line
        drawPolyLine(im, landmarks, 17, 21);        // left eyebrow
        drawPolyLine(im, landmarks, 22, 26);        // right eyebrow
        drawPolyLine(im, landmarks, 27, 30);        // Nose bridge
        drawPolyLine(im, landmarks, 30, 35, true);  // lower nose
        drawPolyLine(im, landmarks, 36, 41, true);  // left eye
        drawPolyLine(im, landmarks, 42, 47, true);  // right eye
        drawPolyLine(im, landmarks, 48, 59, true);  // outer lip
        drawPolyLine(im, landmarks, 60, 67, true);  // inner lip

        // show data
        // std::cout << "====[facemark]====\n"
        //     << "eyebrow: ";
        // for (int i = 17; i <= 26; i++){
        //     std::cout << landmarks[i] << " ";
        // }
        // std::cout << "\nnose bridge: ";
        // for (int i = 27; i <= 30; i++){
        //     std::cout << landmarks[i] << " ";
        // }
        // std::cout << "\neyes: ";
        // for (int i = 36; i<= 47; i++){
        //     std::cout << landmarks[i] << " ";
        // }
        // std::cout << std::endl;
    }
    else {
        // use the circle to identify each landmark
        for (int i=0; i<landmarks.size(); i++){
            circle(im, landmarks[i], 1, LINECOLOR, cv::FILLED);
        }
    }
}
/*===================[image converter]=======================*/
class ImageConverter
{
    // image transport
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // pose
    ros::Publisher pointarr_pub;
    ros::Publisher faceBBox_pub;

    ros::Publisher arucoPoseArr_pub;
    ros::Publisher facePoseArr_pub;

    // YOLO
    ros::Subscriber YOLOBBox_sub;
    ros::Publisher YOLOPose_pub;

    // bbox pose vector
    std::vector<cv::Vec3d> bbox_rvecs, bbox_tvecs;
    std::vector<cv::Point2f> bbox_centers;

    // camera matrix
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    // aruco marker dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    // drone namespace and node name
    std::string ros_ns, ros_nodeName;
    
    // marker size
    // different ids for different size
    std::vector<int > board_ids = {0, 22, 25};         // setting the wall and board ids, 11 --> MD5F, 17 --> MD601
    std::vector<int > wall_ids = {11, 17};       // 0 --> setting as face gt

    // face cascadeclf
    cv::CascadeClassifier faceDetector;
    std::string cascadePath = "/home/chungyu/wayne_temp/tello_ws/files/haarcascade_frontalface_alt2.xml";
    
    // facemark lbf model
    cv::Ptr<cv::face::Facemark> facemark;
    std::string facemarkPath = "/home/chungyu/wayne_temp/tello_ws/files/lbfmodel.yaml";
    
    // variables to store a video frame and grayscale
    cv::Mat frame, gray;
    
    // face detection threshold
    double face_thr;
    double face_bbox_max = 240.0; 
    double face_bbox_min = 100.0;

    // window size
    int imgWidth, imgHeight;

    // flags
    bool missionFlag[3] = {false, false, false};
    bool useNTHU = false;
    bool debugFaceMarker = false;

    // fstream testing
    // std::ofstream myFile;

    /*====================[callbacks]========================*/
    /*
    @imageCb
        sensor_msgs::Image callback
        detect aruco and human face
        render the result on the image
        publish the msg
    */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    /*
    @BBoxesCb
        get the image plane position of the human target
        calculate the position of human target under camera frame
    */
    void BBoxesCb(const darknet_ros_msgs::BoundingBoxes& msg);
    /*==================[functions]==========================*/
    /*
    @getFacingWithAxis
        compute the axis from the face landmarks
        draw the axis
    */
    void getFacingWithAxis(
        cv::Mat &frame, std::vector<cv::Point2f> &landmarks, 
        cv::Mat &faceRvec, cv::Mat &faceTvec, bool useNTHU
    );
    /*
    @ArUcoPoseArrPnP
        input:
            cv::Mat &im
            std::vector< std::vector<cv::Vec3d> > &marker_rvecs;
            std::vector< std::vector<cv::Vec3d> > &marker_tvecs;
            std::vector< std::vector<int> > &marker_ids;
            std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners;

        additional size is preserved for the wall marker and drone marker 
        compute pnp from the image 
        record:
            pose as rvecs, tvecs
            marker ids
            marker corners
    */
    void ArUcoPoseArrPnP(
        cv::Mat &im,
        std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
        std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
        std::vector< std::vector<int> > &marker_ids,
        std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
    );
    /*
    @publishArUcoMsg
    */
    void publishArUcoMsg(
        std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
        std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
        std::vector< std::vector<int> > &marker_ids,
        std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
    );
    /*
    @publishArUcoMsg
    */
    void publishArUcoMsgAsFace(
        std::vector< std::vector<cv::Vec3d> > &marker_rvecs,
        std::vector< std::vector<cv::Vec3d> > &marker_tvecs,
        std::vector< std::vector<int> > &marker_ids,
        std::vector< std::vector< std::vector<cv::Point2f> > > &marker_corners
    );
    /*
    @FacePoseArrPnP
        input
            cv::Mat &im
            std::vector<cv::Mat > &multiFaceRvec
            std::vector<cv::Mat> &multiFaceTvec
            std::vector<int > &face_sizes
            std::vector<std::vector<cv::Point2f> > &face_corners        
    */
    void FacePoseArrPnP(
        cv::Mat &im,
        std::vector<cv::Mat> &multiFaceRvec,
        std::vector<cv::Mat> &multiFaceTvec,
        std::vector<int > &face_sizes,
        std::vector<std::vector<cv::Point2f> > &face_corners
    );
    /*
    @publishFacePoseMsg
        input
            std::vector<cv::Mat > &multiFaceRvec
            std::vector<cv::Mat> &multiFaceTvec
            std::vector<int > &face_sizes
            std::vector<std::vector<cv::Point2f> > &face_corners        
    */
    void publishFacePoseMsg(
        std::vector<cv::Mat> &multiFaceRvec,
        std::vector<cv::Mat> &multiFaceTvec,
        std::vector<int > &face_sizes,
        std::vector<std::vector<cv::Point2f> > &face_corners
    );
    /*
    @YOLOPoseArrPnP
        input:
            std::vector< std::vector<cv::Point2f> > &target_corners
            std::vector< cv::Vec3d> &target_rvecs
            std::vector< cv::Vec3d> &target_tvecs
        calculate the rvec and tvec according to the corners and the drone name
    */
    void YOLOPoseArrPnP(
        std::vector< std::vector<cv::Point2f> > &target_corners,
        std::vector< cv::Vec3d> &target_rvecs,
        std::vector< cv::Vec3d> &target_tvecs
    );
    /*
    @publishYOLOPoseMsg
        input:
            std::vector< cv::Vec3d> &target_rvecs
            std::vector< cv::Vec3d> &target_tvecs
    */
    void publishYOLOPoseMsg(
        std::vector< cv::Vec3d> &target_rvecs,
        std::vector< cv::Vec3d> &target_tvecs
    );
public:
    /*
    @ImageConverter
    */
    // ImageConverter(){}
    // ~ImageConverter(){}
    /*
    @ImageConverter
        explicit constructor
    */
    explicit ImageConverter(ros::NodeHandle& nh_);
    /*
    @~ImageConverter
    */
    ~ImageConverter(){ cv::destroyWindow(ros_ns + OPENCV_WINDOW); };
};
/*====================[image converter realsense]=====================*/
/*
@ImageConverterRS
    subscribe to realsense image_raw
    publish ground-truth poses with timestamps
*/
class ImageConverterRS
{
    // image transport
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // pose
    ros::Publisher arucoPoseArr_pub;

    // param
    std::string ros_ns;
    int buffer_size = 1;
    bool default_cam_info = true;
    bool cam_info_updated = false;


    // camera matrix
    ros::Subscriber cam_info_sub;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    // aruco marker dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // ids
    //      drone 1:    25, 26
    //      drone 2:    22, 23
    //      face debug: 0
    std::vector<int> drone_marker_ids = {0, 22, 23, 25, 26};
    
    // variables to store a video frame
    cv::Mat frame;
    int imgWidth, imgHeight;
    
    /*===================[callback]=====================*/
    /*
    @imageCb
        callback for the image
    */
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    /*
    @camInfoCb
        setting the camera intrinsics
    */
    void camInfoCb(sensor_msgs::CameraInfo const& camInfo);
    /*
    @ArUcoPoseArrPnP
        additional size is preserved for the wall marker and drone marker 
        compute pnp from the image 
        record:
            pose as rvecs, tvecs
            marker ids
            marker corners
    */
    void ArUcoPoseArrPnP(
        cv::Mat &im,
        std::vector<cv::Vec3d> &marker_rvecs,
        std::vector<cv::Vec3d> &marker_tvecs,
        std::vector<int> &marker_ids,
        std::vector< std::vector<cv::Point2f> > &marker_corners
    );
    /*
    @publishArUcoMsg
    */
    void publishArUcoMsg(
        std::vector<cv::Vec3d> &marker_rvecs,
        std::vector<cv::Vec3d> &marker_tvecs,
        std::vector<int> &marker_ids
    );

public:
    explicit ImageConverterRS(ros::NodeHandle& nh_);
    ~ImageConverterRS(){ cv::destroyWindow(ros_ns + OPENCV_WINDOW); };
};

#endif