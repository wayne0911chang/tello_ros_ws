# include "tello_pub/ros_eigen_dep.h"
# include "tello_pub/SE3Pose.h"
# include "tello_pub/GeometricCtrlDebug.h"
# include "tello_pub/math_utils.h"
# include "yaml-cpp/yaml.h"
# include <random>
/*
@debugModeHuman
    human state
*/
enum debugModeHuman {
    STAND_STILL,
    GO_STRAIGHT,
    CONST_VEL_WALKING,
    LOA_IS_FACING
};
/*
@FormCtrlDebugger
    for simulation
    1st-order kinematics in ZOH
*/
class FormCtrlDebugger
{
    // ros-dep
    ros::Subscriber cmd_0_sub, cmd_1_sub, cmd_2_sub,
                    exp_mode_sub, SOT_sub;
    ros::Publisher eskf_0_pub, eskf_1_pub, eskf_2_pub,
                   odom_0_pub, odom_1_pub, odom_2_pub, odom_face_pub,   // in I0 frame
                   eskf_init_1_pub, eskf_init_2_pub,
                   eskf_face_pub, eskf_init_face_pub;
    ros::Timer drone0Timer, drone1Timer, drone2Timer,
               faceTimer;
    ros::Time d0StartTime, d1StartTime, d2StartTime,
              faceStartTime;

    // param 
    int queue_size = 1;
    double t_drones = 0.01;
    double t_face = 0.05;
    double height_offset = 0.0;
    bool StartOfTest = false;       // publish initial states if SOT
    bool isExpMode = false;         // publish drone state and predict
    bool noRollPitch = false;
    int humanMode = 0;              // default stand still

    std::string droneName[3] = {"/tello_0", "/tello_1", "/tello_2"};
    std::string ros_nodeName;

    // states
    Eigen::Vector3d posIner0ToIneri[2];
    Eigen::Matrix3d rotIneriToIner0[2];

    Eigen::VectorXd droneTwist[3];          // in command frame, [omega, v]
    Eigen::Vector3d dronePosition[3];
    Eigen::Quaterniond droneQuat[3];

    Eigen::Vector3d posIner1ToInitFace; 
    Eigen::Matrix3d rotInitFaceToIner1;

    Eigen::VectorXd faceTwist;              // in face frame, [omega, v]
    Eigen::Vector3d facePosition;
    Eigen::Quaterniond faceQuat;

    // constant transformation
    Eigen::Matrix3d rotCmdToBody;


    // functions
    /*
    @exp_mode_cb
        only could be activate if SOT is true
    */
    void exp_mode_cb(std_msgs::Empty const& msg){
        if (!isExpMode && StartOfTest){
            isExpMode = true;
            // debug
            ROS_INFO_STREAM("========[formation debugger]========\nreceived exp mode!");
        }
    }
    void telloSOTCb(std_msgs::Empty const& msg){
        if (!StartOfTest){
            drone0Timer.start();
            drone1Timer.start();
            drone2Timer.start();
            faceTimer.start();
            d0StartTime = ros::Time::now();
            d1StartTime = ros::Time::now();
            d2StartTime = ros::Time::now();
            faceStartTime = ros::Time::now();
            StartOfTest = true;
            ROS_INFO_STREAM("========[formation debugger]========\nreceived SOT!");
        }
    }
    /*
    @cmd_vel_0_cb
        twist cmd in cmd frame
        angular velocity only in yaw axis
    */
    void cmd_vel_0_cb(geometry_msgs::Twist const& twist){
        if (!noRollPitch){
            droneTwist[0](0) = twist.angular.x;
            droneTwist[0](1) = twist.angular.y;
        }
        droneTwist[0](2) = twist.angular.z;
        droneTwist[0](3) = twist.linear.x;
        droneTwist[0](4) = twist.linear.y;
        droneTwist[0](5) = twist.linear.z;        
    }
    void cmd_vel_1_cb(geometry_msgs::Twist const& twist){
        if (!noRollPitch){
            droneTwist[1](0) = twist.angular.x;
            droneTwist[1](1) = twist.angular.y;
        }
        droneTwist[1](2) = twist.angular.z;
        droneTwist[1](3) = twist.linear.x;
        droneTwist[1](4) = twist.linear.y;
        droneTwist[1](5) = twist.linear.z;
    }
    void cmd_vel_2_cb(geometry_msgs::Twist const& twist){
        if (!noRollPitch){
            droneTwist[2](0) = twist.angular.x;
            droneTwist[2](1) = twist.angular.y;
        }
        droneTwist[2](2) = twist.angular.z;
        droneTwist[2](3) = twist.linear.x;
        droneTwist[2](4) = twist.linear.y;
        droneTwist[2](5) = twist.linear.z;
    }
    /*
    @d0TimerCb
        publish true state
        predict drone state
    */
    void d0TimerCb(ros::TimerEvent const& ev){
        publishDroneState(0);
        predictDroneState(0);
    }
    void d1TimerCb(ros::TimerEvent const& ev){
        publishDroneState(1);
        predictDroneState(1);
    }
    void d2TimerCb(ros::TimerEvent const& ev){
        publishDroneState(2);
        predictDroneState(2);
    }
    void faceTimerCb(ros::TimerEvent const& ev){
        publishFaceState();
        predictFaceState();
    }
    /*
    @predictDroneState
        predict drone's state depends on the id
        check SOT here
    */
    void predictDroneState(int ind){
        if (isExpMode && StartOfTest){
            // in body frame
            // Eigen::Quaterniond quat_angvel = 
            //     vector3dToQuaterniond(-droneTwist[ind].head(3) * t_drones);
            // set the angular velocity cmd frame as the opposite of the cmd frame
            // Eigen::Quaterniond quat_angvel = 
            //     vector3dToQuaterniond(-rotCmdToBody * droneTwist[ind].head(3) * t_drones);
            // set the angular velocity cnd frame as the body frame
            Eigen::Quaterniond quat_angvel = 
                vector3dToQuaterniond(droneTwist[ind].head(3) * t_drones);
            // inertial frame
            dronePosition[ind] += 
                droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3) * t_drones;
            // body to inertial
            droneQuat[ind] = droneQuat[ind] * quat_angvel;
        }
    }
    /*
    @predictFaceState
        predict face state depends on the human mode
    */
    void predictFaceState(){
        switch (humanMode)
        {
        case STAND_STILL:
            // pass
            break;
        
        case GO_STRAIGHT:
            if (isExpMode){
                // in inertial frame
                facePosition += faceQuat.toRotationMatrix() * faceTwist.tail(3) * t_face;
            }
            break;
        
        case CONST_VEL_WALKING:
            if (isExpMode){
                // construct random
                // srand(0);
                std::random_device rd;
                std::mt19937 gen(rd());
                std::normal_distribution<double> nd(0.0, 0.005);
                // in Face frame
                Eigen::Quaterniond quat_angvel = 
                    vector3dToQuaterniond(faceTwist.head(3) * t_face);
                // random adjust the linear velocity
                faceTwist(3) += nd(gen);
                faceTwist(5) += nd(gen);
                // in inertial frame
                facePosition += faceQuat.toRotationMatrix() * faceTwist.tail(3) * t_face;
                // face to init face
                faceQuat = faceQuat * quat_angvel;
                // debug
                std::cout << "========[formation debugger]========\nlin vel in F:\n"
                          << faceTwist.tail(3) << std::endl
                          << "ang vel in F:\n" << faceTwist.head(3) << std::endl
                          << "random walk increment: " << nd(gen) << std::endl;
            }
            break;

        case LOA_IS_FACING:
            if (isExpMode){
                // in Face frame
                Eigen::Quaterniond quat_angvel = 
                    vector3dToQuaterniond(faceTwist.head(3) * t_face);
                // in inertial frame
                facePosition += faceQuat.toRotationMatrix() * faceTwist.tail(3) * t_face;
                // face to init face
                faceQuat = faceQuat * quat_angvel;
                // debug
                std::cout << "========[formation debugger]========\nlin vel in F:\n"
                          << faceTwist.tail(3) << std::endl
                          << "ang vel in F:\n" << faceTwist.head(3) << std::endl;
            }
            break;
        default:
            // debug
            std::cout << "========[formation debugger]========\nUnknown human mode!" << std::endl;
            break;
        }
    }
    /*
    @publishTrueState
        publish drone's state depends on the id
    */
    void publishDroneState(int ind){
        if (!isExpMode && StartOfTest){
            switch (ind)
            {
            case 0:
                if (eskf_0_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d0StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_0_pub.publish(trajPt);
                }
                if (odom_0_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    odom_msg.pose.pose.position.x = dronePosition[0](0);
                    odom_msg.pose.pose.position.y = dronePosition[0](1);
                    odom_msg.pose.pose.position.z = dronePosition[0](2) + height_offset;
                    tf::quaternionEigenToMsg(
                        droneQuat[0], odom_msg.pose.pose.orientation
                    );
                    Eigen::Vector3d tmp = 
                        droneQuat[0].toRotationMatrix() * rotCmdToBody * droneTwist[0].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[0](2);
                    odom_0_pub.publish(odom_msg);
                }
                break;
                
            case 1:
                if (eskf_init_1_pub.getNumSubscribers() > 0){
                    tello_pub::SE3Pose SE3Pose_msg;
                    SE3Pose_msg.header.stamp = ros::Time::now();
                    SE3Pose_msg.tvec.x = posIner0ToIneri[ind-1](0);
                    SE3Pose_msg.tvec.y = posIner0ToIneri[ind-1](1);
                    SE3Pose_msg.tvec.z = posIner0ToIneri[ind-1](2);
                    std::memcpy(
                        &SE3Pose_msg.rmat, rotIneriToIner0[ind-1].data(), 9 * sizeof(double)
                    );
                    eskf_init_1_pub.publish(SE3Pose_msg);
                }
                if (eskf_1_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d1StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_1_pub.publish(trajPt);
                }
                if (odom_1_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    // position map to I0 frame
                    Eigen::Vector3d tmp = rotIneriToIner0[0] * Eigen::Vector3d(
                        dronePosition[1](0), 
                        dronePosition[1](1) + height_offset, 
                        dronePosition[1](2)
                    ) + posIner0ToIneri[0];
                    odom_msg.pose.pose.position.x = tmp(0);
                    odom_msg.pose.pose.position.y = tmp(1);
                    odom_msg.pose.pose.position.z = tmp(2);
                    // orientation as B1 --> I0
                    tf::quaternionEigenToMsg(
                        Eigen::Quaterniond(rotIneriToIner0[0] * droneQuat[1].toRotationMatrix()), 
                        odom_msg.pose.pose.orientation
                    );
                    // linear velocity on I0 
                    tmp = rotIneriToIner0[0] * droneQuat[1].toRotationMatrix() * 
                        rotCmdToBody * droneTwist[1].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // angular velocity on body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    odom_msg.twist.twist.angular.x = tmp(0);
                    odom_msg.twist.twist.angular.y = tmp(1);
                    odom_msg.twist.twist.angular.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[1](2);
                    odom_1_pub.publish(odom_msg);
                }
                break;
            
            case 2:
                if (eskf_init_2_pub.getNumSubscribers() > 0){
                    tello_pub::SE3Pose SE3Pose_msg;
                    SE3Pose_msg.header.stamp = ros::Time::now();
                    SE3Pose_msg.tvec.x = posIner0ToIneri[ind-1](0);
                    SE3Pose_msg.tvec.y = posIner0ToIneri[ind-1](1);
                    SE3Pose_msg.tvec.z = posIner0ToIneri[ind-1](2);
                    std::memcpy(
                        &SE3Pose_msg.rmat, rotIneriToIner0[ind-1].data(), 9 * sizeof(double)
                    );
                    eskf_init_2_pub.publish(SE3Pose_msg);
                }
                if (eskf_2_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d2StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_2_pub.publish(trajPt);
                }
                if (odom_2_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    // position map to I0 frame
                    Eigen::Vector3d tmp = rotIneriToIner0[1] * Eigen::Vector3d(
                        dronePosition[2](0),
                        dronePosition[2](1) + height_offset,
                        dronePosition[2](2)
                    ) + posIner0ToIneri[1];
                    odom_msg.pose.pose.position.x = tmp(0);
                    odom_msg.pose.pose.position.y = tmp(1);
                    odom_msg.pose.pose.position.z = tmp(2);
                    // orientation as B2 --> I0
                    tf::quaternionEigenToMsg(
                        Eigen::Quaterniond(rotIneriToIner0[1] * droneQuat[2].toRotationMatrix()), 
                        odom_msg.pose.pose.orientation
                    );
                    // linear velocity on I0 
                    tmp = rotIneriToIner0[1] * droneQuat[2].toRotationMatrix() * 
                        rotCmdToBody * droneTwist[2].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // angular velocity on body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    odom_msg.twist.twist.angular.x = tmp(0);
                    odom_msg.twist.twist.angular.y = tmp(1);
                    odom_msg.twist.twist.angular.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[2](2);
                    odom_2_pub.publish(odom_msg);
                }
                break;
            
            default:
                // std::cout << "========[formation debugger]========\ndrone id = "
                //           << ind << " do not have initial state!" << std::endl;
                break;
            }
        }
        else if (isExpMode){
            // publish drone states
            switch (ind) 
            {
            case 0:
                if (eskf_0_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d0StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_0_pub.publish(trajPt);
                }
                if (odom_0_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    odom_msg.pose.pose.position.x = dronePosition[0](0);
                    odom_msg.pose.pose.position.y = dronePosition[0](1);
                    odom_msg.pose.pose.position.z = dronePosition[0](2) + height_offset;
                    tf::quaternionEigenToMsg(
                        droneQuat[0], odom_msg.pose.pose.orientation
                    );
                    Eigen::Vector3d tmp = 
                        droneQuat[0].toRotationMatrix() * rotCmdToBody * droneTwist[0].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    odom_msg.twist.twist.angular.x = tmp(0);
                    odom_msg.twist.twist.angular.y = tmp(1);
                    odom_msg.twist.twist.angular.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[0](2);
                    odom_0_pub.publish(odom_msg);
                }
                break;

            case 1:
                if (eskf_1_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d1StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_1_pub.publish(trajPt);
                }
                if (odom_1_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    // position map to I0 frame
                    Eigen::Vector3d tmp = rotIneriToIner0[0] * Eigen::Vector3d(
                        dronePosition[1](0), 
                        dronePosition[1](1) + height_offset, 
                        dronePosition[1](2)
                    ) + posIner0ToIneri[0];
                    odom_msg.pose.pose.position.x = tmp(0);
                    odom_msg.pose.pose.position.y = tmp(1);
                    odom_msg.pose.pose.position.z = tmp(2);
                    // orientation as B1 --> I0
                    tf::quaternionEigenToMsg(
                        Eigen::Quaterniond(rotIneriToIner0[0] * droneQuat[1].toRotationMatrix()), 
                        odom_msg.pose.pose.orientation
                    );
                    // linear velocity on I0 
                    tmp = rotIneriToIner0[0] * droneQuat[1].toRotationMatrix() * 
                        rotCmdToBody * droneTwist[1].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // angular velocity on body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    odom_msg.twist.twist.angular.x = tmp(0);
                    odom_msg.twist.twist.angular.y = tmp(1);
                    odom_msg.twist.twist.angular.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[1](2);
                    odom_1_pub.publish(odom_msg);
                }
                break;

            case 2:
                if (eskf_2_pub.getNumSubscribers() > 0){
                    trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                    trajPt.time_from_start = ros::Time::now() - d2StartTime;
                    // pose
                    geometry_msgs::Transform geoTF_msg;
                    geoTF_msg.translation.x = dronePosition[ind](0);
                    geoTF_msg.translation.y = dronePosition[ind](1);
                    geoTF_msg.translation.z = dronePosition[ind](2);
                    tf::quaternionEigenToMsg(droneQuat[ind], geoTF_msg.rotation);
                    // twist
                    // velocity in inertial frame
                    geometry_msgs::Twist twist_msg;
                    Eigen::Vector3d tmp = 
                        droneQuat[ind].toRotationMatrix() * rotCmdToBody * droneTwist[ind].tail(3);
                    twist_msg.linear.x = tmp(0);
                    twist_msg.linear.y = tmp(1);
                    twist_msg.linear.z = tmp(2);
                    // angular velocity in body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);
                    twist_msg.angular.x = tmp(0);
                    twist_msg.angular.y = tmp(1);
                    twist_msg.angular.z = tmp(2);
                    
                    trajPt.velocities.emplace_back(twist_msg);
                    trajPt.transforms.emplace_back(geoTF_msg);
                    eskf_2_pub.publish(trajPt);
                }
                if (odom_2_pub.getNumSubscribers() > 0){
                    nav_msgs::Odometry odom_msg;
                    odom_msg.header.stamp = ros::Time::now();
                    odom_msg.header.frame_id = "I0";
                    // position map to I0 frame
                    Eigen::Vector3d tmp = rotIneriToIner0[1] * Eigen::Vector3d(
                        dronePosition[2](0),
                        dronePosition[2](1) + height_offset,
                        dronePosition[2](2)
                    ) + posIner0ToIneri[1];
                    odom_msg.pose.pose.position.x = tmp(0);
                    odom_msg.pose.pose.position.y = tmp(1);
                    odom_msg.pose.pose.position.z = tmp(2);
                    // orientation as B2 --> I0
                    tf::quaternionEigenToMsg(
                        Eigen::Quaterniond(rotIneriToIner0[1] * droneQuat[2].toRotationMatrix()), 
                        odom_msg.pose.pose.orientation
                    );
                    // linear velocity on I0 
                    tmp = rotIneriToIner0[1] * droneQuat[2].toRotationMatrix() * 
                        rotCmdToBody * droneTwist[2].tail(3);
                    odom_msg.twist.twist.linear.x = tmp(0);
                    odom_msg.twist.twist.linear.y = tmp(1);
                    odom_msg.twist.twist.linear.z = tmp(2);
                    // angular velocity on body frame
                    // tmp = -rotCmdToBody * droneTwist[ind].head(3);
                    // angular velocity frame as body frame
                    tmp = droneTwist[ind].head(3);                    
                    odom_msg.twist.twist.angular.x = tmp(0);
                    odom_msg.twist.twist.angular.y = tmp(1);
                    odom_msg.twist.twist.angular.z = tmp(2);
                    // odom_msg.twist.twist.angular.z = -droneTwist[2](2);
                    odom_2_pub.publish(odom_msg);
                }
                break;

            default:
                // debug
                ROS_INFO_STREAM(
                    "========[formation debugger]========\nNO DRONE ID MATCH: "
                    << ind << "!"
                );
            }
        }
    }
    /*
    @publishFaceState
        publish face state
    */
   void publishFaceState(){
        // check SOT
        if (!isExpMode && StartOfTest){
            if (eskf_init_face_pub.getNumSubscribers() > 0){
                tello_pub::SE3Pose SE3Pose_msg;
                SE3Pose_msg.header.stamp = ros::Time::now();
                SE3Pose_msg.tvec.x = posIner1ToInitFace(0);
                SE3Pose_msg.tvec.y = posIner1ToInitFace(1);
                SE3Pose_msg.tvec.z = posIner1ToInitFace(2);
                std::memcpy(
                    &SE3Pose_msg.rmat, rotInitFaceToIner1.data(), 9 * sizeof(double)
                );
                eskf_init_face_pub.publish(SE3Pose_msg);
            }
            if (eskf_face_pub.getNumSubscribers() > 0){
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - faceStartTime;
                // pose
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = facePosition(0);
                geoTF_msg.translation.y = facePosition(1);
                geoTF_msg.translation.z = facePosition(2);
                tf::quaternionEigenToMsg(faceQuat, geoTF_msg.rotation);
                // twist
                geometry_msgs::Twist twist_msg;
                // velocity in inertial frame
                Eigen::Vector3d tmp = faceQuat.toRotationMatrix() * faceTwist.tail(3);
                twist_msg.linear.x = tmp(0);
                twist_msg.linear.y = tmp(1);
                twist_msg.linear.z = tmp(2);
                twist_msg.angular.x = faceTwist(0);
                twist_msg.angular.y = faceTwist(1);
                twist_msg.angular.z = faceTwist(2);
                
                trajPt.transforms.emplace_back(geoTF_msg);
                trajPt.velocities.emplace_back(twist_msg);
                eskf_face_pub.publish(trajPt);
            }
            if (odom_face_pub.getNumSubscribers() > 0){
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = "I0";
                Eigen::Vector3d tmp = 
                    rotIneriToIner0[0] * 
                    (rotInitFaceToIner1 * facePosition + posIner1ToInitFace) + posIner0ToIneri[0];
                // position in I0
                odom_msg.pose.pose.position.x = tmp(0);
                odom_msg.pose.pose.position.y = tmp(1);
                odom_msg.pose.pose.position.z = tmp(2);
                // orientation F --> I0
                tf::quaternionEigenToMsg(
                    Eigen::Quaterniond(
                        rotIneriToIner0[0] * rotInitFaceToIner1 *
                        faceQuat.toRotationMatrix()
                    ),
                    odom_msg.pose.pose.orientation
                );
                // linear velocity in I0
                tmp = rotIneriToIner0[0] * rotInitFaceToIner1 * 
                    faceQuat.toRotationMatrix() * faceTwist.tail(3);
                odom_msg.twist.twist.linear.x = tmp(0);
                odom_msg.twist.twist.linear.y = tmp(1);
                odom_msg.twist.twist.linear.z = tmp(2);
                odom_msg.twist.twist.angular.x = faceTwist(0);
                odom_msg.twist.twist.angular.y = faceTwist(1);
                odom_msg.twist.twist.angular.z = faceTwist(2);
                odom_face_pub.publish(odom_msg);
            }
        }
        else if (isExpMode){
            // publish face state
            if (eskf_face_pub.getNumSubscribers() > 0){
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - faceStartTime;
                // pose
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = facePosition(0);
                geoTF_msg.translation.y = facePosition(1);
                geoTF_msg.translation.z = facePosition(2);
                tf::quaternionEigenToMsg(faceQuat, geoTF_msg.rotation);
                // twist
                geometry_msgs::Twist twist_msg;
                // velocity in inertial frame
                Eigen::Vector3d tmp = faceQuat.toRotationMatrix() * faceTwist.tail(3);
                twist_msg.linear.x = tmp(0);
                twist_msg.linear.y = tmp(1);
                twist_msg.linear.z = tmp(2);
                twist_msg.angular.x = faceTwist(0);
                twist_msg.angular.y = faceTwist(1);
                twist_msg.angular.z = faceTwist(2);
                
                trajPt.transforms.emplace_back(geoTF_msg);
                trajPt.velocities.emplace_back(twist_msg);
                eskf_face_pub.publish(trajPt);
            }
            if (odom_face_pub.getNumSubscribers() > 0){
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = "I0";
                Eigen::Vector3d tmp = 
                    rotIneriToIner0[0] * 
                    (rotInitFaceToIner1 * facePosition + posIner1ToInitFace) + posIner0ToIneri[0];
                // position in I0
                odom_msg.pose.pose.position.x = tmp(0);
                odom_msg.pose.pose.position.y = tmp(1);
                odom_msg.pose.pose.position.z = tmp(2);
                // orientation F --> I0
                tf::quaternionEigenToMsg(
                    Eigen::Quaterniond(
                        rotIneriToIner0[0] * rotInitFaceToIner1 *
                        faceQuat.toRotationMatrix()
                    ),
                    odom_msg.pose.pose.orientation
                );
                // linear velocity in I0
                tmp = rotIneriToIner0[0] * rotInitFaceToIner1 * 
                    faceQuat.toRotationMatrix() * faceTwist.tail(3);
                odom_msg.twist.twist.linear.x = tmp(0);
                odom_msg.twist.twist.linear.y = tmp(1);
                odom_msg.twist.twist.linear.z = tmp(2);
                odom_msg.twist.twist.angular.x = faceTwist(0);
                odom_msg.twist.twist.angular.y = faceTwist(1);
                odom_msg.twist.twist.angular.z = faceTwist(2);
                odom_face_pub.publish(odom_msg);
            }
        }
   }

public:
    /*
    @FormCtrlDebugger
        load initial states from the YAML file
    */
    explicit FormCtrlDebugger(ros::NodeHandle& nh_) {
        // get ros param
        ros_nodeName = ros::this_node::getName();
        nh_.getParam(ros_nodeName + "/human_mode", humanMode);
        nh_.getParam(ros_nodeName + "/height_offset", height_offset);
        nh_.getParam(ros_nodeName + "/no_roll_pitch", noRollPitch);
        
        // setting initial states from the yaml files
        try {
            // path to config
            std::string configPath = "/tello_pub/cfg/formCtrlDbgInit.yaml";
            nh_.getParam(ros_nodeName + "/init_cfg", configPath);
            YAML::Node config = YAML::LoadFile(configPath);
            
            // assign drone states
            YAML::Node drones[3] = {config["drone0"], config["drone1"], config["drone2"]};
            for (int i=0; i<3; i++){
                // debug
                std::cout << "i = " << i << std::endl;

                if (i != 0){ 
                    posIner0ToIneri[i-1] = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                        drones[i]["position_init"].as<std::vector<double>>().data()
                    );
                    rotIneriToIner0[i-1] = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                        drones[i]["orientation_init"].as<std::vector<double>>().data()
                    );
                    // debug
                    std::cout << "drone init state:\nposition:\n" << posIner0ToIneri[i-1] << std::endl
                              << "orientation:\n" << rotIneriToIner0[i-1] << std::endl;
                }
                dronePosition[i] = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                    drones[i]["position"].as<std::vector<double>>().data()
                );
                // orientation
                std::vector<double> temp = drones[i]["orientation"].as<std::vector<double>>();
                if (temp.size() == 9){
                    // assign a quaternion from rotation matrix
                    droneQuat[i] = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                        temp.data()
                    );
                }
                else if (temp.size() == 4){
                    // assign a quaternion from vec4d
                    Eigen::VectorXd tmp = Eigen::Map<Eigen::Matrix<double, 4, 1, Eigen::ColMajor>>(
                        temp.data()
                    );
                    droneQuat[i].w() = tmp(0);
                    droneQuat[i].vec() = tmp.tail(3);
                    droneQuat[i].normalize();
                }
                droneTwist[i] = Eigen::VectorXd::Zero(6);
                droneTwist[i].tail(3) = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                    drones[i]["velocity"].as<std::vector<double>>().data()
                );
                droneTwist[i].head(3) = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                    drones[i]["angular_velocity"].as<std::vector<double>>().data()
                );
                // debug
                std::cout << "drone states:\nposition:\n" << dronePosition[i] << std::endl
                          << "orientation:\n" << droneQuat[i].w() << std::endl << droneQuat[i].vec() << std::endl
                          << "Body to Inertial rotation matrix:\n" << droneQuat[i].toRotationMatrix() << std::endl
                          << "twist:\n" << droneTwist[i] << std::endl;
            }
            
            // assign human state
            YAML::Node human = config["human"];
            switch (humanMode)
            {
            case STAND_STILL:
                human = human["STAND_STILL"];
                break;
            
            case GO_STRAIGHT:
                human = human["GO_STRAIGHT"];
                break;
            
            case CONST_VEL_WALKING:
                human = human["CONST_VEL_WALKING"];
                break;
            
            case LOA_IS_FACING:
                human = human["LOA_IS_FACING"];
                break;

            default:
                // debug
                std::cout << "========[formation debugger]========\nwrong human mode: "
                          << humanMode << " !" << std::endl;
                break;
            }
            posIner1ToInitFace = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                human["position_init"].as<std::vector<double>>().data()
            );
            rotInitFaceToIner1 = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                human["orientation_init"].as<std::vector<double>>().data()
            );
            facePosition = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                human["position"].as<std::vector<double>>().data()
            );
            faceTwist = Eigen::VectorXd::Zero(6);
            faceTwist.tail(3) = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                human["velocity"].as<std::vector<double>>().data()
            );
            faceTwist.head(3) = Eigen::Map<Eigen::Matrix<double, 3, 1, Eigen::ColMajor>>(
                human["angular_velocity"].as<std::vector<double>>().data()
            );
            // assign quaterniond from vec4d
            Eigen::VectorXd tmp = Eigen::Map<Eigen::Matrix<double, 4, 1, Eigen::ColMajor>>(
                human["orientation"].as<std::vector<double>>().data()
            );
            faceQuat.w() = tmp(0);
            faceQuat.vec() = tmp.tail(3);
            faceQuat.normalize();
            // debug
            std::cout << "human state:\nposition:\n" << facePosition << std::endl
                      << "orientation:\n" << faceQuat.w() << std::endl << faceQuat.vec() << std::endl
                      << "twist:\n" << faceTwist << std::endl
                      << "initial position:\n" << posIner1ToInitFace << std::endl
                      << "initial orientation:\n" << rotInitFaceToIner1 << std::endl;
        }
        catch (...){
            // debug
            std::cout << "========[formation debugger]========\nLoading YAML Error! Set as Default!" << std::endl;
            // setting default states
            for (int i=0; i<3; i++){
                droneTwist[i] = Eigen::VectorXd::Zero(6);
                dronePosition[i] = Eigen::Vector3d::Zero();
                droneQuat[i] = Eigen::Quaterniond::Identity();
            }
            for (int i=0; i<2; i++){
                posIner0ToIneri[i] = Eigen::Vector3d::Zero();
                rotIneriToIner0[i] = Eigen::Matrix3d::Identity();
            }
            posIner1ToInitFace = Eigen::Vector3d::Zero();
            rotInitFaceToIner1 = Eigen::Matrix3d::Zero();
            facePosition = Eigen::Vector3d::Zero();
            faceTwist = Eigen::VectorXd::Zero(6);
            faceQuat = Eigen::Quaterniond::Identity();
        }
        // sub and pub
        exp_mode_sub = nh_.subscribe(
            "/exp_mode", queue_size, 
            &FormCtrlDebugger::exp_mode_cb, this
        );
        SOT_sub = nh_.subscribe(
            "/SOT", queue_size, 
            &FormCtrlDebugger::telloSOTCb, this
        );
        cmd_0_sub = nh_.subscribe(
            droneName[0] + "/cmd_vel", queue_size,
            &FormCtrlDebugger::cmd_vel_0_cb, this
        );
        cmd_1_sub = nh_.subscribe(
            droneName[1] + "/cmd_vel", queue_size,
            &FormCtrlDebugger::cmd_vel_1_cb, this
        );
        cmd_2_sub = nh_.subscribe(
            droneName[2] + "/cmd_vel", queue_size,
            &FormCtrlDebugger::cmd_vel_2_cb, this
        );
        eskf_0_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            droneName[0] + "/eskf_state_full", queue_size
        );
        eskf_1_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            droneName[1] + "/eskf_state_full", queue_size
        );
        eskf_2_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            droneName[2] + "/eskf_state_full", queue_size
        );
        eskf_init_1_pub = nh_.advertise<tello_pub::SE3Pose>(
            droneName[1] + "/eskf_init", queue_size
        );
        eskf_init_2_pub = nh_.advertise<tello_pub::SE3Pose>(
            droneName[2] + "/eskf_init", queue_size
        );
        eskf_face_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/face_eskf", queue_size
        );
        eskf_init_face_pub = nh_.advertise<tello_pub::SE3Pose>(
            "/face_init", queue_size
        );
        odom_0_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName[0] + "/eskf_odom", queue_size
        );
        odom_1_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName[1] + "/eskf_odom", queue_size
        );
        odom_2_pub = nh_.advertise<nav_msgs::Odometry>(
            droneName[2] + "/eskf_odom", queue_size
        );
        odom_face_pub = nh_.advertise<nav_msgs::Odometry>(
            "/face_odom", queue_size
        );
        // timer
        drone0Timer = nh_.createTimer(
            ros::Duration(t_drones), &FormCtrlDebugger::d0TimerCb,
            this, false, false
        );
        drone1Timer = nh_.createTimer(
            ros::Duration(t_drones), &FormCtrlDebugger::d1TimerCb,
            this, false, false
        );
        drone2Timer = nh_.createTimer(
            ros::Duration(t_drones), &FormCtrlDebugger::d2TimerCb,
            this, false, false
        );
        faceTimer = nh_.createTimer(
            ros::Duration(t_face), &FormCtrlDebugger::faceTimerCb,
            this, false, false
        );
        // constant transformation
        // rotCmdToBody << 0.0, -1.0, 0.0,
        //                 1.0, 0.0, 0.0,
        //                 0.0, 0.0, 1.0;
        // use sensing frame as body frame
        rotCmdToBody << 0.0, 1.0, 0.0,
                        1.0, 0.0, 0.0,
                        0.0, 0.0, -1.0;

        // debug
        std::cout << "trans I0 to IF in I0:\n" 
                  << rotIneriToIner0[0] * posIner1ToInitFace + posIner0ToIneri[0]
                  << std::endl;
    }
};


/*
@SingleSE3Ctrller
    test the convergence of 1st-order system of single agent
    without the complex kinematics
*/
class SingleSE3Ctrller
{
    // ros-dep
    ros::Subscriber exp_mode_sub, SOT_sub;
    ros::Publisher cmd_pub, ctrl_debug_pub,
                   state_pub, state_init_pub;
    ros::Timer agentTimer;
    ros::Time startTime;

    // param
    int queue_size = 1;
    double t_smpl = 0.01;
    bool StartOfTest = false;
    bool isExpMode = false;

    // init
    Eigen::Matrix3d rotBodyInitToIner;
    Eigen::Vector3d posInerToBodyInit;
    
    // state
    Eigen::VectorXd posVel;
    Eigen::Vector3d angVel;
    Eigen::Quaterniond quat;

    // desired state
    Eigen::Matrix3d rotBodyDesToIner;
    Eigen::Vector3d posInerToBodyDes;

    // ctrl
    Eigen::VectorXd twistCmd;
    Eigen::Matrix4d grad;
    Eigen::MatrixXd twistGain;
    double errFunc;


    // functions
    /*
    @SOT_cb
        start of test
    */
    void SOT_cb(std_msgs::Empty const& msg){
        if (!StartOfTest){
            agentTimer.start();
            startTime = ros::Time::now();
            StartOfTest = true;
            // debug
            std::cout << "========[single SE3 ctrl]========\nreceived SOT!" << std::endl;
        }
    }
    /*
    @exp_mode_cb

    */
    void exp_mode_cb(std_msgs::Empty const& msg){
        if (!isExpMode){
            isExpMode = true;
            // debug
            std::cout << "========[single SE3 ctrl]========\nreceived exp_mode!" << std::endl;
        }
    }
    /*
    @timerCb
        update the state with twist control
        publish the state and initial
    */
    void timerCb(ros::TimerEvent const& ev){
        stateUpdate();
        publishState();
    }
    /*
    @stateUpdate
        calculate error config
        find the twist feedback ctrl
        update the state
    */
    void stateUpdate(){
        if (isExpMode && StartOfTest){
            // error config
            Eigen::Matrix3d errRmat = 
                rotBodyDesToIner.transpose() * quat.toRotationMatrix();
            Eigen::Vector3d errTvec = rotBodyDesToIner.transpose() * (
                posVel.head(3) - posInerToBodyDes
            );
            // debug
            std::cout << "========[single SE3 ctrl]========\n"
                      << "rot Body to Body des:\n" << errRmat << std::endl
                      << "trans Body des to Body in Body des:\n" << errTvec << std::endl;
            // compute twist
            Eigen::AngleAxisd errRvec;
            if (errRmat.isIdentity()) {
                // debug
                std::cout << "========[single SE3 ctrl]========\n"
                          << "ROTATION ERROR IS IDENTITY:\n" << errRmat << std::endl;
                errRvec = quat.toRotationMatrix();
                errRvec.angle() = 0.0;
                grad.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
                grad.block<3, 1>(0, 3) = errRmat * errTvec;
                twistCmd.head(3) = Eigen::Vector3d::Zero();
                twistCmd.tail(3) = errTvec;
            }
            else {
                errRvec = errRmat;
                twistCmd.head(3) = errRvec.angle() * errRvec.axis();
                twistCmd.tail(3) = leftJacobInvFromRvec(
                    errRvec.axis(), errRvec.angle()
                ) * errTvec;
                grad.block<3, 3>(0, 0) = errRmat * vector3dToSkeySym(
                    errRvec.angle() * errRvec.axis()
                );
                grad.block<3, 1>(0, 3) = errRmat * twistCmd.tail(3);
            }
            errFunc = twistCmd.norm();
            // debug
            std::cout << "========[single SE3 ctrl]========\n"
                      << "se3 relative pose:\n" << twistCmd << std::endl;
            // apply K = -I_6
            twistCmd = twistGain * twistCmd;
            // debug
            std::cout << "inv left jacob:\n" 
                      << leftJacobInvFromRvec(errRvec.axis(), errRvec.angle()) 
                      << std::endl
                      << "rvec axis:\n" << errRvec.axis() << std::endl
                      << "rvec angle:\n" << errRvec.angle() << std::endl
                      << "twist cmd:\n" << twistCmd << std::endl;
            // assign the twist cmd
            posVel.tail(3) = quat.toRotationMatrix() * twistCmd.tail(3);
            angVel = twistCmd.head(3);
            
            // debug
            std::cout << "angular velocity in body frame outer product position in body frame:\n" 
                      << vector3dToSkeySym(twistCmd.head(3)) * quat.toRotationMatrix() * posVel.head(3)
                      << std::endl
                      << "timer derivative of translation from inertial to body in body:\n"
                      << twistCmd.tail(3) - vector3dToSkeySym(twistCmd.head(3)) * quat.toRotationMatrix() * posVel.head(3)
                      << std::endl;


            // update state
            Eigen::Quaterniond quat_angvel = vector3dToQuaterniond(
                angVel * t_smpl
            );
            posVel.head(3) += posVel.tail(3) * t_smpl;
            quat = quat * quat_angvel;
        }
    }
    /*
    @publishState

    */
    void publishState(){
        if (!isExpMode && StartOfTest){
            // publish state, init, cmd
            if (state_init_pub.getNumSubscribers() > 0){
                tello_pub::SE3Pose SE3Pose_msg;
                SE3Pose_msg.header.stamp = ros::Time::now();
                SE3Pose_msg.tvec.x = posInerToBodyInit(0);
                SE3Pose_msg.tvec.y = posInerToBodyInit(1);
                SE3Pose_msg.tvec.z = posInerToBodyInit(2);
                std::memcpy(
                    &SE3Pose_msg.rmat, rotBodyInitToIner.data(), 9 * sizeof(double)
                );
                state_init_pub.publish(SE3Pose_msg);
            }
            if (state_pub.getNumSubscribers() > 0){
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - startTime;
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = posVel(0);
                geoTF_msg.translation.y = posVel(1);
                geoTF_msg.translation.z = posVel(2);
                tf::quaternionEigenToMsg(quat, geoTF_msg.rotation);
                geometry_msgs::Twist twist_msg;
                twist_msg.linear.x = posVel(3);
                twist_msg.linear.y = posVel(4);
                twist_msg.linear.z = posVel(5);
                twist_msg.angular.x = angVel(0);
                twist_msg.angular.y = angVel(1);
                twist_msg.angular.z = angVel(2);
                trajPt.transforms.emplace_back(geoTF_msg);
                trajPt.velocities.emplace_back(twist_msg);
                state_pub.publish(trajPt);
            }
            // publish zero velocity
            geometry_msgs::Twist zeroTwist;
            cmd_pub.publish(zeroTwist);
        }
        else if (isExpMode && StartOfTest){
            // publish state, cmd, debug msg
            if (state_pub.getNumSubscribers() > 0){
                trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
                trajPt.time_from_start = ros::Time::now() - startTime;
                geometry_msgs::Transform geoTF_msg;
                geoTF_msg.translation.x = posVel(0);
                geoTF_msg.translation.y = posVel(1);
                geoTF_msg.translation.z = posVel(2);
                tf::quaternionEigenToMsg(quat, geoTF_msg.rotation);
                geometry_msgs::Twist twist_msg;
                twist_msg.linear.x = posVel(3);
                twist_msg.linear.y = posVel(4);
                twist_msg.linear.z = posVel(5);
                twist_msg.angular.x = angVel(0);
                twist_msg.angular.y = angVel(1);
                twist_msg.angular.z = angVel(2);
                trajPt.transforms.emplace_back(geoTF_msg);
                trajPt.velocities.emplace_back(twist_msg);
                state_pub.publish(trajPt);
            }
            if (ctrl_debug_pub.getNumSubscribers() > 0){
                tello_pub::GeometricCtrlDebug debug_msg;
                debug_msg.header.stamp = ros::Time::now();
                debug_msg.cost_funcs[0] = errFunc;
                std::memcpy(
                    &debug_msg.grad_0, grad.block<3, 4>(0, 0).data(), 12 * sizeof(double)
                );
                ctrl_debug_pub.publish(debug_msg);
            }
            geometry_msgs::Twist cmd_msg;
            cmd_msg.linear.x = twistCmd(3);
            cmd_msg.linear.y = twistCmd(4);
            cmd_msg.linear.z = twistCmd(5);
            cmd_msg.angular.z = twistCmd(2);
            cmd_pub.publish(cmd_msg);
        }
    }

public:
    /*
    @SingleSE3Ctrller

    */
    explicit SingleSE3Ctrller(ros::NodeHandle& nh_){
        // ros-dep
        std::string ros_nodeName = ros::this_node::getName();

        exp_mode_sub = nh_.subscribe(
            "/exp_mode", queue_size,
            &SingleSE3Ctrller::exp_mode_cb, this
        );
        SOT_sub = nh_.subscribe(
            "/SOT", queue_size,
            &SingleSE3Ctrller::SOT_cb, this
        );
        cmd_pub = nh_.advertise<geometry_msgs::Twist>(
            "/cmd_vel", queue_size
        );
        state_init_pub = nh_.advertise<tello_pub::SE3Pose>(
            "/eskf_init", queue_size
        );
        state_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/eskf_state_full", queue_size
        );
        ctrl_debug_pub = nh_.advertise<tello_pub::GeometricCtrlDebug>(
            "/cost_func_and_grad", queue_size
        );
        agentTimer = nh_.createTimer(
            ros::Duration(t_smpl), &SingleSE3Ctrller::timerCb, this, false, false
        );
        // initialize
        posInerToBodyInit = Eigen::Vector3d::Zero();
        rotBodyInitToIner = Eigen::Matrix3d::Identity();
        
        posVel = Eigen::VectorXd::Zero(6);
        angVel = Eigen::Vector3d::Zero();
        quat = Eigen::Quaterniond::Identity();

        twistCmd = Eigen::VectorXd::Zero(6);
        grad = Eigen::Matrix4d::Zero();
        twistGain = -Eigen::MatrixXd::Identity(6, 6);
        // twistGain = -0.1 * Eigen::MatrixXd::Identity(6, 6);

        // desired state 
        posInerToBodyDes << 10.0, 0.0, 0.0;
        rotBodyDesToIner << std::cos(M_PI/2), -std::sin(M_PI/2), 0.0,
                            std::sin(M_PI/2), std::cos(M_PI/2), 0.0,
                            0.0, 0.0, 1.0;
        // debug
        std::cout << "position desired:\n" << posInerToBodyDes << std::endl
                  << "rotation desired:\n" << rotBodyDesToIner << std::endl;
    }

};


/*
@main
*/
int main(int argc, char **argv){

    ros::init(argc, argv, "gm_form_ctrller_dbger", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    bool testSingle = false;
    nh.getParam(ros::this_node::getName() + "/debug_single", testSingle);

    // debug
    std::cout << testSingle << std::endl;


    if (testSingle){
        SingleSE3Ctrller dbgger(nh);
        ros::spin();
    }
    else {
        FormCtrlDebugger dbgger(nh);
        ros::spin();
    }

    // ros::spin();

    return 0;
}