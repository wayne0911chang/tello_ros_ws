#ifndef ROS_EIGEN_DEP_H
#define ROS_EIGEN_DEP_H

// ros-dep
# include <ros/ros.h>
# include <tf/tf.h>
# include <std_msgs/Empty.h>
# include <std_msgs/Float64MultiArray.h>
# include <geometry_msgs/PoseArray.h>
# include <geometry_msgs/Pose.h>
# include <geometry_msgs/Accel.h>
# include <nav_msgs/Odometry.h>
# include <sensor_msgs/Imu.h>
# include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
# include <tf/transform_datatypes.h>

// Eigen
# include <eigen3/Eigen/Dense>
// # include <eigen3/Eigen/Core>        --> in math utils
// # include <eigen3/Eigen/Geometry>    --> in math utils
# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

#endif