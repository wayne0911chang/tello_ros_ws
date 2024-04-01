// testing script for circular motion
# include <ros/ros.h>
# include <sensor_msgs/Imu.h>
# include <nav_msgs/Odometry.h>
# include <std_msgs/Empty.h>
# include <geometry_msgs/Vector3.h>

# include <eigen3/Eigen/Dense>
# include <eigen3/Eigen/Core>
# include <eigen3/Eigen/Geometry>
# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

/*
@SignalGen
    create desired signal with timer
*/
class SignalGen
{
    ros::Timer signalTimer;
    ros::Time startTime;
    ros::Publisher odom_pub, imu_pub;
    ros::Subscriber SOT_sub;

    bool noiseFlag = false;
    int queue_size = 1;
    bool startFlag = false;

    double stdAcc = 0.001;
    double stdAngVel = 0.001;
    double stdVel = 0.01;

    double period = 60;
    double yawRate = 2 * M_PI / period;

    void SOTCallback(std_msgs::Empty const& msg);
    void timerCB(ros::TimerEvent const& ev);

    Eigen::Vector3d angVel = Eigen::Vector3d::Zero();
    Eigen::Vector3d linAcc = Eigen::Vector3d::Zero();
    Eigen::Vector3d nAngVel, nLinAcc, nLinVel;

    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

public:
    SignalGen(){};
    ~SignalGen(){};
    explicit SignalGen(
        ros::NodeHandle& nh_, std::string ns, bool flag
    );

    
};
/*
@SignalGen
    give namespace and noise flag
*/
SignalGen::SignalGen(
    ros::NodeHandle& nh_, std::string ns, bool flag
){
    SOT_sub = nh_.subscribe(
        "/SOT", queue_size, &SignalGen::SOTCallback, this
    );
    odom_pub = nh_.advertise<nav_msgs::Odometry>(
        ns + "/odom", queue_size
    );
    imu_pub = nh_.advertise<sensor_msgs::Imu>(
        ns + "/imu", queue_size
    );
    signalTimer = nh_.createTimer(
        ros::Duration(0.05), &SignalGen::timerCB, this, false
    );

    noiseFlag = flag;
    
    // setting the imu reading with unit G
    linAcc(2) = -1.0;
    angVel(2) = yawRate;

    // initialize the noise
    nAngVel = nLinAcc = nLinVel = Eigen::Vector3d::Zero();
}
/*
SOTCallback
*/
void SignalGen::SOTCallback(std_msgs::Empty const& msg){
    if (!startFlag) {
        startTime = ros::Time::now();
        signalTimer.start();
        startFlag = true;
        ROS_INFO_STREAM("started");
    }
}
/*
@timerCB
    publish numerical result of 
*/
void SignalGen::timerCB(ros::TimerEvent const& ev){
    if (startFlag) {
        // setting the noise
        if (noiseFlag) {
            double rand_ = (double) std::rand() / (RAND_MAX + 1.0);
            nLinAcc(0) =  stdAcc * rand_;
            nLinAcc(1) =  stdAcc * rand_;
            nLinAcc(2) =  stdAcc * rand_;
            nAngVel(0) = stdAngVel * rand_;
            nAngVel(1) = stdAngVel * rand_;
            nAngVel(2) = stdAngVel * rand_;
            nLinVel(0) = stdVel * rand_;
            nLinVel(1) = stdVel * rand_;
            nLinVel(2) = stdVel * rand_;
        }
        // check subscribers
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.twist.twist.linear.x = 0.0 + nLinVel(0);
        odom_msg.twist.twist.linear.y = 0.0 + nLinVel(1);
        odom_msg.twist.twist.linear.z = 0.0 + nLinVel(2);
        odom_msg.twist.twist.angular.x = angVel(0) + nAngVel(0);
        odom_msg.twist.twist.angular.y = angVel(1) + nAngVel(1);
        odom_msg.twist.twist.angular.z = angVel(2) + nAngVel(2);

        odom_pub.publish(odom_msg);
        
        // check subscribers
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.linear_acceleration.x = linAcc(0) + nLinAcc(0);
        imu_msg.linear_acceleration.y = linAcc(1) + nLinAcc(1);
        imu_msg.linear_acceleration.z = linAcc(2) + nLinAcc(2);
        imu_msg.angular_velocity.x = angVel(0) + nAngVel(0);
        imu_msg.angular_velocity.y = angVel(1) + nAngVel(1);
        imu_msg.angular_velocity.z = angVel(2) + nAngVel(2);
        
        Eigen::Vector3d rvec = 
            angVel * (ros::Time::now().toSec() - startTime.toSec());
        Eigen::Vector3d axis = rvec / rvec.norm();
        Eigen::Quaterniond q;
        q.w() = std::cos(rvec.norm() / 2.0f);
        q.vec() = std::sin(rvec.norm() / 2.0f) * axis;
        tf::quaternionEigenToMsg(quat * q, imu_msg.orientation);

        imu_pub.publish(imu_msg);
    }
    else {
        // publish 0
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = startTime;
        odom_pub.publish(odom_msg);

        // publish identity
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = startTime;
        imu_msg.linear_acceleration.z = -1.0;
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        tf::quaternionEigenToMsg(q, imu_msg.orientation);

        imu_pub.publish(imu_msg);
    }
}




int main(int argc, char **argv){
    // testing
    ros::init(argc, argv, "circular_imu_odom_test", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    SignalGen sg(nh, "/tello_0", false);
    
    ros::spin();
}