// ros
# include <ros/ros.h>
# include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
# include <std_msgs/Empty.h>
# include <tf/transform_datatypes.h>

// Eigen
# include <eigen3/Eigen/Dense>
// # include <eigen3/Eigen/Core>
// # include <eigen3/Eigen/Geometry>
# include <tf_conversions/tf_eigen.h>
# include <eigen_conversions/eigen_msg.h>

// self-header
# include <tello_pub/math_utils.h>

/*
@PubTest
    compute eigen stuff
    publish trajectory msg
*/
typedef class PubTest
{
    // ros
    ros::Publisher traj_pub;
    ros::Subscriber SOT_sub, EOT_sub;
    ros::Timer trajTimer;
    ros::Time startTime;

    // Eigen
    Eigen::VectorXd state;      // [p v]
    Eigen::Vector3d accInput, angVelInput;
    Eigen::Quaterniond quat;
    // Eigen::MatrixXd stateTrans;

    // param
    bool startFlag = false;
    double t_smpl = 0.01;
    int vb_counter = 0;
    int id_;

    void SOTCallback(std_msgs::Empty const& msg){
        // call the timer start
        if (!startFlag){
            trajTimer.start();
            startFlag = true;
            startTime = ros::Time::now();
        }
    }
    void EOTcallback(std_msgs::Empty const& msg){
        // call the timer stop
        trajTimer.stop();
        startFlag = false;
    }
    /*
    @timerCallback
        predict the trajectory and publish
        p = p + vdt + 0.5a dt^2
        v = v + adt
    */
    void timerCallback(ros::TimerEvent const& ev){
        // predict
        state.head(3) += state.tail(3)*t_smpl + 0.5 * std::pow(t_smpl, 2) * accInput;
        state.tail(3) += t_smpl * accInput;
        Eigen::Quaterniond quat_incre = vector3dToQuaterniond(angVelInput*t_smpl);
        quat = quat * quat_incre;

        // publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajPt;
        trajPt.time_from_start = ros::Time::now() - startTime;
        geometry_msgs::Transform geoTF_msg;
        geoTF_msg.translation.x = state(0);
        geoTF_msg.translation.y = state(1);
        geoTF_msg.translation.z = state(2);
        tf::quaternionEigenToMsg(quat, geoTF_msg.rotation);
        geometry_msgs::Twist acc_msg, twist_msg;
        acc_msg.linear.x = accInput(0);
        acc_msg.linear.y = accInput(1);
        acc_msg.linear.z = accInput(2);
        twist_msg.linear.x = state(3);
        twist_msg.linear.y = state(4);
        twist_msg.linear.z = state(5);
        twist_msg.angular.x = angVelInput(0);
        twist_msg.angular.y = angVelInput(1);
        twist_msg.angular.z = angVelInput(2);
        trajPt.accelerations.emplace_back(acc_msg);
        trajPt.velocities.emplace_back(twist_msg);
        trajPt.transforms.emplace_back(geoTF_msg);
        traj_pub.publish(trajPt);
        
        // debug
        vb_counter ++;
        if (vb_counter == 20){
            ROS_INFO_STREAM(
                "========[id " << id_ << "]========\n" << trajPt
            );
            vb_counter = 0;
        }
    }

public:
    PubTest(){}
    ~PubTest(){}
    explicit PubTest(ros::NodeHandle& nh_, int id){
        // ros
        SOT_sub = nh_.subscribe("/SOT", 1, &PubTest::SOTCallback, this);
        EOT_sub = nh_.subscribe("/EOT", 1, &PubTest::EOTcallback, this);
        traj_pub = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
            "/traj_pts", 1
        );
        trajTimer = nh_.createTimer(
            ros::Duration(t_smpl), &PubTest::timerCallback, this, false, false
        );
        // trajTimer.stop();
        // parameter initialize
        id_ = id;
        state = Eigen::VectorXd::Zero(6);
        quat = Eigen::Quaterniond::Identity();
        accInput = Eigen::Vector3d::Random() * 0.1;         // random acc ranges [-0.1, 0.1]
        angVelInput = Eigen::Vector3d::Random() * 0.01;     // random angvel ranges [-0.01, 0.01]
        
        // debug
        std::cout << "constructor " << id << std::endl;
    }
};

class ThreeAgent
{
    PubTest t0, t1, t2;
public:
    ThreeAgent(){}
    ~ThreeAgent(){}
    explicit ThreeAgent(ros::NodeHandle& nh_) : t0(nh_, 0), t1(nh_, 1), t2(nh_, 2)
    {
        std::cout << "wrapper function of the PubTest class." << std::endl;
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "single_spinner_test");
    ros::NodeHandle nh;
    
    // get node name
    bool use_spin = false;
    int num_pub = 1;
    int num_thread = 1;
    std::string ros_nodeName = ros::this_node::getName();
    nh.getParam(ros_nodeName + "/use_spin", use_spin);
    nh.getParam(ros_nodeName + "/num_pub", num_pub);
    nh.getParam(ros_nodeName + "/num_thread", num_thread);
    
    // initialize the obj arr
    // PubTest tester[num_pub];
    // std::vector<PubTest> testers;

    // PubTest test0(nh, 0);
    // PubTest test1(nh, 1);
    // PubTest test2(nh, 2);

    ThreeAgent three_agent(nh);


    // testing conidition 
    if (use_spin) {
        // assign the obj
        // for (int i=0; i<num_pub; i++){
            // testers.emplace_back(PubTest(nh, i));
            // tester[i] = PubTest(nh, i);
        // }
        ros::spin();
    }
    else {
        ros::AsyncSpinner asynSpinner(num_thread);
        asynSpinner.start();
        // assign the obj
        // for (int i=0; i<num_pub; i++){
            // testers.emplace_back(PubTest(nh, i));
            // tester[i] = PubTest(nh, i);
        // }
        ros::waitForShutdown();
    }
    // return 0;
}
