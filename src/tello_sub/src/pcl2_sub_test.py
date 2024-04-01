#!/usr/bin/env python3
import rospy;
import numpy as np;
from nav_msgs.msg import Odometry;
from sensor_msgs.msg import PointCloud2;
from sensor_msgs import point_cloud2;

class pclSub(object):
    def __init__(self, ros_ns) -> None:
        # debug
        print(ros_ns)

        self.sub = rospy.Subscriber("/VLP16/pointcloud", PointCloud2, self.callback, queue_size=1);
        self.odom_pub = rospy.Publisher(ros_ns + "odom_lidar", Odometry, queue_size=1)
        
        self.LiDAR_height = 1.0;
        # self.LiDAR_height = 1.013;              # tello_1 circular traj
        self.map_frame_id = "eskf_odom";

        # self.p_bias = np.zeros((3,));
        self.p_bias = np.zeros((4,));

        self.drone_pt_count = 0;



    def callback(self, msg):
        assert isinstance(msg, PointCloud2);

        pts_stamped = point_cloud2.read_points_list(
            msg, field_names=("x", "y", "z", "adjustedtime")
        );
        
        if pts_stamped:
            # store the bias as first pcd
            self.drone_pt_count += 1;
            
            # p_com = np.array([0.0, 0.0, 0.0]);
            p_com_stamped = np.array([0.0, 0.0, 0.0, 0.0]);
            
            # debug
            # print(len(pts));

            for point in pts_stamped:
                # print(point[0], point[1], point[2]);
                # p_com[0] += point[0];
                # p_com[1] += point[1];
                # p_com[2] += point[2];
                
                p_com_stamped[0] += point[0];
                p_com_stamped[1] += point[1];
                p_com_stamped[2] += point[2];
                p_com_stamped[3] += point[3] / 1e6;

            # p_com = p_com / np.float64(len(pts_stamped));
            p_com_stamped = p_com_stamped / np.float64(len(pts_stamped));

            # print(p_com);

            # add the LiDAR height
            # p_com[2] += self.LiDAR_height;
            p_com_stamped[2] += self.LiDAR_height;

            # store the bias as first pcd com
            if self.drone_pt_count == 1:
                # self.p_bias = p_com;
                self.p_bias = p_com_stamped;
                print("bias: ", self.p_bias);
            
            # remove pcd bias from LiDAR
            if self.drone_pt_count >= 1:
                # p_com = p_com - self.p_bias;
                p_com_stamped = p_com_stamped - self.p_bias;
                # print("bias: ", self.p_bias);

            odom_msg = Odometry();
            odom_msg.header.frame_id = self.map_frame_id;
            # odom_msg.header.stamp = rospy.Time.now();
            # odom_msg.pose.pose.position.x = p_com[0];
            # odom_msg.pose.pose.position.y = p_com[1];
            # odom_msg.pose.pose.position.z = p_com[2];

            odom_msg.header.stamp = rospy.rostime.Time.from_sec(p_com_stamped[3]);
            odom_msg.pose.pose.position.x = p_com_stamped[0];
            odom_msg.pose.pose.position.y = p_com_stamped[1];
            odom_msg.pose.pose.position.z = p_com_stamped[2];
            
            print(p_com_stamped);

            # odom_msg.pose.pose.position.x = pts[0][0];
            # odom_msg.pose.pose.position.y = pts[0][1];
            # odom_msg.pose.pose.position.z = pts[0][2];

            self.odom_pub.publish(odom_msg);



if __name__ == '__main__':
    rospy.init_node("pcl_to_odom");

    # ns = "/tello_1";
    # ns = "/tello_0";
    ns = rospy.get_namespace();
    pclSub(ns);

    rospy.spin();