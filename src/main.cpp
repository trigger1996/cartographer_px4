#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <mavros/mavros.h>
#include <mavros_msgs/State.h>

using namespace std;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped pose_2d;
bool is_pos_2d_updated = false;
void pos_2d_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_2d = *msg;
    is_pos_2d_updated = true;
}

sensor_msgs::Imu current_imu;
bool is_imu_updated = false;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    current_imu = *msg;
    is_imu_updated = true;
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "robot_pose_integrator");
    ros::NodeHandle nh, private_nh("~");
    ros::Rate rate(100.0);

    // Get node parameters
    static int         is_publish_tf, is_integrate_imu;
    static std::string self_frame,    child_frame;

    private_nh.param("is_publish_tf",    is_publish_tf,    0);
    private_nh.param("is_integrate_imu", is_integrate_imu, 1);
    private_nh.param<std::string>("self_frame",       self_frame,       "odom");
    private_nh.param<std::string>("child_frame",      child_frame,      "vision_estimate");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/robot_pose", 50, pos_2d_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/imu", 100, imu_cb);

    geometry_msgs::PoseStamped vision_pose;
    ros::Publisher vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>
               ("/mavros/vision_pose/pose", 25);

    tf::TransformBroadcaster mavros_laser_tf_pub;
    tf::Transform mavros_laser_tf;

//    while (ros::ok() && !current_state.connected) {
//        for (int i = 0; i < 1000; i++) {
//            ROS_INFO("FCU not connected ...");

//            ros::spinOnce();
//            rate.sleep();
//        }
//    }

    ROS_INFO("FCU connected ...");
    while (ros::ok()) {
        if (is_pos_2d_updated) {
            vision_pose.header.stamp = ros::Time::now();
            vision_pose.header.frame_id = self_frame;

            // WATCH OUT FOR orientation
            vision_pose.pose.position.x = -pose_2d.pose.position.x;
            vision_pose.pose.position.y = -pose_2d.pose.position.y;
            vision_pose.pose.position.z =  pose_2d.pose.position.z;

            if (is_integrate_imu) {
                vision_pose.pose.orientation = current_imu.orientation;
            }
            vision_pose_pub.publish(vision_pose);


            if (is_publish_tf) {
                mavros_laser_tf.setOrigin( tf::Vector3(-pose_2d.pose.position.x,
                                                       -pose_2d.pose.position.y,
                                                        pose_2d.pose.position.z) );
                if (is_integrate_imu) {
                    tf::Quaternion q;
                    q.setW(vision_pose.pose.orientation.w);
                    q.setX(vision_pose.pose.orientation.x);
                    q.setY(vision_pose.pose.orientation.y);
                    q.setZ(vision_pose.pose.orientation.z);

                    mavros_laser_tf.setRotation(q);
                }
                mavros_laser_tf_pub.sendTransform(tf::StampedTransform(mavros_laser_tf, ros::Time::now(), self_frame, child_frame));
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
    return 0;
}

