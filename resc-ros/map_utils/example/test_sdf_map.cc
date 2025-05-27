//
// Created by Zhaohong Liu on 24-12-22.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "map_utils/SDFMap.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sdf_map_node");
    ros::NodeHandle nh;

    ros::Publisher fake_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    SDFMap sdf_map;
    sdf_map.initMap(nh);

    auto rate = ros::Rate(10);
    while (ros::ok()) {
        pose.header.stamp = ros::Time::now();
        fake_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}