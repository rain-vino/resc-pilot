//
// Created by Zhaohong Liu on 24-9-18.
//

#include <ros/ros.h>

#include "rviz_utils/PoseVisualization.h"

using Ptr = std::unique_ptr<MarkerHandler>;
using RP = RvizParams;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_visualization_node");
    ros::NodeHandle nh("~");

    Ptr pose_marker_handler_ptr = std::make_unique<PoseMarkerHandler>(nh, RP::world_frame_id_);
    pose_marker_handler_ptr->init();

    auto rate = ros::Rate(RP::rviz_frequency_);

    while (ros::ok()) {
        pose_marker_handler_ptr->publish();

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
