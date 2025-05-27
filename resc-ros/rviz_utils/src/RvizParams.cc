//
// Created by Zhaohong Liu on 24-9-19.
//

#include "rviz_utils/RvizParams.h"

// predefined values
std::string RvizParams::pose_marker_pub_topic_ = "/pose_marker";
// ref: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/uav_simulator/Utils/odom_visualization/meshes
std::string RvizParams::mesh_resource_path_ = "package://rviz_utils/meshes/hummingbird.mesh";
std::string RvizParams::world_frame_id_ = "world";
