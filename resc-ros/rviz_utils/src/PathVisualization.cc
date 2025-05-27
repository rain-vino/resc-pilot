//
// Created by Zhaohong Liu on 24-10-19.
//

#include "rviz_utils/PathVisualization.h"

void PathVisualization::updateMarker() {
    marker_.header.stamp = ros::Time::now();
}

void PathVisualization::init() {
    initialized_ = true;
    setCommonMarkerProperties(marker_);

    marker_.type = visualization_msgs::Marker::POINTS;
    marker_.action = visualization_msgs::Marker::ADD;
    marker_.ns = "discrete path";
    marker_.id = 0;
    marker_.color.r = VC::RED.r;
    marker_.color.g = VC::RED.g;
    marker_.color.b = VC::RED.b;
    marker_.scale.x = 0.1;
    marker_.scale.y = 0.1;
    marker_.scale.z = 0.1;

    path_rviz_pub_ = nh_.advertise<visualization_msgs::Marker>(path_rviz_topic_, 5);
}

void PathVisualization::publish() {
    updateMarker();
    path_rviz_pub_.publish(marker_);
}

void PathVisualization::setPath(const std::vector<Eigen::Vector3d> &path) {
    path_ = path;
    path2MarkerPoints();
}

void PathVisualization::clearPath() {
    path_.clear();
}

void PathVisualization::path2MarkerPoints() {
    if (path_.empty()) {
        ROS_WARN("Path is empty! Call setPath() first.");
        return;
    }
    std::vector<geometry_msgs::Point> path_points;
    for (const auto& point : path_) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = cruise_height_;
        path_points.push_back(p);
    }
    marker_.points = path_points;
    clearPath();
}
