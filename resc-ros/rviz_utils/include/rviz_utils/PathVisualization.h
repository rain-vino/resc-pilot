//
// Created by Zhaohong Liu on 24-10-19.
//

#ifndef RVIZ_UTILS_PATHVISUALIZATION_H
#define RVIZ_UTILS_PATHVISUALIZATION_H

#include <geometry_msgs/Point.h>

#include "MarkerHandler.h"
#include "VisualizationColor.h"

using VC = VisualizationColor;

class PathVisualization : public MarkerHandler {
public:
    PathVisualization(const ros::NodeHandle& nh, const std::string& frame_id)
        : MarkerHandler(frame_id, nh) {}
    void updateMarker() override;
    void init() override;
    void publish() override;
    void setPath(const std::vector<Eigen::Vector3d>& path);
    void clearPath();
    void path2MarkerPoints();
private:
    std::vector<Eigen::Vector3d> path_;
    ros::Publisher path_rviz_pub_;
    std::string path_rviz_topic_ = "path_rviz";

    double cruise_height_ = 1.0;
};


#endif //RVIZ_UTILS_PATHVISUALIZATION_H
