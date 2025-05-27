//
// Created by Zhaohong Liu on 24-10-22.
//

#ifndef RVIZ_UTILS_TRAJMARKERHANDLER_H
#define RVIZ_UTILS_TRAJMARKERHANDLER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>

#include "MarkerHandler.h"
#include "VisualizationColor.h"

using VC = VisualizationColor;

class TrajMarkerHandler : public MarkerHandler {
public:
    TrajMarkerHandler(const ros::NodeHandle& nh, const std::string& frame_id)
        : MarkerHandler(frame_id, nh) {
        // color scheme from: http://lcpmgh.com/colors/
        // hex: #BD0026
        traj_color_.a = 1.0;
        traj_color_.r = 0.7411764705882353;
        traj_color_.g = 0.0;
        traj_color_.b = 0.14901960784313725;
    }
    void updateMarker() override;
    void init() override;
    void publish() override;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
//    void velCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
//    Eigen::Vector3f getColor(const double &val);
private:
    bool is_initialized_ = false;
    ros::Subscriber pose_sub_;
    ros::Publisher traj_rviz_pub_;
    ros::Publisher path_pub_;

    double marker_size_ = 0.12;
    size_t max_points_ = 2000;
    double last_time_ = 0.0;
    double duration_pose_ = 0.1;

    std::string pose_topic_ = "/mavros/local_position/pose";
    std::string vel_topic_ = "/mavros/local_position/velocity_local";
    std::string traj_rviz_topic_ = "traj_rviz";
    std::string path_topic_ = "traj_path_rviz";

    nav_msgs::Path path_;

    Eigen::Vector3f color_start_;
    Eigen::Vector3f color_mid1_;
    Eigen::Vector3f color_mid2_;
    Eigen::Vector3f color_mid3_;
    Eigen::Vector3f color_end_;

    std_msgs::ColorRGBA traj_color_;
};


#endif //RVIZ_UTILS_TRAJMARKERHANDLER_H
