//
// Created by Zhaohong Liu on 24-10-21.
//

#ifndef RVIZ_UTILS_CTRLPOINTVISUALIZATION_H
#define RVIZ_UTILS_CTRLPOINTVISUALIZATION_H

#include <geometry_msgs/Point.h>

#include "MarkerHandler.h"
#include "VisualizationColor.h"

using VC = VisualizationColor;

class CtrlPtMarkerHandler : public MarkerHandler {
public:
    CtrlPtMarkerHandler(const ros::NodeHandle& nh, const std::string& frame_id)
            : MarkerHandler(frame_id, nh) {}

    void updateMarker() override;
    void init() override;
    void publish() override;
    void ctrlPt1Callback(const geometry_msgs::Point::ConstPtr& msg);
    void ctrlPt2Callback(const geometry_msgs::Point::ConstPtr& msg);
private:
    bool is_initialized_ = false;
    visualization_msgs::Marker ctrl_pt_second_marker_;
    ros::Publisher ctrl_pt_rviz_pub_;
    std::string ctrl_pt_rviz_topic_ = "ctrl_pt_rviz";
    ros::Subscriber ctrl_pt_1st_sub_;
    ros::Subscriber ctrl_pt_2nd_sub_;
    std::string ctrl_pt_1st_topic_ = "/search/wpt0";
    std::string ctrl_pt_2nd_topic_ = "/search/wpt1";
};


#endif //RVIZ_UTILS_CTRLPOINTVISUALIZATION_H
