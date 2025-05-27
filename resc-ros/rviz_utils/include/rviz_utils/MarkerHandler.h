//
// Created by Zhaohong Liu on 24-9-18.
//

#ifndef RVIZ_UTILS_MARKERHANDLER_H
#define RVIZ_UTILS_MARKERHANDLER_H

#include <utility>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>

class MarkerHandler {
public:
    virtual ~MarkerHandler() = default;

    [[nodiscard]] virtual visualization_msgs::Marker getMarker() const {
        return marker_;
    }
    virtual void updateMarker() = 0;
    virtual void init() = 0;
    virtual void publish() = 0;

protected:
    /**
     * @brief Protected constrictor so that only derived classes can instantiate with handler and frame id
     * @param frame_id std::string, frame id for the marker, no '/' for ubuntu 20.04
     * @param nh ros::NodeHandler, to subscribe/publish if needed
     */
    MarkerHandler(std::string  frame_id, const ros::NodeHandle& nh)
            : nh_(nh), frame_id_(std::move(frame_id)) {}

    /**
     * @brief Set common properties of a marker, such as frame id and timestamp.
     * Virtual so that derived classes can override if needed.
     * @param marker visualization_msgs::Marker, the marker to set properties
     */
    virtual void setCommonMarkerProperties(visualization_msgs::Marker& marker) {
        marker.header.frame_id = frame_id_;
        marker.color.a = 1.0;  // Default alpha value
    }

    bool initialized_ = false;
    ros::NodeHandle nh_;
    std::string frame_id_;
    visualization_msgs::Marker marker_;
};

#endif //RVIZ_UTILS_MARKERHANDLER_H
