//
// Created by Zhaohong Liu on 24-9-18.
//

#ifndef RVIZ_UTILS_RVIZPARAMS_H
#define RVIZ_UTILS_RVIZPARAMS_H

#include <string>

class RvizParams {
public:
    static std::string pose_marker_pub_topic_;

    static std::string mesh_resource_path_;

    static std::string world_frame_id_;

    static constexpr int sub_queue_size_ = 5;
    static constexpr int pub_queue_size_ = 1;

    static constexpr double robot_scale_x_ = 1.0;
    static constexpr double robot_scale_y_ = 1.0;
    static constexpr double robot_scale_z_ = 1.0;

    static constexpr double rviz_frequency_ = 10.0;
};



#endif //RVIZ_UTILS_RVIZPARAMS_H
