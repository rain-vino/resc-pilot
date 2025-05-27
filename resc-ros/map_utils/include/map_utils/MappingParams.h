//
// Created by Zhaohong Liu on 24-12-22.
//

#ifndef MAPPINGPARAMS_H
#define MAPPINGPARAMS_H

#include <Eigen/Eigen>

class MappingParams {
public:
    double map_size_x_ = 40.0;
    double map_size_y_ = 20.0;
    double map_size_z_ = 2.0;
    double ground_height_ = 0.0;
    int ground_index_ = 0;
    Eigen::Vector3d map_origin_ = Eigen::Vector3d(-map_size_x_ / 2,
                                                  -map_size_y_ / 2,
                                                  ground_height_);
    double safe_margin_ = 1.0;

    const int free_threshold_ = 1;
    const int occupied_threshold_ = 0;
    Eigen::Vector3i map_voxel_num_;

    double resolution_ = 0.1;
    double half_resolution_ = 0.05;

    double collision_threshold_ = 0.3;
    double camera_range_max_ = 3.0;
    double inflated_obstacle_range_ = 3.0;
    double sensing_range_ = 5.0;

    bool has_god_view_ = true;

    int cloud_height_ = 1;
};

#endif //MAPPINGPARAMS_H
