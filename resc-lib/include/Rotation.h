//
// Created by Zhaohong Liu on 24-9-10.
//

#ifndef MOGENLIB_ROTATION_H
#define MOGENLIB_ROTATION_H

#include <Eigen/Eigen>
#include <cmath>

class Rotation {
public:
    static Eigen::Matrix3d rotB2A(const Eigen::Vector3d &att);

    static Eigen::Matrix3d rotB2ody2World(const Eigen::Vector3d &att);
};


#endif //MOGENLIB_ROTATION_H
