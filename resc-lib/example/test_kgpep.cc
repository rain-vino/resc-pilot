//
// Created by Zhaohong Liu on 24-12-6.
//

#include <iostream>
#include "K_GPEP.h"

int main() {
    K_GPEP k_gpep;
    Matrix sdf_map(7, 7);
    double map_size = 0.7;

    sdf_map <<
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 0, 1, 1, 1, 1,
        1, 1, 1, 0, 1, 1, 1,
        1, 1, 1, 1, 0, 1, 1,
        1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1;

    k_gpep.set2DESDFMap(sdf_map, map_size, map_size);

    auto test_pos = k_gpep.voxel2Pos(0, 0);
    std::cout << "Test pos: " << test_pos.transpose() << std::endl;

    auto pos = Eigen::Vector3d(0.15, 0.15, 1.0);
    auto ctrl_p = Eigen::Vector3d(0.55, 0.55, 1.0);

    auto edge_info = k_gpep.guidedPseudoRaycast(pos, ctrl_p);
    std::cout << "Edge info: " << edge_info.transpose() << std::endl;

    for (float y = 0.65; y > 0;) {
        for (float x = 0.05; x < 0.7;) {
            auto pos_f = Eigen::Vector2f(x, y);
            auto sdf = k_gpep.getDistanceSDF(pos_f);
            std::cout << sdf << ", ";
            x += 0.1;
        }
        y -= 0.1;
        std::cout << std::endl;
    }

    return 0;
}
