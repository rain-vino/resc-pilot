//
// Created by Zhaohong Liu on 24-9-5.
//

#include <random>
#include <iostream>
#include <Eigen/Eigen>

#include "SignedDistanceField.h"

int main() {
//    SignedDistanceField sdf;
//
//    Eigen::Matrix<double, 120, 120, Eigen::RowMajor> sdf_map;
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_real_distribution<> dis(0.0, 2.0);
//
//    for (int i = 0; i < sdf_map.rows(); ++i) {
//        for (int j = 0; j < sdf_map.cols(); ++j) {
//            sdf_map(i, j) = dis(gen);
//        }
//    }
//
    Eigen::Vector3d pos(1.0, 1.0, 1.0);
    Eigen::Vector3d ctrl_p(2.0, 2.0, 1.0);
//
//    auto edge_info = sdf.rayCastingEdge(pos, ctrl_p, sdf_map);
//    std::cout << "Edge info: " << edge_info << std::endl;
//
//    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> row_major_matrix;
//    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> col_major_matrix;
//
//    row_major_matrix << 1, 2, 3,
//                        4, 5, 6,
//                        7, 8, 9;
//
//    col_major_matrix << 1, 4, 7,
//                        2, 5, 8,
//                        3, 6, 9;
//
//    std::cout << "Row-major matrix:\n" << row_major_matrix << std::endl;
//    std::cout << "Column-major matrix:\n" << col_major_matrix << std::endl;
//    sdf.~SignedDistanceField();
//    std::cout << "sdf is destructed." << std::endl;

    SignedDistanceField sdf2;
    Eigen::Matrix<double, 7, 7, Eigen::RowMajor> sdf_map2;
    double map_size = 0.7;
    sdf2.resetMapSize(map_size);
    sdf_map2 << 1, 1, 1, 1, 1, 1, 1,
                1, 1, 1, 1, 1, 1, 1,
                1, 1, 0, 0, 1, 1, 1,
                1, 1, 1, 0, 1, 1, 1,
                1, 1, 1, 1, 0, 1, 1,
                1, 1, 1, 1, 0, 1, 1,
                1, 1, 1, 1, 1, 1, 1;
    pos = Eigen::Vector3d(0.15, 0.15, 1.0);
    ctrl_p = Eigen::Vector3d(0.55, 0.55, 1.0);
    auto edge_info = sdf2.rayCastingEdge(pos, ctrl_p, sdf_map2);
    std::cout << "Edge info: " << edge_info << std::endl;

    return 0;
}