//
// Created by Zhaohong Liu on 24-9-3.
//

#ifndef MOGENLIB_SIGNEDDISTANCEFIELD_H
#define MOGENLIB_SIGNEDDISTANCEFIELD_H

#include <iostream>
#include <utility>
#include <array>
#include <Eigen/Eigen>

using Index2d = std::pair<int, int>;
using Corner = std::array<Eigen::Vector2d, 4>;
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

enum AngleType {
    MIN,
    MAX
};

class SignedDistanceField {
private:
    double collision_distance_ = 0.3;
    double map_size_ = 12;
    double resolution_ = 0.1;
    int map_size_int_ = static_cast<int>(map_size_ / resolution_);
    double sector_range_ = M_PI / 3;

public:
    void resetMapSize(double & map_size);
    [[nodiscard]] bool isCollide(double distance) const;

    [[nodiscard]] bool isValidIndex(const Index2d& index) const;

    [[nodiscard]] bool isEdgeVoxel(const Index2d& index) const;

    template<typename EigenVector>
    bool isValidPos(const EigenVector& pos) const;

    template<typename EigenVector>
    Index2d pos2Index(const EigenVector& pos) const;

    [[nodiscard]] Corner getGridCorner(const Index2d& index) const;

    static Eigen::Vector2d getPeekCorner(const Eigen::Vector2d& pos2d,
                                         const Eigen::Vector2d& ctrl_p2d,
                                         const Corner& corners,
                                         AngleType type);

    [[nodiscard]] Eigen::Vector4d rayCastingEdge(const Eigen::Vector3d &pos,
                                                 const Eigen::Vector3d &ctrl_p,
                                                 const Eigen::Ref<Matrix> &sdf_map,
                                                 float preset_radius = -1) const;

    [[nodiscard]] Index2d sectorRayCasting(int direction,
                                           const Eigen::Vector2d& pos_2d,
                                           double theta_init,
                                           double d_theta,
                                           double radius,
                                           const std::string& mode,
                                           const Eigen::Ref<Matrix> &sdf_map) const;

    [[nodiscard]] Index2d bresenhamFarthestIndex(const Index2d &start,
                                                 const Index2d &end,
                                                 bool obs_free_mode,
                                                 const Eigen::Ref<Matrix> &sdf_map) const;
};


#endif //MOGENLIB_SIGNEDDISTANCEFIELD_H
