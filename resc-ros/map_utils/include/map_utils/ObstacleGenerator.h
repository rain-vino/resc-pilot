//
// Created by Zhaohong Liu on 24-9-19.
//

#ifndef MAP_UTILS_OBSTACLEGENERATOR_H
#define MAP_UTILS_OBSTACLEGENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

class ObstacleGenerator {
public:
    void setResolution(double res) { resolution_ = res; }
    void setRectPcl(double x, double y, double length, double width, double height,
                    pcl::PointCloud<pcl::PointXYZ> & cloud, bool rd_h = true) const;
private:
    double resolution_ = 0.1;
    int ground_height_ = -1;  // ground_height_ * resolution_
};


#endif //MAP_UTILS_OBSTACLEGENERATOR_H
