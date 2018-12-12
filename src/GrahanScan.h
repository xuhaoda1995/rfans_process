//
// Created by howstar on 18-6-7.
//

#ifndef PROJECT_GRAHANSCAN_H
#define PROJECT_GRAHANSCAN_H


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class GrahanScan {
public:
    GrahanScan(pcl::PointCloud<pcl::PointXYZI>::Ptr points_src);
    ~GrahanScan();

    void convexHull();


private:
    pcl::PointCloud<pcl::PointXYZI>::Ptr data_;

    //以s中的最低点为参考点，对其他所有点进行极角排序（逆时针）
    //共线时离参考点较远的点排在前面，凸包的起始边共线点从近到远排列
    void sortpoint();

public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr convex_hull_points;

};


#endif //PROJECT_GRAHANSCAN_H
