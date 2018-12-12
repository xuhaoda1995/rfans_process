//
// Created by howstar on 18-6-6.
//

#ifndef PROJECT_CLOUDSHOW_H
#define PROJECT_CLOUDSHOW_H

#include "stdafx.h"

namespace pcl_show{

    extern ros::Publisher pub_obs;
    extern ros::Publisher pub_ori;

    void init_pub(ros::NodeHandle nh);

    void publishAll(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data);
}

#endif //PROJECT_CLOUDSHOW_H
