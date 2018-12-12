#ifndef STDAFX_H
#define STDAFX_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>
#include <iostream>

#define CONSTANT_FIGURERESERVE 10
#define RFANS_LINE 32

struct PointSrc
{
    double radios;
    double angle;
};

inline int getIdxVeloDyne(int idx_beam,int idx_sweep)
{
  int idx=idx_beam<16?idx_beam*2:(idx_beam-16)*2+1;
  return idx_sweep*RFANS_LINE+idx;
}

inline int getIdxRslidar(int idx_beam,int idx_sweep)
{
  int idx=idx_beam>=8?23-idx_beam:idx_beam;
  return idx_sweep*RFANS_LINE+idx;
}

#endif // STDAFX_H
