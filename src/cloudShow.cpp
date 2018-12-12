//
// Created by howstar on 18-6-6.
//

#include "cloudShow.h"

ros::Publisher pcl_show::pub_obs;
ros::Publisher pcl_show::pub_ori;

using namespace std;

const string frame_id="velodyne";

void pcl_show::init_pub(ros::NodeHandle nh) {
    pub_obs=nh.advertise<sensor_msgs::PointCloud2>("obs",1);
    pub_ori=nh.advertise<sensor_msgs::PointCloud2>("cloud_ori",1);
    //pub=nh1.advertise<sensor_msgs::PointCloud2>("output",1);
}

void pcl_show::publishAll(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data)
{
  sensor_msgs::PointCloud2 output1,output2;
  pcl::toROSMsg(*cloud_obs,output1);
  output1.header.frame_id=frame_id;
  pcl_show::pub_obs.publish(output1);

  pcl::toROSMsg(*cloud_data,output2);
  output2.header.frame_id=frame_id;
  pcl_show::pub_ori.publish(output2);
}
