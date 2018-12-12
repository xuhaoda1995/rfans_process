#include "stdafx.h"
#include "CObstaclePair.h"
#include "cloudShow.h"

//ros::Publisher pub;
CTrackersCenter trackingCenter;
long g_viewer_frameNumb = 0;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    //transfer msg to PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud_data);
    // Do data processing here...
    cloud_data->width = RFANS_LINE;
    cloud_data->height = cloud_data->size() / RFANS_LINE;
    if (cloud_data->size() % RFANS_LINE != 0) {
        std::stringstream err_str;
        err_str << "the line of rfans is not" << output.width;
        ROS_WARN(err_str.str().c_str());
    }

    pcl::PointCloud<PointSrc>::Ptr cloud_src(new pcl::PointCloud<PointSrc>);
    for(int i=0;i<cloud_data->size();i++) {
        float x = cloud_data->points[i].x;
        float y = cloud_data->points[i].y;
        float z = cloud_data->points[i].z;
        double radios = sqrt(x * x + y * y + z * z);
        double angle = atan2(y, x) / M_PI * 180;
        PointSrc tmp;
        tmp.angle = angle;
        tmp.radios = radios;
        cloud_src->push_back(tmp);
    }

    CObstaclePair obstaclePair(cloud_data,cloud_src);//pair side feature extraction
    trackingCenter.inputSingFrameFigures(obstaclePair.getGroupList(),g_viewer_frameNumb);
//    std::vector<Tracker> objTrackingList = trackingCenter.getTrackerList();

    g_viewer_frameNumb++;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obs(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto one : trackingCenter.getTrackerList())
    {
      *cloud_obs+=*one.FiguresPro[0].cloudInside;
    }
    pcl_show::publishAll(cloud_obs,cloud_data);

//    pcl::toROSMsg(*cloud_data,output);

    // Publish the data.
//    pub.publish(output);
}

int
main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "rfans_process");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/rslidar_points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
//    pub = private_nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    pcl_show::init_pub(private_nh);
    // Spin
    ros::spin();
}
