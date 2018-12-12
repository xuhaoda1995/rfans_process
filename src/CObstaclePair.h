//
// Created by howstar on 18-6-5.
//

#ifndef PROJECT_COBSTACLEPAIR_H
#define PROJECT_COBSTACLEPAIR_H

#include "stdafx.h"
#include "kalmanFilter.h"


struct trackingGroupMemb{
    std::pair<unsigned,unsigned> start_index;
    std::pair<unsigned,unsigned> end_index;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inPoints;
};

struct trackingGroupProp{
    float radius;
    float width_x;
    float length_y;
    pcl::PointXY center;
    pcl::PointXY weightedCenter;
    float density;

    long Fnumber;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInside;
};

struct Tracking_group{
    std::vector<trackingGroupMemb> Gmember;
    trackingGroupProp Gproperties;
};


//input the original point cloud xyzi registered,
//output the pair side of obstacles
//       the inside points of obstacles
class CObstaclePair
{
public:
    CObstaclePair();
    CObstaclePair(pcl::PointCloud<pcl::PointXYZI>::Ptr ori_xyz, pcl::PointCloud<PointSrc>::Ptr ori_scr);
    ~CObstaclePair();

    //function
public:
    std::vector<Tracking_group> getGroupList(){return m_groupList_res;}

private:

    void FeaturePair_Grouping(std::vector<std::vector<std::pair<unsigned, unsigned> > > Start_sidePairFeature,
                              std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature);

    bool GroupMerge(std::vector<Tracking_group> &groupList);

    pcl::PointCloud<pcl::PointXYZI>::Ptr GroupPointCloudColl(Tracking_group group);

    void GroupPropertiesCal(Tracking_group &group);

    unsigned int get_idx(int i, int j);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_group_box(pcl::PointCloud<pcl::PointXYZI>::Ptr group_points);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_line(pcl::PointXYZI p1,pcl::PointXYZI p2);

    pcl::PointCloud<pcl::PointXYZI>::Ptr
    get_lines(pcl::PointCloud<pcl::PointXYZI>::Ptr convex_hull_pts);

    //variable
public:


private:

    pcl::PointCloud<pcl::PointXYZI>::Ptr m_ori_xyz;

    pcl::PointCloud<PointSrc>::Ptr m_ori_src;

    std::vector<Tracking_group> m_groupList_res;

};


class Tracker
{
public:
    Tracker(){countMeb_total = 0; countMeb_total = 0;}
    Tracker(long idIn){ID = idIn;}
    ~Tracker(){}

public:
    //variable
    trackingGroupProp FiguresPro[CONSTANT_FIGURERESERVE];//CONSTANT_FIGURERESERVE 10

    trackingGroupProp PropertiesKF;

    long ID;

    kalmanFilter KF;

    int countMeb_total;

    //function
    void create_WithNewMember(trackingGroupProp &figure);

    void update_WithNewMember(trackingGroupProp &figure);

    void update_WithNoMember();

    long getLastFnumber();

};

class CTrackersCenter{
public:
    CTrackersCenter();
    ~CTrackersCenter();

    //function
public:
    void inputSingFrameFigures(std::vector<Tracking_group> figureList, long frameID);

    std::vector<Tracker> getTrackerList(){return TrackerList;}

private:
    long getNewobjID();

    //variable
public:

private:
    long trackerIDCount;

    std::vector<Tracker> TrackerList;
};

float figureSimiCalculation(Tracker tracker, Tracking_group figure);

#endif //PROJECT_COBSTACLEPAIR_H
