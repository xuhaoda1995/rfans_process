//
// Created by howstar on 18-6-5.
//

#include "CObstaclePair.h"
#include "cloudShow.h"
#include "GrahanScan.h"
#include <vector>
using namespace std;

const int veloline_down_count = 23;

//ros::Publisher pub_group;

const double neboANDsideDistanceThreshold[] = {
        0.80,0.80,0.81,0.83,0.85,//5
        0.87,0.90,0.93,0.97,0.91,//5
        0.98,0.94,0.94,0.96,0.92,//5
        1.00,1.00,1.00,1.00,1.00,1.00,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
        ,1.00
};



CObstaclePair::CObstaclePair()
{

}
CObstaclePair::~CObstaclePair()
{

}

CObstaclePair::CObstaclePair(pcl::PointCloud<pcl::PointXYZI>::Ptr ori_xyz, pcl::PointCloud<PointSrc>::Ptr ori_src)
{

    m_ori_src = ori_src;
    m_ori_xyz = ori_xyz;

    //initialize
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserPointCloud[RFANS_LINE];
    pcl::PointCloud<PointSrc>::Ptr laserPointCloud_src[RFANS_LINE];
    std::vector<std::vector<std::pair<unsigned, unsigned > > > Start_sidePairFeature(RFANS_LINE);
    std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature(RFANS_LINE);


    int numPts = ori_xyz->points.size();
    int numFace = numPts / RFANS_LINE;


    //1.split 32 lines
    for(unsigned ibeam_count = 0 ; ibeam_count < RFANS_LINE ; ibeam_count++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr beam_laserPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<PointSrc>::Ptr beam_laserPointCloud_src(new pcl::PointCloud<PointSrc>);
        for(unsigned jface_count = 0 ; jface_count < numFace ; jface_count++)
        {
//            beam_laserPointCloud->points.push_back(ori_xyz->points[getIdxVeloDyne(ibeam_count,jface_count)]);
//            beam_laserPointCloud_src->points.push_back(ori_src->points[getIdxVeloDyne(ibeam_count,jface_count)]);

//          beam_laserPointCloud->points.push_back(ori_xyz->points[getIdxRslidar(ibeam_count,jface_count)]);
//          beam_laserPointCloud_src->points.push_back(ori_src->points[getIdxRslidar(ibeam_count,jface_count)]);

            beam_laserPointCloud->points.push_back(ori_xyz->points[getIdxRfans(ibeam_count,jface_count,numFace)]);
            beam_laserPointCloud_src->points.push_back(ori_src->points[getIdxRfans(ibeam_count,jface_count,numFace)]);
        }
        laserPointCloud[ibeam_count] = beam_laserPointCloud;
        laserPointCloud_src[ibeam_count] = beam_laserPointCloud_src;
    }


    //2.pair feature extraction
    ///2.1 feature judgment
    for(unsigned ibeam_count = 0 ; ibeam_count < veloline_down_count ; ibeam_count++ )
    {
        //2.1.1 find side and nebo
        vector<bool> isSideFlagList(numFace);
        vector<bool> isNeboFlagList(numFace);

        int neboHood = 15;
        //neighbor judgment
        for(unsigned i = 0 ; i < numFace ; i++)
        {
            float depth0 = laserPointCloud_src[ibeam_count]->points[i].radios;
            float depth1 = depth0;
            unsigned j = 0;
            if(depth0 < 1.5)
            {
                continue;
            }
            else
            {
                //ignore radius = 0.00 or radius < threshold
                for(unsigned m = 1 ; m < neboHood ; m++)
                {
                    unsigned mcount1 = i + m;
                    if(mcount1 >= 0 && mcount1 < numFace)
                    {
                        depth1 = laserPointCloud_src[ibeam_count]->points[mcount1].radios;
                        if(depth1 > 1.5)
                        {
                            j = mcount1;
                            break;
                        }
                    }
                }
                //
                if(depth1 > 1.5 && abs(depth0 - depth1) > neboANDsideDistanceThreshold[ibeam_count] * 0.35/*THRE_NEBODISMAX*/ )
                {
                    if(depth0 < depth1)
                    {
                        isSideFlagList[i] = true;
                        isNeboFlagList[j] = true;
                    }
                    else
                    {
                        isNeboFlagList[i] = true;
                        isSideFlagList[j] = true;
                    }
                }
            }
        }

        //2.1.2 make side pair
        std::vector<std::pair<unsigned,unsigned> > start_sidelist;
        std::vector<std::pair<unsigned,unsigned> > end_sidelist;

        for(unsigned i_index = 0 ; i_index < numFace ; i_index++)
        {
            unsigned j_index = i_index;

            std::pair<unsigned,unsigned> startside;
            std::pair<unsigned,unsigned> endside;
            startside.first = ibeam_count;
            endside.first = ibeam_count;

            if(isSideFlagList[i_index])
            {
                for(unsigned j = 1 ; j < 1500 && (j+i_index) < numFace ; j++)
                {
                    j_index = i_index + j;
                    if(isNeboFlagList[j_index])
                    {
                        break;
                    }
                    else if(isSideFlagList[j_index])
                    {
                        startside.second = i_index;
                        endside.second = j_index;
                        pcl::PointXYZI startpt = laserPointCloud[ibeam_count]->points[i_index];
                        pcl::PointXYZI endpt = laserPointCloud[ibeam_count]->points[j_index];

                        double distance_2 = pow((startpt.x - endpt.x), 2) + pow((startpt.y - endpt.y), 2);
                        //distance between start and end can not be too long
                        //points between start and end can not be too few
                        if(distance_2 < 36.00 && endside.second - startside.second > 3)
                        {
                            start_sidelist.push_back(startside);
                            end_sidelist.push_back(endside);
                        }
                        break;
                    }
                }
                i_index = j_index;
            }
        }
        Start_sidePairFeature[ibeam_count] = start_sidelist;
        End_sidePairFeature[ibeam_count] = end_sidelist;
    }

    FeaturePair_Grouping(Start_sidePairFeature,End_sidePairFeature);
}

unsigned int
CObstaclePair::get_idx(int i, int j) {
    //return j*m_ori_xyz->height+i;
    return i*m_ori_xyz->width+j;
}

void CObstaclePair::FeaturePair_Grouping(std::vector<std::vector<std::pair<unsigned, unsigned> > > Start_sidePairFeature,
                                         std::vector<std::vector<std::pair<unsigned, unsigned> > > End_sidePairFeature)
{
    //resData
    std::vector<Tracking_group> groupList(0);
    std::vector<Tracking_group> groupList_swap(0);

    ////1.group similar segments into a group
    //traverse segment
    for(unsigned ibeam = 0 ; ibeam < RFANS_LINE;  ibeam++)
    {
        if(Start_sidePairFeature[ibeam].size() == 0)
            continue;

        for(unsigned jface = 0 ; jface < Start_sidePairFeature[ibeam].size() ; jface++)
        {
            bool findGroup = false;
            for(unsigned kgroup = 0 ;kgroup < groupList.size() ; kgroup++)
            {
                //similarities calculation
                trackingGroupMemb groupMembNew;
                groupMembNew.start_index = Start_sidePairFeature[ibeam][jface];
                groupMembNew.end_index = End_sidePairFeature[ibeam][jface];
                trackingGroupMemb groupCur = groupList[kgroup].Gmember[groupList[kgroup].Gmember.size() - 1];

                if(groupCur.start_index.first == groupMembNew.start_index.first)
                    continue;

                unsigned curCode_s = get_idx(groupCur.start_index.second, groupCur.start_index.first);
                unsigned curCode_e = get_idx(groupCur.end_index.second, groupCur.end_index.first);
                unsigned newCode_s = get_idx(groupMembNew.start_index.second, groupMembNew.start_index.first);
                unsigned newCode_e = get_idx(groupMembNew.end_index.second, groupMembNew.end_index.first);
                double delt_distance = abs(m_ori_src->points[curCode_s].radios - m_ori_src->points[newCode_s].radios)
                                      + abs(m_ori_src->points[curCode_e].radios - m_ori_src->points[newCode_e].radios);
                double delt_angstart = abs(m_ori_src->points[curCode_s].angle - m_ori_src->points[newCode_s].angle);
                double delt_angend = abs(m_ori_src->points[curCode_e].angle - m_ori_src->points[newCode_e].angle);
                double max_sang = max(m_ori_src->points[curCode_s].angle, m_ori_src->points[newCode_s].angle);
                double min_eang = min(m_ori_src->points[curCode_e].angle, m_ori_src->points[newCode_e].angle);


                //push back a segment to a existed group
                if(delt_distance < 1.0 && (delt_angstart < 5.0 || delt_angend < 5.0 || (min_eang - max_sang) > 0))
                {
                    groupList[kgroup].Gmember.push_back(groupMembNew);
                    findGroup = true;
                    break;
                }
            }
            if(!findGroup)
            {
                //creat a new group with a segment
                Tracking_group groupNew;
                trackingGroupMemb groupMembNew;
                groupMembNew.start_index = Start_sidePairFeature[ibeam][jface];
                groupMembNew.end_index = End_sidePairFeature[ibeam][jface];

                if((groupMembNew.end_index.second - groupMembNew.start_index.second) < 5 && groupMembNew.end_index.first < 18)
                    continue;

                groupNew.Gmember.push_back(groupMembNew);
                groupList.push_back(groupNew);
            }
        }
    }


    ////2.merge group
    //evaluate properties
    bool ifMerge = false;
    for(unsigned icount = 0; icount < 5 && (ifMerge || icount == 0) ; icount++)
        ifMerge = GroupMerge(groupList);


    ////3.filter group List
    groupList_swap.resize(0);
    for(unsigned igroup = 0 ; igroup < groupList.size() ; igroup++)
    {
        //
        if(groupList[igroup].Gmember.size() > 1)
        {
            groupList_swap.push_back(groupList[igroup]);
            //group_properities_swap.push_back(group_properities_swap[igroup]);
        }
    }
    groupList.swap(groupList_swap);

    cout<<"group size is "<<groupList.size()<<endl;

    m_groupList_res = groupList;

}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::GroupPointCloudColl(Tracking_group group) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr Agroup_cloud(new pcl::PointCloud <pcl::PointXYZI>);
    //collect all points belong to a group
    for (unsigned jmemb = 0; jmemb < group.Gmember.size(); jmemb++) {
        int count = group.Gmember[jmemb].end_index.second - group.Gmember[jmemb].start_index.second;
        for (unsigned kindex = 0; kindex <= count; kindex++) {
            int ptIndex = get_idx(group.Gmember[jmemb].start_index.second + kindex,
                                  group.Gmember[jmemb].start_index.first);

//            if (m_ori_src->points[ptIndex].radios > 0)
            Agroup_cloud->points.push_back(m_ori_xyz->points[ptIndex]);
        }
    }
    return Agroup_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
CObstaclePair::get_group_box(pcl::PointCloud<pcl::PointXYZI>::Ptr group_points) {
    int num = group_points->size();
    pcl::PointXYZI l_pt,d_pt,r_pt,t_pt;
    float left=100;
    float right=-100;
    float top=-100;
    float down=100;
    //int l_idx,r_idx,t_idx,d_idx;
    for (int i = 0; i < num; i++) {
        pcl::PointXYZI tmp=group_points->points[i];
        if(left>tmp.x){
            left=tmp.x;
            //l_idx=i;
            l_pt=tmp;
        }
        if(right<tmp.x){
            right=tmp.x;
            //r_idx=i;
            r_pt=tmp;
        }
        if(top<tmp.y){
            top=tmp.y;
            //t_idx=i;
            t_pt=tmp;
        }
        if(down>tmp.y){
            down=tmp.y;
            //d_idx=i;
            d_pt=tmp;
        }

    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr box=get_line(l_pt,t_pt);
    *box+=*(get_line(t_pt,r_pt));
    *box+=*(get_line(r_pt,d_pt));
    *box+=*(get_line(d_pt,l_pt));

    return box;
}

void CObstaclePair::GroupPropertiesCal(Tracking_group &group)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointsIn(GroupPointCloudColl(group));
    group.Gproperties.cloudInside = pointsIn;

    //
    float cx = 0.0,cy = 0.0, radius = 0.0;
    float xMin = pointsIn->points[0].x, xMax = xMin, yMin = pointsIn->points[0].y, yMax = yMin;
    for(unsigned ipoint = 0 ; ipoint < pointsIn->points.size() ; ipoint++)
    {
        float x = pointsIn->points[ipoint].x;
        float y = pointsIn->points[ipoint].y;

        if(abs(x) < 1.5 && abs(y) < 1.5)
            continue;

        xMin = x < xMin ? x: xMin;
        xMax = x > xMax ? x: xMax;
        yMin = y < yMin ? y: yMin;
        yMax = y > yMax ? y: yMax;
    }
    cx = (xMax + xMin) /2.0;
    cy = (yMax + yMin) /2.0;
    if((xMax - xMin) > (yMax - yMin))
    {
        radius = (xMax - xMin) /2.0;
    }
    else
    {
        radius = (yMax - yMin) /2.0;
    }

    if(radius <0.5)
    {
        radius = 0.5;
    }

    group.Gproperties.center.x = cx;
    group.Gproperties.center.y = cy;
    group.Gproperties.radius = radius;
    group.Gproperties.width_x = xMax - xMin;
    group.Gproperties.length_y = yMax - yMin;
    group.Gproperties.density = 1.0 * pointsIn->size() / radius;

}

bool CObstaclePair::GroupMerge(std::vector<Tracking_group> &groupList)
{
    bool ifMergeAgroup = false;

    std::vector<Tracking_group> groupList_swap(0);
    for(unsigned igroup = 0 ; igroup < groupList.size() ; igroup++)
    {
        GroupPropertiesCal(groupList[igroup]);
    }
    for(unsigned igroup = 0 ; igroup < groupList.size() ; igroup++)
    {
        bool findGroup = false;
        for(unsigned jgroup = 0 ; jgroup < groupList_swap.size() ; jgroup++)
        {
            //calculate overlap of two groups
            float disAB, delt_radiusAB;
            disAB = sqrt(pow(groupList[igroup].Gproperties.center.x - groupList[jgroup].Gproperties.center.x,2)
                         + pow(groupList[igroup].Gproperties.center.y - groupList[jgroup].Gproperties.center.y,2));
            delt_radiusAB = max(groupList[igroup].Gproperties.radius, groupList[jgroup].Gproperties.radius);

            if(delt_radiusAB - disAB > -1.0 && groupList[igroup].Gproperties.density > 10 && groupList[jgroup].Gproperties.density > 10)
            {
                //calculate radius acce
                Tracking_group tryMerged_group;
                for(unsigned kmemb = 0 ; kmemb < groupList[igroup].Gmember.size() ; kmemb++)
                {
                    tryMerged_group.Gmember.push_back(groupList[igroup].Gmember[kmemb]);
                }
                for(unsigned kmemb = 0 ; kmemb < groupList_swap[jgroup].Gmember.size() ; kmemb++)
                {
                    tryMerged_group.Gmember.push_back(groupList_swap[jgroup].Gmember[kmemb]);
                }
                GroupPropertiesCal(tryMerged_group);
                float radius_acce = (tryMerged_group.Gproperties.radius - delt_radiusAB)/delt_radiusAB;

                if(radius_acce < 0.2)
                {
                    //merge two groups into one
                    groupList_swap[jgroup] = tryMerged_group;
                    findGroup = true;
                    ifMergeAgroup  = true;
                    break;
                }
            }
        }
        if(!findGroup)
        {
            groupList_swap.push_back(groupList[igroup]);
        }
    }
    groupList.swap(groupList_swap);

    return ifMergeAgroup;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::get_line(pcl::PointXYZI p1, pcl::PointXYZI p2) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    float width=(p2.x-p1.x);
    float height=(p2.y-p1.y);
    int num=20;
    float step_x=width/num;
    float step_y=height/num;
    for(int i=0;i<num;i++)
    {
        pcl::PointXYZI tmp;
        tmp.x=p1.x+step_x*i;
        tmp.y=p1.y+step_y*i;
        tmp_ptr->push_back(tmp);
    }
    return tmp_ptr;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr CObstaclePair::get_lines(pcl::PointCloud<pcl::PointXYZI>::Ptr convex_hull_pts) {
    int nums=convex_hull_pts->size();
    pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
    for(int i=0;i<nums-1;i++){
        *output+=*(get_line(convex_hull_pts->points[i],convex_hull_pts->points[i+1]));
    }
    return output;
}

//grouping end
//------------------------------------------------------------------------------------------------------------------//
//tracking

float figureSimiCalculation(Tracker tracker, Tracking_group figure)
{
    float simi = 0.0;
    float geoSimi = 0.0;
    int posSimi = 0;

    //Position Similarity -- integral part
    float distance = sqrt(pow(tracker.PropertiesKF.center.x - figure.Gproperties.center.x,2)
                          + pow(tracker.PropertiesKF.center.y - figure.Gproperties.center.y,2));
    if(distance > 8.0)
        posSimi = 0.0;
    else
        posSimi = (int)((8.0 - distance) * 10.0 / 8.0 + 0.5);//waiting test

    //Geometry Similarity -- decimal part
    float biggerRadius = max(tracker.PropertiesKF.radius, figure.Gproperties.radius);
    geoSimi = (biggerRadius -abs(tracker.PropertiesKF.radius - figure.Gproperties.radius)) / biggerRadius;

    //
    simi = posSimi + geoSimi;

    return simi;
}

void Tracker::create_WithNewMember(trackingGroupProp &figure)
{
    for(unsigned i = 0 ; i < CONSTANT_FIGURERESERVE ; i++)
    {
        FiguresPro[i] = figure;
    }
    PropertiesKF = figure;

    ////KF
    //KF.inputMeasurement_first(figure.center.x, figure.center.y);
    //KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
    countMeb_total = 1;

}

void Tracker::update_WithNewMember(trackingGroupProp &figure)
{
    for(unsigned i = CONSTANT_FIGURERESERVE - 1 ; i > 0 ; i--)
    {
        FiguresPro[i] = FiguresPro[i - 1];
    }
    FiguresPro[0] = figure;
    PropertiesKF.radius = figure.radius;

    //KF
    if(countMeb_total == 1)
    {
        KF.inputMeasurement_first(figure.center.x, figure.center.y, figure.center.x - FiguresPro[1].center.x, figure.center.y - FiguresPro[1].center.y);
        PropertiesKF = figure;
    }
    else
    {
        KF.inputMeasurement(figure.center.x, figure.center.y);
        KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
    }
    countMeb_total ++;
}

void Tracker::update_WithNoMember()
{
    KF.inputMeasurement(PropertiesKF.center.x, PropertiesKF.center.y);
    KF.outputCorrection(PropertiesKF.center.x, PropertiesKF.center.y);
}

long Tracker::getLastFnumber()
{
    return FiguresPro[0].Fnumber;
}

CTrackersCenter::CTrackersCenter()
{
    trackerIDCount = 0;
}

CTrackersCenter::~CTrackersCenter()
{

}

long CTrackersCenter::getNewobjID()
{
    trackerIDCount++;
    if (trackerIDCount > 100000)
    {
        trackerIDCount = 0;
    }
    return trackerIDCount;
}

void CTrackersCenter::inputSingFrameFigures(std::vector<Tracking_group> figureList, long frameID)
{
    ////0.figureList initialize
    for(unsigned i = 0 ; i < figureList.size() ; i++)
    {
        figureList[i].Gproperties.Fnumber = frameID;
    }
    ////0.trackerList initialize
    if(TrackerList.size() == 0)
    {
        //waiting
        for(unsigned i = 0 ; i < figureList.size() ; i++)
        {
            Tracker trackerNewOne(getNewobjID());
            trackerNewOne.create_WithNewMember(figureList[i].Gproperties);
            TrackerList.push_back(trackerNewOne);
        }
        return;
    }


    ////1.Bayes
    int numFigure = figureList.size();
    int numTracker = TrackerList.size();
    std::vector<int> figureBelong(numFigure);//fill in code of tracker the figure belong to|-1 = new tracker
    std::vector<int> trackerStatus(numTracker);//fill in code of figure the tracker taking |-1 = missing
    std::vector<int> trackerStatus_new(0);//fill in code of figure the tracker taking |-1 = missing
    if(figureList.size() != 0)
    {
        //1.0initialize
        for(unsigned i = 0 ; i < numFigure ; i++)
        {
            figureBelong[i] = -1;
        }
        for(unsigned i = 0 ; i < numTracker ; i++)
        {
            trackerStatus[i] = -1;
        }



        //1.1simi calculation
        Eigen::MatrixXd Matsimi = Eigen::MatrixXd::Zero(numTracker, numFigure);
        for(unsigned itracker = 0 ; itracker < TrackerList.size() ; itracker++)
        {
            for(unsigned jfigure = 0 ; jfigure < figureList.size() ; jfigure++)
            {
                //simi definition //waiting
                Matsimi(itracker, jfigure) = figureSimiCalculation(TrackerList[itracker], figureList[jfigure]);
            }
        }

        //1.2belong
        //find max simi in col(different tracker)
        std::vector<float> figureSimi(numFigure);
        for(unsigned ifigure = 0; ifigure < figureList.size() ; ifigure++)
        {
            float maxSimi = 0.0;
            int codeTracker = -1;
            for(unsigned jtracker = 0 ; jtracker < TrackerList.size() ; jtracker++)
            {
                if(maxSimi < Matsimi(jtracker, ifigure))
                {
                    maxSimi = Matsimi(jtracker, ifigure);
                    codeTracker = jtracker;
                }
            }
            figureBelong[ifigure] = codeTracker;
            figureSimi[ifigure] = maxSimi;
        }
        //solve multi figure match the same tracker
        for(unsigned ifigure = 0; ifigure < figureBelong.size() ; ifigure++)
        {
            for(unsigned jfigure = 1; jfigure < figureBelong.size() ; jfigure++)
            {
                if(figureBelong[ifigure] == figureBelong[jfigure])
                {
                    if(figureSimi[ifigure] > figureSimi[jfigure])
                    {
                        figureBelong[jfigure] = -1;
                    }
                    if(figureSimi[ifigure] < figureSimi[jfigure])
                    {
                        figureBelong[ifigure] = -1;
                    }
                }
            }
        }
        //get the tracker status
        for(unsigned ifigure = 0 ; ifigure < figureBelong.size() ; ifigure++)
        {
            if(figureBelong[ifigure] >= 0)
            {
                trackerStatus[figureBelong[ifigure]] = ifigure;
            }
        }
    }

    ////2.comb list
    //create new tracker with new figure
    trackerStatus_new = trackerStatus;
    for(unsigned ifigure = 0 ; ifigure < figureBelong.size() ; ifigure++)
    {
        if(figureBelong[ifigure] == -1)
        {
            Tracker newOne(getNewobjID());
            TrackerList.push_back(newOne);
            trackerStatus_new.push_back(ifigure);
        }
    }


    //update with new figure
    //update even if no figure
    for(unsigned i = 0 ; i < TrackerList.size() ; i++)
    {
        //update
        if(trackerStatus_new[i] == -1)
        {
            TrackerList[i].update_WithNoMember();
        }
        else if(figureBelong[trackerStatus_new[i]] == -1)
        {
            TrackerList[i].create_WithNewMember(figureList[trackerStatus_new[i]].Gproperties);
        }
        else
        {
            TrackerList[i].update_WithNewMember(figureList[trackerStatus_new[i]].Gproperties);
        }

    }


    //delete missing tracker
    std::vector<Tracker> TrackerList_swap(0);
    for(unsigned i = 0 ; i < TrackerList.size() ; i++)
    {
        if(frameID - TrackerList[i].getLastFnumber() <= 5)
            TrackerList_swap.push_back(TrackerList[i]);
    }
    TrackerList.swap(TrackerList_swap);


    ////show
//    pcl::PointCloud<pcl::PointXYZI>::Ptr AllGroupCloud_color(new pcl::PointCloud<pcl::PointXYZI>);
//    for(unsigned itracker = 0 ; itracker < TrackerList.size() ; itracker++)
//    {
//        pcl::PointCloud<pcl::PointXYZI>::Ptr Agroup_cloud = TrackerList[itracker].FiguresPro[0].cloudInside;
//
//        //color and collecting
//        int colorCount = TrackerList[itracker].ID % 10;
//        float colorIntensity = 10.0 + colorCount * 10.0;
//        for(unsigned kpoint = 0 ; kpoint< Agroup_cloud->points.size() ; kpoint++)
//        {
//            Agroup_cloud->points[kpoint].intensity = colorIntensity;
//            AllGroupCloud_color->push_back(Agroup_cloud->points[kpoint]);
//        }
//        //g_ShowCircleFunction(groupList[igroup].Gproperties.center, groupList[igroup].Gproperties.radius);
//    }
//    int MultiColor[3] = {0,0,0};
//    g_ShowCloudFunction(AllGroupCloud_color, MultiColor, 2, "group");

}
