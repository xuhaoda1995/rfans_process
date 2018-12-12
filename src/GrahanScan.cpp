//
// Created by howstar on 18-6-7.
//

#include "GrahanScan.h"

using namespace pcl;
using namespace std;

void qsortpoint(PointCloud<PointXYZI>::Ptr s,PointXYZI base,int start,int end);
void sort_start_edge(PointCloud<_PointXYZI> s);

//向量（x1,y1）,(x2,y2)的叉积
double CrossMul(double x1,double y1,double x2,double y2)
{
    return x1*y2-x2*y1;
}
//向量（x1,y1）,(x2,y2)的点积
double DotMul(double x1,double y1,double x2,double y2)
{
    return x1*x2+y1*y2;
}
//跨立判断
//判断点c是在向量ab的逆时针方向还是顺时针方向，大于零逆时针，等于0则共线
double CrossMul(PointXYZI a,PointXYZI b,PointXYZI c)
{
    return CrossMul(b.x-a.x,b.y-a.y,c.x-a.x,c.y-a.y);
}
//计算向量ab和ac点积
double DotMul(PointXYZI a,PointXYZI b,PointXYZI c)
{
    return DotMul(b.x-a.x,b.y-a.y,c.x-a.x,c.y-a.y);
}
//判断浮点数符号
int doublecmp(double d)
{
    if(fabs(d)<10e-6)
        return 0;
    return d>0?1:-1;
}
//判断同一直线上的三个点位置，点c是否在点ab之间
bool betweenCmp(PointXYZI a,PointXYZI b,PointXYZI c)
{
    if(doublecmp(DotMul(c,a,b))<=0)
        return true;
    return false;
}
//判断j是否在base->i向量的左边或当共线时j是否位于它们的线段之间
bool isLeftorNearer(pcl::PointXYZI base,PointXYZI i,PointXYZI j)
{
    if(CrossMul(base,i,j)>0)
        return true;
    if(CrossMul(base,i,j)==0 && betweenCmp(base,i,j))
        return true;
    return false;
}
void swap(PointXYZI& a,PointXYZI& b)
{
    PointXYZI temp = b;
    b=a;
    a=temp;
}


void sort_start_edge(PointCloud<PointXYZI>::Ptr s)
{
    int i,j;
    int nums=s->size();
    for(i=2;i<nums;i++)
    {
        if(CrossMul(s->points[0],s->points[1],s->points[i])!=0)
            break;
    }
    for(j=1;j<(i+1)/2;j++)
        swap(s->points[j],s->points[i-j]);
}

bool cmpPoint(const pair<int,double> &a, const pair<int,double> &b)
{
    return a.second<b.second;
}

//将点按极角逆时针排序
void qsortpoint(pcl::PointCloud<pcl::PointXYZI>::Ptr s,PointXYZI base) {
//    if(start>=end)
//        return;
//    pcl::PointXYZI partition = s->points[end-1];
//    int i=start-1,j=start-1;
//    while(++j<end-1)
//    {
//        if(isLeftorNearer(base,s->points[j],partition))
//        {
//            swap(s->points[++i],s->points[j]);
//        }
//    }
//    swap(s->points[++i],s->points[end-1]);
//    qsortpoint(s,base,start,i);
//    qsortpoint(s,base,i+1,end);
    vector<pair<int,double> > ns;

    for (int i = 1; i < s->size(); i++) {
        double tmp_ang=atan2(s->points[i].y-base.y,s->points[i].x-base.x);
        pair<int,double> tmp_idx;
        tmp_idx.first=i;
        tmp_idx.second=tmp_ang;
        ns.push_back(tmp_idx);
    }
    sort(ns.begin(), ns.end(), cmpPoint);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_s(new pcl::PointCloud<pcl::PointXYZI>);
    tmp_s->resize(s->size());
    tmp_s->points[0]=s->points[0];
    for(int i=0;i<ns.size();i++){
        tmp_s->points[i+1]=s->points[ns[i].first];
    }
    tmp_s.swap(s);
}



void GrahanScan::convexHull() {
    sortpoint();

    int nums=data_->size();
    if(nums<=3) {

        *convex_hull_points=*data_;
        return;
    }
    int top=0;
    int i;
    for(i=0;i<2;i++) {
        convex_hull_points->push_back(data_->points[i]);
        top++;
    }
    while(i<nums)
    {
        //用<号判断则包含凸包边上的共线点，<=号判断则只包含凸包顶点
        if(CrossMul(convex_hull_points->points[top-2],convex_hull_points->points[top-1],
                    data_->points[i])<=0)
        {
            convex_hull_points->points.pop_back();
            top--;
            if(top==0)
                break;
        }
        else
        {
            convex_hull_points->points.push_back(data_->points[i++]);
            top++;
        }
    }
    //最后加入起点形成闭包
    while(CrossMul(convex_hull_points->points[top-2],convex_hull_points->points[top-1],
                   data_->points[0])<=0)
    {
        convex_hull_points->points.pop_back();
        top--;
        if(top==0)
            break;
    }
    convex_hull_points->points.push_back(data_->points[0]);

}

void GrahanScan::sortpoint() {

    int nums=data_->size();
    //找最左边的点
    for(int i=1;i<nums;i++)
    {
        if(data_->points[i].x<data_->points[0].x || (data_->points[i].x==data_->points[0].x
           && data_->points[i].y<data_->points[0].y))
            swap(data_->points[0],data_->points[i]);
    }
    qsortpoint(data_,data_->points[0]);
    //将起始边上的共线点重新排列
    sort_start_edge(data_);
}

GrahanScan::GrahanScan(pcl::PointCloud<pcl::PointXYZI>::Ptr points_src)
:convex_hull_points(new PointCloud<PointXYZI>)
,data_(new PointCloud<PointXYZI>){
    data_=points_src;
}

GrahanScan::~GrahanScan() {

}
