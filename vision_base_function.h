#ifndef VISION_BASE_FUNCTION
#define VISION_BASE_FUNCTION

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

typedef struct WheelPoint
{
    double wID;
    Point2f wcenter;
    double wangle;
    double wdst;
}WheelPoint;

typedef struct TrackingBox
{
    int id;
    double objangle; //目标偏转角
    Point2f objcenter;  //目标质心点---最小外接矩形计算---实际中心点
    Point2f warp_objcenter; //像素中心点
    vector<WheelPoint> coveredWheel;//存放当前帧被该目标覆盖的全向轮信息
}TrackingBox;

typedef struct {
    int cid;
    int cobj;
}Location;


vector<TrackingBox> object_detect(vector<Point> contour,vector<vector<Point>> contours,vector<TrackingBox>detData,Mat roatated2,vector<Point2f> tran_coverpoint,vector<WheelPoint>allWheel);
Location object_tracking(Mat roatated2,TrackingBox dbm,int objcount,vector<vector<TrackingBox>> preFrameData);


#endif
