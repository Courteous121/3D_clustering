#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include<pcl-1.13/pcl/point_cloud.h>
#include<pcl-1.13/pcl/point_types.h>
#include<pcl-1.13/pcl/io/pcd_io.h>
#include<pcl-1.13/pcl/visualization/pcl_visualizer.h>
#include<pcl-1.13/pcl/common/common.h>
#include <pcl/console/time.h>



using namespace std;
using namespace cv;

class locate
{
public:
double calDepth(Point2d &p_p);
Mat pixel2camera(Point2d &p_p);
Mat camera2world(Mat &p_c);
double stringTodouble(string a);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCD(std::string pcd_name);           //读入点云文件
int get_minmax_of_pointcloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);      //获取点云最值坐标
void locateCloud(string csv_loc,string cloud_loc);
void convertcsv2pcd(string csv_loc,string cloud_loc);
void convertStr2pcd(vector<vector<double>> str,string cloud_loc);
};