#include <iostream>
#include <pcl-1.13/pcl/io/pcd_io.h>
#include <pcl-1.13/pcl/point_cloud.h>
#include <pcl-1.13/pcl/point_types.h>
#include <pcl-1.13/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.13/pcl/common/common.h>
#include <pcl/io/pcd_io.h>                  //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>                //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h> //点云查看窗口头文件
#include <pcl/console/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <chrono>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <string>

#include <vtkAutoInit.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include<opencv2/opencv.hpp>
#include<cstdlib>
#include <algorithm>
using namespace cv;
using namespace std;
using namespace pcl;

class ignore_cloud
{
private:
//pcl::KdTreeFLANN<pcl::PointXYZ> tree;

//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;

public:
vector<PointIndices> ecel;

void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,double n_s,double n_l,double thresh);
int *rand_rgb();
PointCloud<PointXYZ>::Ptr EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,vector<vector<Point3d>> &objp);

};