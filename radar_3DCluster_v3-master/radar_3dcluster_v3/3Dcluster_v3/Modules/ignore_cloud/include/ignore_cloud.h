/**
 * @file ignore_cloud.h
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
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
#include <pcl/common/transforms.h>

#include <vtkAutoInit.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <algorithm>
// #include "mapping.h"
using namespace cv;
using namespace std;
using namespace pcl;
struct coordinate
{
    Point2d tl;
    double width, height;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clu;
    Point2d big_map, real_map;
};
struct clustering
{
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmax;
    double zmin;
    double mf2, mf1, mf0;
    double linear, plainet;
};
class ignore_cloud
{
private:
public:
    vector<PointIndices> ecel;

    void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, double n_s, double n_l, double thresh);
    int *rand_rgb();
    PointCloud<PointXYZ>::Ptr EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, vector<vector<Point3d>> &objp, vector<double> &stdd, vector<clustering> &clust_box,double ratio,vector<coordinate> &coco);
    Eigen::Vector3f out_box(pcl::PointCloud<pcl::PointXYZ>::Ptr color_point);

    void feature_value(pcl::PointCloud<pcl::PointXYZ>::Ptr color_point, int order, double &linear, double &plainer);
    void sort(double &f1, double &f2, double &f3);
    // PointCloud<PointXYZ>::Ptr EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,vector<vector<Point3d>> &objp,vector<double> &stdd);
    int EucliCluster2(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);
    coordinate get_struct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Matx44d &EXM, cv::Matx33d &INM,double ratio);
};