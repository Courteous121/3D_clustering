/**
 * @file GetDistance.h
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
#include <iostream>
#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace pcl;

class getDis
{
    private:

    public:
    vector<Eigen::Vector4f> getZhiXin(vector<vector<Point3d>> objp,vector<double> stdd);
    vector<double> getDistance(vector<Eigen::Vector4f> center);
};