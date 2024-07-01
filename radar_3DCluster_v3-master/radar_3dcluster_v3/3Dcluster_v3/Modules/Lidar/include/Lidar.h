/**
 * @file Lidar.h
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 激光雷达库头文件
 * @version 1.0
 * @date 2022-01-14
 *
 * @copyright Copyright SCUT RobotLab(c) 2022
 *
 */
#pragma once
#include <fstream>
#include <iostream>           //std
#include <fstream>            //文件处理
#include <future>             //用于关闭激光雷达线程
#include "iomanip"            //小数点处理
#include "client.h"           //ouster客户端库
#include "lidar_scan.h"       //激光雷达扫描头文件
#include "build.h"            //ouster带的文件
#include "types.h"            //激光雷达数据类型
#include "point_viz.h"        //点云可视化头文件
#include "lidar_scan_viz.h"   //雷达扫描可视化头文件
#include <condition_variable> //条件变量
#include <thread>             //多线程
#include <opencv2/opencv.hpp>
#include "Parameter.h"
#include <pcl-1.13/pcl/point_cloud.h>
// #include <pcl-1.13/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.13/pcl/common/common.h>
#include <pcl/io/pcd_io.h>   //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
// #include <pcl/visualization/cloud_viewer.h> //点云查看窗口头文件
#include <pcl/console/time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace ouster;

/*
//映射的参数信息
struct CloudPixel
{
    int u;
    int v;
    bool operator<(const CloudPixel &a) const
    {
        if (u == a.u)
            return v < a.v;
        else
            return u < a.u;
    }
};

struct CloudCam
{
    double x;
    double y;
    double z;
};
*/

class Lidar
{
public:
    int connect_status = 0; // 0代表未连接、1代表连接成功、-1代表连接失败
    cv::Mat depth_mat;      // 深度图

private:
    string sensor_hostname = "os-122139001023.local";      // 传感器地址
    string data_destination = "/home/alyssa/radar_data/1"; // 传感器要传输到的终点
    int lidar_port;                                        // 激光雷达数据端口
    int imu_port;                                          // 加速度计数据端口
    std::shared_ptr<sensor::client> handle;                // 权柄,client是一个结构体
    XYZLut lut;                                            // 查表法所需
    string prod_line;                                      // 激光雷达多少线
    size_t w;                                              // 每帧列数
    size_t h;                                              // 每列像素数
    cv::Size img_size;                                     // 图像尺寸
    bool get_depth_flag = false;                           // 获取深度图标志位
    int column_window_length;                              // 轨道窗口长度
    sensor::sensor_info info;                              // 激光雷达相关信息
    std::shared_ptr<ouster::LidarScan> ls_read;            // 后承接容器
    std::shared_ptr<ouster::LidarScan> ls_write;           // 前承接容器
    // 扫描所使用的格式
    std::shared_ptr<uint8_t[]> lidar_buf;
    std::shared_ptr<ouster::ScanBatcher> batch;

private:
    void connect();   // 连接
    bool configure(); // 配置激光雷达

public:
    Lidar(string sensor_hostname, string data_destination, cv::Size img_size = cv::Size(1280, 1024)); // 激光雷达构造函数
    void showViz();                                                                                   // 可视化点云图
    void calCoord(cv::Matx44d &CEM, cv::Matx33d &CM);                                                 // 计算深度图
    pcl::PointCloud<pcl::PointXYZ>::Ptr openScan();                                                   // 开启轮询扫描
    inline void clearDepth() { this->depth_mat = cv::Mat::zeros(img_size, CV_64FC1); }                // 清空深度图
};

using Lidar_ptr = unique_ptr<Lidar>;