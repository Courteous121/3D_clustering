/**
 * @file main.cpp
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */

#include <iostream>
#include <fstream>
#include <pcl-1.13/pcl/point_cloud.h>
#include <pcl-1.13/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.13/pcl/common/common.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/io/io.h>
#include <pcl/point_types.h>
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
#include "down_sampling.h"
#include "ignore_cloud.h"
#include "locate.h"
#include "GetDistance.h"
#include "segmentation_plane.h"
#include <csignal>
#include "Lidar.h"

using namespace std;
using namespace pcl::io;
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

void stopHandler(int sig)
{
    exit(0);
}

// -----------------------------the whole parameter------------------------------------------
pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);

pcl::ExtractIndices<pcl::PointXYZ> extract;

pcl::SACSegmentation<pcl::PointXYZ> seg;                              // pcl库中自带的采样一致分割算法，即几何分割算法
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 创建点云索引指针
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建模型参数
vector<clustering> clust;
vector<clustering> clust1;
// -----------------------------the whole parameter------------------------------------------

/**
 * @brief show 3D cloud points
 *
 * @param cloud 3D cloud need to dispaly
 * @return
 */
void show3Dpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::CloudViewer viewer("pcd viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

/**
 * @brief prepocess the 3D cloud
 *
 * @param cloud 3D cloud need to be cut
 * @return 3D point cloud which is prepared
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr delete3Dleftright(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        double _x = cloud->points[i].x;

        double _y = cloud->points[i].y;
        double h = cloud->points[i].z;
        double a1 = 0.9 * _x + _y + 4.5;
        double a2 = 2 * _x - _y - 19;
        double a3 = 0.7 * _x + _y - 5.3;
        double a4 = 1.2 * _x - _y + 6;
        if (a1 > 0 && a2 < 0 && a3 < 0 && a4 > 0 && h > -0.5 && h < 2)
        {
            cloud4->push_back(cloud->points[i]);
        }
    }
    cout << "cloud4 size is:" << cloud4->points.size() << endl;

    return cloud4;
}

/**
 * @brief delete the 3D cloud planet
 *
 * @param in_cloud_ptr 3D cloud need to delete the plane
 * @return 3D point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr deletePlanet(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 创建平面分割对象

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCDWriter writer;
    // 设置optimize是true
    seg.setOptimizeCoefficients(true);
    // 设置模型，是0
    seg.setModelType(0);
    // 设置方法还是0
    seg.setMethodType(0);
    // 设置最大轮询次数为100
    seg.setMaxIterations(100);
    // 设置搜索距离为0.02
    seg.setDistanceThreshold(0.02);

    // 把点云中所有的平面全部过滤掉，重复过滤，直到点云数量小于原来的0.3倍
    int i = 0, nr_points = in_cloud_ptr->points.size();
    while (in_cloud_ptr->points.size() > 0.5 * nr_points)
    {
        // 输入点云
        seg.setInputCloud(in_cloud_ptr);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(in_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Write the planar inliers to disk
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        in_cloud_ptr = cloud_f;
    }
    return in_cloud_ptr;
}

bool equal(PointXYZ a, PointXYZ b)
{
    if (a.x == b.x && a.y == b.y && a.z == b.z)
    {
        return true;
    }
    else
        return false;
}

/**
 * @brief emphasis 3D cloud cluster among other cluster
 *
 * @param original all 3D cloud
 * @param cluster 3D cluster
 * @param viewer  a container display the cloud
 * @return 3D point cloud viewer
 */
pcl::visualization::PCLVisualizer::Ptr showClussterInCloud(PointCloud<PointXYZ>::Ptr original, PointCloud<PointXYZ>::Ptr cluster, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    PointCloud<PointXYZRGB>::Ptr color_point(new PointCloud<PointXYZRGB>);
    pcl::PointIndices pi; /// 点云索引index数组
    for (int i = 0; i < original->points.size(); i++)
    {
        for (int j = 0; j < cluster->points.size(); j++)
        {
            if (equal(original->points[i], cluster->points[j]))
            {
                pi.indices.push_back(i);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_indice(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IndicesPtr index_ptr(new vector<int>(pi.indices)); /// 将自定义的pi数组进行智能指针的转换
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(original);
    extract.setIndices(index_ptr);
    extract.setNegative(true); /// 提取索引外的点云，若设置为true,则与copyPointCloud提取结果相同
    extract.filter(*cloud_indice);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_indice, "other cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cluster");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "other cloud");

    return viewer;
    // viewer->spinOnce(2000);
    // viewer->spin();
}

double stringTodouble(string a)
{
    stringstream s(a);
    double b;
    s >> b;
    return b;
}

struct CallbackArgs
{
    PointCloudT::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

/**
 * @brief mouse show function
 *
 * @param event
 * @param args
 * @return
 */
void pickPointCallback(const pcl::visualization::PointPickingEvent &event, void *args)
{
    CallbackArgs *data = (CallbackArgs *)args;
    if (event.getPointIndex() == -1)
        return;
    PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // 绘制红色点
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud<pcl::PointXYZ>(data->clicked_points_3d, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "clicked_points");
    // std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;

    Point3d p(current_point.x, current_point.y, current_point.z);
    double x = current_point.x;
    double y = current_point.y;
    double z = current_point.z;
    int j = 0;
    for (int i = 0; i < clust.size(); i++)
    {
        if (x > clust[i].xmin && x < clust[i].xmax && y > clust[i].ymin && y < clust[i].ymax && z > clust[i].zmin && z < clust[i].zmax)
        {
            cout << "*****第" << i + 1 << "组*****" << endl;
            cout << "mf2=" << clust[i].mf2 << endl;
            cout << "mf1=" << clust[i].mf1 << endl;
            cout << "mf0=" << clust[i].mf0 << endl;
            cout << "linear=" << clust[i].linear << endl;
            cout << "plainet=" << clust[i].plainet << endl;
            cout << clust[i].mf2 << " " << clust[i].mf1 << " " << clust[i].mf0 << " " << clust[i].linear << " " << clust[i].plainet << endl;
        }
    }
}

/**
 * @brief single_debug
 *
 * @return
 */
void single_debug()
{
    // 需要调试的pcd文件数量与编号
    int pcd_number = 33;
    int pcd_file[pcd_number];

    for (int q = 1; q < 34; q++)
    {

        pcd_file[q - 1] = q;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
    for (int pcd_n = 0; pcd_n < pcd_number; pcd_n++)
    {

        string pcd_addre = "/home/alyssa/save_point/record/4/" + to_string(pcd_file[pcd_n]) + ".pcd";
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_addre, *cloud_ori) == -1) //* load the file
        {
            cout << "Couldn't read file test_pcd.pcd " << endl;
        }

        else if (!cloud_ori->empty())
        {
            // viewer->removeAllPointClouds();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
            cloud2 = deletePlanet(cloud_ori);
            VoxelCentriodDownSample vcds; // 创建三维体素格网化对象
            vcds.setInputCloud(cloud2);   // 设置输入点云
            vcds.setGridStep(0.2);        // 设置体素格网边长
            vcds.downSample(cloud3);      // 执行体素质心下采样，并将采样结果保存到cloud_downSample中
            ignore_cloud ign;
            vector<vector<Point3d>> objp;
            vector<double> stdd;
            double ratio = 1;
            vector<coordinate> coco;
            cloud4 = ign.EucliCluster(cloud3, objp, stdd, clust, ratio, coco);
            cout << "第" << pcd_n + 1 << "组" << endl;
            cout << "clust的大小为：" << clust.size() << endl;

            int d;
            pcl::visualization::PCLVisualizer::Ptr viewer2(new pcl::visualization::PCLVisualizer("viewer2"));
            // viewer2->addPointCloud(cloud3, "cloud");
            viewer2 = showClussterInCloud(cloud3, cloud4, viewer2);

            CallbackArgs cb_args;
            PointCloudT::Ptr clicked_points_3d(new PointCloudT);
            cb_args.clicked_points_3d = clicked_points_3d;
            cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer2);

            viewer2->registerPointPickingCallback(pickPointCallback, (void *)&cb_args);
            std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
            viewer2->spin();
        }
        clust.clear();
        vector<clustering>(clust).swap(clust); // 清除容器并最小化它的容量
    }
}

/**
 * @brief run_debug
 *
 * @return
 */
void run_debug()
{
    // 雷达初始化
    Lidar lid("os-122139001023.local", "");
    // pcl::visualization::CloudViewer viewer("pcd viewer");
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer0"));

    // 确定需要记录的pcd文件编号
    ifstream in;
    string file_name = "/home/alyssa/save_point/record/order.txt";
    in.open(file_name);
    int pcd_num;
    in >> pcd_num;
    // 得到编号之后清空txt文档
    while (true)
    {
        // 循环轮询雷达
        future<void> poll_future; // 与promise关联的future对象
        promise<void> poll_exit;  // 激光雷达扫描线程退出信号
        thread poll;              // 激光雷达扫描线程
        poll_future = poll_exit.get_future();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_ori = lid.openScan();
        if (!cloud_ori->empty())
        {
            viewer->removeAllPointClouds();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
            cloud2 = deletePlanet(cloud_ori);
            VoxelCentriodDownSample vcds;  // 创建三维体素格网化对象
            vcds.setInputCloud(cloud_ori); // 设置输入点云
            vcds.setGridStep(0.2);         // 设置体素格网边长
            vcds.downSample(cloud3);       // 执行体素质心下采样，并将采样结果保存到cloud_downSample中
            ignore_cloud ign;
            vector<vector<Point3d>> objp;
            vector<double> stdd;
            double ratio = 1;
            vector<coordinate> coco;
            cloud4 = ign.EucliCluster(cloud3, objp, stdd, clust1, ratio, coco);
            pcd_num++;

            // 可视化点云
            // viewer->addPointCloud<pcl::PointXYZ>(cloud4, "cluster");
            // viewer->addPointCloud<pcl::PointXYZ>(cloud3, "other cloud");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cluster");
            // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "other cloud");
            showClussterInCloud(cloud3, cloud4, viewer);
            viewer->spinOnce();
            // 轮询存储为pcd文件
            string save_pcd = "/home/alyssa/save_point/record/4/" + to_string(pcd_num) + ".pcd";
            cloud_ori->width = 1;
            cloud_ori->height = cloud_ori->points.size();
            savePCDFileASCII<pcl::PointXYZ>(save_pcd, *cloud_ori);
            ofstream file_writer(file_name, ios_base::out);
            ofstream out;
            out.open(file_name);
            out << to_string(pcd_num);
        }
    }
}

int main(int argc, char **argv)
{

    single_debug();
    run_debug();
    return (0);
}
