#include <iostream>
#include <pcl-1.13/pcl/point_cloud.h>
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
#include "down_sampling.h"
#include "ignore_cloud.h"
#include "locate.h"
#include "GetDistance.h"
#include <csignal>
#include "Lidar.h"

using namespace std;

void stopHandler(int sig)
{
    exit(0);
}
// the whole parameter-----------------------------------------------------------------
pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);

pcl::ExtractIndices<pcl::PointXYZ> extract;

pcl::SACSegmentation<pcl::PointXYZ> seg;                              // pcl库中自带的采样一致分割算法，即几何分割算法
pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                // 创建点云索引指针
pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); // 创建模型参数
struct CallbackArgs
{
    PointCloud<PointXYZRGB>::Ptr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

// the whole parameter-----------------------------------------------------------------
void pickPointCallback(const pcl::visualization::PointPickingEvent &event, void *args)
{
    CallbackArgs *data = (CallbackArgs *)args;
    if (event.getPointIndex() == -1)
        return;
    PointXYZRGB current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // 绘制红色点
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGB> red(data->clicked_points_3d, 0, 0, 255);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
                                                      "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}
// show the 3D point
void show3Dpoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::CloudViewer viewer("pcd viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
    }
}

// delete point which is too near from 0 point
// pcl::PointCloud<pcl::PointXYZ>::Ptr delete3DNear(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

//     for (int i = 0; i < cloud->points.size(); i++)
//     {
//         double dis = sqrt(cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y);
//         // cout<<"the"<<i<<"distance is"<<dis<<endl;
//         if (dis > 2)
//         {
//             cloud2->push_back(cloud->points[i]);
//         }
//     }
//     cout << "cloud2 size is:" << cloud2->points.size() << endl;

//     return cloud2;
// }

vector<vector<double>> delete3DNear(vector<vector<double>> strArr)
{
    pcl::console::TicToc time;
    time.tic();
    vector<vector<double>> str;
    vector<double> strr;
    for (int i = 0; i < strArr.size(); i++)
    {
        double dis = sqrt(strArr[i][0] * strArr[i][0] + strArr[i][1] * strArr[i][1]);
        if (dis > 2)
        {
            strr.push_back(strArr[i][0]);
            strr.push_back(strArr[i][1]);
            strr.push_back(strArr[i][2]);
            str.push_back(strr);
        }
    }
    cout << "筛选耗时为：" << time.toc() << "ms" << endl;
    return str;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr delete3Dheight(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        double h = cloud->points[i].z;

        if (h > -1.5 && h < -1.49)
        {
            cloud3->push_back(cloud->points[i]);
        }
    }
    cout << "cloud3 size is:" << cloud3->points.size() << endl;
    return cloud;
}

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

void getIndex(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4)
{

    for (int j = 1; j < cloud4->points.size() + 1; j++)
    {
        far_indices->indices.push_back(j);
    }

    extract.setInputCloud(cloud4);   // 输入点云指针
    extract.setIndices(far_indices); // 输入待处理的点云索引

    // 设置为true，则去掉索引值代表的点云再进行输出；设置为false，则直接输出索引值代表的点云
    extract.setNegative(false);
    PointCloud<PointXYZ>::Ptr out_cloud_ptr;
    extract.filter(*out_cloud_ptr); // 输出点云(非指针)

    // cout << "out_cloud_ptr size is:" << out_cloud_ptr->points.size() << endl;
}

// pcl::PointCloud<pcl::PointXYZ>::Ptr deletePlanet(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,vector<vector<double>> strArr)
// {
//     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//     // 创建平面分割对象

//     seg_plane::SACSegmentation<pcl::PointXYZ> seg;
//     plane_struct::PointIn::Ptr inliers(new plane_struct::PointIn);
//     plane_struct::ModelCo::Ptr coefficients(new plane_struct::ModelCo);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());

//     pcl::PCDWriter writer;
//     //设置optimize是true
//     seg.setOptimizeCoefficients(true);
//     //设置模型，是0
//     seg.setModelType(0);
//     //设置方法还是0
//     seg.setMethodType(0);
//     //设置最大轮询次数为100
//     seg.setMaxIterations(100);
//     //设置搜索距离为0.02
//     seg.setDistanceThreshold(0.02);

//     // 把点云中所有的平面全部过滤掉，重复过滤，直到点云数量小于原来的0.3倍
//     int i = 0, nr_points = strArr.size();
//     while (in_cloud_ptr->points.size() > 0.5 * nr_points)
//     {
//         // 输入点云
//         seg.setInputPoint(strArr);
//         //seg.setInputCloud(in_cloud_ptr);
//         seg.segment_plane(*inliers, *coefficients);
//         if (inliers->indices.size() == 0)
//         {
//             std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//             break;
//         }
//         // Extract the planar inliers from the input cloud
//         pcl::ExtractIndices<pcl::PointXYZ> extract;
//         extract.setInputCloud(in_cloud_ptr);
//         extract.setIndices(inliers);
//         extract.setNegative(false);

//         // Write the planar inliers to disk
//         extract.filter(*cloud_plane);
//         std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

//         // Remove the planar inliers, extract the rest
//         extract.setNegative(true);
//         extract.filter(*cloud_f);
//         in_cloud_ptr = cloud_f;
//     }
//     return in_cloud_ptr;
// }

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

void deleteGroundModel(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr)
{
    //============对要分割的平面模型进行设置============
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); // 设置分割模型，此时设置的分割模型为垂直于轴线的平面模型
    seg.setAxis(Eigen::Vector3f(0, 0, 1));               // 设置分割模型垂直的轴线。因此要对地面进行滤波，所以此处平面模型设置为垂直于Z轴
    seg.setEpsAngle(0.1);                                // 允许平面模型法线与Z轴的最大偏差为0.1弧度，也就是说大于0.1弧度的平面模型不考虑
    //==============================================

    seg.setMethodType(pcl::SAC_RANSAC);   // 设置分割方法为随机采样一致算法
    seg.setMaxIterations(100);            // 设置最大迭代次数为100
    seg.setDistanceThreshold(0.2);        // 设置点云到平面的最大距离阈值。若小于该阈值，则认定为内点
    seg.setOptimizeCoefficients(true);    // 再次进行参数优化
    seg.setInputCloud(in_cloud_ptr);      // 输入待处理点云集合
    seg.segment(*inliers, *coefficients); // 向分割算法中输入模型参数，输出分割模型的点云索引对象

    // if (inliers->indices.size() == 0)
    // {
    //     cout << "Could not estimate a planar model for the given dataset." << endl;
    //     // PCL_ERROR("Could not estimate a planar model for the given dataset.");
    //     // return -1;
    // }

    // 输出平面模型的系数 a,b,c,d
    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;
    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    // 输出点集合的坐标
    for (size_t i = 0; i < inliers->indices.size(); ++i)
        std::cerr << inliers->indices[i] << "    " << in_cloud_ptr->points[inliers->indices[i]].x << " "
                  << in_cloud_ptr->points[inliers->indices[i]].y << " "
                  << in_cloud_ptr->points[inliers->indices[i]].z << std::endl;

    // 可视化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3d viewer"));
    // 左边窗口显示输入的点云（待拟合）
    int v1(0);
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(in_cloud_ptr->makeShared(), 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(in_cloud_ptr->makeShared(), color_in, "cloud_in", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud_in", v1);

    // 右边的窗口显示拟合之后的点云
    int v2(0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>); // 存放最后拟合好的平面
    cloud_out->width = inliers->indices.size();
    cloud_out->height = 1;
    cloud_out->is_dense = false;
    cloud_out->resize(cloud_out->height * cloud_out->width);
    for (size_t i = 0; i < inliers->indices.size(); i++)
    {
        cloud_out->points[i].x = in_cloud_ptr->points[inliers->indices[i]].x;
        cloud_out->points[i].y = in_cloud_ptr->points[inliers->indices[i]].y;
        cloud_out->points[i].z = in_cloud_ptr->points[inliers->indices[i]].z;
    }
    std::cout << "convert succeed!" << std::endl;
    // viewer->createViewPort(0.5, 0, 1, 1, v2);
    // viewer->setBackgroundColor(255, 255, 255, v2);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_out, "cloud_out", v2);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud_out", v2);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud_out", v2);

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce();
    // }
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
void showClussterInCloud(PointCloud<PointXYZ>::Ptr original, PointCloud<PointXYZ>::Ptr cluster)
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

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("点云索引提取结果"));
    viewer->addPointCloud<pcl::PointXYZ>(cluster, "cluster");
    viewer->addPointCloud<pcl::PointXYZ>(cloud_indice, "other cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cluster");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "other cloud");
    viewer->spin();
}

vector<vector<Point2d>> locate2pic(PointCloud<PointXYZ>::Ptr cluster, vector<pcl::PointIndices> ecel)
{
    Mat extrisicMat(4, 4, cv::DataType<double>::type); // Intrisic matrix
    extrisicMat.at<double>(0, 0) = -7.1907391850483116e-03;
    extrisicMat.at<double>(1, 0) = -9.9997142335005851e-01;
    extrisicMat.at<double>(2, 0) = -2.3336137706425064e-03;
    extrisicMat.at<double>(3, 0) = 0;
    extrisicMat.at<double>(0, 1) = 1.0494953004635377e-02;
    extrisicMat.at<double>(1, 1) = 2.2580773589691017e-03;
    extrisicMat.at<double>(2, 1) = -9.9994237686382270e-01;
    extrisicMat.at<double>(3, 1) = 0;
    extrisicMat.at<double>(0, 2) = 9.9991907134097757e-01;
    extrisicMat.at<double>(1, 2) = -7.2148159989590677e-03;
    extrisicMat.at<double>(2, 2) = 1.0478415848689249e-02;
    extrisicMat.at<double>(3, 2) = 0;
    extrisicMat.at<double>(0, 3) = 1.0984281510814174e-01;
    extrisicMat.at<double>(1, 3) = -1.8261670813403203e-02;
    extrisicMat.at<double>(2, 3) = 1.7323651488230618e-01;
    extrisicMat.at<double>(3, 3) = 1;

    Mat ConextrisicMat(4, 4, cv::DataType<double>::type); // Intrisic matrix
    ConextrisicMat.at<double>(0, 0) = -0.00719074;
    ConextrisicMat.at<double>(1, 0) = 0.01049495;
    ConextrisicMat.at<double>(2, 0) = 0.99991907;
    ConextrisicMat.at<double>(3, 0) = 0;
    ConextrisicMat.at<double>(0, 1) = -0.99997142;
    ConextrisicMat.at<double>(1, 1) = 0.00225808;
    ConextrisicMat.at<double>(2, 1) = -0.00721482;
    ConextrisicMat.at<double>(3, 1) = 0;
    ConextrisicMat.at<double>(0, 2) = -0.00233361;
    ConextrisicMat.at<double>(1, 2) = -0.99994238;
    ConextrisicMat.at<double>(2, 2) = 0.01047842;
    ConextrisicMat.at<double>(3, 2) = 0;
    ConextrisicMat.at<double>(0, 3) = -0.01706703;
    ConextrisicMat.at<double>(1, 3) = 0.17211497;
    ConextrisicMat.at<double>(2, 3) = -0.11178092;
    ConextrisicMat.at<double>(3, 3) = 1;

    Mat intrisicMat(3, 4, cv::DataType<double>::type); // Intrisic matrix
    intrisicMat.at<double>(0, 0) = 1.3859739625395162e+03;
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;
    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 1.3815353250336800e+03;
    intrisicMat.at<double>(2, 1) = 0;
    intrisicMat.at<double>(0, 2) = 9.3622464596653492e+02;
    intrisicMat.at<double>(1, 2) = 4.9459467170828475e+02;
    intrisicMat.at<double>(2, 2) = 1;
    intrisicMat.at<double>(0, 3) = 1;
    intrisicMat.at<double>(1, 3) = 1;
    intrisicMat.at<double>(2, 3) = 1;

    vector<vector<Point3d>> zhxin;
    zhxin.resize(ecel.size());
    vector<vector<Point2d>> imgpoint;
    imgpoint.resize(ecel.size());

    for (int i = 0; i < ecel.size(); i++)
    {
        int clusters_size = ecel[i].indices.size();
        // cout << "第" << i << "组聚类的坐标是：" << endl;
        for (int j = 0; j < clusters_size; j++)
        {

            Point3d t;
            // cout << "(" << cluster->points[ecel[i].indices[j]].x << "," << cluster->points[ecel[i].indices[j]].y << "," << cluster->points[ecel[i].indices[j]].z << ")" << endl;
            t.x = cluster->points[ecel[i].indices[j]].x;
            t.y = cluster->points[ecel[i].indices[j]].y;
            t.z = cluster->points[ecel[i].indices[j]].z;

            zhxin[i].push_back(t);
        }
        // cout<<"zhxin第"<<i<<"组的数量是："<<zhxin[i].size()<<endl;
    }

    imgpoint.resize(ecel.size());
    for (int i = 0; i < ecel.size(); i++)
    {
        imgpoint[i].resize(zhxin[i].size());

        // cout << "第" << i << "类聚类的像素坐标是：" << endl;
        //  cout << "zhxin " << i << "size is:" << zhxin[i].size() << endl;
        for (int j = 0; j < zhxin[i].size(); j++)
        {

            Mat m(4, 1, cv::DataType<double>::type);
            m.at<double>(0) = zhxin[i][j].x;
            m.at<double>(1) = zhxin[i][j].y;
            m.at<double>(2) = zhxin[i][j].z;
            m.at<double>(3) = 1;

            Mat cam = ConextrisicMat * m;

            double zc = cam.at<double>(2);
            Mat imgp = intrisicMat * cam;

            imgpoint[i][j].x = static_cast<int>(imgp.at<double>(0) / zc);
            imgpoint[i][j].y = static_cast<int>(imgp.at<double>(1) / zc);
            // cout << "(" << imgpoint[i][j].x << "," << imgpoint[i][j].y << ")" << endl;
        }
    }
    return imgpoint;
}
double stringTodouble(string a)
{
    stringstream s(a);
    double b;
    s >> b;
    return b;
}

int main(int argc, char **argv)
{
    cv::Size img_size = cv::Size(1280, 1024);
    string des_locate = "/home/alyssa/radar_datas/1";
    Lidar lid("os-122139001023.local", "");
    future<void> poll_future; // 与promise关联的future对象
    promise<void> poll_exit;  // 激光雷达扫描线程退出信号
    thread poll;              // 激光雷达扫描线程
    poll_future = poll_exit.get_future();

    poll = thread([&]
                  { lid.openScan(poll_future); });

    // lid.showViz();
    pcl::console::TicToc time;
    pcl::console::TicToc time1;
    pcl::console::TicToc time2;
    pcl::console::TicToc time3;
    pcl::console::TicToc time4;
    pcl::console::TicToc time5;
    pcl::console::TicToc time6;
    pcl::console::TicToc time7;
    pcl::console::TicToc time8;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud5(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud6(new pcl::PointCloud<pcl::PointXYZ>);

    locate loc;
    string csv_location = "/home/alyssa/save_point/7/cloud_0.csv";

    // string csv_location = "/home/alyssa/Radar/1.pcd";

    // int group =15;

    string lineStr00;
    vector<vector<double>> strArray, strArray1;

    ifstream m00(csv_location);
    while (getline(m00, lineStr00))
    {
        stringstream ss00(lineStr00);
        string str00;
        vector<double> lineArray00;
        while (getline(ss00, str00, ','))
        {
            lineArray00.push_back(stringTodouble(str00));
        }
        strArray.push_back(lineArray00);
    }
    // strArray1.resize(strArray.size());

    time.tic();
    time1.tic();
    vector<double> t;
    t.resize(3);
    cloud->points.resize(strArray.size());
    int j = 0;
    for (int i = 0; i < strArray.size(); i++)
    {
        double _x = strArray[i][0];
        double _y = strArray[i][1];
        double _z = strArray[i][2];

        // 条件一是dis>2
        double dis = _x * _x + _y * _y;

        double a1 = 0.9 * _x + _y + 4.5;
        double a2 = 2 * _x - _y - 19;
        double a3 = 0.7 * _x + _y - 5.3;
        double a4 = 1.2 * _x - _y + 6;

        if (dis > 2 && a1 > 0 && a2 < 0 && a3 < 0 && a4 > 0 && _z > -0.5 && _z < 2)
        {

            cloud->points[j].x = _x;
            cloud->points[j].y = _y;
            cloud->points[j].z = _z;
            j++;
        }
    }
    double t1 = time1.toc();

    time2.tic();
    cloud2 = deletePlanet(cloud);
    pcl::PointCloud<PointT>::Ptr cloud_downSample(new pcl::PointCloud<PointT>);
    VoxelCentriodDownSample vcds; // 创建三维体素格网化对象
    vcds.setInputCloud(cloud2);   // 设置输入点云
    vcds.setGridStep(0.2);        // 设置体素格网边长
    vcds.downSample(cloud3);      // 执行体素质心下采样，并将采样结果保存到cloud_downSample中
    cout << "点云数量为：" << cloud->points.size() << endl;

    cout << "删除平面之后点数为：" << cloud2->points.size() << endl;

    cout << "下采样之后点数：" << cloud3->points.size() << endl;
    double t2 = time2.toc();
    time3.tic();
    vector<vector<Point3d>> objp;
    vector<double> stdd;
    ignore_cloud ig_cloud;

    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);
    cloud6 = ig_cloud.EucliCluster(cloud3, objp, stdd);

    double t3 = time3.toc();
    cout << "->筛选点云时间为：" << t1 << "ms" << endl;
    cout << "->删除平面+下采样的时间为：" << t2 << "ms" << endl;
    cout << "->聚类时间为：" << t3 << "ms" << endl;
    cout << "->总共用时为：" << time.toc() << "ms" << endl;
    // cout<<"组别："<<group<<endl;

    showClussterInCloud(cloud3, cloud6);
    showClussterInCloud(cloud, cloud6);

    // double t2 = time2.toc();

    // time5.tic();
    // // getIndex(cloud5);

    // vector<double> cloud_dis;
    // ignore_cloud ig_cloud;
    // vector<vector<Point3d>> objp;
    // vector<double> stdd;
    // cloud6 = ig_cloud.EucliCluster(cloud_downSample, objp, stdd);
    // getDis getd;
    // vector<Eigen::Vector4f> zxgroup = getd.getZhiXin(objp, stdd);
    // getd.getDistance(zxgroup);
    // double t5 = time5.toc();

    // //  传参之后一切正常
    // //   cout << "传参数之后：" << endl;
    // //   for (int i = 0; i < imgp.size(); i++)
    // //   {
    // //       cout << "第" << i << "组坐标是：" << endl;
    // //       for (int j = 0; j < imgp[i].size(); j++)
    // //       {
    // //           cout << "(" << imgp[i][j].x << "," << imgp[i][j].y << ")" << endl;
    // //       }
    // //   }

    // // 对点云进行定位
    // time8.tic();
    // // vector<vector<Point2d>> imgp = locate2pic(cloud6, ig_cloud.ecel);
    // double t8 = time8.toc();
    // //  cout << "cloud5 size is:" << cloud5->points.size() << endl;
    // //  展示聚类点云------------------------------------------------------------
    // time6.tic();
    // // show3Dpoint(cloud);
    // // show3Dpoint(cloud2);
    // // show3Dpoint(cloud3);
    // // show3Dpoint(cloud4); // 删除xyz方向的点
    // // show3Dpoint(cloud5); // 删除平面
    // // show3Dpoint(cloud_downSample);//下采样之后

    // // showClussterInCloud(cloud2, cloud4);
    // showClussterInCloud(cloud2, cloud6);
    // showClussterInCloud(cloud4, cloud6);
    // showClussterInCloud(cloud_downSample, cloud6);

    // // show3Dpoint(cloud6);

    // // 鼠标回调函数，获取x/y/z的值
    // // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("viewer"));
    // // viewer->addPointCloud(cloud3, "cloud3");
    // // viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    // // CallbackArgs cb_args;
    // // PointCloud<PointXYZRGB>::Ptr clicked_points_3d(new PointCloud<PointXYZRGB>);
    // // cb_args.clicked_points_3d = clicked_points_3d;
    // // cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    // // viewer->registerPointPickingCallback(pickPointCallback, (void *)&cb_args);
    // // std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
    // // viewer->spin();

    // double t6 = time6.toc();
    // // 展示结束------------------------------------------------------------
    // cout << "->转换为pcd文件的时间为:" << t7 / 1000 << " s" << endl;
    // cout << "->删除附近点的时间为:" << t1 / 1000 << " s" << endl;
    // cout << "->下采样的时间为:" << t2 / 1000 << " s" << endl;
    // cout << "->删除y方向点的时间为:" << t3 / 1000 << " s" << endl;
    // cout << "->删除平面的时间为:" << t4 / 1000 << " s" << endl;
    // // cout << "->定位的时间为:" << t8 / 1000 << " s" << endl;
    // cout << "->3D聚类的时间为:" << t5 / 1000 << " s" << endl;
    // cout << "->最终用时:" << (time.toc() - t6) / 1000 << " s" << endl;

    return (0);
}
