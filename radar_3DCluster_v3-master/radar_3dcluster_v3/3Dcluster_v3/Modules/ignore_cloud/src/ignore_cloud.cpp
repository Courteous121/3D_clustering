/**
 * @file ignore_cloud.cpp
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
#include "ignore_cloud.h"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;
using namespace cv;
using namespace pcl;

// -----------------------------parameter------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale;
pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale;
pcl::PointCloud<pcl::PointNormal>::Ptr doncloud;
pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
pcl::ConditionalRemoval<pcl::PointNormal> cond_remove;
pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond;
pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered;
// -----------------------------parameter------------------------------------------

/**
 * @brief 滤波
 *
 * @param in_cloud
 * @param n_s
 * @param n_l
 * @return
 */
void ignore_cloud::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, double n_s, double n_l, double thresh)
{
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
    double scale_s, scale_l;
    double threshold0;
    scale_s = n_s;
    scale_l = n_l;
    threshold0 = thresh;
    normal_estimation.setInputCloud(in_cloud);
    tree->setInputCloud(in_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setViewPoint(0, 0, 0);
    normal_estimation.setRadiusSearch(scale_s);
    normal_estimation.compute(*normals_small_scale);
    normal_estimation.setRadiusSearch(scale_l);
    normal_estimation.compute(*normals_large_scale);
    std::cout << "read in" << std::endl;

    diffnormals_estimator.setInputCloud(in_cloud);
    diffnormals_estimator.setNormalScaleSmall(normals_small_scale);
    diffnormals_estimator.setNormalScaleLarge(normals_large_scale);

    if (!diffnormals_estimator.initCompute())
    {
        std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
        exit(EXIT_FAILURE);
    }
    copyPointCloud(*in_cloud, *doncloud);
    diffnormals_estimator.computeFeature(*doncloud);
    pcl::PCDWriter writer;
    writer.write<pcl::PointNormal>("/home/alyssa/save_point/3Dclu/don.pcd", *doncloud, false);

    range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
        new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold0)));
    cond_remove.setCondition(range_cond);
    cond_remove.setInputCloud(doncloud);
    cond_remove.filter(*doncloud_filtered);
    doncloud = doncloud_filtered;
    writer.write<pcl::PointNormal>("/home/alyssa/save_point/3Dclu/don_filtered.pcd", *doncloud_filtered, false);
    std::cout << "Filtered Pointcloud: " << doncloud->size() << " data points." << std::endl;
}

/**
 * @brief 随机生成颜色
 *
 * @return
 */
int *ignore_cloud::rand_rgb()
{ // 随机产生颜色
    int *rgb = new int[3];
    rgb[0] = rand() % 255;
    rgb[1] = rand() % 255;
    rgb[2] = rand() % 255;
    return rgb;
}

/**
 * @brief 将三维坐标转换为二维像素坐标
 *
 * @param cloud 三维坐标
 * @param EXM 外参
 * @param INM 内参
 * @return
 */
coordinate get_struct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, cv::Matx44d &EXM, cv::Matx33d &INM, double ratio)
{
    coordinate co;
    co.cloud_clu = cloud;
    vector<Point2d> pixel_box;
    double xmin, xmax, ymin, ymax;
    Matx41d l_point = {cloud->points[0].x, cloud->points[0].y, cloud->points[0].z, 1}; // 4*1
    cv::Matx41d c_point = EXM.inv() * l_point;                                         // 4*4 右乘 4*1
    cv::Matx31d c_p = {c_point(0, 0), c_point(0, 1), c_point(0, 2)};
    cv::Matx31d pi = INM * c_p;
    xmin = (int)(pi(0, 0) / pi(0, 2));
    xmax = (int)(pi(0, 0) / pi(0, 2));
    ymin = (int)(pi(0, 1) / pi(0, 2));
    ymax = (int)(pi(0, 1) / pi(0, 2));
    Eigen::Vector4f centroid;                 // 质心
    pcl::compute3DCentroid(*cloud, centroid); // 齐次坐标，（c0,c1,c2,1）
    co.big_map.x = centroid.x();
    co.big_map.y = centroid.y();
    co.real_map.x = centroid.x() / ratio;
    co.real_map.y = centroid.y() / ratio;
    for (int i = 0; i < cloud->points.size(); i++)
    {
        Matx41d lidar_point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1}; // 4*1
        cv::Matx41d camera_point = EXM.inv() * lidar_point;                                    // 4*4 右乘 4*1
        cv::Matx31d camera_p = {camera_point(0, 0), camera_point(0, 1), camera_point(0, 2)};
        cv::Matx31d pixel = INM * camera_p;
        int u = (int)(pixel(0, 0) / pixel(0, 2));
        int v = (int)(pixel(0, 1) / pixel(0, 2));
        Point2d p(u, v);
        pixel_box.push_back(p);
        if (xmin > u)
            xmin = u;
        if (xmax < u)
            xmax = u;
        if (ymin > v)
            ymin = v;
        if (ymax < v)
            ymax = v;
    }
    co.height = ymax - ymin;
    co.width = xmax - xmin;
    co.tl.x = xmin;
    co.tl.y = ymin;
    return co;
}

/**
 * @brief 聚类函数
 *
 * @param in_cloud 需要聚类的点云
 * @param objp
 * @param stdd
 * @param clust_box 聚类
 * @param ratio 地图和真实点云缩放比例
 * @param coco 筛选聚类的结构体
 * @return
 */
PointCloud<PointXYZ>::Ptr ignore_cloud::EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, vector<vector<Point3d>> &objp, vector<double> &stdd, vector<clustering> &clust_box, double ratio, vector<coordinate> &coco)
{
    // 欧式聚类*******************************************************
    vector<PointIndices> ece_inlier;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    // tree->setInputCloud(out_cloud_ptr);
    EuclideanClusterExtraction<PointXYZ> ece;
    ece.setInputCloud(in_cloud);
    // 0.22 3 20
    ece.setClusterTolerance(0.2); // 值越小，聚类数量越多，这是搜索半径 0.22
    ece.setMinClusterSize(5);
    ece.setMaxClusterSize(25);

    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);
    // ecel = ece_inlier;
    cout << "输入的点云数量是:" << in_cloud->points.size() << endl;
    // 聚类结果展示***************************************************
    srand((unsigned)time(NULL));

    int color_index = 0;
    PointCloud<PointXYZ>::Ptr color_point(new PointCloud<PointXYZ>);
    int ooo = 0;
    // 初次聚类得到的坐标
    for (int i_seg = 0; i_seg < ece_inlier.size(); i_seg++)
    {
        int clusters_size = ece_inlier[i_seg].indices.size();
        PointCloud<PointXYZ>::Ptr box_point(new PointCloud<PointXYZ>);

        for (int i_idx = 0; i_idx < clusters_size; i_idx++)
        {
            PointXYZ point0;
            point0.x = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].x;
            point0.y = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].y;
            point0.z = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].z;
            box_point->push_back(point0);

            // 求得三维坐标的最值
        }
        cv::Matx44d EXM;
        cv::Matx33d INM;
        // 外接矩形判别
        Eigen::Vector3f mf = this->out_box(box_point);
        //&& mf[2] < 0.7&& mf[1] > 0.2 && mf[1] < 0.6 && mf[0] < 0.3 && mf[0] >0.1
        //&& mf[1] > 0.2 && mf[1] < 0.5 && mf[0] < 0.2
        if (mf[2] > 0)
        {
            // 特征值判别
            int value_choose = 1;
            double linear;
            double plainer;
            this->feature_value(box_point, value_choose, linear, plainer);
            // if (linear < -1 && linear > -40 && plainer > -15 && plainer < -1)
            // {
            double x1 = in_cloud->points[ece_inlier[i_seg].indices[0]].x;
            double x2 = in_cloud->points[ece_inlier[i_seg].indices[0]].x;
            double y1 = in_cloud->points[ece_inlier[i_seg].indices[0]].y;
            double y2 = in_cloud->points[ece_inlier[i_seg].indices[0]].y;
            double z1 = in_cloud->points[ece_inlier[i_seg].indices[0]].z;
            double z2 = in_cloud->points[ece_inlier[i_seg].indices[0]].z;
            PointCloud<PointXYZ>::Ptr standard_cloud(new PointCloud<PointXYZ>);
            for (int i_idx = 0; i_idx < clusters_size; i_idx++)
            {
                PointXYZ point0;
                point0.x = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].x;
                point0.y = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].y;
                point0.z = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].z;
                color_point->push_back(point0);
                standard_cloud->push_back(point0);
                if (x1 > point0.x)
                    x1 = point0.x;
                if (x2 < point0.x)
                    x2 = point0.x;
                if (y1 > point0.y)
                    y1 = point0.y;
                if (y2 < point0.y)
                    y2 = point0.y;
                if (z1 > point0.z)
                    z1 = point0.z;
                if (z2 < point0.z)
                    z2 = point0.z;
            }
            // coordinate co = get_struct(standard_cloud, EXM, INM, ratio);
            // coco.push_back(co);
            clustering clust;
            clust.xmin = x1;
            clust.xmax = x2;
            clust.ymin = y1;
            clust.ymax = y2;
            clust.zmin = z1;
            clust.zmax = z2;
            ooo++;
            // cout << "===========>>  " << ooo << "组" << endl;
            // cout << "线性为：" << linear << endl;
            // cout << "平面性为：" << plainer << endl;
            // cout << "外接矩形为：" << mf[0] << "," << mf[1] << "," << mf[2] << endl;
            clust.linear = linear;
            clust.plainet = plainer;
            clust.mf0 = mf[0];
            clust.mf1 = mf[1];
            clust.mf2 = mf[2];
            clust_box.push_back(clust);
            // }
        }
    }
    cout << "聚类中的clust数量为 " << clust_box.size() << endl;
    return color_point;
}

/**
 * @brief 聚类函数
 *
 * @param color_point 聚类
 * @return 外接最小长方体
 */
Eigen::Vector3f ignore_cloud::out_box(pcl::PointCloud<pcl::PointXYZ>::Ptr color_point)
{
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*color_point, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*color_point, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // 校正主方向间垂直
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    // std::cout << "特征值va(3x1):\n"
    //           << eigenValuesPCA << std::endl;
    // std::cout << "特征向量ve(3x3):\n"
    //           << eigenVectorsPCA << std::endl;
    // std::cout << "质心点(4x1):\n"
    //           << pcaCentroid << std::endl;
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();                                     // R.
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>()); //  -R*t
    tm_inv = tm.inverse();
    // std::cout << "变换矩阵tm(4x4):\n"
    //           << tm << std::endl;
    // std::cout << "逆变矩阵tm'(4x4):\n"
    //           << tm_inv << std::endl;
    pcl::PointCloud<PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*color_point, *transformedCloud, tm);
    PointXYZ min_p1, max_p1;
    Eigen::Vector3f c1, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

    // std::cout << "型心c1(3x1):\n"
    //           << c1 << std::endl;

    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);

    Eigen::Vector3f whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    return whd1;
}

/**
 * @brief 排序函数
 *
 */
void ignore_cloud::sort(double &f1, double &f2, double &f3)
{
    double a;
    if (f1 > f2)
    {
        a = f1;
        f1 = f2;
        f2 = a;
    }
    if (f1 > f3)
    {
        a = f1;
        f1 = f3;
        f3 = a;
    }
    if (f2 > f3)
    {
        a = f2;
        f2 = f3;
        f3 = a;
    }
}

/**
 * @brief 聚类性质筛选函数
 *
 * @param color_point 聚类
 * @param linear 线性
 * @param plainer 平面性
 * @return 
 */
void ignore_cloud::feature_value(pcl::PointCloud<pcl::PointXYZ>::Ptr color_point, int order, double &linear, double &plainer)
{
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*color_point, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*color_point, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    double f1 = eigenValuesPCA[0];
    double f2 = eigenValuesPCA[1];
    double f3 = eigenValuesPCA[2];
    this->sort(f1, f2, f3);
    double e1 = f1 / (f1 + f2 + f3);
    double e2 = f2 / (f1 + f2 + f3);
    double e3 = f3 / (f1 + f2 + f3);

    // 线性
    linear = (f1 - f2) / f1;
    // 平面型
    plainer = (f2 - f3) / f1;
}

/**
 * @brief 聚类中聚类筛选函数
 *
 * @param in_cloud 聚类
 * @return 
 */
int ignore_cloud::EucliCluster2(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud)
{
    // 欧式聚类*******************************************************
    vector<PointIndices> ece_inlier3;
    search::KdTree<PointXYZ>::Ptr tree3(new search::KdTree<PointXYZ>);
    // tree->setInputCloud(out_cloud_ptr);
    EuclideanClusterExtraction<PointXYZ> ece3;
    ece3.setInputCloud(in_cloud);
    // 0.11只能聚类到大聚类中的一部分
    ece3.setClusterTolerance(0.1); // 值越小，聚类数量越多，这是搜索半径
    ece3.setMinClusterSize(2);
    ece3.setMaxClusterSize(5);

    ece3.setSearchMethod(tree3);
    ece3.extract(ece_inlier3);
    // ecel = ece_inlier;
    cout << "小型聚类得到的聚类数量是：" << ece_inlier3.size() << endl;
    // cout << "输入的点云数量是:" << in_cloud->points.size() << endl;
    // 聚类结果展示***************************************************
    srand((unsigned)time(NULL));

    int color_index = 0;
    // vector<PointCloud<PointXYZ>::Ptr> color_point0 = vector<PointCloud<PointXYZ>::Ptr>();
    PointCloud<PointXYZ>::Ptr color_point(new PointCloud<PointXYZ>);

    vector<vector<Point3d>> zhxin;
    zhxin.resize(ece_inlier3.size());
    vector<Point2d> imgpoint;
    imgpoint.resize(ece_inlier3.size());
    int m = 0;
    vector<double> verticle_vector;

    for (int i_seg = 0; i_seg < ece_inlier3.size(); i_seg++)
    {
        double x1 = in_cloud->points[ece_inlier3[i_seg].indices[0]].x;
        double x2 = in_cloud->points[ece_inlier3[i_seg].indices[1]].x;
        double x3 = in_cloud->points[ece_inlier3[i_seg].indices[2]].x;

        double y1 = in_cloud->points[ece_inlier3[i_seg].indices[0]].y;
        double y2 = in_cloud->points[ece_inlier3[i_seg].indices[1]].y;
        double y3 = in_cloud->points[ece_inlier3[i_seg].indices[2]].y;

        double z1 = in_cloud->points[ece_inlier3[i_seg].indices[0]].z;
        double z2 = in_cloud->points[ece_inlier3[i_seg].indices[1]].z;
        double z3 = in_cloud->points[ece_inlier3[i_seg].indices[2]].z;

        double x = 1;
        double y = (x3 * z1 - x3 * y2 + x1 * z2 - x2 * z1 - x1 * z3 + x2 * z3) / (-z1 * y3 - z2 * y1 + z2 * y3 + z1 * y2 - z3 * y2 + z3 * y1);
        double z = (x2 - x1 + y * (y2 - y1)) / (z1 - z2);
        cout << "第" << i_seg + 1 << "个小聚类的法向量是：" << endl;
        cout << "(" << x << "," << y << "," << z << ")" << endl;
    }
    return 0;
}
