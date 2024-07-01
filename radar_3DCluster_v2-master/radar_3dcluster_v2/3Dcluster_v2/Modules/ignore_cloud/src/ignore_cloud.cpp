#include "ignore_cloud.h"
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigenvalues>
using namespace Eigen;
using namespace std;
using namespace cv;

using namespace pcl;
pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals;

pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale;
pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale;
pcl::PointCloud<pcl::PointNormal>::Ptr doncloud;
pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
pcl::ConditionalRemoval<pcl::PointNormal> cond_remove;
pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond;
pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered;

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
    // okay
    tree->setInputCloud(in_cloud);
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setViewPoint(0, 0, 0);
    normal_estimation.setRadiusSearch(scale_s);
    normal_estimation.compute(*normals_small_scale);
    normal_estimation.setRadiusSearch(scale_l);
    normal_estimation.compute(*normals_large_scale);
    std::cout << "read in" << std::endl;

    // no
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

int *ignore_cloud::rand_rgb()
{ // 随机产生颜色
    int *rgb = new int[3];
    rgb[0] = rand() % 255;
    rgb[1] = rand() % 255;
    rgb[2] = rand() % 255;
    return rgb;
}
PointCloud<PointXYZ>::Ptr ignore_cloud::EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, vector<vector<Point3d>> &objp, vector<double> &stdd)
{
    // 欧式聚类*******************************************************
    vector<PointIndices> ece_inlier;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    // tree->setInputCloud(out_cloud_ptr);
    EuclideanClusterExtraction<PointXYZ> ece;
    ece.setInputCloud(in_cloud);
    // 0.22 3 15
    ece.setClusterTolerance(0.22); // 值越小，聚类数量越多，这是搜索半径 0.22
    ece.setMinClusterSize(5);
    ece.setMaxClusterSize(20);

    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);
    // ecel = ece_inlier;
    cout << "输入的点云数量是:" << in_cloud->points.size() << endl;
    // 聚类结果展示***************************************************
    srand((unsigned)time(NULL));

    int color_index = 0;
    // vector<PointCloud<PointXYZ>::Ptr> color_point0 = vector<PointCloud<PointXYZ>::Ptr>();
    PointCloud<PointXYZ>::Ptr color_point(new PointCloud<PointXYZ>);

    // color_point0.resize(ece_inlier.size());
    // for (int i_seg = 0; i_seg < ece_inlier.size(); i_seg++)
    // {
    // cout << "第" << i_seg << "组聚类的坐标是：" << endl;

    //}

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
    zhxin.resize(ece_inlier.size());
    vector<Point2d> imgpoint;
    imgpoint.resize(ece_inlier.size());
    int m = 0;

    for (int i_seg = 0; i_seg < ece_inlier.size(); i_seg++)
    {
        int clusters_size = ece_inlier[i_seg].indices.size();
        for (int i_idx = 0; i_idx < clusters_size; i_idx++)
        {

            PointXYZ point0;
            // color_point0[i_seg]->resize(clusters_size);
            point0.x = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].x;
            point0.y = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].y;
            point0.z = in_cloud->points[ece_inlier[i_seg].indices[i_idx]].z;
            color_point->push_back(point0);

            // color_point0[i_seg]->push_back(point0);
        }
    }

    for (int i = 0; i < ece_inlier.size(); i++)
    {
        int clusters_size0 = ece_inlier[i].indices.size();
        // cout << "第" << i << "组聚类的坐标是：" << endl;
        for (int j = m; j < m + clusters_size0; j++)
        {

            Point3d t;
            // cout << "(" << color_point0->points[ece_inlier[i].indices[j]].x << "," << color_point0->points[ece_inlier[i].indices[j]].y << "," << color_point0->points[ece_inlier[i].indices[j]].z << ")" << endl;
            t.x = color_point->points[j].x;
            t.y = color_point->points[j].y;
            t.z = color_point->points[j].z;
            // cout << "(" << t.x << "," << t.y << "," << t.z << ")" << endl;

            zhxin[i].push_back(t);
        }
        m += clusters_size0;
        // cout<<"zhxin第"<<i<<"组的数量是："<<zhxin[i].size()<<endl;
    }

    objp = zhxin;
    imgpoint.resize(ece_inlier.size());

    vector<Eigen::Vector4f> cenbox;
    for (int i = 0; i < objp.size(); i++)
    {
        // cout << objp[i].size();
        // cenbox[i].resize(objp[i].size());
        // cout << "read in" << endl;
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        Eigen::Vector4f centroid; // 质心
        cloud->resize(objp[i].size());
        cloud->points.resize(objp[i].size());
        for (int j = 0; j < objp[i].size(); j++)
        {
            cloud->points[j].x = objp[i][j].x;
            cloud->points[j].y = objp[i][j].y;
            cloud->points[j].z = objp[i][j].z;
        }
        // cout << "第" << i << "份数据的质心是：" << endl;
        pcl::compute3DCentroid(*cloud, centroid); // 齐次坐标，（c0,c1,c2,1）
        // cout << "(" << centroid.x() << "," << centroid.y() << "," << centroid.z() << ")" << endl;

        cenbox.push_back(centroid);
    }

    vector<double> standard;
    for (int i = 0; i < ece_inlier.size(); i++)
    {
        imgpoint.resize(zhxin[i].size());

        // cout << "第" << i << "类聚类的像素坐标是：" << endl;
        Mat m(4, 1, cv::DataType<double>::type);
        m.at<double>(0) = cenbox[i].x();
        m.at<double>(1) = cenbox[i].y();
        m.at<double>(2) = cenbox[i].z();
        m.at<double>(3) = 1;

        Mat cam = ConextrisicMat * m;
        double zc = cam.at<double>(2);
        Mat imgp = intrisicMat * cam;

        imgpoint[i].x = static_cast<int>(imgp.at<double>(0) / zc);
        imgpoint[i].y = static_cast<int>(imgp.at<double>(1) / zc);
        // cout << "(" << imgpoint[i].x << "," << imgpoint[i].y << ")" << endl;
        double x0 = imgpoint[i].x;
        double y0 = imgpoint[i].y;
        //**************速度判定
        if (x0 > 1000 && 0 < y0 && y0 < 960)
        {
            standard.push_back(i);
            cout << "&&&&&&&&&&&&&&&&符合要求的组别为：" << i << endl;
        }
    }

    cout << "聚类的数量是:" << standard.size() << endl;

    stdd = standard;

    PointCloud<PointXYZ>::Ptr cloud_stan(new PointCloud<PointXYZ>);
    PointXYZ point;

    for (int i = 0; i < standard.size(); i++)
    {
        int n = standard[i];
        PointCloud<PointXYZ>::Ptr cloud_stan1(new PointCloud<PointXYZ>);

        int u = zhxin[n].size() - 1;
        double length, weight, height;
        double x1 = 0;
        double x2 = 0;
        double y1 = 0;
        double y2 = 0;
        double z1 = 0;
        double z2 = 0;
        for (int p = 0; p < zhxin[n].size(); p++)
        {
            if (x1 > zhxin[n][p].x)
                x1 = zhxin[n][p].x;
            if (x2 < zhxin[n][p].x)
                x2 = zhxin[n][p].x;
            if (y1 > zhxin[n][p].y)
                y1 = zhxin[n][p].y;
            if (y2 < zhxin[n][p].y)
                y2 = zhxin[n][p].y;
            if (z1 > zhxin[n][p].z)
                z1 = zhxin[n][p].z;
            if (z2 < zhxin[n][p].z)
                z2 = zhxin[n][p].z;
        }
        length = x2 - x1;
        weight = y2 - y1;
        height = z2 - z1;
        // cout << "长宽高分别是："
        //      << "(" << length << "," << weight << "," << height << ")" << endl;
        double dif = abs(length - weight);
        if (length < 5.5 && weight < 5 && height > 0.4)
        {
            cout << "===>>>选中的大型聚类号数为：" << n << endl;

            cout << "长宽高分别是："
                 << "(" << length << "," << weight << "," << height << ")" << endl;

            Mat s0(3, 3, cv::DataType<double>::type);               // Intrisic matrix
            Mat s(3, zhxin[n].size(), cv::DataType<double>::type);  // Intrisic matrix
            Mat s1(zhxin[n].size(), 3, cv::DataType<double>::type); // Intrisic matrix

            for (int j = 0; j < zhxin[n].size(); j++)
            {

                s.at<double>(0,j) = zhxin[n][j].x;
                s.at<double>(1,j) = zhxin[n][j].y;
                s.at<double>(2,j) = zhxin[n][j].z;
                s1.at<double>(j,0) = zhxin[n][j].x;
                s1.at<double>(j,1) = zhxin[n][j].y;
                s1.at<double>(j,2) = zhxin[n][j].z;
            }

            Matrix3d A;
            s0 = s * s1;

            double f1 = s0.at<double>(0, 0);
            double f2 = s0.at<double>(0, 1);
            double f3 = s0.at<double>(0, 2);
            double f4 = s0.at<double>(1, 0);
            double f5 = s0.at<double>(1, 1);
            double f6 = s0.at<double>(1, 2);
            double f7 = s0.at<double>(2, 0);
            double f8 = s0.at<double>(2, 1);
            double f9 = s0.at<double>(2, 2);

            A << f1, f2, f3, f4, f5, f6, f7, f8, f9;

            EigenSolver<Matrix3d> es(A);
            Matrix3d D = es.pseudoEigenvalueMatrix();
            double h1 = D(0, 0);
            double h2 = D(1, 1);
            double h3 = D(2, 2);
            double max;
            if (h1 < h2)
            {
                max = h1, h1 = h2, h2 = max;
            }
            if (h1 < h3)
            {
                max = h1, h1 = h3, h3 = max;
            }
            if (h2 < h3)
            {
                max = h2, h2 = h3, h3 = max;
            }
            if (h1 < h2)
            {
                max = h1, h1 = h2, h2 = max;
            }
            double liner = (h1 - h2) / h1;
            double plane = (h2 - h3) / h1;
            cout << liner << " " << plane << endl;
            if (plane < 0.001)
            {
                for (int o = 0; o < zhxin[n].size(); o++)
                {
                    point.x = zhxin[n][o].x;
                    point.y = zhxin[n][o].y;
                    point.z = zhxin[n][o].z;
                    cloud_stan->push_back(point);
                }
            }
        }
        cout << "read in" << endl;
    }

    return cloud_stan; // 坐标筛选后
}
// 方法可行！！！
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
