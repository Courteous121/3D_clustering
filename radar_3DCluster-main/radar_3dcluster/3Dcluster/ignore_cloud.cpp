#include "ignore_cloud.h"

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
PointCloud<PointXYZ>::Ptr ignore_cloud::EucliCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, vector<vector<Point3d>> &objp)
{
    // 欧式聚类*******************************************************
    vector<PointIndices> ece_inlier;
    search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
    // tree->setInputCloud(out_cloud_ptr);
    EuclideanClusterExtraction<PointXYZ> ece;
    ece.setInputCloud(in_cloud);
    ece.setClusterTolerance(0.22); // 值越小，聚类数量越多，这是搜索半径
    ece.setMinClusterSize(5);
    ece.setMaxClusterSize(15);

    ece.setSearchMethod(tree);
    ece.extract(ece_inlier);
    // ecel = ece_inlier;
    cout << "输入的点云数量是:" << in_cloud->points.size() << endl;
    cout << "聚类的数量是:" << ece_inlier.size() << endl;
    // 聚类结果展示***************************************************
    srand((unsigned)time(NULL));

    int color_index = 0;
    // vector<PointCloud<PointXYZ>::Ptr> color_point0 = vector<PointCloud<PointXYZ>::Ptr>();
    PointCloud<PointXYZ>::Ptr color_point(new PointCloud<PointXYZ>);

    // color_point0.resize(ece_inlier.size());
    for (int i_seg = 0; i_seg < ece_inlier.size(); i_seg++)
    {
        // cout << "第" << i_seg << "组聚类的坐标是：" << endl;

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
    vector<vector<Point2d>> imgpoint;
    imgpoint.resize(ece_inlier.size());
    int m = 0;
    for (int i = 0; i < ece_inlier.size(); i++)
    {
        int clusters_size = ece_inlier[i].indices.size();
        // cout << "第" << i << "组聚类的坐标是：" << endl;
        for (int j = m; j < m + clusters_size; j++)
        {

            Point3d t;
            // cout << "(" << color_point0->points[ece_inlier[i].indices[j]].x << "," << color_point0->points[ece_inlier[i].indices[j]].y << "," << color_point0->points[ece_inlier[i].indices[j]].z << ")" << endl;
            t.x = color_point->points[j].x;
            t.y = color_point->points[j].y;
            t.z = color_point->points[j].z;
            // cout << "(" << t.x << "," << t.y << "," << t.z << ")" << endl;

            zhxin[i].push_back(t);
        }
        m += clusters_size;
        // cout<<"zhxin第"<<i<<"组的数量是："<<zhxin[i].size()<<endl;
    }
    objp = zhxin;
    imgpoint.resize(ece_inlier.size());
    for (int i = 0; i < ece_inlier.size(); i++)
    {
        imgpoint[i].resize(zhxin[i].size());

        cout << "第" << i << "类聚类的像素坐标是：" << endl;
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
            cout << "(" << imgpoint[i][j].x << "," << imgpoint[i][j].y << ")" << endl;
        }
    }

    return color_point;
}