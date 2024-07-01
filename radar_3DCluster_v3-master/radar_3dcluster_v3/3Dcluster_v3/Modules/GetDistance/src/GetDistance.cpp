/**
 * @file GetDistance.cpp
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
#include "GetDistance.h"

/**
 * @brief get center func
 *
 * @param objp 
 * @param stdd cluster  
 * @return
 */
vector<Eigen::Vector4f> getDis::getZhiXin(vector<vector<Point3d>> objp, vector<double> stdd)
{

    vector<Eigen::Vector4f> cenbox;
    for (int i = 0; i < stdd.size(); i++)
    {
        int m = stdd[i];
        // cout << objp[i].size();
        // cenbox[i].resize(objp[i].size());
        // cout << "read in" << endl;
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
        Eigen::Vector4f centroid; // 质心
        cloud->resize(objp[m].size());
        cloud->points.resize(objp[m].size());
        for (int j = 0; j < objp[m].size(); j++)
        {
            cloud->points[j].x = objp[m][j].x;
            cloud->points[j].y = objp[m][j].y;
            cloud->points[j].z = objp[m][j].z;
        }

        pcl::compute3DCentroid(*cloud, centroid); // 齐次坐标，（c0,c1,c2,1）

        cenbox.push_back(centroid);
    }
    return cenbox;
}

/**
 * @brief get distance func
 *
 * @param center 
 * @return all kind of distance
 */
vector<double> getDis::getDistance(vector<Eigen::Vector4f> center)
{
 
    vector<double> cloud_dis;
    for (int i = 0; i < center.size() - 1; i++)
    {

        for (int j = i + 1; j < center.size(); j++)
        {

            double x = center[i].x();
            double y = center[i].y();
            double z = center[i].z();
            double x1 = center[j].x();
            double y1 = center[j].y();
            double z1 = center[j].z();
            double d = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1) + (z - z1) * (z - z1));
            cout << "第" << i + 1 << "个聚类和第" << j + 1 << "个聚类的距离是：\t" << d << "m" << endl;
            // cloud_dis.push_back(sqrt(x * x1 + y * y1 + z * z1));
        }
    }
    for (int i = 0; i < center.size(); i++)
    {
        double x = center[i].x();
        double y = center[i].y();
        double z = center[i].z();

        cout << "第" << i + 1 << "个聚类离原点的距离为：\t" << sqrt(x * x + y * y + z * z) << "m" << endl;
        // cloud_dis.push_back(sqrt(x * x + y * y + z * z));
    }

    return cloud_dis;
}
