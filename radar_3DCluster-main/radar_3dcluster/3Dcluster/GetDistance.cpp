#include "GetDistance.h"

vector<Eigen::Vector4f> getDis::getZhiXin(vector<vector<Point3d>> objp)
{
    vector<Eigen::Vector4f> cenbox;
    // cenbox.resize(objp.size());
    // cout << objp.size() << endl;17

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
        // cout << "第" << i << "份数据的质心是："<<endl;
        pcl::compute3DCentroid(*cloud, centroid); // 齐次坐标，（c0,c1,c2,1）
        // cout << "(" << centroid.x() << "," << centroid.y() << "," << centroid.z() << ")" << endl;

        cenbox.push_back(centroid);
    }
    // for (int i = 0; i < cenbox.size(); i++)
    // {
    //     cout<<"第" << i << "份数据的质心是："<<endl;
    //     cout << "(" << cenbox[i].x() << "," << cenbox[i].y() << "," << cenbox[i].z() << ")" << endl;

    // }
    // cout << "聚类的数量为：" << cenbox.size() << endl;
    return cenbox;
}

vector<double> getDis::getDistance(vector<Eigen::Vector4f> center)
{
    // cout << "聚类的数量为：" << center.size() << endl;
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
            // cout<<"("<<x<<","<<y<<","<<z<<") "<<"("<<x1<<","<<y1<<","<<z1<<")"<<endl;
            cloud_dis.push_back(sqrt(x * x1 + y * y1 + z * z1));
        }
    }
    for (int i = 0; i < center.size(); i++)
    {
        double x = center[i].x();
        double y = center[i].y();
        double z = center[i].z();

        cout << "第" << i + 1 << "个聚类离原点的距离为：\t" << sqrt(x * x + y * y + z * z) << "m" << endl;
        cloud_dis.push_back(sqrt(x * x + y * y + z * z));
    }

    return cloud_dis;
}
