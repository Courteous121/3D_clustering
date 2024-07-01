#include "locate.h"

// 内参矩阵
double cameraD[3][3] = {{
                            1604.1191539594568,
                            0.,
                            639.83687194220943,

                        },
                        {
                            0.,
                            1604.7833493341714,
                            512.22951297937527,

                        },
                        {
                            0.,
                            0.,
                            1.,

                        }};
Mat cameraMatrix(3, 3, cv::DataType<double>::type, cameraD);

double camerad[3][3] = {
    {0.00062339514711813774414, 0, -3.9887121840711318798},
    {0, 0.00062313720920523210926, -3.1918926110259144071},
    {0, 0, 1}};
Mat cameraMatrix_reverse(3, 3, cv::DataType<double>::type, camerad);

// 外参矩阵

double cameraF[4][4] = {
    {-0.00719074, -0.99997142, -0.00233361, -0.01706703},
    {0.01049495, 0.00225808, -0.99994238, 0.17211497},
    {0.99991907, -0.00721482, 0.01047842, -0.11178092},
    {0, 0, 0, 1}};
Mat exMatrix(4, 4, cv::DataType<double>::type, cameraF);

double cameraf[4][4] = {
    {-0.0071907431823030508, 0.010494957118714257, 0.99991907263570464351, 0.109842810005512436},
    {-0.99997142666810094945, 0.00225807355052380062, -0.0072148168007321332337, -0.01826166945911077088},
    {-0.0023336164322706440315, -0.99994237371863765101, 0.010478412803982456703, 0.17323651037602299033},
    {0, 0, 0, 1}};

Mat exMatrix_reverse(4, 4, cv::DataType<double>::type, cameraf);

// 旋转矩阵
double r[3][3] = {{
                      -0.00719074,
                      -0.99997142,
                      -0.00233361,
                  },
                  {
                      0.01049495,
                      0.00225808,
                      -0.99994238,
                  },
                  {
                      0.99991907,
                      -0.00721482,
                      0.01047842,
                  }};
Mat R(3, 3, cv::DataType<double>::type, r);

double r_re[3][3] = {{-0.0071907431823030508, 0.010494957118714257, 0.99991907263570464351},
                     {-0.99997142666810094945, 0.00225807355052380062, -0.0072148168007321332337},
                     {-0.0023336164322706440315, -0.99994237371863765101, 0.010478412803982456703}};
Mat r_reverse(3, 3, cv::DataType<double>::type, r_re);

// 平移矩阵
double t[3][1] = {-0.01706703, 0.17211497, -0.11178092};
Mat T(3, 1, cv::DataType<double>::type, t);

double locate::calDepth(Point2d &p_p)
{
    // p_p_q是像素齐次矩阵
    double p_p_q0[3][1]{p_p.x, p_p.y, 1};
    Mat p_p_q(3, 1, cv::DataType<double>::type, p_p_q0);
    Mat rightMatrix = r_reverse * cameraMatrix_reverse * p_p_q;
    Mat leftMatrix = r_reverse * T;
    return leftMatrix.at<double>(2, 0) / rightMatrix.at<double>(2, 0);
}

Mat locate::pixel2camera(Point2d &p_p)
{
    double depth = calDepth(p_p);
    double m1 = (p_p.x - cameraMatrix.at<double>(0, 2)) * depth / cameraMatrix.at<double>(0, 0);
    double m2 = (p_p.y - cameraMatrix.at<double>(1, 2)) * depth / cameraMatrix.at<double>(1, 1);
    cout << "(" << p_p.y << "-" << cameraMatrix.at<double>(1, 2) << ")*" << depth << "/" << cameraMatrix.at<double>(1, 1) << "=" << m2 << endl;
    // cout<<"("<<m1<<","<<m2<<","<<depth<<")"<<endl;
    double cam[3][1] = {m1, m2, depth};
    Mat came(3, 1, cv::DataType<double>::type, cam);
    return came;
}

Mat locate::camera2world(Mat &p_c)
{
    double p_c_q0[4][1]{p_c.at<double>(0, 0), p_c.at<double>(1, 0), p_c.at<double>(2.0), 1};
    Mat p_c_q(4, 1, cv::DataType<double>::type, p_c_q0);
    Mat p_w_q = exMatrix_reverse * p_c_q;
    // cout<<p_w_q<<endl;
    return p_w_q;
}
double locate::stringTodouble(string a)
{
    stringstream s(a);
    double b;
    s >> b;
    return b;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr locate::readPCD(std::string pcd_name) // 读入点云文件
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_name, *cloud) < 0)
    {
        PCL_ERROR("无法打开该文件,请检查是该pcd文件是否存在\n");
    }
    return cloud;
}
pcl::PointXYZRGB minpt;                                                       // xyz的最小值
pcl::PointXYZRGB maxpt;                                                       // xyz的最大值
int locate::get_minmax_of_pointcloud(pcl::PointCloud<pcl::PointXYZRGB> cloud) // 获取点云最值坐标
{

    pcl::getMinMax3D(cloud, minpt, maxpt);
    return 1;
}

void locate::locateCloud(string csv_loc, string cloud_loc)
{
    pcl::console::TicToc time;
    time.tic();
    // 读取点云数据
    string lineStr;
    vector<vector<string>> strArray, result;
    ifstream m(csv_loc);

    while (getline(m, lineStr))
    {
        stringstream ss(lineStr);
        string str;
        vector<string> lineArray;
        while (getline(ss, str, ','))
        {
            lineArray.push_back(str);
        }
        strArray.push_back(lineArray);
    }

    // 定义像素坐标,二维坐标转三维坐标不稳定
    double _x, _y, width, height;
    _x = 100000;
    _y = 2400;
    width = 1000000;
    height = 2400;
    Point2d a(_x, _y);                  // tl
    Point2d b(_x + width, _y);          // tr
    Point2d c(_x + width, _y + height); // br
    Point2d d(_x, _y + height);         // bl

    Mat pa_camera = pixel2camera(a);
    Mat pa_world = camera2world(pa_camera);
    // cout << pa_world << endl;

    Mat pb_camera = pixel2camera(b);
    Mat pb_world = camera2world(pb_camera);
    // cout << pb_world << endl;

    Mat pc_camera = pixel2camera(c);
    Mat pc_world = camera2world(pc_camera);
    // cout << pc_world << endl;

    Mat pd_camera = pixel2camera(d);
    Mat pd_world = camera2world(pd_camera);
    // cout << pd_world << endl;

    double x1 = pd_world.at<double>(0, 0);
    double x2 = pb_world.at<double>(0, 0);
    double y1 = pd_world.at<double>(1, 0);
    double y2 = pb_world.at<double>(1, 0);
    double z1 = pa_world.at<double>(2, 0);
    double z2 = pc_world.at<double>(2, 0);
    cout << "(" << x1 << "," << x2 << "),(" << y1 << "," << y2 << "),(" << z1 << "," << z2 << ")" << endl;
    cout << "the point as blew" << endl;
    for (int i = 0; i < strArray.size(); i++)
    {
        double x = stringTodouble(strArray[i][0]);
        double y = stringTodouble(strArray[i][1]);
        double z = stringTodouble(strArray[i][2]);
        // if(x>x1&&x<x2)
        result.push_back(strArray[i]);
        // cout<<strArray[i][0]<<" "<<strArray[i][1]<<" "<<strArray[i][2]<<endl;
    }
    // string cloud_loc = "/home/alyssa/save_point/test_point/test24.pcd";
    ofstream outfile(cloud_loc);
    outfile << "# .PCD v0.7 - Point Cloud Data file format" << endl;
    outfile << "VERSION 0.7" << endl;
    outfile << "FIELDS x y z" << endl;
    outfile << "SIZE 4 4 4" << endl;
    outfile << "TYPE F F F" << endl;
    outfile << "COUNT 1 1 1" << endl;
    outfile << "WIDTH " << result.size() << endl;
    outfile << "HEIGHT 1" << endl;
    outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    outfile << "POINTS " << result.size() << endl;
    outfile << "DATA ascii" << endl;
    for (int i = 0; i < result.size(); i++)
    {
        outfile << result[i][0] << " " << result[i][1] << " " << result[i][2] << endl;
        // cout<<result[i][0]<<" "<<result[i][1]<<" "<<result[i][2]<<endl;
    }

    cout << "选出个数为：" << result.size() << endl;
    cout << "->转换得到pcd文件时间长度为:" << time.toc() / 1000 << " s" << endl;

    // result.clear();

    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=readPCD("/home/alyssa/下载/save_point/test_point/test8.pcd");       //读入点云
    // get_minmax_of_pointcloud(*cloud);       //得到点云*cloud的坐标最大小值
}
void locate::convertcsv2pcd(string csv_loc, string cloud_loc)
{
    pcl::console::TicToc time;
    time.tic();
    // 读取点云数据
    string lineStr;
    vector<vector<string>> strArray;

    ifstream m(csv_loc);

    while (getline(m, lineStr))
    {
        stringstream ss(lineStr);
        string str;
        vector<string> lineArray;
        while (getline(ss, str, ','))
        {
            lineArray.push_back(str);
        }
        strArray.push_back(lineArray);
    }

    ofstream outfile(cloud_loc);
    outfile << "# .PCD v0.7 - Point Cloud Data file format" << endl;
    outfile << "VERSION 0.7" << endl;
    outfile << "FIELDS x y z" << endl;
    outfile << "SIZE 4 4 4" << endl;
    outfile << "TYPE F F F" << endl;
    outfile << "COUNT 1 1 1" << endl;
    outfile << "WIDTH " << strArray.size() << endl;
    outfile << "HEIGHT 1" << endl;
    outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    outfile << "POINTS " << strArray.size() << endl;
    outfile << "DATA ascii" << endl;
    for (int i = 0; i < strArray.size(); i++)
    {

        outfile << strArray[i][0] << " " << strArray[i][1] << " " << strArray[i][2] << endl;
        // cout<<result[i][0]<<" "<<result[i][1]<<" "<<result[i][2]<<endl;
    }
    cout << "->转换得到pcd文件时间长度为:" << time.toc() / 1000 << " s" << endl;
}
