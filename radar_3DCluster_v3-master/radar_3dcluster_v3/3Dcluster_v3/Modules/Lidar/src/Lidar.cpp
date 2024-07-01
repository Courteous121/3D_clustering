/**
 * @file Lidar.cpp
 * @author 梁俊玮 (1609205169@qq.com)
 * @brief 激光雷达库
 * @version 1.0
 * @date 2022-01-14
 *
 * @copyright Copyright SCUT RobotLab(c) 2022
 *
 */
#include "lidar_scan.h"
#include "Lidar.h"

/**
 * @brief 构造函数
 *
 * @param  sensor_hostname  传感器地址
 * @param  data_destination 传感器要传输到的终点
 */
Lidar::Lidar(string sensor_hostname, string data_destination, cv::Size img_size)
{
    this->img_size = img_size;
    if (sensor_hostname != "")
    {
        // 赋值
        this->sensor_hostname = sensor_hostname;
        this->data_destination = data_destination;
        // 连接
        this->connect();
        // 配置
        // this->configure();

        // 转为XYZ点云
        // 预计算表格，以便从范围有效计算点云
        this->lut = ouster::make_xyz_lut(this->info);
        // 扫描用中间变量
    }
    else
    {
        // 解析元数据
        FileStorage fs("../Setting_file/PathParam.yml", FileStorage::READ);
        string meta_path;

        fs["meta_path"] >> meta_path;
        fs.release();
        this->info = sensor::metadata_from_json(meta_path);
    }

    // 构建初始深度图
    // this->depth_mat = cv::Mat(1280, 1024, CV_64FC1, cv::Scalar::all(0));
    this->depth_mat = cv::Mat(img_size.width, img_size.height, CV_64FC1, cv::Scalar::all(0));
    // 解析数据
    this->prod_line = this->info.prod_line;        // 获取线
    this->w = this->info.format.columns_per_frame; // 每帧列数
    this->h = this->info.format.pixels_per_column; // 每列像素数
    sensor::ColumnWindow column_window = info.format.column_window;
    this->column_window_length = (column_window.second - column_window.first + w) % w + 1;
    // 输出相关信息
    cout << this->prod_line << "线" << endl;
    cout << "每帧列数：" << this->w << endl;
    cout << "每列像素数：" << this->h << endl;
    cout << "列窗口点一：" << column_window.first << endl;
    cout << "列窗口点二：" << column_window.second << endl;
    cout << "滑动窗口大小：" << this->column_window_length << endl;
    // 构建扫描所需要用到的格式
    if (sensor_hostname != "")
    {
        // 准备扫描
        ls_read = shared_ptr<ouster::LidarScan>(
            new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});
        ls_write = shared_ptr<ouster::LidarScan>(
            new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});

        auto packet_format = sensor::get_format(this->info);
        this->batch = shared_ptr<ouster::ScanBatcher>(new ouster::ScanBatcher{this->w, packet_format});

        // 构建雷达数据包与imu数据包
        lidar_buf = std::shared_ptr<uint8_t[]>(
            new uint8_t[packet_format.lidar_packet_size + 1]);
    }
}

/**
 * @brief 连接到激光雷达
 *
 */
void Lidar::connect()
{
    // 获得权柄
    cout << "开始构造雷达" << endl;
    this->handle = sensor::init_client(sensor_hostname, "");
    cout << "权柄获取完成" << endl;
    if (!this->handle)
    {
        this->connect_status = -1;
        cout << "连接失败" << endl;
        return;
    }
    else
    {
        this->connect_status = 1;
        cout << "连接成功" << endl;
    }

    // 获得元数据
    auto metadata = sensor::get_metadata(*(this->handle));
    // 解析元数据
    this->info = sensor::parse_metadata(metadata);
}

/**
 * @brief 开启轮询扫描
 *
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Lidar::openScan()
{
    vector<Point3d> pointbox;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

    // 轮询
    // while (true)
    int g = 0;
    // while (thread_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    // {

    // 轮询等待激光雷达数据
    sensor::client_state st = sensor::poll_client(*(this->handle));
    if (st & sensor::client_state::CLIENT_ERROR)
    {
        std::cerr << "客户端返回错误状态" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    if (st & sensor::client_state::LIDAR_DATA)
    {

        if (sensor::read_lidar_packet(*(this->handle), lidar_buf.get(),
                                      get_format(this->info)))
        {
            if ((*batch)(lidar_buf.get(), *ls_write))
            {

                g++;
                swap(ls_read, ls_write); // 将右边值交给左方
                Eigen::Ref<img_t<uint32_t>> range = (*ls_read).field(sensor::ChanField::RANGE);
                // cloud->points.resize(1024 * 64);
                string na = "/home/alyssa/save_point/code_point/" + to_string(g) + ".txt";
                ofstream out(na);
                int j = 0;
                // 计算三维坐标点
                for (int row = 0; row < range.rows(); row++)
                {
                    for (int col = this->w / 3; col < this->w / 3 * 2; col++)
                    {
                        if (range(row, col) != 0)
                        {

                            double nooffset_x = lut.direction(row * w + col, 0) * range(row, col);
                            double nooffset_y = lut.direction(row * w + col, 1) * range(row, col);
                            double nooffset_z = lut.direction(row * w + col, 2) * range(row, col);
                            double x = nooffset_x == 0 ? 0 : nooffset_x + lut.offset(row * w + col, 0);
                            double y = nooffset_y == 0 ? 0 : nooffset_y + lut.offset(row * w + col, 1);
                            double z = nooffset_z == 0 ? 0 : nooffset_z + lut.offset(row * w + col, 2);
                            double dis = x * x + y * y;
                            double a1 = 0.9 * x + y + 4.5;
                            double a2 = 2 * x - y - 19;
                            double a3 = 0.7 * x + y - 5.3;
                            double a4 = 1.2 * x - y + 6;
                            // if (dis > 2 && a1 > 0 && a2 < 0 && a3 < 0 && a4 > 0 && z > -0.5 && z < 2)
                            // if (dis > 2  && z > -0.5 && z < 2)
                            // {

                                pcl::PointXYZ m(x, y, z);
                                cloud->points.push_back(m);
                            // }
                            // out << to_string(x) << "\t" << to_string(y) << "\t" << to_string(z) << "\n";
                        }
                    }
                }
                return cloud;
            }
        }
    }
    return cloud1;
}
// }

/**
 * @brief 计算深度图
 *
 * @param  CEM              激光雷达到相机的外参矩阵
 * @param  CM               相机内参
 */
// void Lidar::calCoord(cv::Matx44d &CEM, cv::Matx33d &CM)
// {
//     double ts = (double)cv::getTickCount();
//     Eigen::Ref<img_t<uint32_t>> range = (*this->ls_read).field(sensor::ChanField::RANGE);
//     //计算三维坐标点
//     for (int row = 0; row < range.rows(); row++)
//     {
//         for (int col = this->w / 3; col < this->w / 3 * 2; col++)
//         {
//             double nooffset_x = lut.direction(row * w + col, 0) * range(row, col);
// double nooffset_y = lut.direction(row * w + col, 1) * range(row, col);
// double nooffset_z = lut.direction(row * w + col, 2) * range(row, col);
// double x = nooffset_x == 0 ? 0 : nooffset_x + lut.offset(row * w + col, 0);
// double y = nooffset_y == 0 ? 0 : nooffset_y + lut.offset(row * w + col, 1);
// double z = nooffset_z == 0 ? 0 : nooffset_z + lut.offset(row * w + col, 2);
// cv::Matx41d lidar_point = {x, y, z, 1}; //4*1
// //激光雷达坐标系转为相机坐标系
// cv::Matx41d camera_point = CEM.inv() * lidar_point; //4*4 右乘 4*1
// //与像素坐标系建立关系
// cv::Matx31d correct_point;
// if (cal_param.undistcoeff_flag)
// {
//     // 畸变处理。由于标定出来的畸变矩阵不太准确，畸变纠正的效果也不行
//     double r = sqrt(camera_point(0, 0) * camera_point(0, 0) + camera_point(0, 1) * camera_point(0, 1));
//     double k1 = *(main_camera_param.distCoeff.ptr<double>(0, 0));
//     double k2 = *(main_camera_param.distCoeff.ptr<double>(0, 1));
//     double p1 = *(main_camera_param.distCoeff.ptr<double>(0, 2));
//     double p2 = *(main_camera_param.distCoeff.ptr<double>(0, 3));
//     x=camera_point(0, 0);
//     y=camera_point(0, 1);
//     double distort_x = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
//     double distort_y = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
//     correct_point = {distort_x, distort_y, camera_point(0, 2)};
// }
// else
// {
//     correct_point = {camera_point(0, 0), camera_point(0, 1), camera_point(0, 2)};
// }

// cv::Matx31d pixel = CM * correct_point;
// int u = (int)(pixel(0, 0) / pixel(0, 2));
// int v = (int)(pixel(0, 1) / pixel(0, 2));
// if(cal_param.polynomial_flag){
//     // 多项式处理。
//     double detla_u=u-this->img_size.width/2.0;
//     double detla_v=v-this->img_size.height/2.0;
//     double move_u=cal_param.ax*detla_u*detla_u+cal_param.bx*detla_u+cal_param.cx;
//     double move_v=cal_param.ay*detla_v*detla_v+cal_param.by*detla_v+cal_param.cy;
//     u+=detla_u>0?(int)move_u:(int)-move_u;
//     v+=detla_v>0?(int)move_v:(int)-move_v;
// }
// double cam_z = camera_point(0, 2);

// if (u <= 0 || v <= 0 || u >= this->img_size.width || v >= this->img_size.height)
// {
//     continue;
// }
// if (cam_z < 0)
// {
//     continue;
// }
// double original = *(depth_mat.ptr<double>(v, u));
//         if (original != 0)
//         {
//             *(depth_mat.ptr<double>(v, u)) = min(original, cam_z);
//         }
//         else
//         {
//             *(depth_mat.ptr<double>(v, u)) = cam_z;
//         }
//     }
// }
// cv::imshow("img", depth_mat); //显示深度图
// cv::waitKey(1);

// 计算深度图时间
//     double te = (double)cv::getTickCount();
//     double T = (te - ts) * 1000 / cv::getTickFrequency();
//     //cout << "深度图计算时间为：" << T << endl;
// }

/**
 * @brief 可视化
 *
 */
void Lidar::showViz()
{
    // 预计算表格，以便从范围有效计算点云
    const auto xyz_lut = make_xyz_lut(this->info);
    cout << xyz_lut.direction.size() << endl;
    // cout<<xyz_lut.direction.x()<<","<<xyz_lut.direction.y()<<","<<xyz_lut.direction.z()<<endl;
    Eigen::Ref<img_t<uint32_t>> range = (*this->ls_read).field(sensor::ChanField::RANGE);

    // for (int row = 0; row < range.rows(); row++)
    // for (int i = 0; i < xyz_lut.direction.size(); i++)
    //     {
    // for (int col = this->w / 3; col < this->w / 3 * 2; col++)
    // {
    //     double nooffset_x = xyz_lut.direction(row * w + col, 0) * range(row, col);
    //     double nooffset_y = xyz_lut.direction(row * w + col, 1) * range(row, col);
    //     double nooffset_z = xyz_lut.direction(row * w + col, 2) * range(row, col);
    //     double x = nooffset_x == 0 ? 0 : nooffset_x + xyz_lut.offset(row * w + col, 0);
    //     double y = nooffset_y == 0 ? 0 : nooffset_y + xyz_lut.offset(row * w + col, 1);
    //     double z = nooffset_z == 0 ? 0 : nooffset_z + xyz_lut.offset(row * w + col, 2);
    //     cout<<x<<","<<y<<","<<z<<endl;
    // }
    // cout<<xyz_lut.direction(i,0)<<","<<xyz_lut.direction(i,1)<<","<<xyz_lut.direction(i,2)<<endl;
    // }
    // 构建点云可视化类，参数分别为点云配置参数、可视化器的名字、是否在单独的线程中循环运行
    viz::PointViz point_viz(
        {viz::CloudSetup{xyz_lut.direction.data(), xyz_lut.offset.data(), this->h * this->w,
                         this->w, info.extrinsic.data()},
         viz::CloudSetup{xyz_lut.direction.data(), xyz_lut.offset.data(), this->h * this->w,
                         this->w, info.extrinsic.data()}},
        "clouds_viz", false);

    // 雷达扫描可视化类
    viz::LidarScanViz lidar_scan_viz(this->info, point_viz);
    // 条件变量
    std::condition_variable cv;
    // 互斥锁
    std::mutex swap_mtx;
    // 设置雷达扫描准备为未准备
    bool lidar_scan_ready = false;

    // 准备扫描
    std::unique_ptr<ouster::LidarScan> ls_read(
        new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});
    std::unique_ptr<ouster::LidarScan> ls_write(
        new ouster::LidarScan{this->w, this->h, info.format.udp_profile_lidar});

    // 包的种类
    auto packet_format = sensor::get_format(this->info);
    auto batch = ouster::ScanBatcher(this->w, packet_format);

    // 构建雷达数据包与imu数据包
    std::unique_ptr<uint8_t[]> lidar_buf(
        new uint8_t[packet_format.lidar_packet_size + 1]);
    std::unique_ptr<uint8_t[]> imu_buf(
        new uint8_t[packet_format.imu_packet_size + 1]);

    std::thread poll([&]
                     {
                         //只要可视化器不退出则继续执行，开始不断轮询
                         while (!point_viz.quit)
                         {
                             // 轮询等待激光雷达数据
                             sensor::client_state st = sensor::poll_client(*(this->handle));
                             if (st & sensor::client_state::CLIENT_ERROR)
                             {
                                 std::cerr << "客户端返回错误状态" << std::endl;
                                 std::exit(EXIT_FAILURE);
                             }
                             if (st & sensor::client_state::LIDAR_DATA)
                             {
                                 if (sensor::read_lidar_packet(*(this->handle), lidar_buf.get(),
                                                               packet_format))
                                 {
                                     if (batch(lidar_buf.get(), *ls_write))
                                     {
                                         std::lock_guard<std::mutex> lk(swap_mtx);
                                         std::swap(ls_read, ls_write); //将右边值交给左方
                                         //激光扫描完成，数据已经到位
                                         lidar_scan_ready = true;
                                         cv.notify_one(); //随机唤醒一个等待的线程（实际是另外一个线程）
                                     }
                                 }
                             }
                             if (st & sensor::client_state::IMU_DATA)
                             {
                                 //读取，但不发挥作用
                                 sensor::read_imu_packet(*(this->handle), imu_buf.get(), packet_format);
                             }
                             if (st & sensor::EXIT)
                             {
                                 point_viz.quit = true;
                                 break;
                             }
                         } });

    std::thread update_draw([&]()
                            {
                                while (!point_viz.quit)
                                {
                                    std::unique_lock<std::mutex> lk2(swap_mtx);
                                    cv.wait(lk2, [&]()
                                            { return lidar_scan_ready || point_viz.quit; });

                                    if (point_viz.quit)
                                        break;

                                    lidar_scan_ready = false;
                                    double ts = (double)cv::getTickCount();
                                    lidar_scan_viz.draw(*ls_read);
                                    double te = (double)cv::getTickCount();
                                    double T = (te - ts) * 1000 / cv::getTickFrequency();
                                    cout << "扫描时间为：" << T << endl;
                                } });

    point_viz.drawLoop();
    cv.notify_one();
    poll.join();
    update_draw.join();
}