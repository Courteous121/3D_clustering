/**
 * @file down_sampling.cpp
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
#include "down_sampling.h"


/**
* @brief   ：设置输入点云
* @param[I]：cloud_in（输入点云）
* @param[O]：none
* @return  : none
**/
void VoxelCentriodDownSample::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_in)
{
	m_cloud_in = cloud_in;
	is_setInputCloud = true;
}

/**
* @brief   ：设置体素格网边长
* @param[I]：step（体素格网边长）
* @param[O]：none
* @return  : none
**/
void VoxelCentriodDownSample::setGridStep(double step)
{
	if (step > 0)
	{
		m_step = step;
		is_setGridStep = true;
	}
	else
	{
		PCL_ERROR("\a->格网边长应为正数！\n");
		system("pause");
		abort();
	}
}

/**
* @brief   ：体素质心下采样
* @param[I]：none
* @param[O]：downSampleCloud（下采样点云）
* @return  : none
**/
void VoxelCentriodDownSample::downSample(pcl::PointCloud<PointT>::Ptr downSampleCloud)
{
	if (!is_setGridStep)
	{
		PCL_ERROR("\a->请先设置格网边长！\n");
		system("pause");
		abort();
	}
	pcl::PointXYZ min;
	pcl::PointXYZ max;
	pcl::getMinMax3D(*m_cloud_in, min, max);

	m_row = (int)((max.y - min.y) / m_step) + 1;
	m_col = (int)((max.x - min.x) / m_step) + 1;
	m_lay = (int)((max.z - min.z) / m_step) + 1;

	int row_i;			//每一点的行号，从1开始
	int col_i;			//每一点的列号，从1开始
	int lay_i;		//每一点的层号，从1开始
	int grid_id_pt;		//逐行对应的一维格网编号id，从1开始
	multimap<int, PointT> m_grid3D;	//存放水平面格网点云的容器

	//遍历点云，进行三维体素格网化
	size_t num_cp = m_cloud_in->points.size();
	for (size_t i = 0; i < num_cp; i++)
	{
		row_i = (int)((m_cloud_in->points[i].y - min.y) / m_step) + 1;	//每一点的行号，从1开始
		col_i = (int)((m_cloud_in->points[i].x - min.x) / m_step) + 1;	//每一点的列号，从1开始
		lay_i = (int)((m_cloud_in->points[i].z - min.z) / m_step) + 1;//每一点的列号，从1开始

		grid_id_pt = (lay_i - 1) * (m_row * m_col) + (row_i - 1) * m_col + col_i;	//格网一维索引，从1开始
		m_grid3D.insert(pair<int, PointT>(grid_id_pt, m_cloud_in->points[i]));		//将每一个id对应的点坐标存入容器grids2D中
	}

	//判断体素是否为空，若非空，则计算体素内点云质心，以质心代替该体素内的所有点
	for (int lay = 1; lay <= m_lay; lay++)			//层扫描
	{
		for (int row = 1; row <= m_row; row++)		//行扫描
		{
			for (int col = 1; col <= m_col; col++)	//列扫描
			{
				int grid_id;	//逐行对应的一维格网编号id，从1开始
				grid_id = (lay - 1) * (m_row * m_col) + (row - 1) * m_col + col;
				if (m_grid3D.count(grid_id))		//若体素格网内有点，则计算体素质心
				{
					float sum_x, sum_y, sum_z;
					sum_x = sum_y = sum_z = 0;

					auto range = m_grid3D.equal_range(grid_id);
					for(auto it = range.first;it!=range.second; ++it)
					{
						sum_x += (*it).second.x;
						sum_y += (*it).second.y;
						sum_z += (*it).second.z;
					}

					PointT temp;	//临时存放体素质心
					temp.x = sum_x / m_grid3D.count(grid_id);
					temp.y = sum_y / m_grid3D.count(grid_id);
					temp.z = sum_z / m_grid3D.count(grid_id);

					downSampleCloud->push_back(temp);
				}
			}
		}
	}
}
