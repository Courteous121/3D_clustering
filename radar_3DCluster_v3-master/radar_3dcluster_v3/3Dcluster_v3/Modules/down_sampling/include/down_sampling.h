/**
 * @file down_sampling.h
 * @author alyssa
 * @brief
 * @version 3.0
 * @date 2023-4-25
 *
 */
#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

using namespace std;

typedef pcl::PointXYZ PointT;

//体素质心下采样类
class VoxelCentriodDownSample
{
public:
	/**
	* @brief   ：设置输入点云
	* @param[I]：cloud_in（输入点云）
	* @param[O]：none
	* @return  : none
	* @note    ：
	**/
	void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_in);

	/**
	* @brief   ：设置体素格网边长
	* @param[I]：step（体素格网边长）
	* @param[O]：none
	* @return  : none
	* @note    ：
	**/
	void setGridStep(double step);

	/**
	* @brief   ：体素下采样
	* @param[I]：none
	* @param[O]：downSampleCloud（下采样点云）
	* @return  : none
	* @note    ：
	**/
	void downSample(pcl::PointCloud<PointT>::Ptr downSampleCloud);

private:
	pcl::PointCloud<PointT>::Ptr m_cloud_in;	//输入点云
	bool is_setInputCloud = false;

	double m_step;		//格网边长
	bool is_setGridStep = false;

	int m_row;			//格网总行数
	int m_col;			//格网总列数
	int m_lay;			//格网总层数

};
