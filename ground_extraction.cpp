#include <StdAfx.h>
#include "ground_extraction.h"
#include "dataio.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace groundExtraction;

CGroundExtraction::CGroundExtraction()
{
	grid_resolution_ = 0.04;
	min_pt_num_in_grid_ = 10;
	max_height_difference_ = 0.1;
}

CGroundExtraction::~CGroundExtraction()
{

}

void  CGroundExtraction::CalculateGridAndMinZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	double max_x, double max_y, double min_x, double min_y,
	int row, int list, int num_voxel, Voxel* grid)
{
	/*遍历每个点，判断点的voxel ID，以及每个voxel的min_z;*/
	int temp_num_voxel;
	int ExpandGrid_List, ExpandGrid_Row;
	ExpandGrid_List = list + 2;
	ExpandGrid_Row = row + 2;
	temp_num_voxel = ExpandGrid_Row*ExpandGrid_List;
	Voxel *temp_grid = new Voxel[temp_num_voxel];

	/*遍历每个点，判断点的voxel ID，以及每个voxel的min_z;*/
	for (int i = 0; i < cloud->points.size(); i++)
	{
		int temp_row, temp_list, temp_num;
		temp_list = floor((cloud->points[i].x - min_x) / grid_resolution_);
		temp_row = floor((cloud->points[i].y - min_y) / grid_resolution_);
		temp_num = temp_row*list + temp_list;
		if (temp_num >= 0 && temp_num < num_voxel)
		{
			grid[temp_num].point_id.push_back(i);
			grid[temp_num].PointsNumber++;
			if (cloud->points[i].z < grid[temp_num].min_z)
			{
				grid[temp_num].min_z = cloud->points[i].z;
				grid[temp_num].NeighborMin_z = cloud->points[i].z;
			}
		}
	}
	//为增广格网赋值
	for (int i = 0; i < num_voxel; i++)
	{
		int ExpandGrid_TempRow, ExpandGrid_TempList, ExpandGrid_TempNum;
		ExpandGrid_TempRow = i / list + 1;
		ExpandGrid_TempList = i%list + 1;
		ExpandGrid_TempNum = ExpandGrid_TempRow*ExpandGrid_List + ExpandGrid_TempList;
		temp_grid[ExpandGrid_TempNum].min_z = grid[i].min_z;
		if (ExpandGrid_TempList == 1 || ExpandGrid_TempRow == 1 || ExpandGrid_TempList == list || ExpandGrid_TempRow == row)
		{
			if (ExpandGrid_TempList == 1)
			{
				temp_grid[ExpandGrid_TempNum - 1].min_z = grid[i].min_z;
				if (ExpandGrid_TempRow == 1)
					temp_grid[ExpandGrid_TempNum - 1 - ExpandGrid_TempList].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempList == list)
				{
					temp_grid[ExpandGrid_TempNum + 1].min_z = grid[i].min_z;
					if (ExpandGrid_TempRow == list)
						temp_grid[ExpandGrid_TempNum + 1 + ExpandGrid_TempList].min_z = grid[i].min_z;
				}
			}
			if (ExpandGrid_TempRow == 1)
			{
				temp_grid[ExpandGrid_TempNum - ExpandGrid_List].min_z = grid[i].min_z;
				if (ExpandGrid_TempList == list)
					temp_grid[ExpandGrid_TempNum + 1 - ExpandGrid_TempList].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempRow == row)
				{
					temp_grid[ExpandGrid_TempNum + ExpandGrid_List].min_z = grid[i].min_z;
					if (ExpandGrid_TempList == 1)
						temp_grid[ExpandGrid_TempNum - 1 + ExpandGrid_TempList].min_z = grid[i].min_z;

				}
			}
		}
	}
	for (int i = 0; i < num_voxel; i++)
	{
		int ExpandGrid_TempRow, ExpandGrid_TempList, ExpandGrid_TempNum;
		ExpandGrid_TempRow = i / list + 1;
		ExpandGrid_TempList = i%list + 1;
		ExpandGrid_TempNum = ExpandGrid_TempRow*ExpandGrid_List + ExpandGrid_TempList;
		for (int j = -1; j < 2; j++)
		{
			for (int k = -1; k<2; k++)
			{
				if (grid[i].NeighborMin_z>temp_grid[ExpandGrid_TempNum + j*ExpandGrid_List + k].min_z && (j != 0 || k != 0))
					grid[i].NeighborMin_z = temp_grid[ExpandGrid_TempNum + j*ExpandGrid_List + k].min_z;
			}
		}
	}
	delete[] temp_grid;
}


void CGroundExtraction::JudgeGroundAndNonGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
	Voxel* grid, int num_voxel)
{
	int n = 0, m = 0;
	for (int i = 0; i < num_voxel; i++)
	{
		for (int j = 0; j < grid[i].point_id.size(); j++)
		{
			if (cloud->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference_&&grid[i].min_z - grid[i].NeighborMin_z < 2 * grid_resolution_)
			{
				ground_cloud->points.push_back(cloud->points[grid[i].point_id[j]]);
			}
			else
			{
				no_ground_cloud->points.push_back(cloud->points[grid[i].point_id[j]]);
			}
		}
	}
}


void CGroundExtraction::FilterNotGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	vector<int>is_ground_points;
	pcl::PointXYZI groundpointsTotempgroudpoints;
	for (int i = 0; i < ground_cloud->points.size(); i++)
	{
		groundpointsTotempgroudpoints = ground_cloud->points[i];
		temp_ground_cloud->push_back(groundpointsTotempgroudpoints);
		is_ground_points.push_back(i);
	}
	ground_cloud->clear();
	set<int, less<int>>Unsearch;
	set<int, less<int>>::iterator iterUnsearch;
	vector<int>isearse;
	for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		isearse.push_back(i);
		Unsearch.insert(i);
	}
	pcl::KdTreeFLANN<pcl::PointXYZI>Kdtree_search_ground_cloud;
	pcl::KdTreeFLANN<pcl::PointXYZI>Kdtree_search_cloud;
	Kdtree_search_ground_cloud.setInputCloud(temp_ground_cloud);
	Kdtree_search_cloud.setInputCloud(cloud);
	float radius = 1.0;
	std::vector<int>PointIdSearch_ground_cloud;
	std::vector<float>PointDistanceSearch_ground_cloud;
	std::vector<int>PointIdSearch_cloud;
	std::vector<float>PointDistanceSearch_cloud;
	pcl::PointXYZI Searchpoint;
	int Pointsub;
	while (!Unsearch.empty())
	{
		iterUnsearch = Unsearch.begin();
		Pointsub = *iterUnsearch;
		Searchpoint = temp_ground_cloud->points[Pointsub];
		Unsearch.erase(Pointsub);
		Kdtree_search_ground_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_ground_cloud, PointDistanceSearch_ground_cloud);
		if (PointIdSearch_ground_cloud.size()<5)
		{
			if (isearse[Pointsub] != -1)
			{
				is_ground_points[Pointsub] = -2;//若搜索到的点数小于20，则将搜索中心点滤除;
				isearse[Pointsub] = -1;
			}
		}
		else
		{
			if (PointIdSearch_ground_cloud.size()>10)
			{
				for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
				{
					if (isearse[PointIdSearch_ground_cloud[i]] != -1)
					{
						Unsearch.erase(PointIdSearch_ground_cloud[i]);
						isearse[PointIdSearch_ground_cloud[i]] = -1;
					}
				}
			}
			else
			{
				Kdtree_search_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_cloud, PointDistanceSearch_cloud);
				if (PointIdSearch_cloud.size() > 2 * PointIdSearch_ground_cloud.size())
				{
					for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
					{
						if (isearse[PointIdSearch_ground_cloud[i]] != -1)
						{
							is_ground_points[PointIdSearch_ground_cloud[i]] = -1;//若在原始点云中搜索到的点数大于地面点云中搜索的点数的二倍，则修改为非地面点;
							Unsearch.erase(PointIdSearch_ground_cloud[i]);
							isearse[PointIdSearch_ground_cloud[i]] = -1;
						}
					}
				}
				else
				{
					if (isearse[Pointsub] != -1)
					{
						Unsearch.erase(Pointsub);
						isearse[Pointsub] = -1;
					}
				}
			}

		}
	}
	isearse.clear();
	vector<int>().swap(isearse);
	PointIdSearch_cloud.clear();
	vector<int>().swap(PointIdSearch_cloud);
	PointDistanceSearch_cloud.clear();
	vector<float>().swap(PointDistanceSearch_cloud);
	PointIdSearch_ground_cloud.clear();
	vector<int>().swap(PointIdSearch_ground_cloud);
	PointDistanceSearch_ground_cloud.clear();
	vector<float>().swap(PointDistanceSearch_ground_cloud);
	for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		if (is_ground_points[i] != -1)//不是非地面点;
		{
			if (is_ground_points[i] != -2)//不是直接剔出点;
			{
				ground_cloud->push_back(temp_ground_cloud->points[i]);
			}
		}
		else
		{
			no_ground_cloud->points.push_back(temp_ground_cloud->points[i]);
		}
	}
	temp_ground_cloud->clear();
	pcl::PointCloud<pcl::PointXYZI>().swap(*temp_ground_cloud);
	is_ground_points.clear();
	vector<int>().swap(is_ground_points);
}



void CGroundExtraction::ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	                                       pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
	                                       pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
	Bounds bounds, CenterPoint center_pt)
{
	/*将点云划分格网，并初始化;*/
	int row, list, num_voxel;//格网的行、列以及格网个数;
	row = ceil((bounds.max_y - bounds.min_y) / grid_resolution_);
	list = ceil((bounds.max_x - bounds.min_x) / grid_resolution_);
	num_voxel = row*list;

	Voxel* grid = new Voxel[num_voxel];
	for (int i = 0; i < num_voxel; i++)
	{
		grid[i].min_z = FLT_MAX;
	}
	/*计算每个格网中的最低点高程;*/
	CalculateGridAndMinZ(cloud, bounds.max_x, bounds.max_y, bounds.min_x, bounds.min_y, row, list, num_voxel, grid);
	/*根据高程差值判断地面点;*/
	JudgeGroundAndNonGroundPoints(cloud, ground_cloud, no_ground_cloud, grid, num_voxel);
	/*滤除误分非地面点*/
	FilterNotGroundPoint(cloud, ground_cloud, no_ground_cloud);

	for (int i = 0; i < ground_cloud->points.size(); i++)
	{
		ground_cloud->points[i].data[3] = i;
	}

	for (int j = 0; j < no_ground_cloud->points.size(); j++)
	{
		no_ground_cloud->points[j].data[3] = j;
	}
	delete[]grid;
}


