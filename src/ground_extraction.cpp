#include "ground_extraction.h"
#include <pcl/io/pcd_io.h>


//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
using namespace roadmarking;

Ground_Extraction::Ground_Extraction() 
{
	//use the default value
	grid_res_ = 1.5;       
	min_pt_num_grid_ = 50;     
	max_height_diff_ = 0.25; 
	max_nei_height_diff_= 1.0;
}

void  Ground_Extraction::Get_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	double max_x, double max_y, double min_x, double min_y,
	int row, int list, int num_voxel, Voxel* grid, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud)
{
	int temp_num_voxel;
	int ExpandGrid_List, ExpandGrid_Row;
	ExpandGrid_List = list + 2;
	ExpandGrid_Row = row + 2;
	temp_num_voxel = ExpandGrid_Row*ExpandGrid_List;
	Voxel *temp_grid = new Voxel[temp_num_voxel];

	float non_ground_z_min = FLT_MAX;

	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].z > non_ground_z_min)
		{
			no_ground_cloud->points.push_back(cloud->points[i]);
			continue;
		}

		int temp_row, temp_list, temp_num;
		temp_list = floor((cloud->points[i].x - min_x) / grid_res_);
		temp_row = floor((cloud->points[i].y - min_y) / grid_res_);
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
	delete[] temp_grid;
}


void Ground_Extraction::Seg_ground_nground_pts(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
	pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
	Voxel* grid, int num_voxel)
{
	int n = 0, m = 0;
	for (int i = 0; i < num_voxel; i++)
	{
		for (int j = 0; j < grid[i].point_id.size(); j++)
		{
			if (cloud->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_diff_
				&&grid[i].min_z - grid[i].NeighborMin_z < max_nei_height_diff_)
				ground_cloud->points.push_back(cloud->points[grid[i].point_id[j]]);
			else
				no_ground_cloud->points.push_back(cloud->points[grid[i].point_id[j]]);
		}
	}
}

void Ground_Extraction::Extract_ground_pts(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
	                                       pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
	                                       pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
	                                       Bounds bounds, CenterPoint center_pt)
{
	int row, list, num_voxel; 
	row = ceil((bounds.max_y - bounds.min_y) / grid_res_);
	list = ceil((bounds.max_x - bounds.min_x) / grid_res_);
	num_voxel = row*list;

	Voxel* grid = new Voxel[num_voxel];
	for (int i = 0; i < num_voxel; i++)
	{
		grid[i].min_z = FLT_MAX;
	}
	Get_grid(cloud, bounds.max_x, bounds.max_y, bounds.min_x, bounds.min_y, row, list, num_voxel, grid, no_ground_cloud);
	Seg_ground_nground_pts(cloud, ground_cloud, no_ground_cloud, grid, num_voxel);
	delete[]grid;
}


