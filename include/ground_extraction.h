#ifndef EXTRACTGROUND_H
#define EXTRACTGROUND_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>

#include "utility.h"


//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
namespace roadmarking
{  
	class Ground_Extraction
	{
	public:

		Ground_Extraction();
		void Extract_ground_pts(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Bounds bounds, CenterPoint center_pt);
		
	protected:

	private:

		void Get_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			double max_x, double max_y, double min_x, double min_y,
			int row, int list, int num_voxel, Voxel* grid, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);

		void Seg_ground_nground_pts(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Voxel* grid, int num_voxel);

		float grid_res_;
		int   min_pt_num_grid_;
		float max_height_diff_;
		float max_nei_height_diff_; 
	};
}
#endif