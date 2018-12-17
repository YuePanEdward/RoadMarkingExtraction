#ifndef GROUNDEXTRACTION
#define GROUNDEXTRACTION

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace groundExtraction
{   
		struct CenterPoint
		{
			double x;
			double y;
			double z;
			CenterPoint(double x = 0, double y = 0, double z = 0) :
				x(x), y(y), z(z)
			{
				z = 0.0;
				x = y = 0.0;
			}

		};

		struct Bounds
		{
			double min_x;
			double min_y;
			double min_z;
			double max_x;
			double max_y;
			double max_z;
			Bounds()
			{
				min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
			}
		};



		struct Grid
		{
			bool is_empty;
			Grid()
			{
				is_empty = true;
			}
		};

		struct Voxel
		{
			vector<int>point_id;
			float min_z;
			float max_z;
			float dertaz;
			float min_z_x;//格网最低点的X坐标;
			float min_z_y;//格网最低点的y坐标;
			float NeighborMin_z;
			int PointsNumber;
			float mean_z;

			Voxel()
			{
				min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
				PointsNumber = 1;
				dertaz = 0.0;
			}
		};

		struct SimplifiedVoxel
		{
			vector<int>point_id;
			float max_curvature;
			int max_curvature_point_id;
			bool has_keypoint;
			SimplifiedVoxel()
			{
				has_keypoint = false;
			}
		};


		class StructOperator
		{
		public:
			//获取中心
			void getCloudCenterPoint(const pcl::PointCloud<pcl::PointXYZ> & cloud, CenterPoint & centerPoint)
			{
				double cx = 0, cy = 0, cz = 0;

				for (int i = 0; i < cloud.size(); i++)
				{
					cx += cloud.points[i].x / cloud.size();
					cy += cloud.points[i].y / cloud.size();
					cz += cloud.points[i].z / cloud.size();
				}
				centerPoint.x = cx;
				centerPoint.y = cy;
				centerPoint.z = cz;
			}
			//获取边界
			void getCloudBound(const pcl::PointCloud<pcl::PointXYZ> & cloud, Bounds & bound)
			{
				double min_x = cloud[0].x;
				double min_y = cloud[0].y;
				double min_z = cloud[0].z;
				double max_x = cloud[0].x;
				double max_y = cloud[0].y;
				double max_z = cloud[0].z;

				for (int i = 0; i<cloud.size(); i++)
				{
					//获取边界
					if (min_x>cloud.points[i].x)
						min_x = cloud.points[i].x;
					if (min_y > cloud.points[i].y)
						min_y = cloud.points[i].y;
					if (min_z > cloud.points[i].z)
						min_z = cloud.points[i].z;
					if (max_x < cloud.points[i].x)
						max_x = cloud.points[i].x;
					if (max_y < cloud.points[i].y)
						max_y = cloud.points[i].y;
					if (max_z < cloud.points[i].z)
						max_z = cloud.points[i].z;
				}
				bound.min_x = min_x;
				bound.max_x = max_x;
				bound.min_y = min_y;
				bound.max_y = max_y;
				bound.min_z = min_z;
				bound.max_z = max_z;
			}

			void getCloudBound(const pcl::PointCloud<pcl::PointXYZI> & cloud, Bounds & bound)
			{
				double min_x = cloud[0].x;
				double min_y = cloud[0].y;
				double min_z = cloud[0].z;
				double max_x = cloud[0].x;
				double max_y = cloud[0].y;
				double max_z = cloud[0].z;

				for (int i = 0; i<cloud.size(); i++)
				{
					//获取边界
					if (min_x>cloud.points[i].x)
						min_x = cloud.points[i].x;
					if (min_y > cloud.points[i].y)
						min_y = cloud.points[i].y;
					if (min_z > cloud.points[i].z)
						min_z = cloud.points[i].z;
					if (max_x < cloud.points[i].x)
						max_x = cloud.points[i].x;
					if (max_y < cloud.points[i].y)
						max_y = cloud.points[i].y;
					if (max_z < cloud.points[i].z)
						max_z = cloud.points[i].z;
				}
				bound.min_x = min_x;
				bound.max_x = max_x;
				bound.min_y = min_y;
				bound.max_y = max_y;
				bound.min_z = min_z;
				bound.max_z = max_z;
			}

			//获取中心和边界
			void getBoundAndCenter(const pcl::PointCloud<pcl::PointXYZI> & cloud, Bounds & bound, CenterPoint& centerPoint)
			{
				double min_x = cloud[0].x;
				double min_y = cloud[0].y;
				double min_z = cloud[0].z;
				double max_x = cloud[0].x;
				double max_y = cloud[0].y;
				double max_z = cloud[0].z;

				double cx = 0, cy = 0, cz = 0;

				for (int i = 0; i<cloud.size(); i++)
				{
					//获取边界
					if (min_x>cloud.points[i].x)
						min_x = cloud.points[i].x;
					if (min_y > cloud.points[i].y)
						min_y = cloud.points[i].y;
					if (min_z > cloud.points[i].z)
						min_z = cloud.points[i].z;
					if (max_x < cloud.points[i].x)
						max_x = cloud.points[i].x;
					if (max_y < cloud.points[i].y)
						max_y = cloud.points[i].y;
					if (max_z < cloud.points[i].z)
						max_z = cloud.points[i].z;


					cx += cloud.points[i].x / cloud.size();
					cy += cloud.points[i].y / cloud.size();
					cz += cloud.points[i].z / cloud.size();
				}
				bound.min_x = min_x;
				bound.max_x = max_x;
				bound.min_y = min_y;
				bound.max_y = max_y;
				bound.min_z = min_z;
				bound.max_z = max_z;


				centerPoint.x = cx;
				centerPoint.y = cy;
				centerPoint.z = cz;
			}

			void GetSubsetBoundary(pcl::PointCloud<pcl::PointXYZ>::Ptr & plane_wall_cloud,
				vector<int> & index, Bounds & bound)
			{
				//构建点云
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				for (int i = 0; i < index.size(); i++)
				{
					temp_cloud->push_back(plane_wall_cloud->points[index[i]]);
				}
				getCloudBound(*temp_cloud, bound);
			}

	protected:

	private:
	};

	class CGroundExtraction
	{
	public:

		CGroundExtraction();
		~CGroundExtraction();

		/*地面点提取,该函数用来识别地面点和非地面点;*/
		void ExtractGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Bounds bounds, CenterPoint center_pt);

		void SetMaxHeightDifference(float max_height_difference){ max_height_difference_ = max_height_difference; }
		void SetMinPointNumInGrid(int min_pt_num_in_grid){ min_pt_num_in_grid_ = min_pt_num_in_grid; }
		void SetGridResolution(float grid_resolution){ grid_resolution_ = grid_resolution; }

		

	protected:

	private:

	

		/*计算格网中的最低点高程;*/
		void CalculateGridAndMinZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			double max_x, double max_y, double min_x, double min_y,
			int row, int list, int num_voxel, Voxel* grid);

		/*根据点跟Grid中最低点的高程差，判断地面点和非地面点;*/
		void JudgeGroundAndNonGroundPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
			Voxel* grid, int num_voxel);


		/*对地面点进一步判断，如果该地面点高于周围地面点很多，则判断为非地面点;*/
		void FilterNotGroundPoint(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud);

		float grid_resolution_;//格网的分辨率;
		int   min_pt_num_in_grid_;//格网中最小的点个数，小于该点数据认为该格网为噪声点;
		float max_height_difference_;//个网内的点与个网内最低点的高程差,大于该值被认为是非地面点;


	};
}
#endif