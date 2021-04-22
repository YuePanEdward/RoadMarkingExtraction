#ifndef DATAIO_H
#define DATAIO_H

#include "utility.h"
#include "pointcloudprocess.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

//#include <LASlib/lasreader.hpp>
//#include <LASlib/laswriter.hpp>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

// #include <gdal.h>
// #include <gdal_alg.h>
// #include <gdal_priv.h>
// #include <ogrsf_frmts.h>

#include <strstream>
#include <fstream>
#include <vector>
//#include <io.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace roadmarking
{
	class DataIo
	{
	public:
		struct Paralist
		{
			// Default value
			int road_type = 2; //Urban road
			float expected_point_num_per_m_square = 900.0;
			float grid_resolution = 0.08;

			float intensity_scale = 5.0;
			float density_threshold = 2.5;

			float model_matching_heading_increment = 25.0;
			float model_matching_correct_match_fitness_thre = 0.02;
			float model_matching_overlapping_dist_thre = 0.15;
			float model_matching_overlapping_ratio_thre = 0.75;

			float sideline_vector_distance = 4.5;
			bool visualization_on = false;
		};
		/*
		Parameters Notification
		1  road_type （1: highway, 2: urban road) default: 2
		2  point_density (expected average point density on road surface, X point/m^2) default: 900.0
		3  grid_resolution (projected grid size [unit:m]) default: 0.1
		4  intensity_scale (controls the value range of point intensity. Generally, the smaller the value is, the more markings are found, and the more noise are found accordingly). recommended: 2.0-8.0, default: 5.0
		5  density_threshold (describes the approximate number of ground points in a single pixel on the road) recommended: 2.5-5.0, default: 2.5
		6  rotation_increment_step  (controls the increase step size of the initial estimate of the rotation angle during template matching. The larger the value, the more robust the template matching, but the longer time it would take) recommended: 15-45, default: 25.0
		7  matching_fitness_thre (controls the error threshold during template matching. The larger the value, the lower the accuracy and the higher the recall) preferred: 0.01-0.05, default: 0.02
		8  overlaping_distance_thre (controls the search radius [unit:m] of neighboring points when calculating the template matching overlap ratio. The larger the value, the higher the overlap ratio would be) recommended: 0.1-0.2 default: 0.15
		9  overlaping_ratio_thre (controls the criterion for judging that the template matching is successful, that is, the overlapping ratio between the template model and the scene point cloud. The larger this value is, the corresponding matching accuracy would be higher and the recall would be lower) recommended: 0.7-0.8 default: 0.75
		10 sideline_vector_distance (controls the distance threshold [unit: m] for vectorization of long edges. Adjacent edges whose endpoint distance is less than this threshold will be connected into a segment; in addition, the vectorized sampling points will also use this value as the sampling interval) recommended: 3.0-6.0 default: 4.5
		11 visualization_on (1:on, 0:off) default: 0*/

		//Parameter List Related
		void readParalist(string paralistfile);
		void displayparameter(int datatype, int roadtype, int is_road_extracted);

		//Read from folder
		bool batchReadFileNamesInFolders(const std::string &folderName,
										 const std::string &extension,
										 std::vector<std::string> &fileNames);

		bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName,
													  const std::string &extension,
													  std::vector<std::string> &fileNames);

		//pcd file
		bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud, Bounds &bound_3d);
		bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool writePcdFile(const string &fileName, const pcXYZRGBPtr &pointCloud);
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds);
		bool writePcdAll(const string &foldername, const string &fileName, const pcXYZIPtr &NGCloud, const pcXYZIPtr &GCloud, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings); 
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings, double minX, double minY);							
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings);													

		//las file
		//using liblas (used)
		bool readLasFileHeader(const std::string &fileName, liblas::Header &header);
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds &bound_3d);
		bool writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, liblas::Color lasColor, double minX, double minY); 
		bool writeLasAll(int file_index, const string &fileName, vector<pcl::PointCloud<pcl::PointXYZI>> &pointCloud, const RoadMarkings &roadmarkings, double minX, double minY);
		bool writeLasAll(const std::string &folderName, int file_index, const string &fileName, vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds, const RoadMarkings &roadmarkings, double minX, double minY);
		bool batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames, std::vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds);
		bool batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames, std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &pointClouds);
		bool mergeLasFileHeaders(const std::vector<liblas::Header> &header, liblas::Header &mergeFileHeader);
		bool mergeLasFilesColoredByFile(const string &folderName);
		bool mergeLasFilesIntensity(const string &folderName);

		//using LASlib (deprecated)
		//bool readPointCloudFromLasFile(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, std::string &filename, Bounds &bound_3d);
		bool writecolorLASPointCloud(const std::string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
									 int R, int G, int B, double Min_x, double Min_y, double Min_z); // with color and global shift
		// bool writeLasAll(const string &fileName, int file_index, vector<pcl::PointCloud<pcl::PointXYZI>> &pointCloud,
		// 	const RoadMarkings &roadmarkings, double minX, double minY, double minZ);

		//ply file
		bool readPlyFile(const string &fileName, const pcXYZIPtr &pointCloud);

		//txt file
		bool writemarkVectTXT(const RoadMarkings &roadmarkings);

		//dxf file
		bool writemarkVectDXF(int file_index, const std::string &foldername, const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings,
							  double d_X, double d_Y);

		//shp file
		bool writeRoadmarkingShpWithOffset(const std::string &filename, RoadMarkings &roadmarkings, int file_index,
										   float d_X, float d_Y); //need GDAL

		//display
		void displayroad(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloud);
		void displaymark(const vector<pcXYZI> &clouds);
		void displaymarkwithng(const vector<pcXYZI> &clouds, const pcXYZIPtr &ngcloud);
		void displaymarkbycategory(const vector<pcXYZI> &clouds, const RoadMarkings &roadmarkings);
		void displaymarkVect(const RoadMarkings &roadmarkings);
		void displaymarkVect(const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings);

		bool GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds &bound);
		bool GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, Bounds &bound);

		Paralist paralist;

	private:
		void combineCloudBound(const Bounds &bound1, const Bounds &bound2, Bounds &bound);
		void getCloudBound(const pcXYZIPtr &cloud, Bounds &bound);
	};
}

#endif
