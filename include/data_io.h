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

		struct Paralist {
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

		//Parameter List Related
		void readParalist(string paralistfile);
		void displayparameter(int datatype, int roadtype, int is_road_extracted);

		//Read from folder
		bool batchReadFileNamesInFolders(const std::string &folderName,
			const std::string & extension,
			std::vector<std::string> &fileNames);

		bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName,
			const std::string & extension,
			std::vector<std::string> &fileNames);

		//pcd file
		bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
		bool writePcdFile(const string &fileName, const pcXYZRGBPtr &pointCloud);
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds);
		bool writePcdAll(const string &foldername, const string &fileName, const pcXYZIPtr &NGCloud, const pcXYZIPtr &GCloud, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings); //���أ� ���� �ǵ����Ҳһ�����;
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings, double minX, double minY); //����  ������ߵ��ƾ������� (pcd�е�ë������;
		bool writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings); //����  ������ߵ����������;

		//las file  
		//using liblas (used)
		bool readLasFileHeader(const std::string &fileName, liblas::Header& header);
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud); // ����bounding box
		bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds & bound_3d);
		bool writeLasFile(const std::string &fileName,  pcl::PointCloud<pcl::PointXYZI> &pointCloud, liblas::Color lasColor, double minX, double minY); //����
		bool writeLasAll(int file_index, const string &fileName, vector<pcl::PointCloud<pcl::PointXYZI>> &pointCloud, const RoadMarkings &roadmarkings, double minX, double minY);
		bool writeLasAll(const std::string &folderName, int file_index, const string &fileName,  vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds, const RoadMarkings &roadmarkings, double minX, double minY);
		bool batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames,std::vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds);
		bool batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames,std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &pointClouds);
		bool mergeLasFileHeaders(const std::vector<liblas::Header>& header, liblas::Header& mergeFileHeader);
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
		bool writemarkVectDXF(const RoadMarkings &roadmarkings);
		bool writemarkVectDXF(int file_index, const std::string &foldername, const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings,
			double d_X, double d_Y);
		
		//shp file 
		bool writeRoadmarkingShpWithOffset(const std::string &filename, RoadMarkings & roadmarkings, int file_index,
			float d_x, float d_y, float d_z);

		//display
		void displayroad(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloud);
		void displaymark(const vector<pcXYZI> &clouds);
		void displaymarkwithng(const vector<pcXYZI> &clouds, const pcXYZIPtr &ngcloud);
		void displaymarkbycategory(const vector<pcXYZI> &clouds, const RoadMarkings &roadmarkings);
		void displaymarkVect(const RoadMarkings &roadmarkings);
		void displaymarkVect(const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings); //�������л���ʾ

		bool GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds &bound);
		bool GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, Bounds &bound);
	

		Paralist paralist;
	private:

		void combineCloudBound(const Bounds & bound1, const Bounds & bound2, Bounds & bound);
		void getCloudBound(const pcXYZIPtr &cloud, Bounds & bound);

	};
}

#endif 
