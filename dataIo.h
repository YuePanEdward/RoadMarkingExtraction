#ifndef DATAIO_H
#define DATAIO_H


#include "utility.h"
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <vector>
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

//为了不造成Flann冲突（因为OpenCV和PCL里都用了），建议别importCV了这儿
using namespace std;
using namespace utility;


class DataIo: public Cutility
{
public:
	void readPointXYZLC(const string &fileName, vector<PointXYZLC> &pts);
	
	//pcd file
	bool readPcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
	bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);

	//las file  
	bool readLasFileHeader(const std::string &fileName, liblas::Header& header);
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud); // This is great.
	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
	bool mergeLasFileHeaders(const std::vector<liblas::Header>& header, liblas::Header& mergeFileHeader);
	bool mergeLasFilesColoredByFile(const string &folderName);
	bool mergeLasFilesIntensity(const string &folderName);
	
	//display
	void display(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloudT);


private:

};

#endif 
