#include "dataIo.h"
#include "utility.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>
#include <pcl/visualization/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iosfwd>
#include <fstream>
#include <intsafe.h>

using namespace  std;
using namespace utility;

void DataIo::readPointXYZLC(const string &fileName, vector<PointXYZLC> &ptlcs)
{
	ifstream ifs(fileName);

	if (ifs.fail())
	{
		cout << "can not open the file." << endl;
		return;
	}

	PointXYZLC ptlc;
	size_t ptIndex = 0;
	size_t i = 0;
	while (!ifs.eof())
	{
		//if (i % 10 == 0){

			ifs >> ptlc.pt[0] >> ptlc.pt[1] >> ptlc.pt[2] >> ptlc.objectLabel >> ptlc.objectClass;  //注意ptlc.pt的格式，是Vector型，不是点啊
			ptlc.ptIndex = ptIndex;
			ptlcs.push_back(ptlc);

			ptIndex++;
			//i++;
			//cout << ptIndex << endl;
			ifs.get();
			if (ifs.peek() == EOF)
				break;
		//}
	}
}
bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::loadPCDFile(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}

bool DataIo::writePcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}


void DataIo::display(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;
	float maxi,maxvi,mini,maxz,minz;
	maxi = -FLT_MAX;
	mini = FLT_MAX;
	maxz = -FLT_MAX;
	minz = FLT_MAX;

	int startcolor[3];
	int endcolor[3];
	int rr, gg, bb;
	float kk;

	//cout << "Input the color scale that you want to render the point cloud." << endl;
	//cout << "Start Color R,G,B:" << endl;

	//cin >> startcolor[0] >> startcolor[1] >> startcolor[2];

	//cout << "End Color R,G,B:" << endl;
	//cin >> endcolor[0] >> endcolor[1] >> endcolor[2];
	

	startcolor[0] = 255; startcolor[1] = 0; startcolor[2] = 0;
	endcolor[0] = 0; endcolor[1] = 255; endcolor[2] = 0;

	rr = startcolor[0] - endcolor[0];
	gg = startcolor[1] - endcolor[1];
	bb = startcolor[2] - endcolor[2];

	//maxvi = 10000;
	pcXYZRGBPtr GC(new pcXYZRGB());
	pcXYZRGBPtr NGC(new pcXYZRGB());

	for (size_t i = 0; i < ngcloud->points.size(); ++i)
	{
		if (ngcloud->points[i].z > maxz) maxz = ngcloud->points[i].z;
		if (ngcloud->points[i].z < minz) minz = ngcloud->points[i].z;
	}


	for (size_t i = 0; i < gcloud->points.size(); ++i)
	{
		if (gcloud->points[i].intensity > maxi) maxi = gcloud->points[i].intensity;
		if (gcloud->points[i].intensity < mini) mini = gcloud->points[i].intensity;
	}


	//cout << "max intensity: " << maxi << "\tmin intensity: " << mini<<endl;

	for (size_t i = 0; i < ngcloud->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = ngcloud->points[i].x;
		pt.y = ngcloud->points[i].y;
		pt.z = ngcloud->points[i].z;
		
		kk = (ngcloud->points[i].z - minz) / (maxz - minz);
		pt.r = endcolor[0] + rr * kk;
		pt.g = endcolor[1] + gg * kk;
		pt.b = endcolor[2] + bb * kk;
		NGC->points.push_back(pt);
	}
	
	viewer->addPointCloud(NGC, "Non-ground");

	// Ground points are rendered in intensity
	for (size_t i = 0; i < gcloud->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = gcloud->points[i].x;
		pt.y = gcloud->points[i].y;
		pt.z = gcloud->points[i].z;
		/*if (gcloud->points[i].intensity<maxvi)
		{
			pt.r = 255;
			pt.g = 255;
			pt.b = 255;
		}
		else{
			pt.r = 0;
			pt.g = 0;
			pt.b = 0;
		}*/
		//pt.r = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);
		//pt.g = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);
		//pt.b = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);

		pt.r = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
		pt.g = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
		pt.b = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
		GC->points.push_back(pt);
	}

	viewer->addPointCloud(GC, "Ground");

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


bool DataIo::readLasFileHeader(const std::string &fileName, liblas::Header& header)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}
	else
	{
		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			return 0;
		}

		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		header = reader.GetHeader();
	}

	return 1;
}

bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();
	
	//header里可以直接提bounding box 出来
	double Xmin, Ymin, Xmax, Ymax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();

	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		pcl::PointXYZI  pt;
		/*将重心化后的坐标和强度值赋值给PCL中的点;*/
		/*做一个平移，否则在WGS84 下的点坐标太大了，会造成精度损失的 因为las的读取点数据是double的，而pcd是int的*/
		pt.x = p.GetX()-Xmin;
		pt.y = p.GetY()-Ymin;  
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		pointCloud.points.push_back(pt);
	}

	return 1;
}

