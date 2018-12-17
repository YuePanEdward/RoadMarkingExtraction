#ifndef UTILITY_H
#define UTILITY_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr      pcXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI>           pcXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>           pcXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr       pcXYPtr;
typedef  pcl::PointCloud<pcl::PointXY>            pcXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      pcXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>           pcXYZRGB;

typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr       pcXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA>            pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;

namespace utility
{
	struct PointXYZLC
	{
		Eigen::Vector3d pt;
		int ptIndex;
		int kptIndex;
		int objectLabel;
		int objectClass;
		bool isKeypoint;
		PointXYZLC()
		{
			ptIndex = kptIndex = objectClass = objectLabel =-1;
			isKeypoint = false;
		}
	};

	struct PointCloudBound
	{
		double minx;
		double maxx;
		double miny;
		double maxy;
		double minz;
		double maxz;
		PointCloudBound()
		{
			minx = DBL_MAX;
			miny = DBL_MAX;
			minz = DBL_MAX;
			maxx = -DBL_MAX;
			maxy = -DBL_MAX;
			maxz = -DBL_MAX;
		}
	};

	typedef std::vector<PointXYZLC> VePointXYZLC;
	
	class Cutility
	{
	public:
		void getPointXYZLCBound(const VePointXYZLC & ptlcs, PointCloudBound &bound);
		void mergeBound(const PointCloudBound &bound1, const PointCloudBound &bound2, PointCloudBound &bound);
		void vePointXYZLC2PointCloud(const VePointXYZLC & ptlcs, pcXYZ &pc);
		void vePointXYZLC2PointCloud(const VePointXYZLC & ptlcs, pcXYZI &pc);

	protected:
	private:
	};

	
}
#endif