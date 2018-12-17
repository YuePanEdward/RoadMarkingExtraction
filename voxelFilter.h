#ifndef VOXELFILTER_H
#define VOXELFILTER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/common/common.h>
#include <vector>
#include <limits>
#include <iostream>

#include "utility.h"

using namespace utility;

class VoxelFilter: public Cutility
{
public:
	
	float _voxel_size;

	VoxelFilter(float voxel_size) :_voxel_size(voxel_size)
	{

	}

	struct IDPair
	{
		IDPair() :idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;
		//重载比较函数;
		bool operator<(const IDPair& pair) { return voxel_idx < pair.voxel_idx; }
	};

	//进行抽稀;
	void voxelizationxyzlc(const std::vector<PointXYZLC> &ptlcs, std::vector<PointXYZLC> &voxelPtlcs);

	void outputVoxelPtlcs(const std::string &fileName, const std::vector<PointXYZLC> &voxelPtlcs);

	pcXYZIPtr filter(const pcXYZIPtr& cloud_ptr);

	bool outputfcloud(const std::string &fileName, const pcXYZIPtr &pointCloud);
};

#endif