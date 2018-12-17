#include "voxelFilter.h"

#include <fstream>
#include <iomanip>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

using namespace std;

void VoxelFilter::voxelizationxyzlc(const vector<PointXYZLC> &ptlcs, vector<PointXYZLC> &voxelPtlcs)
{
	//voxel大小的倒数;
	float inverse_voxel_size = 1.0f / _voxel_size;

	//获取最大最小;
	PointCloudBound bound;
	getPointXYZLCBound(ptlcs, bound);

	//计算总共的格子数量;
	double xx, yy, zz;
	xx = bound.maxx - bound.minx;
	yy = bound.maxy - bound.miny;
	zz = bound.maxz - bound.minz;
	unsigned long long max_vx = ceil(xx*inverse_voxel_size) + 1;
	unsigned long long max_vy = ceil(yy*inverse_voxel_size) + 1;
	unsigned long long max_vz = ceil(zz*inverse_voxel_size) + 1;

	//判定格子数量是否超过给定值;
	if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
	{
		std::cout << "fail due to so many voxels;"<<endl;
	}

	//计算乘子;
	unsigned long long mul_vx = max_vy*max_vz;
	unsigned long long mul_vy = max_vz;
	unsigned long long mul_vz = 1;

	//计算所有点的位置;
	std::vector<IDPair> id_pairs(ptlcs.size());
	unsigned int idx = 0;

	for (vector<PointXYZLC>::const_iterator it = ptlcs.begin(); it != ptlcs.end(); it++)
	{
		//计算编号;
		unsigned long long vx = floor((it->pt.x() - bound.minx)*inverse_voxel_size);
		unsigned long long vy = floor((it->pt.y() - bound.miny)*inverse_voxel_size);
		unsigned long long vz = floor((it->pt.z() - bound.minz)*inverse_voxel_size);

		//计算格子编号;
		unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

		IDPair pair;
		pair.idx = idx;
		pair.voxel_idx = voxel_idx;
		id_pairs.push_back(pair);

		idx++;
	}

	//进行排序;
	std::sort(id_pairs.begin(), id_pairs.end());

	//保留每个格子中的一个点;
	unsigned int begin_id = 0;


	while (begin_id < id_pairs.size())
	{
		//保留第一个点;
		voxelPtlcs.push_back(ptlcs[id_pairs[begin_id].idx]);

		//往后相同格子的点都不保留;
		unsigned int compare_id = begin_id + 1;
		while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
			compare_id++;
		begin_id = compare_id;
	}

	for (size_t i = 0; i < voxelPtlcs.size(); ++i)
	{
		voxelPtlcs[i].ptIndex = i;
	}
}

void VoxelFilter::outputVoxelPtlcs(const string &filename, const vector<PointXYZLC> &voxelPtlcs)
{
	PointCloudBound bound;
	getPointXYZLCBound(voxelPtlcs, bound);

	ofstream ofs;
	ofs.open(filename, std::ios::out | std::ios::binary);

	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.minx, bound.miny, bound.minz);
		header.SetMax(bound.maxx, bound.maxy, bound.maxz);
		header.SetOffset((bound.minx + bound.maxx) / 2.0, (bound.miny + bound.maxy) / 2.0, (bound.minz + bound.maxz) / 2.0);

		header.SetScale(0.0000001, 0.0000001, 0.0000001);
		header.SetPointRecordsCount(voxelPtlcs.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (size_t i = 0; i < voxelPtlcs.size(); i++)
		{
			pt.SetCoordinates(voxelPtlcs[i].pt.x(), voxelPtlcs[i].pt.y(), voxelPtlcs[i].pt.z());
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}

}

pcXYZIPtr VoxelFilter::filter(const pcXYZIPtr& cloud_ptr)
{
	//voxel大小的倒数
	float inverse_voxel_size = 1.0f / _voxel_size;

	//获取最大最小
	Eigen::Vector4f min_p, max_p;
	pcl::getMinMax3D(*cloud_ptr, min_p, max_p);

	//计算总共的格子数量
	Eigen::Vector4f gap_p;  //boundingbox gap
	gap_p = max_p - min_p;

	unsigned long long max_vx = ceil(gap_p.coeff(0)*inverse_voxel_size) + 1;
	unsigned long long max_vy = ceil(gap_p.coeff(1)*inverse_voxel_size) + 1;
	unsigned long long max_vz = ceil(gap_p.coeff(2)*inverse_voxel_size) + 1;

	//判定格子数量是否超过给定值
	if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
	{
		std::cout << "抽稀失败，最大格子数量过多";
	}

	//计算乘子
	unsigned long long mul_vx = max_vy*max_vz;
	unsigned long long mul_vy = max_vz;
	unsigned long long mul_vz = 1;

	//计算所有点的位置
	std::vector<IDPair> id_pairs(cloud_ptr->size());
	unsigned int idx = 0;
	for (pcXYZI::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++)
	{
		//计算编号
		unsigned long long vx = floor((it->x - min_p.coeff(0))*inverse_voxel_size);
		unsigned long long vy = floor((it->y - min_p.coeff(1))*inverse_voxel_size);
		unsigned long long vz = floor((it->z - min_p.coeff(2))*inverse_voxel_size);

		//计算格子编号
		unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

		IDPair pair;
		pair.idx = idx;
		pair.voxel_idx = voxel_idx;
		id_pairs.push_back(pair);

		idx++;
	}

	//进行排序
	std::sort(id_pairs.begin(), id_pairs.end());

	//保留每个格子中的一个点
	unsigned int begin_id = 0;
	pcXYZIPtr result_ptr(new pcXYZI());
	while (begin_id < id_pairs.size())
	{
		//保留第一个点
		result_ptr->push_back(cloud_ptr->points[id_pairs[begin_id].idx]);

		//往后相同格子的点都不保留
		unsigned int compare_id = begin_id + 1;
		while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
			compare_id++;
		begin_id = compare_id;
	}

	return result_ptr;

}

bool VoxelFilter::outputfcloud(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}