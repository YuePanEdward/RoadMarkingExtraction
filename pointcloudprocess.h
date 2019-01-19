#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "stdafx.h"
#include "utility.h"
#include "KMeans.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace utility;
using namespace std;

namespace PCprocess
{
	typedef std::vector<float> BagOfVisualWords;
	typedef std::vector<float> VisualWordsHistogram;

	struct RoadMarking  
	{
		int category;  //Category,from 1 to 5
		
		vector<pcl::PointXYZI> polyline; //For vectorization

		//Other semantic or topological information
	};

	struct BoundingFeature
	{
		double corner; //角度制
		vector<double> sortingEdges; //排序后的四条边 （升序排列）
	};

	struct SegMetric
	{
		size_t detectedNum;
		size_t groundTruthNum;
		size_t correctNum;
		float averageRatio;
		float precision;
		float recall;
		float f1;
	};

	struct Object
	{
		size_t objectLabel;
		size_t objectClass;
		int topic;
		PointCloudBound boundingBox;
		std::vector<size_t> ptIndexes;
		std::vector<size_t> kptIndexes;
		std::vector<int> vwIndexes;
		std::vector<int> vw2Indexes;
		BagOfVisualWords bovw;
		VisualWordsHistogram vwh;
		float difference;
		bool isChanged;
		Object()
		{
			objectLabel = objectClass = 0;
			isChanged = true;
		}

	};

	typedef std::vector<Object> OneSegmentation;
	typedef std::vector<OneSegmentation> multiSegmentation;


	class Csegmentation
	{
	public:
		
		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud);   // PMF 法地面滤波;
		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud, int max_window_size, float slope, float initial_distance, float max_distance);   // PMF 法地面滤波 参数重载;
		
		pcXYZIPtr planesegRansac(const pcXYZIPtr &cloud, float threshold);   //Ransac 平面拟合;
		pcXYZIPtr groundprojection(const pcXYZIPtr &cloud);                  //点云降维打击(投影）;
		void cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int N, int MeanK, double std);  //Otsu Intensity Thresholding + SOR Filter 
		void SORFilter(const pcXYZI &inclouds, pcXYZI &outclouds, int MeanK, double std); //SOR （Statisics Outliers Remover) 这个也应该写成对所有点云的处理比较好;
		void NFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int K); //按总点数再滤一次，小于K个点的删去

		void BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas); //bounding 4 points 提取;
		void BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures); //bounding 4 points 几何特征计算;
		void BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds);                 //批提取边界点;
		void CornerExtraction(const vector<pcXYZI> &boundaryclouds, vector<pcXYZI> &cornerclouds, bool UseRadius, int K, float radius, float dis_threshold, float maxcos);     //批提取边界上的角点;
		
		void CategoryJudgementBox_highway(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings); //高速公路 道路标线粗分类(基于bounding box信息）;
		void CategoryJudgementBox_cityroad(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings); //城市道路 道路标线粗分类(基于bounding box信息）;

		void CombineSideLines(const vector<RoadMarking> & roadmarkings, double combine_length, vector <RoadMarking> & combine_sideline_markings);  // 道路边线序列化连接;
		void Find_head2tail(int head_index, const vector<vector<double>> & d_head_tail, vector<bool> & line_used, vector<int> & combineline, double combine_length);   //递归函数
		void Find_tail2head(int tail_index, const vector<vector<double>> & d_tail_head, vector<bool> & line_used, vector<int> & combineline, double combine_length);   //递归函数
		
		void MarkingVectorization_highway(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, const vector<vector<pcl::PointXYZI>> &modeldatas, vector<RoadMarking> & roadmarkings, const vector<bool> & is_rights, double line_sample_dl, double ambiguousRatio);  //道路标线分类及矢量化 //line_sample_dl 线采样点间隔   // ambiguousRatio 模糊搜索比例 //0.2
		void MarkingVectorization_cityroad(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, const vector<vector<pcl::PointXYZI>> &modeldatas, vector<RoadMarking> & roadmarkings, const vector<bool> & is_rights, double line_sample_dl, double ambiguousRatio);  //道路标线分类及矢量化 //line_sample_dl 线采样点间隔   // ambiguousRatio 模糊搜索比例 //0.2

		pcXYZI alphashape(const pcXYZI &cloud, float alpha_value);   //Concave Hull Generation
		
		pcXYZI CornerpointKNN(const pcXYZI &boundarycloud, int K, float disthreshold, float maxcos);                   //KNN corner point extraction 
		pcXYZI CornerpointRadius(const pcXYZI &boundarycloud, float radius, float disthreshold, float maxcos);         //Radius corner point extraction 

		pcXYZI CornerClusteringKMeans(const pcXYZI &cornercloud, int K);


	protected:
	
	private:
		
		
	};
}
#endif