#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "utility.h"
#include "stereoBinaryFeature.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace utility;
using namespace std;

namespace segmentation
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


	class Csegmentation:public Cutility
	{
	public:
		pcXYZIPtr planesegRansac(const pcXYZIPtr &cloud, float threshold);   //Ransac 平面拟合
		pcXYZIPtr groundprojection(const pcXYZIPtr &cloud);                  //点云降维打击
		void cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int N, int MeanK, double std);  //Otsu Intensity Thresholding + SOR Filter
		void SORFilter(const pcXYZI &inclouds, pcXYZI &outclouds, int MeanK, double std); //SOR （Statisics Outliers Remover) 这个也应该写成对所有点云的处理比较好
		void NFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int K); //按总点数再滤一次，小于K个点的删去

		void BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas); //bounding 4 points 提取
		void BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures); //bounding 4 points 几何特征计算
		void CategoryJudgement(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings); //道路标线粗分类
		void MarkingVectorization(const vector<pcXYZI> &clouds, vector<RoadMarking> & roadmarkings, double line_sample_dl);  //道路标线分类及矢量化 //line_sample_dl 线采样点间隔

		void getVisualWordsIndex(const vectorSBF &bscFeats, multiSegmentation &multiObjects);  //视觉单词计算
		void  getGroundTruth(const VePointXYZLC &ptlcs, vector<Object> &groundTruth);

	protected:
		void  getObjectBound(const VePointXYZLC &ptlcs, Object &object);
		bool  isOverlapping(const Object &object1, const Object &object2);
	private:
		
		void  findOverlappingGT(const Object &object, const vector<Object> &groundTruth, vector<Object> &overlappingGT);
		float findMaxRatio(const Object &object, const vector<Object> &overlappingGT);
		size_t calculateIntersection(const std::vector<size_t> &indexes1, const std::vector<size_t> &indexes2);
		size_t calculateUnion(const std::vector<size_t> &indexes1, const std::vector<size_t> &indexes2);
	};
}
#endif