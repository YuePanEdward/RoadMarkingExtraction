#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "utility.h"
#include "stereoBinaryFeature.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

using namespace utility;
using namespace std;

namespace segmentation
{
	typedef std::vector<float> BagOfVisualWords;
	typedef std::vector<float> VisualWordsHistogram;

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
		pcXYZIPtr planesegRansac(const pcXYZIPtr &cloud, float threshold);
		pcXYZIPtr groundprojection(const pcXYZIPtr &cloud);

		void getVisualWordsIndex(const vectorSBF &bscFeats, multiSegmentation &multiObjects);

		void  getGroundTruth(const VePointXYZLC &ptlcs, vector<Object> &groundTruth);

		SegMetric evaluateSegQuality(const VePointXYZLC &ptlcs, const vector<Object> &objects, const vector<Object> &groundTruth, float overlapT);

		void multiSegmentation2Documents(const multiSegmentation &multiObjects, vector<vector<int>> &documents);


		void  displayObjectsByLabel(const VePointXYZLC &ptlcs, const vector<Object> &objects);
		void  displayObjectsByClass(const VePointXYZLC &ptlcs, const vector<Object> &objects);

		void  outputMultiSegmentation(const string &foldername, const VePointXYZLC &ptlcs, const multiSegmentation &multiObject);
		void  outputMultiSegmentationKpt(const string &foldername, const VePointXYZLC &ptlcs, const multiSegmentation &multiObject);

		void  outputOneSegmentation(const string &filename, const VePointXYZLC &ptlcs, const OneSegmentation &objects);
		void  outputOneSegmentationKpt(const string &filename, const VePointXYZLC &ptlcs, const OneSegmentation &objects);
		void  output(const string &filename, const multiSegmentation &multiObjects);
		void  outputSegQuality(const SegMetric &metric);

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