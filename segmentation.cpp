#include "segmentation.h"
#include "utility.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <intsafe.h>
#include <set>
#include <unordered_set>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

using namespace segmentation;
using namespace utility;

bool cmpLabel(const PointXYZLC &pt1, const PointXYZLC &pt2)
{
	if (pt1.objectLabel < pt2.objectLabel)
	{
		return true;
	}
	return false;
}

pcXYZIPtr Csegmentation::planesegRansac(const pcXYZIPtr &cloud,float threshold)
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}

	/*cout << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;*/

	cout << "Model inliers number: " << inliers->indices.size() << std::endl;
	pcXYZIPtr fitcloud (new pcXYZI());
	//fitcloud->width = inliers->indices.size();
	//fitcloud->height = 1;
	//fitcloud->points.resize(cloud->width * cloud->height); //用pushback就不用先给空间了

	for (size_t i = 0; i < inliers->indices.size(); ++i)
	{
		fitcloud->push_back(cloud->points[inliers->indices[i]]);
	}
	return fitcloud;
}
void Csegmentation::getVisualWordsIndex(const vectorSBF &bscFeats, multiSegmentation &multiObjects)
{
	size_t interval = bscFeats.size() / 2;
	for (size_t i = 0; i < multiObjects.size(); ++i)
	{
		for (size_t j = 0; j < multiObjects[i].size(); ++j)
		{
			if (multiObjects[i][j].isChanged)
			{
				multiObjects[i][j].vwIndexes.swap(vector<int>());
				multiObjects[i][j].vw2Indexes.swap(vector<int>());

				for (size_t m = 0; m < multiObjects[i][j].kptIndexes.size(); ++m)
				{
					size_t index = multiObjects[i][j].kptIndexes[m];
					multiObjects[i][j].vwIndexes.push_back(bscFeats[index].bscVisualWordsIndex_);
					multiObjects[i][j].vw2Indexes.push_back(bscFeats[index + interval].bscVisualWordsIndex_);
				}
			}
		}
	}
}

void Csegmentation::getObjectBound(const VePointXYZLC &ptlcs, Object &object)
{
	object.boundingBox.minx = DBL_MAX;
	object.boundingBox.miny = DBL_MAX;
	object.boundingBox.minz = DBL_MAX;
	object.boundingBox.maxx = -DBL_MAX;
	object.boundingBox.maxy = -DBL_MAX;
	object.boundingBox.maxz = -DBL_MAX;

	for (size_t i = 0; i<object.ptIndexes.size(); ++i)
	{
		if (object.boundingBox.minx > ptlcs[object.ptIndexes[i]].pt.x())
			object.boundingBox.minx = ptlcs[object.ptIndexes[i]].pt.x();
		if (object.boundingBox.miny > ptlcs[object.ptIndexes[i]].pt.y())
			object.boundingBox.miny = ptlcs[object.ptIndexes[i]].pt.y();
		if (object.boundingBox.minz > ptlcs[object.ptIndexes[i]].pt.z())
			object.boundingBox.minz = ptlcs[object.ptIndexes[i]].pt.z();

		if (object.boundingBox.maxx < ptlcs[object.ptIndexes[i]].pt.x())
			object.boundingBox.maxx = ptlcs[object.ptIndexes[i]].pt.x();
		if (object.boundingBox.maxy < ptlcs[object.ptIndexes[i]].pt.y())
			object.boundingBox.maxy = ptlcs[object.ptIndexes[i]].pt.y();
		if (object.boundingBox.maxz < ptlcs[object.ptIndexes[i]].pt.z())
			object.boundingBox.maxz = ptlcs[object.ptIndexes[i]].pt.z();
	}
}

void Csegmentation::getGroundTruth(const VePointXYZLC &ptlcs,  vector<Object> &groundTruth)
{
	vector<PointXYZLC> orderPtlcs;
	orderPtlcs = ptlcs;
	sort(orderPtlcs.begin(), orderPtlcs.end(), cmpLabel);

	Object object;
	object.objectLabel = orderPtlcs[0].objectLabel;
	object.objectClass = orderPtlcs[0].objectClass;

	for (size_t i = 0; i < orderPtlcs.size(); ++i)
	{
		if (orderPtlcs[i].objectLabel == object.objectLabel)
		{
			object.ptIndexes.push_back(orderPtlcs[i].ptIndex);
		}
		else
		{
			getObjectBound(ptlcs, object);
			groundTruth.push_back(object);
			object.objectLabel = orderPtlcs[i].objectLabel;
			object.objectClass = orderPtlcs[i].objectClass;
			object.ptIndexes.clear();
			object.ptIndexes.push_back(orderPtlcs[i].ptIndex);
		}
	}
	getObjectBound(ptlcs, object);
	groundTruth.push_back(object);
	orderPtlcs.swap(vector<PointXYZLC>());
}


SegMetric Csegmentation::evaluateSegQuality(const VePointXYZLC &ptlcs, const vector<Object> &objects, const vector<Object> &groundTruth, float overlapT)
{
	SegMetric metric;
	vector<Object> overlappingGT;
	size_t correctNum = 0;
	double averageRatio = 0.0;

	for (size_t i = 0; i < objects.size(); ++i)
	{
		overlappingGT.swap(vector<Object>());
		findOverlappingGT(objects[i], groundTruth, overlappingGT);
		float ratio = findMaxRatio(objects[i], overlappingGT);
		averageRatio += ratio;
		if (ratio > overlapT)
		{
			correctNum++;
		}
	}

	averageRatio /= objects.size();
	metric.averageRatio = averageRatio;
	metric.precision = (double)correctNum /(double)objects.size();
	metric.recall = (double)correctNum / (double)groundTruth.size();
	metric.f1 = 2.0 * (double)correctNum / (double)(objects.size()+groundTruth.size());
	metric.detectedNum = objects.size();
	metric.groundTruthNum = groundTruth.size();
	metric.correctNum = correctNum;

	return metric;
}

bool Csegmentation::isOverlapping(const Object &object1, const Object &object2)
{
	double o1Minx, o1Miny, o1Minz, o1Maxx, o1Maxy, o1Maxz;
	o1Minx = object1.boundingBox.minx;
	o1Miny = object1.boundingBox.miny;
	o1Minz = object1.boundingBox.minz;
	o1Maxx = object1.boundingBox.maxx;
	o1Maxy = object1.boundingBox.maxy;
	o1Maxz = object1.boundingBox.maxz;

	double o2Minx, o2Miny, o2Minz, o2Maxx, o2Maxy, o2Maxz;
	o2Minx = object2.boundingBox.minx;
	o2Miny = object2.boundingBox.miny;
	o2Minz = object2.boundingBox.minz;
	o2Maxx = object2.boundingBox.maxx;
	o2Maxy = object2.boundingBox.maxy;
	o2Maxz = object2.boundingBox.maxz;

	if (o1Minx > o2Maxx || o1Maxx < o2Minx
		|| o1Miny > o2Maxy || o1Maxy < o2Miny
		|| o1Minz > o2Maxz || o1Maxz <o2Minz)
	{
		return false;
	}
	else
	{
		return true;
	}
}

void Csegmentation::findOverlappingGT(const Object &object, const vector<Object> &groundTruth, vector<Object> &overlappingGT)
{
	double oMinx, oMiny, oMinz, oMaxx, oMaxy, oMaxz;
	oMinx = object.boundingBox.minx;
	oMiny = object.boundingBox.miny;
	oMinz = object.boundingBox.minz;
	oMaxx = object.boundingBox.maxx;
	oMaxy = object.boundingBox.maxy;
	oMaxz = object.boundingBox.maxz;

	double gtMinx, gtMiny, gtMinz, gtMaxx, gtMaxy, gtMaxz;
	for (size_t i = 0; i < groundTruth.size(); ++i)
	{
		gtMinx = groundTruth[i].boundingBox.minx;
		gtMiny = groundTruth[i].boundingBox.miny;
		gtMinz = groundTruth[i].boundingBox.minz;
		gtMaxx = groundTruth[i].boundingBox.maxx;
		gtMaxy = groundTruth[i].boundingBox.maxy;
		gtMaxz = groundTruth[i].boundingBox.maxz;

		if (   oMinx > gtMaxx || oMaxx < gtMinx
			|| oMiny > gtMaxy || oMaxy < gtMiny
			|| oMinz > gtMaxz || oMaxz < gtMinz)
		{
			continue;
		}
		else
		{
			overlappingGT.push_back(groundTruth[i]);
		}
	}
}

float Csegmentation::findMaxRatio(const Object &object, const vector<Object> &overlappingGT)
{
	float maxRatio = 0.0f;
	for (size_t i = 0; i < overlappingGT.size(); ++i)
	{
		size_t intersectionNum = calculateIntersection(object.ptIndexes, overlappingGT[i].ptIndexes);
		size_t unionNum = calculateUnion(object.ptIndexes, overlappingGT[i].ptIndexes);
		if (unionNum == 0)
		{
			continue;
		}
		float  ratio = (float)intersectionNum / (float)unionNum;
		if (ratio > maxRatio)
		{
			maxRatio = ratio;
		}
	}
	return maxRatio;
}

size_t Csegmentation::calculateIntersection(const vector<size_t> &indexes1, const vector<size_t> &indexes2)
{
	size_t counter = 0;
	
	unordered_set<size_t> mSet; 
	for (size_t i = 0; i < indexes1.size(); i++)
	{
		mSet.insert(indexes1[i]);
	}

	for (size_t i = 0; i < indexes2.size(); i++)
	{
		//注意调用set的find方法,这个方法会通过计算Hash值来比较; 
		if (mSet.end() != mSet.find(indexes2[i]))
			counter++;
	}
	return counter;
}

size_t Csegmentation::calculateUnion(const vector<size_t> &indexes1, const vector<size_t> &indexes2)
{
	vector<size_t> indexes(indexes1.size() + indexes2.size());
	for (size_t i = 0; i < indexes1.size(); ++i)
	{
		indexes[i] = indexes1[i];
	}

	for (size_t i = 0; i < indexes2.size(); ++i)
	{
		indexes[i + indexes1.size()] = indexes2[i];
	}

	sort(indexes.begin(), indexes.end());
	vector<size_t>::iterator new_end;
	new_end = unique(indexes.begin(), indexes.end());    
	indexes.erase(new_end, indexes.end()); 

	return indexes.size();
}


void Csegmentation::multiSegmentation2Documents(const multiSegmentation &multiObjects, vector<vector<int>> &documents)
{
	vector<int> vwIndexes;
	for (int i = 0; i < multiObjects.size(); i++)
	{
		for (int j = 0; j < multiObjects[i].size(); j++)
		{
			vwIndexes.swap(vector<int>());
			vwIndexes = multiObjects[i][j].vwIndexes;
			for (size_t m = 0; m < multiObjects[i][j].vw2Indexes.size(); ++m)
			{
				vwIndexes.push_back(multiObjects[i][j].vw2Indexes[m]);
			}
			documents.push_back(vwIndexes);	
		}
	}
	vwIndexes.swap(vector<int>());
}

void  Csegmentation::outputSegQuality(const SegMetric &metric)
{
	cout << "-----------------------------------------------------------" << endl;
	cout << "the number of detected objects is:" << metric.detectedNum << endl;
	cout << "the number of ground truth is:" << metric.groundTruthNum << endl;
	cout << "the number of correct objects is:" << metric.correctNum << endl;
	cout << "the average ratio is:" <<setiosflags(ios::fixed)<<setprecision(3)<<metric.averageRatio << endl;
	cout << "the precision is:" << setiosflags(ios::fixed) << setprecision(3) << metric.precision << endl;
	cout << "the recall is:" << setiosflags(ios::fixed) << setprecision(3) << metric.recall << endl;
	cout << "the f1 is:" << setiosflags(ios::fixed) << setprecision(3) << metric.f1 << endl;
	cout << "-----------------------------------------------------------" << endl;
}

void Csegmentation::displayObjectsByLabel(const VePointXYZLC &ptlcs, const vector<Object> &objects)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("displayObjectsByLabel"));
	viewer->setBackgroundColor(255, 255, 255);

	//display points;
	pcXYZRGB pointCloudC;
	srand((unsigned)time(NULL));
	for (size_t i = 0; i < objects.size(); ++i)
	{
		UINT8 r, g, b;
		r = rand() % 255;
		g = rand() % 255;
		b = rand() % 255;
		for (size_t j = 0; j < objects[i].ptIndexes.size(); ++j)
		{
			pcl::PointXYZRGB pt;
			pt.x = ptlcs[objects[i].ptIndexes[j]].pt.x();
			pt.y = ptlcs[objects[i].ptIndexes[j]].pt.y();
			pt.z = ptlcs[objects[i].ptIndexes[j]].pt.z();
			pt.r = r;
			pt.g = g;
			pt.b = b;
			pointCloudC.points.push_back(pt);
		}
	}
	viewer->addPointCloud(pointCloudC.makeShared());

	//display bounding box;
	char t[256];
	string s;
	size_t index = 0;
	for (size_t i = 0; i < objects.size(); ++i)
	{
		pcl::PointXYZ pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;

		pt1.x = objects[i].boundingBox.minx;
		pt1.y = objects[i].boundingBox.miny;
		pt1.z = objects[i].boundingBox.minz;

		pt2.x = objects[i].boundingBox.maxx;
		pt2.y = objects[i].boundingBox.miny;
		pt2.z = objects[i].boundingBox.minz;

		pt3.x = objects[i].boundingBox.minx;
		pt3.y = objects[i].boundingBox.maxy;
		pt3.z = objects[i].boundingBox.minz;

		pt4.x = objects[i].boundingBox.minx;
		pt4.y = objects[i].boundingBox.miny;
		pt4.z = objects[i].boundingBox.maxz;


		pt5.x = objects[i].boundingBox.maxx;
		pt5.y = objects[i].boundingBox.maxy;
		pt5.z = objects[i].boundingBox.maxz;


		pt6.x = objects[i].boundingBox.minx;
		pt6.y = objects[i].boundingBox.maxy;
		pt6.z = objects[i].boundingBox.maxz;

		pt7.x = objects[i].boundingBox.maxx;
		pt7.y = objects[i].boundingBox.miny;
		pt7.z = objects[i].boundingBox.maxz;

		pt8.x = objects[i].boundingBox.maxx;
		pt8.y = objects[i].boundingBox.maxy;
		pt8.z = objects[i].boundingBox.minz;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt1, pt3, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt1, pt4, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt5, pt6, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt5, pt7, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt5, pt8, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt2, pt7, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt2, pt8, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt3, pt6, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt3, pt8, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt4, pt6, 1.0, 0.0, 0.0, s);
		index++;

		sprintf(t, "%d", index);
		s = t;
		viewer->addLine(pt4, pt7, 1.0, 0.0, 0.0, s);
		index++;
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	pointCloudC.swap(pcXYZRGB());
}

void Csegmentation::outputMultiSegmentation(const string &foldername, const VePointXYZLC &ptlcs, const multiSegmentation &multiObject)
{
	boost::filesystem::path dir(foldername);
	string  outputFolder;
	outputFolder = dir.stem().string();

	if (!boost::filesystem::exists(outputFolder))
	{
		boost::filesystem::create_directory(outputFolder);
	}

	for (size_t i = 0; i < multiObject.size(); ++i)
	{
		char t[256];
		string s;
		sprintf(t, "%d", i);
		s = t;
		string  outputFileName;
		outputFileName = outputFolder + "\\" + "_" + s + "_objects.las";
		outputOneSegmentation(outputFileName, ptlcs, multiObject[i]);
	}
}

void  Csegmentation::outputMultiSegmentationKpt(const string &foldername, const VePointXYZLC &ptlcs, const multiSegmentation &multiObject)
{
	boost::filesystem::path dir(foldername);
	string  outputFolder;
	outputFolder = dir.stem().string();

	if (!boost::filesystem::exists(outputFolder))
	{
		boost::filesystem::create_directory(outputFolder);
	}

	for (size_t i = 0; i < multiObject.size(); ++i)
	{
		char t[256];
		string s;
		sprintf(t, "%d", i);
		s = t;
		string  outputFileName;
		outputFileName = outputFolder + "\\" + "_" + s + "_objectsKpt.las";
		outputOneSegmentationKpt(outputFileName, ptlcs, multiObject[i]);
	}
}

void Csegmentation::outputOneSegmentation(const string &filename, const VePointXYZLC &ptlcs, const OneSegmentation &objects)
{
	srand((unsigned)time(NULL));

	PointCloudBound bound;
	getPointXYZLCBound(ptlcs, bound);

	size_t pt_num = 0;
	for (size_t i = 0; i < objects.size(); i++)
	{
		pt_num += objects[i].ptIndexes.size();
	}

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

		header.SetScale(0.001, 0.001, 0.01);
		header.SetPointRecordsCount(pt_num);

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (size_t i = 0; i < objects.size(); ++i)
		{
			UINT8 r, g, b;
			r = rand() % 255;
			g = rand() % 255;
			b = rand() % 255;
			for (size_t j = 0; j < objects[i].ptIndexes.size(); ++j)
			{
				pt.SetCoordinates(ptlcs[objects[i].ptIndexes[j]].pt.x(),
					              ptlcs[objects[i].ptIndexes[j]].pt.y(),
								  ptlcs[objects[i].ptIndexes[j]].pt.z());

				pt.SetIntensity(10);
				pt.SetColor(liblas::Color(r, g, b));
				writer.WritePoint(pt);
			}
		}

		ofs.flush();
		ofs.close();
	}
}

void Csegmentation::outputOneSegmentationKpt(const string &filename, const VePointXYZLC &ptlcs, const OneSegmentation &objects)
{
	srand((unsigned)time(NULL));

	PointCloudBound bound;
	getPointXYZLCBound(ptlcs, bound);

	size_t pt_num = 0;
	for (size_t i = 0; i < objects.size(); i++)
	{
		pt_num += objects[i].kptIndexes.size();
	}

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

		header.SetScale(0.001, 0.001, 0.01);
		header.SetPointRecordsCount(pt_num);

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (size_t i = 0; i < objects.size(); ++i)
		{
			UINT8 r, g, b;
			r = rand() % 255;
			g = rand() % 255;
			b = rand() % 255;
			for (size_t j = 0; j < objects[i].ptIndexes.size(); ++j)
			{
				if (ptlcs[objects[i].ptIndexes[j]].isKeypoint)
				{
					pt.SetCoordinates(ptlcs[objects[i].ptIndexes[j]].pt.x(),
						ptlcs[objects[i].ptIndexes[j]].pt.y(),
						ptlcs[objects[i].ptIndexes[j]].pt.z());

					pt.SetIntensity(10);
					pt.SetColor(liblas::Color(r, g, b));
					writer.WritePoint(pt);
				}		
			}
		}

		ofs.flush();
		ofs.close();
	}
}

void  Csegmentation::output(const string &filename, const multiSegmentation &multiObjects)
{
	ofstream ofs(filename);
	for (size_t i = 0; i < multiObjects.size(); ++i)
	{
		for (size_t j = 0; j < multiObjects[i].size(); ++j)
		{
			cout << multiObjects[i][j].topic << endl;
			for (size_t m = 0; m < multiObjects[i][j].kptIndexes.size(); ++m)
			{
				ofs << multiObjects[i][j].kptIndexes[m] << endl;
			}
		}
	}
}

pcXYZIPtr Csegmentation::groundprojection(const pcXYZIPtr &cloud)
{
	pcXYZIPtr cloud_projected(new pcXYZI());

	// Create a set of planar coefficients with X=Y=Z=0
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	coefficients->values.resize(4);
	coefficients->values[0] = coefficients->values[1]= coefficients->values[3] = 0;
	coefficients->values[2] = cloud->points[0].z;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZI> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);

	cout << "Cloud projection completed" << endl;
	return cloud_projected;

}