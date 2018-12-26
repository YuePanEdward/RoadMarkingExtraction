#include "segmentation.h"
#include "utility.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <algorithm>

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <intsafe.h>
#include <set>
#include <unordered_set>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

using namespace segmentation;
using namespace utility;
using namespace cv;

bool cmpLabel(const PointXYZLC &pt1, const PointXYZLC &pt2)
{
	if (pt1.objectLabel < pt2.objectLabel)
	{
		return true;
	}
	return false;
}

void Csegmentation::NFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int K)
{
	for (int i = 0; i < inclouds.size(); i++)
	{
		if ( inclouds[i].size()>K)  // 要满足 总点数大于K，才接受这个子点云
		{
			outclouds.push_back(inclouds[i]);
		}
	}
	cout << "Filtered Cloud Number: " << outclouds.size() << endl;
}

void Csegmentation::SORFilter(const pcXYZI &incloud, pcXYZI &outcloud, int MeanK, double std)
{
	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
	
	sor.setInputCloud(incloud.makeShared());
	sor.setMeanK(MeanK);//50
	sor.setStddevMulThresh(std);//1.0
	sor.filter(outcloud);
	
}

void Csegmentation::cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outSORclouds, int N, int MeanK, double std)
{
	int cloudnumber = inclouds.size();
	vector<pcXYZI> outclouds;
	
	outclouds.resize(cloudnumber);
	outSORclouds.resize(cloudnumber);

	//Otsu method thresholding
	
	for (int i = 0; i < cloudnumber; i++)
	{
		//store intensity [integer] 
		vector<int>  intensitylist;
		int pointnumber = inclouds[i].size();
		intensitylist.resize(pointnumber);
		for (int j = 0; j < pointnumber; j++)
		{
			intensitylist[j] = inclouds[i].points[j].intensity;   //vector 如果已经分配好内存就不能push_back了？
		}
		int intensitymax, intensitymin;
		intensitymax = *(max_element(intensitylist.begin(), intensitylist.end()));
		//intensitymin = *(min_element(intensitylist.begin(), intensitylist.end()));

		//OTSU Method
		
		//Define Histogram
		vector<int> h0;
		vector<float> h;
		h0.resize(N );
		h.resize(N );
		
		for (int k = 0; k < N; k++) h0[k] = 0;
	
		//generate histogram
		for (int j = 0; j < pointnumber; j++)
		{
			int bin = (N-1) * intensitylist[j] / intensitymax;
			h0[bin]++;
		}

		//求取比例直方图
		for (int k = 0; k < N; k++)
		{
			h[k] = (float)h0[k] / pointnumber;
		}

		int threshold=0;

		float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
		for (int k = 0; k < N; k++)
		{
			w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

			for (int t = 0; t < N; t++)
			{
				if (t <= k) //背景部分  
				{
					//以i为阈值分类，第一类总的概率  
					w0 += h[t];
					u0tmp += t * h[t];
				}
				else       //前景部分  
				{
					//以i为阈值分类，第二类总的概率  
					w1 += h[t];
					u1tmp += t * h[t];
				}
			}

			u0 = u0tmp / w0;        //第一类的平均灰度  
			u1 = u1tmp / w1;        //第二类的平均灰度  
			u = u0tmp + u1tmp;      //整幅图像的平均灰度  
			
			//计算类间方差  
			deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);
			
			//找出最大类间方差以及对应的阈值  
			if (deltaTmp > deltaMax)
			{
				deltaMax = deltaTmp;
				threshold = k;
			}
		}

		//cout << "Threshold : "<<threshold<< endl;
	    
		//根据该阈值进行点云分割
		for (int j = 0; j < pointnumber; j++)
		{
			int bin = (N-1) * intensitylist[j] / intensitymax;
			if (bin>threshold)
			{
				outclouds[i].push_back(inclouds[i].points[j]);
			}
		}
		//cout << "Cloud " << i << " 's number after Otsu Thresholding is: " <<outclouds[i].size()<< endl;

		//SOR: Statistics Outlier Remover
		SORFilter(outclouds[i], outSORclouds[i], MeanK, std);

		//cout << "Cloud " << i << " 's number after SOR is: " <<outclouds[i].size()<< endl;
		
	}

}

void Csegmentation::BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas)//存bounding box 极值点序号 //顺序 minx,miny,maxx,maxy
{
	boundingdatas.resize(clouds.size());

	for (int i = 0; i < clouds.size(); i++)
	{
		double max_x, max_y, min_x, min_y;
		int max_x_j, max_y_j, min_x_j, min_y_j;
		max_x = -DBL_MAX; max_x_j = 0;
		max_y = -DBL_MAX; max_y_j = 0;
		min_x = DBL_MAX; min_x_j= 0;
		min_y = DBL_MAX; min_y_j = 0;
		
		for (int j = 0; j < clouds[i].size(); j++){
			if (clouds[i].points[j].x > max_x)
			{
				max_x = clouds[i].points[j].x;
				max_x_j = j;
			}
			if (clouds[i].points[j].x < min_x)
			{
				min_x = clouds[i].points[j].x;
				min_x_j = j;
			}
			if (clouds[i].points[j].y > max_y)
			{
				max_y = clouds[i].points[j].y;
				max_y_j = j;
			}
			if (clouds[i].points[j].y < min_y)
			{
				min_y = clouds[i].points[j].y;
				min_y_j = j;
			}

		}
		//顺序 minx,miny,maxx,maxy  
		boundingdatas[i].push_back(clouds[i].points[min_x_j]);
		boundingdatas[i].push_back(clouds[i].points[min_y_j]);
		boundingdatas[i].push_back(clouds[i].points[max_x_j]);
		boundingdatas[i].push_back(clouds[i].points[max_y_j]);

		//对于一般四边形路标，矢量化就是 minx,miny,maxx,maxy 这四个点 
	}

}
void Csegmentation::BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures)
{
	boundingfeatures.resize(boundingdatas.size());
	const double pi = 3.1415926;
	for (int i = 0; i < boundingdatas.size(); i++)
	{
		// minx,miny,maxx,maxy 四个点 分别标为 A，B，C，D
		double Ax, Ay, Bx, By, Cx, Cy, Dx, Dy; //点坐标
		double AB, BC, CD, DA, AC, BD;  //6条边长
		double ACpmBD;   //对角线向量点乘
		double cornerrad; //对角线夹角（弧度制）
		//记录点坐标
		Ax = boundingdatas[i][0].x;
		Bx = boundingdatas[i][1].x;
		Cx = boundingdatas[i][2].x;
		Dx = boundingdatas[i][3].x;
		Ay = boundingdatas[i][0].y;
		By = boundingdatas[i][1].y;
		Cy = boundingdatas[i][2].y;
		Dy = boundingdatas[i][3].y;

		// 计算边长 
		AB = sqrt((Ax - Bx)*(Ax - Bx) + (Ay - By)*(Ay - By));
		BC = sqrt((Bx - Cx)*(Bx - Cx) + (By - Cy)*(By - Cy));
		CD = sqrt((Cx - Dx)*(Cx - Dx) + (Cy - Dy)*(Cy - Dy));
		DA = sqrt((Dx - Ax)*(Dx - Ax) + (Dy - Ay)*(Dy - Ay));
		AC = sqrt((Ax - Cx)*(Ax - Cx) + (Ay - Cy)*(Ay - Cy));
		BD = sqrt((Bx - Dx)*(Bx - Dx) + (By - Dy)*(By - Dy));

		//计算对角线向量点乘
		ACpmBD = abs((Ax - Cx)*(Bx - Dx) + (Ay - Cy)*(By - Dy)); //取个绝对值，都看锐角夹角

		cornerrad = acos(ACpmBD / AC / BD); //弧度制夹角
		boundingfeatures[i].corner = cornerrad / pi * 180; //角度制夹角
		boundingfeatures[i].sortingEdges.push_back(AB);
		boundingfeatures[i].sortingEdges.push_back(BC);
		boundingfeatures[i].sortingEdges.push_back(CD);
		boundingfeatures[i].sortingEdges.push_back(DA);

		sort(boundingfeatures[i].sortingEdges.begin(), boundingfeatures[i].sortingEdges.end()); //默认升序排列
		
	}
	
}

void Csegmentation::CategoryJudgement(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings)
{
	//先验阈值设定
	double angt1, angt2, lt1, lt2, lt3, lt4, lt5, ratio1, ratio2;
	//Bounding四点组 对角线夹角
	angt1 = 1;
	angt2 = 28;
	//Bounding四点组 边长
	lt1 = 0.05;
	lt2 = 0.5;
	lt3 = 2;
	lt4 = 7;
	lt5 = 0.2;
	//Bounding四点组 边长比值
	ratio1 = 1.25;

	//根据先验知识对每个Bounding四点组进行判断，分类
	for (int i = 0; i < boundingfeatures.size(); i++)
	{
		//想办法简化下，且有层次一点
		if (boundingfeatures[i].corner > angt1 && boundingfeatures[i].corner < angt2
			&& boundingfeatures[i].sortingEdges[3]<lt4 && boundingfeatures[i].sortingEdges[3]>lt3
			&& boundingfeatures[i].sortingEdges[2]<lt4 && boundingfeatures[i].sortingEdges[2]>lt3
			&& boundingfeatures[i].sortingEdges[1]<lt2 && boundingfeatures[i].sortingEdges[1]>lt1
			&& boundingfeatures[i].sortingEdges[0]<lt2 && boundingfeatures[i].sortingEdges[0]>lt1
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1
			&& boundingfeatures[i].sortingEdges[1] / boundingfeatures[i].sortingEdges[0] < ratio1)
		{
			roadmarkings[i].category = 1;   // 1 类 长方形规则标线
		}

		if (/*boundingfeatures[i].corner < angt1
			&& */boundingfeatures[i].sortingEdges[3]>lt4 && boundingfeatures[i].sortingEdges[2]>lt4 
			&& boundingfeatures[i].sortingEdges[1]<lt5 && boundingfeatures[i].sortingEdges[0]<lt5
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1)
		{
			roadmarkings[i].category = 2;   // 2 类 线型长边线
		}
	
		
	}
}

void Csegmentation::MarkingVectorization(const vector<pcXYZI> &clouds, vector<RoadMarking> & roadmarkings, double line_sample_dl)
{
	vector<vector<pcl::PointXYZI>> boundingdatas;
	vector<BoundingFeature> boundingfeatures;
	vector<int> categorynumber;
	
	roadmarkings.resize(clouds.size());
	
	categorynumber.resize(6);  //共有6类
	for (int m = 0; m < 6; m++)
	{
		categorynumber[m] = 0;
	}

	BoundingInformation(clouds, boundingdatas);
	BoundingFeatureCalculation(boundingdatas, boundingfeatures);
	CategoryJudgement(boundingfeatures, roadmarkings);
	
	for (int i = 0; i < clouds.size(); i++)
	{
		switch (roadmarkings[i].category)
		{
		   case 1: //1 类 长方形规则标线
			   categorynumber[1]++;

			   roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
			   roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
			   roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
			   roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
			   break;
		   case 2: //2类 线型长边线
			   categorynumber[2]++;

			   double Xmin, Xmax, Ymin, Ymax;
			   Xmin = boundingdatas[i][0].x;
			   Ymin = boundingdatas[i][1].y;
			   Xmax = boundingdatas[i][2].x;
			   Ymax = boundingdatas[i][3].y;

			   double dX, dY;
			   dX = Xmax - Xmin;
			   dY = Ymax - Ymin;

			   double deltaX;  //采样点X间隔
			   double sampleK_d;
			   int sampleK; //采样点数

			   
			   deltaX = line_sample_dl*dX / sqrt(dX*dX + dY*dY);
			   sampleK_d = dX /deltaX;
			   sampleK = sampleK_d;
			   
			   //cout << "Cloud  " << i << "  Sample number: " << sampleK<<endl;

			   roadmarkings[i].polyline.resize(sampleK + 2);
			   roadmarkings[i].polyline[0] = boundingdatas[i][0]; //这里不够精确，可取0和1的平均点
			   roadmarkings[i].polyline[sampleK+1] = boundingdatas[i][3]; //这里不够精确，可取2和3的平均点

			   vector<double> Xlist; //存采样点大致X坐标
			   for (int k = 1; k <= sampleK; k++){
				   Xlist.push_back(Xmin + k*deltaX);
			   }

			   double ambiguousRatio = 0.25;  //模糊搜索比例 
			   for (int j = 0; j < clouds[i].size(); j++)
			   {
				   for (int k = 0; k<Xlist.size();k++)
				   {
					   auto iter = Xlist.begin();
					   if (clouds[i].points[j].x>Xlist[k] - ambiguousRatio*deltaX && clouds[i].points[j].x < Xlist[k] + ambiguousRatio*deltaX)
					   {
						   double this_k_d = (Xlist[k] - Xmin + 0.001) / deltaX;
						   int this_k = this_k_d;
						   roadmarkings[i].polyline[this_k] = clouds[i].points[j]; //存为采样点
						   Xlist.erase(iter); // Xlist中删除该位置
						   break;
					   }
					   iter++;
				   }
			   }
			   break;

		}
	}
	cout << "Rectangle Road Markings Number: " << categorynumber[1] << endl;
	cout << "Side Line Road Markings Number: " << categorynumber[2] << endl;

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