#include "stdafx.h"
#include "pointcloudprocess.h"
#include "utility.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <algorithm>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/concave_hull.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <intsafe.h>
#include <set>
#include <unordered_set>


#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

using namespace PCprocess;
using namespace utility;
using namespace cv;
using namespace std;


void Csegmentation::GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud)
{
	pcl::PointIndicesPtr ground_points(new pcl::PointIndices);
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(20);
	pmf.setSlope(1.0f);
	pmf.setInitialDistance(0.5f);
	pmf.setMaxDistance(3.0f);
	pmf.extract(ground_points->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground_points);
	extract.filter(*gcloud);

	//std::cerr << "Ground cloud after filtering (PMF): " << std::endl;
	//std::cerr << *gcloud << std::endl;

	// Extract non-ground returns
	extract.setNegative(true);
	extract.filter(*ngcloud);

	//std::cerr << "Non-ground cloud after filtering (PMF): " << std::endl;
	//std::cerr << *ngcloud << std::endl;
}

void Csegmentation::GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud, int max_window_size, float slope, float initial_distance, float max_distance)
{
	pcl::PointIndicesPtr ground_points(new pcl::PointIndices);
	pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
	pmf.setInputCloud(cloud);
	pmf.setMaxWindowSize(max_window_size);  //20
	pmf.setSlope(slope);//1.0f
	pmf.setInitialDistance(initial_distance);//0.5f
	pmf.setMaxDistance(max_distance);//3.0f
	pmf.extract(ground_points->indices);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(ground_points);
	extract.filter(*gcloud);

	//std::cerr << "Ground cloud after filtering (PMF): " << std::endl;
	//std::cerr << *gcloud << std::endl;

	// Extract non-ground returns
	extract.setNegative(true);
	extract.filter(*ngcloud);

	//std::cerr << "Non-ground cloud after filtering (PMF): " << std::endl;
	//std::cerr << *ngcloud << std::endl;
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

void Csegmentation::CategoryJudgementBox_highway(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings)
{
	roadmarkings.resize(boundingfeatures.size());
	
	//先验阈值设定
	double angt1, angt2, lt1, lt2, lt3, lt4, lt5, ratio1, ratio2;
	//Bounding四点组 对角线夹角
	angt1 = 1;
	angt2 = 30;
	//Bounding四点组 边长
	lt1 = 0.05;
	lt2 = 0.5;
	lt3 = 1.5;
	lt4 = 8;
	lt5 = 0.3;
	//Bounding四点组 边长比值
	ratio1 = 1.35;
	ratio2 = 1.5;
	//根据先验知识对每个Bounding四点组进行判断，分类
	for (int i = 0; i < boundingfeatures.size(); i++)
	{
		
		//想办法简化下，且有层次一点
		
		if (/*boundingfeatures[i].corner < angt1
			&& */boundingfeatures[i].sortingEdges[3]>lt4 && boundingfeatures[i].sortingEdges[2]>lt4
			&& boundingfeatures[i].sortingEdges[1]<lt5 && boundingfeatures[i].sortingEdges[0]<lt5
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio2)
		{
			roadmarkings[i].category = 1;   // 1 类 线型长边线
		}
		//这个也用模板匹配做吧
		
		if (boundingfeatures[i].corner > angt1 && boundingfeatures[i].corner < angt2
			&& boundingfeatures[i].sortingEdges[3]<lt4 && boundingfeatures[i].sortingEdges[3]>lt3
			&& boundingfeatures[i].sortingEdges[2]<lt4 && boundingfeatures[i].sortingEdges[2]>lt3
			&& boundingfeatures[i].sortingEdges[1]<lt2 && boundingfeatures[i].sortingEdges[1]>lt1
			&& boundingfeatures[i].sortingEdges[0]<lt2 && boundingfeatures[i].sortingEdges[0]>lt1
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1
			&& boundingfeatures[i].sortingEdges[1] / boundingfeatures[i].sortingEdges[0] < ratio1)
		{
			roadmarkings[i].category = 2;   // 2 类 长方形规则标线
		} 

	}
	cout << "Classification based on Bounding Box Information done" << endl;
}


void Csegmentation::CategoryJudgementBox_cityroad(const vector<BoundingFeature> & boundingfeatures, vector<RoadMarking> & roadmarkings)
{
	roadmarkings.resize(boundingfeatures.size());

	//先验阈值设定
	double angt1, angt2, lt1, lt2, lt3, lt4, lt5, ratio1, ratio2;
	//Bounding四点组 对角线夹角
	angt1 = 1;
	angt2 = 55;
	//Bounding四点组 边长
	lt1 = 0.05;
	lt2 = 0.45;
	lt3 = 1.5;
	lt4 = 6.5;
	lt5 = 0.3;
	//Bounding四点组 边长比值
	ratio1 = 1.4;
	ratio2 = 1.5;
	//根据先验知识对每个Bounding四点组进行判断，分类
	for (int i = 0; i < boundingfeatures.size(); i++)
	{

		//想办法简化下，且有层次一点

		if (/*boundingfeatures[i].corner < angt1
			&& */boundingfeatures[i].sortingEdges[3]>lt4 && boundingfeatures[i].sortingEdges[2]>lt4
			&& boundingfeatures[i].sortingEdges[1]<lt5 && boundingfeatures[i].sortingEdges[0]<lt5
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio2)
		{
			roadmarkings[i].category = 1;   // 1 类 线型长边线
		}
		//这个也用模板匹配做吧

		if (boundingfeatures[i].corner > angt1 && boundingfeatures[i].corner < angt2
			&& boundingfeatures[i].sortingEdges[3]<lt4 && boundingfeatures[i].sortingEdges[3]>lt3
			&& boundingfeatures[i].sortingEdges[2]<lt4 && boundingfeatures[i].sortingEdges[2]>lt3
			&& boundingfeatures[i].sortingEdges[1]<lt2 && boundingfeatures[i].sortingEdges[1]>lt1
			&& boundingfeatures[i].sortingEdges[0]<lt2 && boundingfeatures[i].sortingEdges[0]>lt1
			&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1
			&& boundingfeatures[i].sortingEdges[1] / boundingfeatures[i].sortingEdges[0] < ratio1)
		{
			roadmarkings[i].category = 2;   // 2 类 长方形规则标线
		}

	}
	cout << "Classification based on Bounding Box Information done" << endl;
}

void Csegmentation::Find_tail2head(int tail_index, const vector<vector<double>> & d_tail_head, vector < bool > & line_used, vector<int> & combineline, double combine_length)
{
	int sideline_number = line_used.size();
	for (int j = 0; j < sideline_number; j++)
	{
		if (line_used[j] == false && tail_index != j && d_tail_head[tail_index][j] < combine_length){
			combineline.push_back(j); //在最后加
			line_used[j] = true;
			Find_tail2head(j, d_tail_head, line_used, combineline, combine_length);
			break;
		}
	}
}

void Csegmentation::Find_head2tail(int head_index, const vector<vector<double>> & d_head_tail, vector<bool> & line_used, vector<int> & combineline, double combine_length)
{
	int sideline_number = line_used.size();
	for (int j = 0; j < sideline_number; j++)
	{
		if (line_used[j] == false && head_index != j && d_head_tail[head_index][j] < combine_length){
			combineline.insert(combineline.begin(), j); // 在最前面加;
			line_used[j] = true;
			Find_head2tail(j, d_head_tail, line_used, combineline, combine_length);
			break;
		}
	}

}

void Csegmentation::CombineSideLines(const vector<RoadMarking> & roadmarkings, double Combine_length, vector <RoadMarking> & combine_sideline_markings)
{
	vector<pair<pcl::PointXYZI, pcl::PointXYZI>> sidelines;
	vector<int> sidelines_index_in_roadmarkings;

	//Sidelines 存 起点，终点;
	for (int i = 0; i < roadmarkings.size(); i++)
	{
		if (roadmarkings[i].category == 1)
		{
			pair<pcl::PointXYZI, pcl::PointXYZI> sideline;
			sideline.first = roadmarkings[i].polyline[0];
			sideline.second = roadmarkings[i].polyline[roadmarkings[i].polyline.size() - 1];
			sidelines.push_back(sideline);
			sidelines_index_in_roadmarkings.push_back(i);
		}
	}
	
	int sideline_number = sidelines.size();
	//cout << "Side Line Number before Combination: " << sideline_number << endl;

	vector<vector<double>> D_tail_head(sideline_number, vector<double>(sideline_number));
	vector<vector<double>> D_head_tail(sideline_number, vector<double>(sideline_number));

	//起点到终点，终点到起点距离表 计算;
	for (int i = 0; i < sideline_number; i++)
	{
		for (int j = 0; j < sideline_number; j++)
		{
			if (i != j){

				D_tail_head[i][j] = sqrt((sidelines[i].second.x - sidelines[j].first.x)*(sidelines[i].second.x - sidelines[j].first.x)
					+ (sidelines[i].second.y - sidelines[j].first.y)*(sidelines[i].second.y - sidelines[j].first.y));
				D_head_tail[i][j] = sqrt((sidelines[i].first.x - sidelines[j].second.x)*(sidelines[i].first.x - sidelines[j].second.x)
					+ (sidelines[i].second.y - sidelines[j].first.y)*(sidelines[i].second.y - sidelines[j].first.y));

			}
		}
	}

	vector<vector<int>> Combinelines; //合并边线集;
	vector<bool> Line_used(sideline_number);   //是否已处理; 
	
    for (int i = 0; i < sideline_number; i++) Line_used[i] = false;  //初始化为false

	// 递归找可能的断线，以距离Combine_length作为判断依据;
	for (int i = 0; i < sideline_number; i++)
	{
		if (Line_used[i] == false){
			vector<int> Combineline;
			Combineline.push_back(i);
			Line_used[i] = true;

			Find_tail2head(i, D_tail_head, Line_used, Combineline, Combine_length);// 递归往后找;
			Find_head2tail(i, D_head_tail, Line_used, Combineline, Combine_length);// 递归往前找;

			Combinelines.push_back(Combineline);
		}
	}
	
	combine_sideline_markings.resize(Combinelines.size());

	cout << "Side Line Number after Combination: " << Combinelines.size() << endl;

	
	//断线序列化;
	for (int i = 0; i < Combinelines.size(); i++)
	{
		for (int j = 0; j < Combinelines[i].size(); j++){
			int roadmarkings_index=sidelines_index_in_roadmarkings[Combinelines[i][j]];
			for (int k = 0; k < roadmarkings[roadmarkings_index].polyline.size();k++)
			{
				combine_sideline_markings[i].polyline.push_back(roadmarkings[roadmarkings_index].polyline[k]);
			}	
		}
	}
	
}


void Csegmentation::MarkingVectorization_highway(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, const vector<vector<pcl::PointXYZI>> &modeldatas, vector<RoadMarking> & roadmarkings, const vector<bool> & is_rights, double line_sample_dl, double ambiguousRatio)//模糊搜索比例
{

	vector<int> categorynumber;
	vector<double> Xlist; //存采样点大致X坐标 //For 第二类长边线
	const double pi = 3.1415926;

	categorynumber.resize(6);  //共有6类
	for (int m = 0; m < 6; m++)
	{
		categorynumber[m] = 0;
	}

	for (int i = 0; i < clouds.size(); i++)
	{
		switch (roadmarkings[i].category)
		{
		case 1: //1类 线型长边线
			categorynumber[1]++;

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
			sampleK_d = dX / deltaX;
			sampleK = sampleK_d;

			//cout << "Cloud  " << i << "  Sample number: " << sampleK<<endl;

			roadmarkings[i].polyline.resize(sampleK + 2);
			roadmarkings[i].polyline[0] = boundingdatas[i][0]; //这里不够精确，可取0和1的平均点
			roadmarkings[i].polyline[sampleK + 1] = boundingdatas[i][3]; //这里不够精确，可取2和3的平均点


			for (int k = 1; k <= sampleK; k++){
				Xlist.push_back(Xmin + k*deltaX);
			}

			for (int j = 0; j < clouds[i].size(); j++)
			{
				for (int k = 0; k<Xlist.size(); k++)
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

			for (auto iter = roadmarkings[i].polyline.begin() + 1; iter != roadmarkings[i].polyline.end() - 1; iter++)
			{
				if ((*iter).x == 0) *iter = *(iter - 1);
			}

			break;

		case 2: //2 类 长方形规则标线
			categorynumber[2]++;

			roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
			break;

		case 3: //3类 普通向前箭头
			categorynumber[3]++;
			//法一：边缘点抽稀，直接连，因为alpha-shape之后直接就是顺序排列的;

			//法二：记录模型点位，确定主方向，再用规格化参数生成矢量图; 
			/*
					 *
					 * *
					 **********  *
					 *            *
					 **********  *
					 * *
					 *

					 已知XA,YA;XB,YB  【注意都是测绘左手系下的】
					 顺时针顺序
					 ID     X'       Y'
					 0      0        9
					 1    -0.675     5.4
					 2    -0.225     5.4
					 3    -0.225     0
					 4     0.225     0
					 5     0.225     5.4
					 6     0.675     5.4

					 theta=arctan((XA-XB)/(YA-YB))

					 XA-XB     YA-YB      theta'(顺时针转）
					 >0        >0         theta
					 >0        <0         180+theta
					 <0        <0         180+theta
					 <0        >0         360+theta


					 X=cos(theta)*X' -sin(theta)*Y'+XB
					 Y=sin(theta)*X' +cos(theta)*Y'+YB
					 */

			roadmarkings[i].polyline.resize(7);


			double Xa_3, Xb_3, Ya_3, Yb_3, Za_3, Zb_3, Z_ave_3, theta_3, delta_X_3, delta_Y_3;
			double X_3[7];
			double Y_3[7];

			
				Xa_3 = modeldatas[i][0].x;
				Ya_3 = modeldatas[i][0].y;
				Za_3 = modeldatas[i][0].z;
				Xb_3 = modeldatas[i][1].x;
				Yb_3 = modeldatas[i][1].y;
				Zb_3 = modeldatas[i][1].z;
			
			Z_ave_3 = (Za_3 + Zb_3) / 2;

			delta_X_3 = Xa_3 - Xb_3;
			delta_Y_3 = Ya_3 - Yb_3;

			theta_3 = atan(delta_X_3 / delta_Y_3);

			//注意这是测绘坐标系，是左手系，与一般的数学坐标系（右手系）不同;

			if ((delta_X_3 < 0 && delta_Y_3 > 0) || (delta_X_3 < 0 && delta_Y_3 < 0)) theta_3 = pi + theta_3;
			else if (delta_X_3 > 0 && delta_Y_3 < 0) theta_3 = 2 * pi + theta_3;

			
				X_3[0] = 0; Y_3[0] = 9;
				X_3[1] = -0.675; Y_3[1] = 5.4;
				X_3[2] = -0.225; Y_3[2] = 5.4;
				X_3[3] = -0.225; Y_3[3] = 0;
				X_3[4] = 0.225; Y_3[4] = 0;
				X_3[5] = 0.225; Y_3[5] = 5.4;
				X_3[6] = 0.675; Y_3[6] = 5.4;
		

			for (int j = 0; j < 7; j++)
			{
				//顺时针转动theta 
				roadmarkings[i].polyline[j].x = cos(theta_3)*X_3[j] + sin(theta_3)*Y_3[j] + Xb_3;
				roadmarkings[i].polyline[j].y = -sin(theta_3)*X_3[j] + cos(theta_3)*Y_3[j] + Yb_3;
				roadmarkings[i].polyline[j].z = Z_ave_3;

			}

			break;
		case 4: //4类 向前向右箭头
			categorynumber[4]++;

			//法一：边缘点抽稀，直接连，因为alpha-shape之后直接就是顺序排列的;

			//法二：记录模型点位，确定主方向，再用规格化参数生成矢量图; 
			/*

			已知XA,YA;XB,YB  【注意都是测绘左手系下的】
			顺时针顺序
			ID     X'       Y'
			0      0        9
			1    -0.675     5.4
			2    -0.225     5.4
			3    -0.225     0
			4     0.225     0
			5     0.225     0.6
			6     1.425     1.5
			7     1.425     0.15
			8     2.025     2.4
			9     1.425     4.8
			10    1.425     3.3
			11    0.225     2.4
			12    0.225     5.4
			13    0.675     5.4

			theta=arctan((XA-XB)/(YA-YB))

			XA-XB     YA-YB      theta'(顺时针转）
			>0        >0         theta
			>0        <0         180+theta
			<0        <0         180+theta
			<0        >0         360+theta


			X=cos(theta)*X' -sin(theta)*Y'+XB
			Y=sin(theta)*X' +cos(theta)*Y'+YB
			*/

			roadmarkings[i].polyline.resize(14);


			double Xa_4, Xb_4, Ya_4, Yb_4, Za_4, Zb_4, Z_ave_4, theta_4, delta_X_4, delta_Y_4;
			double X_4[14];
			double Y_4[14];

			Xa_4 = modeldatas[i][0].x;
			Ya_4 = modeldatas[i][0].y;
			Za_4 = modeldatas[i][0].z;
			Xb_4 = modeldatas[i][1].x;
			Yb_4 = modeldatas[i][1].y;
			Zb_4 = modeldatas[i][1].z;

			Z_ave_4 = (Za_4 + Zb_4) / 2;

			delta_X_4 = Xa_4 - Xb_4;
			delta_Y_4 = Ya_4 - Yb_4;

			theta_4 = atan(delta_X_4 / delta_Y_4);

			//注意这是测绘坐标系，是左手系，与一般的数学坐标系（右手系）不同;

			if ((delta_X_4 < 0 && delta_Y_4 > 0) || (delta_X_4 < 0 && delta_Y_4 < 0)) theta_4 = pi + theta_4;
			else if (delta_X_4 > 0 && delta_Y_4 < 0) theta_4 = 2 * pi + theta_4;

			if (is_rights[i] == true){
				X_4[0] = 0; Y_4[0] = 9;
				X_4[1] = -0.675; Y_4[1] = 5.4;
				X_4[2] = -0.225; Y_4[2] = 5.4;
				X_4[3] = -0.225; Y_4[3] = 0;
				X_4[4] = 0.225; Y_4[4] = 0;
				X_4[5] = 0.225; Y_4[5] = 0.6;
				X_4[6] = 1.425; Y_4[6] = 1.5;
				X_4[7] = 1.425; Y_4[7] = 0.15;
				X_4[8] = 2.025; Y_4[8] = 2.4;
				X_4[9] = 1.425; Y_4[9] = 4.8;
				X_4[10] = 1.425; Y_4[10] = 3.3;
				X_4[11] = 0.225; Y_4[11] = 2.4;
				X_4[12] = 0.225; Y_4[12] = 5.4;
				X_4[13] = 0.675; Y_4[13] = 5.4;
			}
			else{
				X_4[0] = 0; Y_4[0] = 9;
				X_4[1] = 0.675; Y_4[1] = 5.4;
				X_4[2] = 0.225; Y_4[2] = 5.4;
				X_4[3] = 0.225; Y_4[3] = 0;
				X_4[4] = -0.225; Y_4[4] = 0;
				X_4[5] = -0.225; Y_4[5] = 0.6;
				X_4[6] = -1.425; Y_4[6] = 1.5;
				X_4[7] = -1.425; Y_4[7] = 0.15;
				X_4[8] = -2.025; Y_4[8] = 2.4;
				X_4[9] = -1.425; Y_4[9] = 4.8;
				X_4[10] = -1.425; Y_4[10] = 3.3;
				X_4[11] = -0.225; Y_4[11] = 2.4;
				X_4[12] = -0.225; Y_4[12] = 5.4;
				X_4[13] = -0.675; Y_4[13] = 5.4;

			}
			for (int j = 0; j < 14; j++)
			{
				//顺时针转动theta 
				roadmarkings[i].polyline[j].x = cos(theta_4)*X_4[j] + sin(theta_4)*Y_4[j] + Xb_4;
				roadmarkings[i].polyline[j].y = -sin(theta_4)*X_4[j] + cos(theta_4)*Y_4[j] + Yb_4;
				roadmarkings[i].polyline[j].z = Z_ave_4;

			}

			break;
		case 5: //5类 向右箭头
			categorynumber[5]++;
			break;
		default:
			break;
		}
	}



	cout << "Side Line Road Markings Number: " << categorynumber[1] << endl;
	cout << "Rectangle Road Markings Number: " << categorynumber[2] << endl;
	cout << "Arrow Road Markings (Forward) Number: " << categorynumber[3] << endl;
	cout << "Arrow Road Markings (Forward and Rightward / Leftward) Number: " << categorynumber[4] << endl;
	cout << "Arrow Road Markings (Rightward / Leftward) Number: " << categorynumber[5] << endl;

}
void Csegmentation::MarkingVectorization_cityroad(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, const vector<vector<pcl::PointXYZI>> &modeldatas, vector<RoadMarking> & roadmarkings, const vector<bool> & is_rights, double line_sample_dl, double ambiguousRatio)//模糊搜索比例
{

	vector<int> categorynumber;
	vector<double> Xlist; //存采样点大致X坐标 //For 第二类长边线
	const double pi = 3.1415926;

	categorynumber.resize(6);  //共有6类
	for (int m = 0; m < 6; m++)
	{
		categorynumber[m] = 0;
	}

	for (int i = 0; i < clouds.size(); i++)
	{
		switch (roadmarkings[i].category)
		{
		case 1: //1类 线型长边线
			categorynumber[1]++;

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
			sampleK_d = dX / deltaX;
			sampleK = sampleK_d;

			//cout << "Cloud  " << i << "  Sample number: " << sampleK<<endl;

			roadmarkings[i].polyline.resize(sampleK + 2);
			roadmarkings[i].polyline[0] = boundingdatas[i][0]; //这里不够精确，可取0和1的平均点
			roadmarkings[i].polyline[sampleK + 1] = boundingdatas[i][3]; //这里不够精确，可取2和3的平均点


			for (int k = 1; k <= sampleK; k++){
				Xlist.push_back(Xmin + k*deltaX);
			}

			for (int j = 0; j < clouds[i].size(); j++)
			{
				for (int k = 0; k<Xlist.size(); k++)
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

			for (auto iter = roadmarkings[i].polyline.begin() + 1; iter != roadmarkings[i].polyline.end() - 1; iter++)
			{
				if ((*iter).x == 0) *iter = *(iter - 1);
			}

			break;

		case 2: //2 类 长方形规则标线
			categorynumber[2]++;

			roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
			roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
			break;

		case 3: //3类 普通向前箭头
			categorynumber[3]++;
			//法一：边缘点抽稀，直接连，因为alpha-shape之后直接就是顺序排列的;

			//法二：记录模型点位，确定主方向，再用规格化参数生成矢量图; 
			/*
			
			已知XA,YA;XB,YB  【注意都是测绘左手系下的】
			顺时针顺序
			ID     X'       Y'
			0      0        6
			1    -0.45     3.6
			2    -0.15     3.6
			3    -0.15      0
			4     0.15      0
			5     0.15     3.6
			6     0.45     3.6

			theta=arctan((XA-XB)/(YA-YB))

			XA-XB     YA-YB      theta'(顺时针转）
			>0        >0         theta
			>0        <0         180+theta
			<0        <0         180+theta
			<0        >0         360+theta


			X=cos(theta)*X' -sin(theta)*Y'+XB
			Y=sin(theta)*X' +cos(theta)*Y'+YB
			*/

			roadmarkings[i].polyline.resize(7);


			double Xa_3, Xb_3, Ya_3, Yb_3, Za_3, Zb_3, Z_ave_3, theta_3, delta_X_3, delta_Y_3;
			double X_3[7];
			double Y_3[7];

			Xa_3 = modeldatas[i][0].x;
			Ya_3 = modeldatas[i][0].y;
			Za_3 = modeldatas[i][0].z;
			Xb_3 = modeldatas[i][1].x;
			Yb_3 = modeldatas[i][1].y;
			Zb_3 = modeldatas[i][1].z;

			Z_ave_3 = (Za_3 + Zb_3) / 2;

			delta_X_3 = Xa_3 - Xb_3;
			delta_Y_3 = Ya_3 - Yb_3;

			theta_3 = atan(delta_X_3 / delta_Y_3);

			//注意这是测绘坐标系，是左手系，与一般的数学坐标系（右手系）不同;

			if ((delta_X_3 < 0 && delta_Y_3 > 0) || (delta_X_3 < 0 && delta_Y_3 < 0)) theta_3 = pi + theta_3;
			else if (delta_X_3 > 0 && delta_Y_3 < 0) theta_3 = 2 * pi + theta_3;
		
			X_3[0] = 0; Y_3[0] = 6;
			X_3[1] = -0.45; Y_3[1] = 3.6;
			X_3[2] = -0.15; Y_3[2] = 3.6;
			X_3[3] = -0.15; Y_3[3] = 0;
			X_3[4] = 0.15; Y_3[4] = 0;
			X_3[5] = 0.15; Y_3[5] = 3.6;
			X_3[6] = 0.45; Y_3[6] = 3.6;

			for (int j = 0; j < 7; j++)
			{
				//顺时针转动theta 
				roadmarkings[i].polyline[j].x = cos(theta_3)*X_3[j] + sin(theta_3)*Y_3[j] + Xb_3;
				roadmarkings[i].polyline[j].y = -sin(theta_3)*X_3[j] + cos(theta_3)*Y_3[j] + Yb_3;
				roadmarkings[i].polyline[j].z = Z_ave_3;

			}

			break;
		case 4: //4类 向前向右箭头
			categorynumber[4]++;

			//法一：边缘点抽稀，直接连，因为alpha-shape之后直接就是顺序排列的;

			//法二：记录模型点位，确定主方向，再用规格化参数生成矢量图; 
			/*

			已知XA,YA;XB,YB  【注意都是测绘左手系下的】
			顺时针顺序
			ID     X'       Y'
			0      0        6
			1    -0.45     3.6
			2    -0.15     3.6
			3    -0.15     0
			4     0.15     0
			5     0.15     0.4
			6     0.95     1.0
			7     0.95     0.1
			8     1.35     1.6
			9     0.95     3.2
			10    0.95     2.2
			11    0.15     1.6
			12    0.15     3.6
			13    0.45     3.6

			theta=arctan((XA-XB)/(YA-YB))

			XA-XB     YA-YB      theta'(顺时针转）
			>0        >0         theta
			>0        <0         180+theta
			<0        <0         180+theta
			<0        >0         360+theta


			X=cos(theta)*X' -sin(theta)*Y'+XB
			Y=sin(theta)*X' +cos(theta)*Y'+YB
			*/

			roadmarkings[i].polyline.resize(14);


			double Xa_4, Xb_4, Ya_4, Yb_4, Za_4, Zb_4, Z_ave_4, theta_4, delta_X_4, delta_Y_4;
			double X_4[14];
			double Y_4[14];

			Xa_4 = modeldatas[i][0].x;
			Ya_4 = modeldatas[i][0].y;
			Za_4 = modeldatas[i][0].z;
			Xb_4 = modeldatas[i][1].x;
			Yb_4 = modeldatas[i][1].y;
			Zb_4 = modeldatas[i][1].z;

			Z_ave_4 = (Za_4 + Zb_4) / 2;

			delta_X_4 = Xa_4 - Xb_4;
			delta_Y_4 = Ya_4 - Yb_4;

			theta_4 = atan(delta_X_4 / delta_Y_4);

			//注意这是测绘坐标系，是左手系，与一般的数学坐标系（右手系）不同;

			if ((delta_X_4 < 0 && delta_Y_4 > 0) || (delta_X_4 < 0 && delta_Y_4 < 0)) theta_4 = pi + theta_4;
			else if (delta_X_4 > 0 && delta_Y_4 < 0) theta_4 = 2 * pi + theta_4;
			
			if (is_rights[i] == true){
				X_4[0] = 0; Y_4[0] = 6;
				X_4[1] = -0.45; Y_4[1] = 3.6;
				X_4[2] = -0.15; Y_4[2] = 3.6;
				X_4[3] = -0.15; Y_4[3] = 0;
				X_4[4] = 0.15; Y_4[4] = 0;
				X_4[5] = 0.15; Y_4[5] = 0.4;
				X_4[6] = 0.95; Y_4[6] = 1.0;
				X_4[7] = 0.95; Y_4[7] = 0.1;
				X_4[8] = 1.35; Y_4[8] = 1.6;
				X_4[9] = 0.95; Y_4[9] = 3.2;
				X_4[10] = 0.95; Y_4[10] = 2.2;
				X_4[11] = 0.15; Y_4[11] = 1.6;
				X_4[12] = 0.15; Y_4[12] = 3.6;
				X_4[13] = 0.45; Y_4[13] = 3.6;
			}
			else{
				X_4[0] = 0; Y_4[0] = 6;
				X_4[1] = 0.45; Y_4[1] = 3.6;
				X_4[2] = 0.15; Y_4[2] = 3.6;
				X_4[3] = 0.15; Y_4[3] = 0;
				X_4[4] = -0.15; Y_4[4] = 0;
				X_4[5] = -0.15; Y_4[5] = 0.4;
				X_4[6] = -0.95; Y_4[6] = 1.0;
				X_4[7] = -0.95; Y_4[7] = 0.1;
				X_4[8] = -1.35; Y_4[8] = 1.6;
				X_4[9] = -0.95; Y_4[9] = 3.2;
				X_4[10] = -0.95; Y_4[10] = 2.2;
				X_4[11] = -0.15; Y_4[11] = 1.6;
				X_4[12] = -0.15; Y_4[12] = 3.6;
				X_4[13] = -0.45; Y_4[13] = 3.6;

			}
			for (int j = 0; j < 14; j++)
			{
				//顺时针转动theta 
				roadmarkings[i].polyline[j].x = cos(theta_4)*X_4[j] + sin(theta_4)*Y_4[j] + Xb_4;
				roadmarkings[i].polyline[j].y = -sin(theta_4)*X_4[j] + cos(theta_4)*Y_4[j] + Yb_4;
				roadmarkings[i].polyline[j].z = Z_ave_4;

			}

			break;
		case 5: //5类 向右箭头
			categorynumber[5]++;

			//法一：边缘点抽稀，直接连，因为alpha-shape之后直接就是顺序排列的;

			//法二：记录模型点位，确定主方向，再用规格化参数生成矢量图; 
			/*

			已知XA,YA;XB,YB  【注意都是测绘左手系下的】
			顺时针顺序
			ID     X'       Y'
			0    -0.95      6
			1    -1.35     4.4
			2    -0.95     2.9
			3    -0.95     3.8
			4    -0.15     3
			5    -0.15     0
			6     0.15     0
			7     0.15     3.9
			8    -0.95     5.0

			theta=arctan((XA-XB)/(YA-YB))

			XA-XB     YA-YB      theta'(顺时针转）
			>0        >0         theta
			>0        <0         180+theta
			<0        <0         180+theta
			<0        >0         360+theta


			X=cos(theta)*X' -sin(theta)*Y'+XB
			Y=sin(theta)*X' +cos(theta)*Y'+YB
			*/

			roadmarkings[i].polyline.resize(9);


			double Xa_5, Xb_5, Ya_5, Yb_5, Za_5, Zb_5, Z_ave_5, theta_5, delta_X_5, delta_Y_5;
			double X_5[9];
			double Y_5[9];

			Xa_5 = modeldatas[i][0].x;
			Ya_5 = modeldatas[i][0].y;
			Za_5 = modeldatas[i][0].z;
			Xb_5 = modeldatas[i][1].x;
			Yb_5 = modeldatas[i][1].y;
			Zb_5 = modeldatas[i][1].z;

			Z_ave_5 = (Za_5 + Zb_5) / 2;

			delta_X_5 = Xa_5 - Xb_5;
			delta_Y_5 = Ya_5 - Yb_5;

			theta_5 = atan(delta_X_5 / delta_Y_5);

			//注意这是测绘坐标系，是左手系，与一般的数学坐标系（右手系）不同;

			if ((delta_X_5 < 0 && delta_Y_5 > 0) || (delta_X_5 < 0 && delta_Y_5 < 0)) theta_5 = pi + theta_5;
			else if (delta_X_5 > 0 && delta_Y_5 < 0) theta_5 = 2 * pi + theta_5;
			
			if (is_rights[i] == true){
				X_5[0] = 0.95; Y_5[0] = 6;
				X_5[1] = 1.35; Y_5[1] = 4.4;
				X_5[2] = 0.95; Y_5[2] = 2.9;
				X_5[3] = 0.95; Y_5[3] = 3.8;
				X_5[4] = 0.15; Y_5[4] = 3;
				X_5[5] = 0.15; Y_5[5] = 0;
				X_5[6] = -0.15; Y_5[6] = 0;
				X_5[7] = -0.15; Y_5[7] = 3.9;
				X_5[8] = 0.95; Y_5[8] = 5.0;
			}
			else{
				X_5[0] = -0.95; Y_5[0] = 6;
				X_5[1] = -1.35; Y_5[1] = 4.4;
				X_5[2] = -0.95; Y_5[2] = 2.9;
				X_5[3] = -0.95; Y_5[3] = 3.8;
				X_5[4] = -0.15; Y_5[4] = 3;
				X_5[5] = -0.15; Y_5[5] = 0;
				X_5[6] = 0.15; Y_5[6] = 0;
				X_5[7] = 0.15; Y_5[7] = 3.9;
				X_5[8] = -0.95; Y_5[8] = 5.0;
			}

			
			for (int j = 0; j < 9; j++)
			{
				//顺时针转动theta 
				roadmarkings[i].polyline[j].x = cos(theta_5)*X_5[j] + sin(theta_5)*Y_5[j] + Xb_5;
				roadmarkings[i].polyline[j].y = -sin(theta_5)*X_5[j] + cos(theta_5)*Y_5[j] + Yb_5;
				roadmarkings[i].polyline[j].z = Z_ave_5;

			}

			break;

		default:
			break;
		}
	}

	cout << "Side Line Road Markings Number: " << categorynumber[1] << endl;
	cout << "Rectangle Road Markings Number: " << categorynumber[2] << endl;
	cout << "Arrow Road Markings (Forward) Number: " << categorynumber[3] << endl;
	cout << "Arrow Road Markings (Forward and Rightward / Leftward) Number: " << categorynumber[4] << endl;
	cout << "Arrow Road Markings (Rightward / Leftward) Number: " << categorynumber[5] << endl;
}


void Csegmentation::BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds)
{
	boundaryclouds.resize(clouds.size());
	for (int i = 0; i < clouds.size(); i++)
	{
		boundaryclouds[i] = alphashape(clouds[i], 0.1);
	}
	cout << "Boundary Extraction Done" << endl;
}



pcXYZI Csegmentation::alphashape(const pcXYZI &cloud, float alpha_value) //Concave Hull Generation
{
	pcXYZI cloud_hull;    
	pcl::ConcaveHull<pcl::PointXYZI> chull;        //创建多边形提取对象
	chull.setInputCloud(cloud.makeShared());                    //设置输入点云为提取后点云
	chull.setAlpha(alpha_value);                   //0.1
	chull.reconstruct(cloud_hull);                //创建提取凹多边形

	//std::cout<< "Concave hull has: " << cloud_hull->points.size() << " data points." << endl;
	return cloud_hull;
}

pcXYZI Csegmentation::CornerpointKNN(const pcXYZI &boundarycloud, int K, float disthreshold, float maxcos)
{
	pcXYZI conrnerPoints;
	
	// 先建KDtree
	pcl::KdTreeFLANN <pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(boundarycloud.makeShared());

	vector<int> pointIdxNKNSearch(K);   //距离升序排列
	vector<float> pointNKNSquaredDistance(K); //注意是距离平方

	for (int i = 0; i < boundarycloud.size(); i++){


		kdtree.nearestKSearch(boundarycloud.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);  // K 近邻搜索结果
		float max1_max2;
		max1_max2 = sqrt(pointNKNSquaredDistance[K - 1]) - sqrt(pointNKNSquaredDistance[K - 2]);
		
		float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

		Xo = boundarycloud.points[i].x;
		Yo = boundarycloud.points[i].y;
		Xa = boundarycloud.points[pointIdxNKNSearch[K - 1]].x;
		Ya = boundarycloud.points[pointIdxNKNSearch[K - 1]].y;

		if (max1_max2 < disthreshold)  //若距离最大与次大点间距小于阈值，认为它们处于同侧，找异侧点
		{
			float maxdis=0; 
			int maxindex=-1;
			float Xc, Yc, Xd, Yd;
			Xc = boundarycloud.points[pointIdxNKNSearch[K - 2]].x;
			Yc = boundarycloud.points[pointIdxNKNSearch[K - 2]].y;
			//次远点找之前邻域点中的最远点
			for (int j = 0; j < K - 2; j++){
				Xd = boundarycloud.points[pointIdxNKNSearch[j]].x;
				Yd = boundarycloud.points[pointIdxNKNSearch[j]].y;

				float dis = sqrt((Xd - Xc)*(Xd - Xc) + (Yd - Yc)*(Yd - Yc));

				if (dis > maxdis) {
					maxdis = dis;
					maxindex = j;
				}
			}
			Xb = boundarycloud.points[pointIdxNKNSearch[maxindex]].x;
			Yb = boundarycloud.points[pointIdxNKNSearch[maxindex]].y;
		}

		//否则直接接受
		else{
			Xb = boundarycloud.points[pointIdxNKNSearch[K - 2]].x;
			Yb = boundarycloud.points[pointIdxNKNSearch[K - 2]].y;
		}
		//夹角计算
		AOpBO = (Xa - Xo)*(Xb - Xo) + (Ya - Yo)*(Yb - Yo);
		AO = sqrt((Xa - Xo)*(Xa - Xo) + (Ya - Yo)*(Ya - Yo));
		BO = sqrt((Xb - Xo)*(Xb - Xo) + (Yb - Yo)*(Yb - Yo));
		cosAOB = abs(AOpBO / AO / BO);

		if (cosAOB < maxcos) conrnerPoints.points.push_back(boundarycloud.points[i]);  //夹角满足条件，认为是角点
	}
	
	return conrnerPoints;
}

pcXYZI Csegmentation::CornerpointRadius(const pcXYZI &boundarycloud, float radius, float disthreshold, float maxcos)
{
	pcXYZI conrnerPoints;

	// 先建KDtree
	pcl::KdTreeFLANN <pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(boundarycloud.makeShared());

	// Neighbors within radius search

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	for (int i = 0; i < boundarycloud.size(); i++){

		if (kdtree.radiusSearch(boundarycloud.points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance)>2){

			int K = pointIdxRadiusSearch.size(); // Radius 内的近邻点数

			float max1_max2;
			max1_max2 = sqrt(pointRadiusSquaredDistance[K - 1]) - sqrt(pointRadiusSquaredDistance[K - 2]);

			float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

			Xo = boundarycloud.points[i].x;
			Yo = boundarycloud.points[i].y;
			Xa = boundarycloud.points[pointIdxRadiusSearch[K - 1]].x;
			Ya = boundarycloud.points[pointIdxRadiusSearch[K - 1]].y;

			if (max1_max2 < disthreshold)  //若距离最大与次大点间距小于阈值，认为它们处于同侧，找异侧点
			{
				float maxdis = 0;
				int maxindex = -1;
				float Xc, Yc, Xd, Yd;
				Xc = boundarycloud.points[pointIdxRadiusSearch[K - 2]].x;
				Yc = boundarycloud.points[pointIdxRadiusSearch[K - 2]].y;
				//次远点找之前邻域点中的最远点
				for (int j = 0; j < K - 2; j++){
					Xd = boundarycloud.points[pointIdxRadiusSearch[j]].x;
					Yd = boundarycloud.points[pointIdxRadiusSearch[j]].y;

					float dis = sqrt((Xd - Xc)*(Xd - Xc) + (Yd - Yc)*(Yd - Yc));

					if (dis > maxdis) {
						maxdis = dis;
						maxindex = j;
					}
				}
				Xb = boundarycloud.points[pointIdxRadiusSearch[maxindex]].x;
				Yb = boundarycloud.points[pointIdxRadiusSearch[maxindex]].y;
			}

			//否则直接接受
			else{
				Xb = boundarycloud.points[pointIdxRadiusSearch[K - 2]].x;
				Yb = boundarycloud.points[pointIdxRadiusSearch[K - 2]].y;
			}
			//夹角计算
			AOpBO = (Xa - Xo)*(Xb - Xo) + (Ya - Yo)*(Yb - Yo);
			AO = sqrt((Xa - Xo)*(Xa - Xo) + (Ya - Yo)*(Ya - Yo));
			BO = sqrt((Xb - Xo)*(Xb - Xo) + (Yb - Yo)*(Yb - Yo));
			cosAOB = abs(AOpBO / AO / BO);

			if (cosAOB < maxcos) conrnerPoints.points.push_back(boundarycloud.points[i]);  //夹角满足条件，认为是角点
		}
	}

	
	return conrnerPoints;
}

void Csegmentation::CornerExtraction(const vector<pcXYZI> &boundaryclouds, vector<pcXYZI> &corners, bool UseRadius, int K, float radius, float dis_threshold, float maxcos)
{
	// 参数比较多 解释一下
	// bool UseRadius                是用什么策略搜索近邻，1. 一定距离内 (radius)  0. KNN
	// int K                         KNN中的点数
	// radius                        Radius搜索中的搜索半径
	// dis_threshold                 同侧点判断距离阈值 （不是很科学啊） 0.1
	// maxcos                        夹角限制阈值   0.94
	
	//vector<pcXYZI> corners;
	corners.resize(boundaryclouds.size());


	if (UseRadius) {
		for (int i = 0; i < corners.size(); i++)
		{
			corners[i] = CornerpointRadius(boundaryclouds[i], radius, dis_threshold, maxcos);
			/*if (corners[i].size() < 20 && corners[i].size() > 5) {
				cornerclouds.push_back(CornerClusteringKMeans(corners[i], 5));
			}// 判断是箭头，再KMeans聚类*/
		}
	}
	else{
		for (int i = 0; i < corners.size(); i++)
		{
			corners[i] = CornerpointKNN(boundaryclouds[i], K, dis_threshold, maxcos);  
			/*if (corners[i].size() < 20 && corners[i].size() > 5) {
				cornerclouds.push_back(CornerClusteringKMeans(corners[i], 5));
			}// 判断是箭头，再KMeans聚类*/
		}
	}
	
	/*for (int j = 0; j < corners.size(); j++){
		cout << j<<"  Cloud  "<< corners[j].size() << endl;
	}*/


	cout << "Corner Extraction Done" << endl;
}

pcXYZI Csegmentation::CornerClusteringKMeans(const pcXYZI &cornercloud, int K)
{
	//cout << "Kmeans began" << endl;
	KMeans kmeans;
	
	st_pointxyz center_arr[5] = {
		{ 0, 0, 0 },
		{ 2.5, 2.5, 2.5 },
		{ 3, 3, -3 },
		{ 1, 1, 1 },
		{ 2, 2, 2 }
	};

	kmeans.InputCloud(cornercloud.makeShared());
	kmeans.SetK(K);
	kmeans.InitKCenter(center_arr);
	kmeans.Cluster();
	pcXYZI CenteriodCloud;
	for (int i = 0; i < K; i++){
		pcl::PointXYZI pt;
		pt.x = kmeans.mv_center[i].x;
		pt.y = kmeans.mv_center[i].y;
		pt.z = kmeans.mv_center[i].z;
		pt.intensity = cornercloud.points[0].intensity;

		CenteriodCloud.push_back(pt);
	}
	
	cout << "Kmeans done.  point number " << CenteriodCloud.size()<< endl;
	return CenteriodCloud;
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