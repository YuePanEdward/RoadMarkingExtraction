#include "pointcloudprocess.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/surface/concave_hull.h>  
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>

#include <set>
#include <unordered_set>
#include <algorithm>

#include "utility.h"

using namespace std;

namespace roadmarking
{

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
			if (inclouds[i].size() > K)
			{
				outclouds.push_back(inclouds[i]);
			}
		}
		//cout << "Filtered Cloud Number: " << outclouds.size() << endl;
	}

	void Csegmentation::SORFilter(const pcXYZI &incloud, pcXYZI &outcloud, int MeanK, double std)
	{
		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;

		sor.setInputCloud(incloud.makeShared());
		sor.setMeanK(MeanK);//example 50
		sor.setStddevMulThresh(std);//exampel 1.0
		sor.filter(outcloud);

	}

	void Csegmentation::cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outSORclouds, int N, int MeanK, double std)
	{
		int cloudnumber = inclouds.size();
        cout<< "Candidate segement number: "<< cloudnumber<<endl;

		vector<pcXYZI> outclouds;

		outclouds.resize(cloudnumber);
		outSORclouds.resize(cloudnumber);

		//Otsu method thresholding
		int i;
#pragma omp parallel for private(i) //Multi-thread
		for (i = 0; i < cloudnumber; i++)
		{
			//store intensity [integer] 
			vector<int>  intensitylist;
			int pointnumber = inclouds[i].size();
			intensitylist.resize(pointnumber);
			for (int j = 0; j < pointnumber; j++)
			{
				intensitylist[j] = inclouds[i].points[j].intensity;   
			}
			int intensitymax, intensitymin;
			intensitymax = *(max_element(intensitylist.begin(), intensitylist.end()));
			//intensitymin = *(min_element(intensitylist.begin(), intensitylist.end()));

			//OTSU Method

			//Define Histogram
			vector<int> h0;
			vector<float> h;
			h0.resize(N);
			h.resize(N);

			for (int k = 0; k < N; k++) h0[k] = 0;

			//generate histogram
			for (int j = 0; j < pointnumber; j++)
			{
				int bin = (N - 1) * intensitylist[j] / intensitymax;
				h0[bin]++;
			}
			for (int k = 0; k < N; k++)
			{
				h[k] = (float)h0[k] / pointnumber;
			}

			int threshold = 0;

			float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
			for (int k = 0; k < N; k++)
			{
				w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;

				for (int t = 0; t < N; t++)
				{
					if (t <= k)
					{
						w0 += h[t];
						u0tmp += t * h[t];
					}
					else     
					{
						w1 += h[t];
						u1tmp += t * h[t];
					}
				}
				u0 = u0tmp / w0;       
				u1 = u1tmp / w1;     
				u = u0tmp + u1tmp;    

				deltaTmp = w0 * (u0 - u)*(u0 - u) + w1 * (u1 - u)*(u1 - u);

				if (deltaTmp > deltaMax)
				{
					deltaMax = deltaTmp;
					threshold = k;
				}
			}

			//cout << "Threshold : "<<threshold<< endl;
			for (int j = 0; j < pointnumber; j++)
			{
				int bin = (N - 1) * intensitylist[j] / intensitymax;
				if (bin > threshold)
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

	void Csegmentation::BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas)//��bounding box ��ֵ����� //˳�� minx,miny,maxx,maxy
	{
		boundingdatas.resize(clouds.size());

		int i;
#pragma omp parallel for private(i) //Multi-thread

		for (i = 0; i < clouds.size(); i++)
		{
			double max_x, max_y, min_x, min_y;
			int max_x_j, max_y_j, min_x_j, min_y_j;
			max_x = -DBL_MAX; max_x_j = 0;
			max_y = -DBL_MAX; max_y_j = 0;
			min_x = DBL_MAX; min_x_j = 0;
			min_y = DBL_MAX; min_y_j = 0;

			for (int j = 0; j < clouds[i].size(); j++) {
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
			boundingdatas[i].push_back(clouds[i].points[min_x_j]);
			boundingdatas[i].push_back(clouds[i].points[min_y_j]);
			boundingdatas[i].push_back(clouds[i].points[max_x_j]);
			boundingdatas[i].push_back(clouds[i].points[max_y_j]);
		}

	}
	void Csegmentation::BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures)
	{
		boundingfeatures.resize(boundingdatas.size());
		const double pi = 3.1415926;

		int i;
#pragma omp parallel for private(i) //Multi-thread
		for (i = 0; i < boundingdatas.size(); i++)
		{
			double Ax, Ay, Bx, By, Cx, Cy, Dx, Dy; 
			double AB, BC, CD, DA, AC, BD; 
			double ACpmBD;  
			double cornerrad;

			Ax = boundingdatas[i][0].x;
			Bx = boundingdatas[i][1].x;
			Cx = boundingdatas[i][2].x;
			Dx = boundingdatas[i][3].x;
			Ay = boundingdatas[i][0].y;
			By = boundingdatas[i][1].y;
			Cy = boundingdatas[i][2].y;
			Dy = boundingdatas[i][3].y;

			AB = sqrt((Ax - Bx)*(Ax - Bx) + (Ay - By)*(Ay - By));
			BC = sqrt((Bx - Cx)*(Bx - Cx) + (By - Cy)*(By - Cy));
			CD = sqrt((Cx - Dx)*(Cx - Dx) + (Cy - Dy)*(Cy - Dy));
			DA = sqrt((Dx - Ax)*(Dx - Ax) + (Dy - Ay)*(Dy - Ay));
			AC = sqrt((Ax - Cx)*(Ax - Cx) + (Ay - Cy)*(Ay - Cy));
			BD = sqrt((Bx - Dx)*(Bx - Dx) + (By - Dy)*(By - Dy));

		
			ACpmBD = abs((Ax - Cx)*(Bx - Dx) + (Ay - Cy)*(By - Dy)); 

			cornerrad = acos(ACpmBD / AC / BD); 
			boundingfeatures[i].corner = cornerrad / pi * 180; 
			boundingfeatures[i].sortingEdges.push_back(AB);
			boundingfeatures[i].sortingEdges.push_back(BC);
			boundingfeatures[i].sortingEdges.push_back(CD);
			boundingfeatures[i].sortingEdges.push_back(DA);

			sort(boundingfeatures[i].sortingEdges.begin(), boundingfeatures[i].sortingEdges.end()); 
		}

	}

	void Csegmentation::CategoryJudgementBox_highway(const vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings)
	{
		roadmarkings.resize(boundingfeatures.size());


		double angt1, angt2, lt1, lt2, lt3, lt4, lt5, ratio1, ratio2;

		angt1 = 1.0;
		angt2 = 45;

		lt1 = 0.03;
		lt2 = 0.6;
		lt3 = 3.0;
		lt4 = 8.5;
		lt5 = 0.4;
	
		ratio1 = 1.35;
		ratio2 = 1.5;
		for (int i = 0; i < boundingfeatures.size(); i++)
		{
			bool rough_classified = false;

			if (/*boundingfeatures[i].corner < angt1
				&& */boundingfeatures[i].sortingEdges[3] > lt4 && boundingfeatures[i].sortingEdges[2] > lt4
				&& boundingfeatures[i].sortingEdges[1] < lt5 && boundingfeatures[i].sortingEdges[0] < lt5
				&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio2)
			{
				roadmarkings[i].category = 1;  
				rough_classified = true;
			}

			if (boundingfeatures[i].corner > angt1 && boundingfeatures[i].corner < angt2
				&& boundingfeatures[i].sortingEdges[3]<lt4 && boundingfeatures[i].sortingEdges[3]>lt3
				&& boundingfeatures[i].sortingEdges[2]<lt4 && boundingfeatures[i].sortingEdges[2]>lt3
				&& boundingfeatures[i].sortingEdges[1]<lt2 && boundingfeatures[i].sortingEdges[1]>lt1
				&& boundingfeatures[i].sortingEdges[0]<lt2 && boundingfeatures[i].sortingEdges[0]>lt1
				&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1
				&& boundingfeatures[i].sortingEdges[1] / boundingfeatures[i].sortingEdges[0] < ratio1)
			{
				roadmarkings[i].category = 2;  
				rough_classified = true;
			}
			if (rough_classified)
				cout << "Object [" << i << "] ---> Category [" << roadmarkings[i].category << "]" << endl;

		}
		//cout << "Classification based on Bounding Box Information done" << endl;
	}


	void Csegmentation::CategoryJudgementBox_cityroad(const std::vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings)
	{
		roadmarkings.resize(boundingfeatures.size());

		double angt1, angt2, angt3, angt4, angt5, lt1, lt2, lt3, lt4, lt5, lt6, lt7, lt8, ratio1, ratio2;
		angt1 = 2.0;
		angt2 = 50.0;

		angt3 = 70.0;
		angt4 = 110.0;

		angt5 = 5.5;

		lt1 = 0.05;
		lt2 = 0.55;
		lt3 = 0.8;
		lt4 = 7.0;
		lt5 = 0.4;
		lt6 = 7.5;
		lt7 = 2.2;
		lt8 = 1.0;

		ratio1 = 2.0;
		ratio2 = 1.3;

		for (int i = 0; i < boundingfeatures.size(); i++)
		{
			bool rough_classified = false;
			if (boundingfeatures[i].corner < angt5 &&
				boundingfeatures[i].sortingEdges[3] > lt6 && boundingfeatures[i].sortingEdges[2] > lt6
				&& (boundingfeatures[i].sortingEdges[1] < lt5 || boundingfeatures[i].sortingEdges[0] < lt5)
				&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio2)
			{
				roadmarkings[i].category = 1;  
				rough_classified = true;
			}

			if (boundingfeatures[i].corner > angt1 && boundingfeatures[i].corner < angt2
				&& boundingfeatures[i].sortingEdges[3]<lt4 && boundingfeatures[i].sortingEdges[3]>lt3
				&& boundingfeatures[i].sortingEdges[2]<lt4 && boundingfeatures[i].sortingEdges[2]>lt3
				&& boundingfeatures[i].sortingEdges[1]<lt2 && boundingfeatures[i].sortingEdges[1]>lt1
				&& boundingfeatures[i].sortingEdges[0]<lt2 && boundingfeatures[i].sortingEdges[0]>lt1
				&& boundingfeatures[i].sortingEdges[3] / boundingfeatures[i].sortingEdges[2] < ratio1
				&& boundingfeatures[i].sortingEdges[1] / boundingfeatures[i].sortingEdges[0] < ratio1)
			{
				roadmarkings[i].category = 2;  
				rough_classified = true;
			}

			if (boundingfeatures[i].corner > angt3 && boundingfeatures[i].corner < angt4
				&& boundingfeatures[i].sortingEdges[3]>lt8 && boundingfeatures[i].sortingEdges[3] < lt7
				&& boundingfeatures[i].sortingEdges[2]>lt8 && boundingfeatures[i].sortingEdges[2] < lt7
				&& boundingfeatures[i].sortingEdges[1]>lt8 && boundingfeatures[i].sortingEdges[1] < lt7
				&& boundingfeatures[i].sortingEdges[0]>lt8 && boundingfeatures[i].sortingEdges[0] < lt7)
			{
				roadmarkings[i].category = 8;  
				rough_classified = true;
			}
			if (rough_classified)
				cout << "Object [" << i << "] ---> Category [" << roadmarkings[i].category << "]" << endl;
		}
		//cout << "Classification based on Bounding Box Information done" << endl;
	}

	void Csegmentation::Find_tail2head(int tail_index, const vector<vector<double>> & d_tail_head, vector < bool > & line_used, vector<int> & combineline, double combine_length)
	{
		int sideline_number = line_used.size();
		for (int j = 0; j < sideline_number; j++)
		{
			if (line_used[j] == false && tail_index != j && d_tail_head[tail_index][j] < combine_length) {
				combineline.push_back(j); 
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
			if (line_used[j] == false && head_index != j && d_head_tail[head_index][j] < combine_length) {
				combineline.insert(combineline.begin(), j); 
				line_used[j] = true;
				Find_head2tail(j, d_head_tail, line_used, combineline, combine_length);
				break;
			}
		}

	}

	void Csegmentation::CombineSideLines(const RoadMarkings & roadmarkings, double Combine_length, RoadMarkings & combine_sideline_markings)
	{
		vector<pair<pcl::PointXYZI, pcl::PointXYZI>> sidelines;
		vector<int> sidelines_index_in_roadmarkings;
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

		for (int i = 0; i < sideline_number; i++)
		{
			for (int j = 0; j < sideline_number; j++)
			{
				if (i != j) {

					D_tail_head[i][j] = sqrt((sidelines[i].second.x - sidelines[j].first.x)*(sidelines[i].second.x - sidelines[j].first.x)
						+ (sidelines[i].second.y - sidelines[j].first.y)*(sidelines[i].second.y - sidelines[j].first.y));
					D_head_tail[i][j] = sqrt((sidelines[i].first.x - sidelines[j].second.x)*(sidelines[i].first.x - sidelines[j].second.x)
						+ (sidelines[i].second.y - sidelines[j].first.y)*(sidelines[i].second.y - sidelines[j].first.y));

				}
			}
		}

		vector<vector<int>> Combinelines; 
		vector<bool> Line_used(sideline_number);  

		for (int i = 0; i < sideline_number; i++) Line_used[i] = false;  
		for (int i = 0; i < sideline_number; i++)
		{
			if (Line_used[i] == false) {
				vector<int> Combineline;
				Combineline.push_back(i);
				Line_used[i] = true;

				Find_tail2head(i, D_tail_head, Line_used, Combineline, Combine_length);
				Find_head2tail(i, D_head_tail, Line_used, Combineline, Combine_length);

				Combinelines.push_back(Combineline);
			}
		}

		combine_sideline_markings.resize(Combinelines.size());

		for (int i = 0; i < Combinelines.size(); i++)
		{
			for (int j = 0; j < Combinelines[i].size(); j++) {
				int roadmarkings_index = sidelines_index_in_roadmarkings[Combinelines[i][j]];
				for (int k = 0; k < roadmarkings[roadmarkings_index].polyline.size(); k++)
				{
					combine_sideline_markings[i].polyline.push_back(roadmarkings[roadmarkings_index].polyline[k]);
				}
			}
		}

	}


	void Csegmentation::GetRoadmarkingsForVect(RoadMarkings & roadmarkings,	RoadMarkings & roadmarkings_sideline, RoadMarkings & roadmarkings_vect)
	{
		for (int i = 0; i < roadmarkings_sideline.size(); i++)
			roadmarkings_vect.push_back(roadmarkings_sideline[i]);

		for (int i = 0; i < roadmarkings.size(); i++)
		{
			if ((roadmarkings[i].category > 1 && roadmarkings[i].category < 6) || roadmarkings[i].category == 8)
				roadmarkings_vect.push_back(roadmarkings[i]);
		}
	}

	void Csegmentation::MarkingVectorization_highway(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas,
		RoadMarkings & roadmarkings, double line_sample_dl, double ambiguousRatio)
	{
        vector<int> categorynumber;
		vector<double> Xlist; 
		const double pi = 3.1415926;

		categorynumber.resize(10); 
		for (int m = 0; m < 10; m++)
		{
			categorynumber[m] = 0;
		}

		for (int i = 0; i < clouds.size(); i++)
		{
			pcl::PointXYZ pt_temp;

			switch (roadmarkings[i].category)
			{
			case 1:
			{
				categorynumber[1]++;

				double Xmin, Xmax, Ymin, Ymax;
				Xmin = boundingdatas[i][0].x;
				Ymin = boundingdatas[i][1].y;
				Xmax = boundingdatas[i][2].x;
				Ymax = boundingdatas[i][3].y;

				double dX, dY;
				dX = Xmax - Xmin;
				dY = Ymax - Ymin;

				double deltaX; 
				double sampleK_d;
				int sampleK;

				deltaX = 0.75 * line_sample_dl*dX / sqrt(dX*dX + dY * dY);    
				sampleK_d = dX / deltaX;
				sampleK = sampleK_d;

				//cout << "Cloud  " << i << "  Sample number: " << sampleK<<endl;

				roadmarkings[i].polyline.resize(sampleK + 2);
				roadmarkings[i].polyline[0] = boundingdatas[i][0];
				roadmarkings[i].polyline[sampleK + 1] = boundingdatas[i][3]; 


				for (int k = 1; k <= sampleK; k++) {
					Xlist.push_back(Xmin + k * deltaX);
				}

				for (int j = 0; j < clouds[i].size(); j++)
				{
					for (int k = 0; k < Xlist.size(); k++)
					{
						auto iter = Xlist.begin();
						if (clouds[i].points[j].x > Xlist[k] - ambiguousRatio * deltaX && clouds[i].points[j].x < Xlist[k] + ambiguousRatio * deltaX)
						{
							double this_k_d = (Xlist[k] - Xmin + 0.001) / deltaX;
							int this_k = this_k_d;
							roadmarkings[i].polyline[this_k] = clouds[i].points[j]; 
							Xlist.erase(iter); 
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
			}
			case 2: 
			{
				categorynumber[2]++;

				roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
				break;
			}
			case 3: //3 forward arrow (ready)
			{
				categorynumber[3]++;
				pcXYZPtr forward_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(-0.225, -4.5, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, -4.5, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, 0.9, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.675, 0.9, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0, 4.5, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.675, 0.9, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.225, 0.9, 0.0);
				forward_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*forward_arrow_points, *forward_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(7);

				for (int j = 0; j < 7; j++)
				{
					roadmarkings[i].polyline[j].x = forward_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = forward_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = forward_arrow_points->points[j].z;
				}
                /*
				theta=arctan((XA-XB)/(YA-YB))
				XA-XB     YA-YB      theta
				>0        >0         theta
				>0        <0         180+theta
				<0        <0         180+theta
				<0        >0         360+theta
				X=cos(theta)*X' -sin(theta)*Y'+XB
				Y=sin(theta)*X' +cos(theta)*Y'+YB
				*/
				break;
			}
			case 4: //4 forward right arrow (ready)
			{
				categorynumber[4]++;

				pcXYZPtr forward_right_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(0, 4.5, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.675, 0.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.225, 0.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.225, -4.5, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, -4.5, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, -3.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.425, -3.0, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.425, -4.35, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(2.025, -2.1, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.425, 0.3, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.425, -1.2, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, -2.1, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.225, 0.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.675, 0.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*forward_right_arrow_points, *forward_right_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(14);

				for (int j = 0; j < 14; j++)
				{
					roadmarkings[i].polyline[j].x = forward_right_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = forward_right_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = forward_right_arrow_points->points[j].z;
				}

				/*
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
				XA-XB     YA-YB      theta
				>0        >0         theta
				>0        <0         180+theta
				<0        <0         180+theta
				<0        >0         360+theta
				X=cos(theta)*X' -sin(theta)*Y'+XB
				Y=sin(theta)*X' +cos(theta)*Y'+YB
				*/

				break;
			}
			case 5: 
			{
				categorynumber[5]++;

				pcXYZPtr right_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(0.95, 3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.35, 1.4, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, -0.1, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, 0.8, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, 0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, -3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, 0.9, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, 2.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*right_arrow_points, *right_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(9);

				for (int j = 0; j < 9; j++)
				{
					roadmarkings[i].polyline[j].x = right_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = right_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = right_arrow_points->points[j].z;
				}
				break;
			}
			case 6: //6
			{
				categorynumber[6]++;

				//TODO: add the vectorization model (vertex coordinates)

				break;
			}
			case 8: //8 
			{
				categorynumber[8]++;
				roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
				break;
			}
			default:
				break;
			}
		}
		/*
		cout << "Side Line Road Markings Number: " << categorynumber[1] << endl;
		cout << "Rectangle Road Markings Number: " << categorynumber[2] << endl;
		cout << "Arrow Road Markings (Forward) Number: " << categorynumber[3] << endl;
		cout << "Arrow Road Markings (Forward and Rightward / Leftward) Number: " << categorynumber[4] << endl;
		cout << "Arrow Road Markings (Rightward / Leftward) Number: " << categorynumber[5] << endl;
		cout << "Arrow Road Markings (U-turn) Number: " << categorynumber[6] << endl;
		cout << "Pedestrian Warning Markings Number: " << categorynumber[8] << endl;
		*/
	}

	void Csegmentation::MarkingVectorization_cityroad(const vector<pcXYZI> &clouds,
		const vector<vector<pcl::PointXYZI>> &boundingdatas,
		RoadMarkings & roadmarkings, double line_sample_dl, double ambiguousRatio)
	{

		vector<int> categorynumber;
		vector<double> Xlist; 
		const double pi = 3.1415926;

		categorynumber.resize(10); 
		for (int m = 0; m < 10; m++)
		{
			categorynumber[m] = 0;
		}

		for (int i = 0; i < clouds.size(); i++)
		{
			pcl::PointXYZ pt_temp;

			switch (roadmarkings[i].category)
			{
			case 1: 
			{
				categorynumber[1]++;

				double Xmin, Xmax, Ymin, Ymax;
				Xmin = boundingdatas[i][0].x;
				Ymin = boundingdatas[i][1].y;
				Xmax = boundingdatas[i][2].x;
				Ymax = boundingdatas[i][3].y;

				double dX, dY;
				dX = Xmax - Xmin;
				dY = Ymax - Ymin;

				double deltaX;  
				double sampleK_d;
				int sampleK; 

				deltaX = 0.75 * line_sample_dl*dX / sqrt(dX*dX + dY * dY);    
				sampleK_d = dX / deltaX;
				sampleK = sampleK_d;

				//cout << "Cloud  " << i << "  Sample number: " << sampleK<<endl;

				roadmarkings[i].polyline.resize(sampleK + 2);
				roadmarkings[i].polyline[0] = boundingdatas[i][0]; 
				roadmarkings[i].polyline[sampleK + 1] = boundingdatas[i][3]; 


				for (int k = 1; k <= sampleK; k++) {
					Xlist.push_back(Xmin + k * deltaX);
				}

				for (int j = 0; j < clouds[i].size(); j++)
				{
					for (int k = 0; k < Xlist.size(); k++)
					{
						auto iter = Xlist.begin();
						if (clouds[i].points[j].x > Xlist[k] - ambiguousRatio * deltaX && clouds[i].points[j].x < Xlist[k] + ambiguousRatio * deltaX)
						{
							double this_k_d = (Xlist[k] - Xmin + 0.001) / deltaX;
							int this_k = this_k_d;
							roadmarkings[i].polyline[this_k] = clouds[i].points[j]; 
							Xlist.erase(iter); 
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
			}
			case 2: 
			{
				categorynumber[2]++;

				roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
				break;
			}
			case 3: 
			{
				categorynumber[3]++;

				pcXYZPtr forward_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(-0.15, -3.0, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -3.0, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, 0.6, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.45, 0.6, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0, 3.0, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.45, 0.6, 0.0);
				forward_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, 0.6, 0.0);
				forward_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*forward_arrow_points, *forward_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(7);

				for (int j = 0; j < 7; j++)
				{
					roadmarkings[i].polyline[j].x = forward_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = forward_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = forward_arrow_points->points[j].z;
				}

				/*
				ID     X'       Y'
				0      0        6
				1    -0.45     3.6
				2    -0.15     3.6
				3    -0.15      0
				4     0.15      0
				5     0.15     3.6
				6     0.45     3.6
				theta=arctan((XA-XB)/(YA-YB))
				XA-XB     YA-YB      theta
				>0        >0         theta
				>0        <0         180+theta
				<0        <0         180+theta
				<0        >0         360+theta
				X=cos(theta)*X' -sin(theta)*Y'+XB
				Y=sin(theta)*X' +cos(theta)*Y'+YB
				*/
				break;
			}//4�� ��ǰ���Ҽ�ͷ
			{
				categorynumber[4]++;

				pcXYZPtr forward_right_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(0, 3.0, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.45, 0.6, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, 0.6, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, -3.0, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -3.0, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -2.4, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, -2.0, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, -2.9, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.35, -1.4, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, 0.2, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, -0.8, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -1.4, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, 0.6, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.45, 0.6, 0.0);
				forward_right_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*forward_right_arrow_points, *forward_right_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(14);

				for (int j = 0; j < 14; j++)
				{
					roadmarkings[i].polyline[j].x = forward_right_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = forward_right_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = forward_right_arrow_points->points[j].z;
				}

				/*
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
				XA-XB     YA-YB      theta
				>0        >0         theta
				>0        <0         180+theta
				<0        <0         180+theta
				<0        >0         360+theta
				X=cos(theta)*X' -sin(theta)*Y'+XB
				Y=sin(theta)*X' +cos(theta)*Y'+YB
				*/

				break;
			}
			case 5: 
			{
				categorynumber[5]++;

				pcXYZPtr right_arrow_points(new pcXYZ());
				pt_temp = pcl::PointXYZ(0.95, 3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(1.35, 1.4, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, -0.1, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, 0.8, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, 0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.15, -3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, -3.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(-0.15, 0.9, 0.0);
				right_arrow_points->points.push_back(pt_temp);
				pt_temp = pcl::PointXYZ(0.95, 2.0, 0.0);
				right_arrow_points->points.push_back(pt_temp);

				pcl::transformPointCloud(*right_arrow_points, *right_arrow_points, roadmarkings[i].localization_tranmat_m2s);

				roadmarkings[i].polyline.resize(9);

				for (int j = 0; j < 9; j++)
				{
					roadmarkings[i].polyline[j].x = right_arrow_points->points[j].x;
					roadmarkings[i].polyline[j].y = right_arrow_points->points[j].y;
					roadmarkings[i].polyline[j].z = right_arrow_points->points[j].z;
				}
				/*
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
				XA-XB     YA-YB      theta
				>0        >0         theta
				>0        <0         180+theta
				<0        <0         180+theta
				<0        >0         360+theta
				X=cos(theta)*X' -sin(theta)*Y'+XB
				Y=sin(theta)*X' +cos(theta)*Y'+YB
				*/

				break;
			}
			case 6: //6  //TODO: add model
			{
				categorynumber[6]++;
				break;
			}
			case 7: //7  //TODO: add model
			{
				categorynumber[7]++;
				break;
			}
			case 8: //8 //TODO: add model
			{
				categorynumber[8]++;
				roadmarkings[i].polyline.push_back(boundingdatas[i][0]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][1]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][2]);
				roadmarkings[i].polyline.push_back(boundingdatas[i][3]);
				break;
			}
			default:
				break;
			}
		}
		/*
		cout << "Side Line Road Markings Number: " << categorynumber[1] << endl;
		cout << "Rectangle Road Markings Number: " << categorynumber[2] << endl;
		cout << "Arrow Road Markings (Forward) Number: " << categorynumber[3] << endl;
		cout << "Arrow Road Markings (Forward and Rightward / Leftward) Number: " << categorynumber[4] << endl;
		cout << "Arrow Road Markings (Rightward / Leftward) Number: " << categorynumber[5] << endl;
		cout << "Arrow Road Markings (U-turn) Number: " << categorynumber[6] << endl;
		cout << "Cross Marking Number: " << categorynumber[7] << endl;
		cout << "Pedestrian Warning Markings Number: " << categorynumber[8] << endl;
		*/
	}


	void Csegmentation::BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds, int down_rate, float alpha_value_scale)
	{
		boundaryclouds.resize(clouds.size());
		int i;
		//#pragma omp parallel for private(i) //Multi-thread

		for (i = 0; i < clouds.size(); i++)
		{
			boundaryclouds[i] = alphashape(clouds[i], resolution*alpha_value_scale); //this is the parameter for alpha-shape, very important

			pcXYZIPtr tempcloud(new pcXYZI);
			for (int j = 0; j < boundaryclouds[i].points.size(); j++)
			{
				if (j % down_rate == 0)
					tempcloud->points.push_back(boundaryclouds[i].points[j]);
			}
			tempcloud->points.swap(boundaryclouds[i].points);
		}
		//cout << "Boundary Extraction Done" << endl;
	}


	pcXYZI Csegmentation::alphashape(const pcXYZI &cloud, float alpha_value) //Concave Hull Generation
	{
		pcXYZI cloud_hull;
		pcl::ConcaveHull<pcl::PointXYZI> chull;       
		chull.setInputCloud(cloud.makeShared());       
		chull.setAlpha(alpha_value);              
		chull.reconstruct(cloud_hull);

		//std::cout<< "Concave hull has: " << cloud_hull->points.size() << " data points." << endl;
		return cloud_hull;
	}

	pcXYZI Csegmentation::CornerpointKNN(const pcXYZI &boundarycloud, int K, float disthreshold, float maxcos)
	{
		pcXYZI conrnerPoints;

		// �Ƚ�KDtree
		pcl::KdTreeFLANN <pcl::PointXYZI> kdtree;
		kdtree.setInputCloud(boundarycloud.makeShared());

		vector<int> pointIdxNKNSearch(K);   
		vector<float> pointNKNSquaredDistance(K); 

		for (int i = 0; i < boundarycloud.size(); i++) {

			kdtree.nearestKSearch(boundarycloud.points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance);  
			float max1_max2;
			max1_max2 = sqrt(pointNKNSquaredDistance[K - 1]) - sqrt(pointNKNSquaredDistance[K - 2]);

			float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

			Xo = boundarycloud.points[i].x;
			Yo = boundarycloud.points[i].y;
			Xa = boundarycloud.points[pointIdxNKNSearch[K - 1]].x;
			Ya = boundarycloud.points[pointIdxNKNSearch[K - 1]].y;

			if (max1_max2 < disthreshold)  
			{
				float maxdis = 0;
				int maxindex = -1;
				float Xc, Yc, Xd, Yd;
				Xc = boundarycloud.points[pointIdxNKNSearch[K - 2]].x;
				Yc = boundarycloud.points[pointIdxNKNSearch[K - 2]].y;
				
				for (int j = 0; j < K - 2; j++) {
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

			else {
				Xb = boundarycloud.points[pointIdxNKNSearch[K - 2]].x;
				Yb = boundarycloud.points[pointIdxNKNSearch[K - 2]].y;
			}

			AOpBO = (Xa - Xo)*(Xb - Xo) + (Ya - Yo)*(Yb - Yo);
			AO = sqrt((Xa - Xo)*(Xa - Xo) + (Ya - Yo)*(Ya - Yo));
			BO = sqrt((Xb - Xo)*(Xb - Xo) + (Yb - Yo)*(Yb - Yo));
			cosAOB = abs(AOpBO / AO / BO);

			if (cosAOB < maxcos) conrnerPoints.points.push_back(boundarycloud.points[i]); 
		}

		return conrnerPoints;
	}

	pcXYZI Csegmentation::CornerpointRadius(const pcXYZI &boundarycloud, float radius, float disthreshold, float maxcos)
	{
		pcXYZI conrnerPoints;

		// �Ƚ�KDtree
		pcl::KdTreeFLANN <pcl::PointXYZI> kdtree;
		kdtree.setInputCloud(boundarycloud.makeShared());

		// Neighbors within radius search

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;

		for (int i = 0; i < boundarycloud.size(); i++) {

			if (kdtree.radiusSearch(boundarycloud.points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 2) {

				int K = pointIdxRadiusSearch.size(); 

				float max1_max2;
				max1_max2 = sqrt(pointRadiusSquaredDistance[K - 1]) - sqrt(pointRadiusSquaredDistance[K - 2]);

				float Xa, Xb, Xo, Ya, Yb, Yo, AOpBO, AO, BO, cosAOB;

				Xo = boundarycloud.points[i].x;
				Yo = boundarycloud.points[i].y;
				Xa = boundarycloud.points[pointIdxRadiusSearch[K - 1]].x;
				Ya = boundarycloud.points[pointIdxRadiusSearch[K - 1]].y;

				if (max1_max2 < disthreshold)  
				{
					float maxdis = 0;
					int maxindex = -1;
					float Xc, Yc, Xd, Yd;
					Xc = boundarycloud.points[pointIdxRadiusSearch[K - 2]].x;
					Yc = boundarycloud.points[pointIdxRadiusSearch[K - 2]].y;
					for (int j = 0; j < K - 2; j++) {
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

				else {
					Xb = boundarycloud.points[pointIdxRadiusSearch[K - 2]].x;
					Yb = boundarycloud.points[pointIdxRadiusSearch[K - 2]].y;
				}
				AOpBO = (Xa - Xo)*(Xb - Xo) + (Ya - Yo)*(Yb - Yo);
				AO = sqrt((Xa - Xo)*(Xa - Xo) + (Ya - Yo)*(Ya - Yo));
				BO = sqrt((Xb - Xo)*(Xb - Xo) + (Yb - Yo)*(Yb - Yo));
				cosAOB = abs(AOpBO / AO / BO);

				if (cosAOB < maxcos) conrnerPoints.points.push_back(boundarycloud.points[i]);  
			}
		}


		return conrnerPoints;
	}

	void Csegmentation::CornerExtraction(const vector<pcXYZI> &boundaryclouds, vector<pcXYZI> &corners, bool UseRadius, int K, float radius, float dis_threshold, float maxcos)
	{
		corners.resize(boundaryclouds.size());


		if (UseRadius) {
			for (int i = 0; i < corners.size(); i++)
			{
				corners[i] = CornerpointRadius(boundaryclouds[i], radius, dis_threshold, maxcos);
			}
		}
		else {
			for (int i = 0; i < corners.size(); i++)
				corners[i] = CornerpointKNN(boundaryclouds[i], K, dis_threshold, maxcos);	
		}
		cout << "Corner Extraction Done" << endl;
	}

# if 0
	pcXYZI Csegmentation::CornerClusteringKMeans(const pcXYZI &cornercloud, int K)
	{
		// KMeans kmeans;

		// st_pointxyz center_arr[5] = {
		// 	{ 0, 0, 0 },
		// 	{ 2.5, 2.5, 2.5 },
		// 	{ 3, 3, -3 },
		// 	{ 1, 1, 1 },
		// 	{ 2, 2, 2 }
		// };

		// kmeans.InputCloud(cornercloud.makeShared());
		// kmeans.SetK(K);
		// kmeans.InitKCenter(center_arr);
		// kmeans.Cluster();
		// pcXYZI CenteriodCloud;
		// for (int i = 0; i < K; i++) {
		// 	pcl::PointXYZI pt;
		// 	pt.x = kmeans.mv_center[i].x;
		// 	pt.y = kmeans.mv_center[i].y;
		// 	pt.z = kmeans.mv_center[i].z;
		// 	pt.intensity = cornercloud.points[0].intensity;

		// 	CenteriodCloud.push_back(pt);
		// }

		// cout << "Kmeans done.  point number " << CenteriodCloud.size() << endl;
		// return CenteriodCloud;
	}
#endif

	pcXYZIPtr Csegmentation::planesegRansac(const pcXYZIPtr &cloud, float threshold)
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

		cout << "Model inliers number: " << inliers->indices.size() << std::endl;
		pcXYZIPtr fitcloud(new pcXYZI());

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
		coefficients->values[0] = coefficients->values[1] = coefficients->values[3] = 0;
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
}