#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include "utility.h"

using namespace std;

namespace roadmarking
{

	class Csegmentation
	{
	public:
		
		Csegmentation(float res){ resolution = res; };

		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud);   // PMF �������˲�;
		void GroundFilter_PMF(const pcXYZIPtr &cloud, pcXYZIPtr &gcloud, pcXYZIPtr &ngcloud, int max_window_size, float slope, float initial_distance, float max_distance);   // PMF �������˲� ��������;
		
		pcXYZIPtr planesegRansac(const pcXYZIPtr &cloud, float threshold);   //Ransac ƽ�����;
		pcXYZIPtr groundprojection(const pcXYZIPtr &cloud);                  //���ƽ�ά���(ͶӰ��;
		void cloudFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int N, int MeanK, double std);  //Otsu Intensity Thresholding + SOR Filter 
		void SORFilter(const pcXYZI &inclouds, pcXYZI &outclouds, int MeanK, double std); //SOR ��Statisics Outliers Remover) ���ҲӦ��д�ɶ����е��ƵĴ����ȽϺ�;
		void NFilter(const vector<pcXYZI> &inclouds, vector<pcXYZI> &outclouds, int K); //���ܵ�������һ�Σ�С��K�����ɾȥ

		void BoundingInformation(const vector<pcXYZI> &clouds, vector<vector<pcl::PointXYZI>> & boundingdatas); //bounding 4 points ��ȡ;
		void BoundingFeatureCalculation(const vector<vector<pcl::PointXYZI>> & boundingdatas, vector<BoundingFeature> & boundingfeatures); //bounding 4 points ������������;
		void BoundaryExtraction(const vector<pcXYZI> &clouds, vector<pcXYZI> &boundaryclouds, int down_rate=1, float alpha_value_scale = 0.8);                 //����ȡ�߽��;
		void CornerExtraction(const vector<pcXYZI> &boundaryclouds, vector<pcXYZI> &cornerclouds, bool UseRadius, int K, float radius, float dis_threshold, float maxcos);     //����ȡ�߽��ϵĽǵ�;
		
		void CategoryJudgementBox_highway(const vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings); //���ٹ�· ��·���ߴַ���(����bounding box��Ϣ��;
		void CategoryJudgementBox_cityroad(const vector<BoundingFeature> & boundingfeatures, RoadMarkings & roadmarkings); //���е�· ��·���ߴַ���(����bounding box��Ϣ��;

		void CombineSideLines(const RoadMarkings & roadmarkings, double combine_length, RoadMarkings & combine_sideline_markings);  // ��·�������л�����;
		void Find_head2tail(int head_index, const vector<vector<double>> & d_head_tail, vector<bool> & line_used, vector<int> & combineline, double combine_length);   //�ݹ麯��
		void Find_tail2head(int tail_index, const vector<vector<double>> & d_tail_head, vector<bool> & line_used, vector<int> & combineline, double combine_length);   //�ݹ麯��
		
		void MarkingVectorization_highway(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas,RoadMarkings & roadmarkings,  double line_sample_dl, double ambiguousRatio);  //��·���߷��༰ʸ���� //line_sample_dl �߲�������   // ambiguousRatio ģ���������� //0.2
		void MarkingVectorization_cityroad(const vector<pcXYZI> &clouds, const vector<vector<pcl::PointXYZI>> &boundingdatas, RoadMarkings & roadmarkings, double line_sample_dl, double ambiguousRatio);  //��·���߷��༰ʸ���� //line_sample_dl �߲�������   // ambiguousRatio ģ���������� //0.2
		void GetRoadmarkingsForVect(RoadMarkings & roadmarkings, RoadMarkings & roadmarkings_sideline, RoadMarkings & roadmarkings_vect);

		pcXYZI alphashape(const pcXYZI &cloud, float alpha_value);   //Concave Hull Generation
		
		pcXYZI CornerpointKNN(const pcXYZI &boundarycloud, int K, float disthreshold, float maxcos);                   //KNN corner point extraction 
		pcXYZI CornerpointRadius(const pcXYZI &boundarycloud, float radius, float disthreshold, float maxcos);         //Radius corner point extraction 

		pcXYZI CornerClusteringKMeans(const pcXYZI &cornercloud, int K);

		
	protected:
	
	private:
		float resolution;
		
	};
}
#endif