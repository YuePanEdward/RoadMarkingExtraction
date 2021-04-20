#ifndef INCLUDE_MODEL_MATCH_H
#define INCLUDE_MODEL_MATCH_H

#include <pcl/kdtree/kdtree_flann.h>
#include "utility.h"

using namespace std;

namespace roadmarking 
{
	class ModelMatch {
	public:

		ModelMatch()
		{
			iter_num_ = 12;
			correct_match_fitness_thre_ = 0.2;
			overlapDis_ = 0.15;
			tolerantMinOverlap_ = 0.75;
			heading_increment_ = 20.0;
		}

		//Constructor
		ModelMatch(int iter_num_0, float correct_match_fitness_thre_0, float heading_increment_0, float overlapDis_0, float tolerantMinOverlap_0) {
			iter_num_ = iter_num_0;
			correct_match_fitness_thre_ = correct_match_fitness_thre_0;
			overlapDis_ = overlapDis_0;
			tolerantMinOverlap_ = tolerantMinOverlap_0;
			heading_increment_ = heading_increment_0;
		}// order: KeypointDetection's RadiusNonMax,tolerantEuclideanDis,tolerantHammingDis,tolerantMinOverlap ratio;

		bool model_match(std::string model_file_path, vector<pcXYZI> &sceneclouds, RoadMarkings & roadmarkings);  // pathname: Model Folder's name

	protected:


	private:

		float icp_reg_4dof_global(const pcXYZIPtr & ModelCloud, const pcXYZIPtr & SceneCloud, bool with_reflection,
			Eigen::Matrix4f & tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre);

		float icp_reg(const pcXYZIPtr & SourceCloud, const pcXYZIPtr & TargetCloud,
			Eigen::Matrix4f & initial_guess, Eigen::Matrix4f & transformationS2T,
			int max_iter_num, float dis_thre);

		float cal_overlap_ratio(const pcXYZIPtr & search_cloud, pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree, float thre_dis);
			

		//Parameters
		float tolerantMinOverlap_;                                   
		int iter_num_;
		float heading_increment_ ;
		float correct_match_fitness_thre_;
		float overlapDis_;                                              

	};
}
#endif //INCLUDE_MODEL_MATCH_H