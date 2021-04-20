

#include "modelmatch.h"
#include "data_io.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#define BOOST_TYPEOF_EMULATION
#include <pcl/registration/icp.h> 


namespace roadmarking
{

	bool ModelMatch::model_match(std::string model_folder_path, vector<pcXYZI> &scenePointClouds, RoadMarkings & roadmarkings) {

		float correct_match_fitness_thre = correct_match_fitness_thre_;
		float overlapping_dist_thre = overlapDis_;
		float heading_increment = heading_increment_;
		int iter_num = iter_num_;
		
		float corr_dist_thre = FLT_MAX;

		//modeldatas.resize(scenePointClouds.size());
		//is_rights.resize(scenePointClouds.size());

		vector<string> modelFileNames;
		string extension = ".pcd"; 
		string pathName = model_folder_path; //path of the model pool

		DataIo io;
		io.batchReadFileNamesInFolders(pathName, extension, modelFileNames);

		cout << "Load [" << modelFileNames.size() << "] models from the model pool." << endl;

		std::vector<pcXYZIPtr> modelPointClouds(modelFileNames.size());

		for (int j = 0; j < modelFileNames.size(); j++) //for different models
		{
			modelPointClouds[j] = boost::make_shared<pcXYZI>();
			
			//liblas
			//Bounds bound_3d_temp;
			//io.readPointCloudFromLasFile(modelPointClouds[j], modelFileNames[j], bound_3d_temp);
			//io.readLasFile(modelFileNames[j], *modelPointClouds[j]);
			io.readPcdFile(modelFileNames[j], modelPointClouds[j]);

			if (modelPointClouds[j]->points.empty())
			{
				cout << "Error, "<< modelFileNames[j] << " is empty." << endl;
				return 0;
			}
		}
        

		int min_point_num_for_match = 50;
		int i;
#pragma omp parallel for private(i) //Multi-thread
		//select the model resulting in the largest overlapping ratio with its fitness score larger than a threshold
		for (i = 0; i < scenePointClouds.size(); i++)
		{
			//cout << "Size of No. " << i << "Pointcloud: " << scenePointClouds[i].points.size() << endl;
			if (scenePointClouds[i].points.size() < min_point_num_for_match) continue;
			if (roadmarkings[i].category != 1 && roadmarkings[i].category != 2)
			{
				pcXYZIPtr current_scenecloud(new pcXYZI(scenePointClouds[i]));
				
				float best_overlapping_ratio = tolerantMinOverlap_;
				float match_fitness_best;
				Eigen::Matrix4f tran_mat_m2s_best_match;
				int best_model_index = -1;

				pcl::KdTreeFLANN<pcl::PointXYZI> scene_kdtree;
				scene_kdtree.setInputCloud(current_scenecloud);

				for (int j = 0; j < modelPointClouds.size(); j++)
				{
					float temp_match_fitness, overlapping_ratio;

					bool need_judge_reflect = false;
					if (j ==2 || j==3) //axis symmetric roadmarkings
						need_judge_reflect = true;

					pcXYZIPtr modelPointClouds_tran(new pcXYZI);
					Eigen::Matrix4f tran_mat_m2s_temp;

					temp_match_fitness = icp_reg_4dof_global(modelPointClouds[j], current_scenecloud, need_judge_reflect,
						tran_mat_m2s_temp, heading_increment, iter_num, corr_dist_thre);

					pcl::transformPointCloud(*modelPointClouds[j], *modelPointClouds_tran, tran_mat_m2s_temp);

					overlapping_ratio = cal_overlap_ratio(modelPointClouds_tran, scene_kdtree.makeShared(), overlapping_dist_thre);
                    
					if(overlapping_ratio > tolerantMinOverlap_-0.05)
					{
						//check overlap ratio calculated w.r.t. the scene point cloud->points
						pcl::KdTreeFLANN<pcl::PointXYZI> model_kdtree;
				        model_kdtree.setInputCloud(modelPointClouds_tran);
						overlapping_ratio = 0.5 * (overlapping_ratio + cal_overlap_ratio(current_scenecloud, model_kdtree.makeShared(), overlapping_dist_thre)); //average overlapping_ratio
					}

					//balanced weight
					if (j == 0) // rectangle 
						overlapping_ratio -= 0.08;
					else if (j == 1)  // forward arrow
						overlapping_ratio += 0.03;
					else if (j == 2)    //right-forward arrow
					    overlapping_ratio -= 0.01;
					else if (j == 6)    //dimond
					    overlapping_ratio -= 0.1;
					
					if (overlapping_ratio > best_overlapping_ratio && temp_match_fitness < correct_match_fitness_thre)
					{
						best_overlapping_ratio = overlapping_ratio;
						match_fitness_best = temp_match_fitness;
						tran_mat_m2s_best_match = tran_mat_m2s_temp;
						best_model_index = j;
					}
				}

				if (best_model_index >= 0)
				{
					roadmarkings[i].category = best_model_index + 2;
					roadmarkings[i].localization_tranmat_m2s = tran_mat_m2s_best_match;
					cout << "Object [" << i << "] ---> Category [" << roadmarkings[i].category <<
						"], overlap: [" << best_overlapping_ratio * 100.0 << "% ], score: [" << match_fitness_best << "]" << endl;
				}
			}
		}

		return true;
	}

	/**
	* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
	* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
	* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
	* \param[out] thre_dis : It acts as the search radius of overlapping estimation
	* \return : The estimated overlap ratio [from 0 to 1]
	*/
	float ModelMatch::cal_overlap_ratio(const pcXYZIPtr & search_cloud,
		pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree, float thre_dis)
	{
		int overlap_point_num = 0;
		float overlap_ratio;

		/*pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
		kdtree.setInputCloud(ModelCloud);*/

		std::vector<int> search_indices; //point index Vector
		std::vector<float> distances_square;	//distance Vector

		//cout << "calculate nn\n";

		int down_rate = 3;

		for (int i = 0; i < search_cloud->points.size(); i++) {
			if (i % down_rate == 0)
			{
				tree->nearestKSearch(search_cloud->points[i], 1, search_indices, distances_square); //1nn
				if (distances_square[0] < thre_dis* thre_dis)
					overlap_point_num++;
				std::vector<int>().swap(search_indices);
				std::vector<float>().swap(distances_square);
			}
		}
		overlap_ratio = (0.001 + overlap_point_num) / (search_cloud->points.size() / down_rate + down_rate - 1);

		//cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;
		//LOG(INFO) << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

		return overlap_ratio;
	}


	/**
	* \brief Point-to-Point metric ICP
	* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
	* \param[in]  TargetCloud : A pointer of the Target Point Cloud
	* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
	* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
	* \param[in]  max_iter : A parameter controls the max iteration number for the registration
	* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
	* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
	* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
	*/

	float ModelMatch::icp_reg(const pcXYZIPtr & SourceCloud, const pcXYZIPtr &TargetCloud,
		Eigen::Matrix4f & initial_guess, Eigen::Matrix4f & transformationS2T,
		int max_iter, float thre_dis)
	{
		clock_t t0, t1;
		t0 = clock();

		pcXYZIPtr tranSource(new pcXYZI);
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

		icp.setInputSource(SourceCloud);
		icp.setInputTarget(TargetCloud);

		icp.setMaxCorrespondenceDistance(thre_dis);

		// Converge criterion ( 'Or' Relation ) 
		// Set the maximum number of iterations [ n>x ] (criterion 1)
		icp.setMaximumIterations(max_iter);     //Most likely to happen
		// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
		icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
		// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
		icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen

		icp.align(*tranSource, initial_guess);  //Use closed-form SVD to estimate transformation for each iteration [You can switch to L-M Optimization]
		transformationS2T = icp.getFinalTransformation().cast<float>();

		t1 = clock();

		float fitness_score = icp.getFitnessScore();

		// Commented these out if you don't want to output the registration log
		/*LOG(INFO) << "Point-to-Point ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
		LOG(INFO) << transformationS2T;
		LOG(INFO) << "The fitness score of this registration is " << icp.getFitnessScore();
		if (icp.getFitnessScore() > 5000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
		LOG(INFO) << "-----------------------------------------------------------------------------";*/

		//
		/*cout << "Point-to-Point ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
		cout << "The fitness score of this registration is " << icp.getFitnessScore() << endl;
		cout << "-----------------------------------------------------------------------------" << endl;*/
		return fitness_score;
	}

	float ModelMatch::icp_reg_4dof_global(const pcXYZIPtr & ModelCloud, const pcXYZIPtr & SceneCloud, bool with_reflection,
		Eigen::Matrix4f & tran_mat_m2s_best, float heading_step_d, int max_iter_num, float dis_thre)
	{
		// translate the centers of point cloud to be together
		StructOperator so;
		CenterPoint center_model;
		CenterPoint center_reflect_model;
		CenterPoint center_scene;

		Eigen::Matrix4f reflect_mat;
		reflect_mat.setIdentity();
		reflect_mat(0, 0) = -1.0;
		pcXYZIPtr reflect_model(new pcXYZI);

		so.getCloudCenterPoint(*ModelCloud, center_model);
		so.getCloudCenterPoint(*SceneCloud, center_scene);

		float current_best_fitness = FLT_MAX;
		float current_best_heading_d;

		float temp_fitness;

		float heading_d = 0.0;
		float heading_rad;

		Eigen::Matrix4f temp_rot_z_mat;
		Eigen::Matrix4f tran_mat_m2s;

		bool successful_reg = false;

		float t_x, t_y, t_z;
		t_x = center_scene.x - center_model.x;
		t_y = center_scene.y - center_model.y;
		t_z = center_scene.z - center_model.z;

		while (heading_d < 360.0)
		{
			heading_rad = heading_d * M_PI / 180.0;

			tran_mat_m2s.setIdentity();
			temp_rot_z_mat.setIdentity();
			temp_rot_z_mat(0, 0) = cos(heading_rad);
			temp_rot_z_mat(0, 1) = sin(heading_rad);
			temp_rot_z_mat(1, 0) = -sin(heading_rad);
			temp_rot_z_mat(1, 1) = cos(heading_rad);
			temp_rot_z_mat(0, 3) = t_x;
			temp_rot_z_mat(1, 3) = t_y;
			temp_rot_z_mat(2, 3) = t_z;

			temp_fitness = icp_reg(ModelCloud, SceneCloud, temp_rot_z_mat, tran_mat_m2s,
				max_iter_num, dis_thre);

			if (temp_fitness < current_best_fitness)
			{
				current_best_fitness = temp_fitness;
				current_best_heading_d = heading_d;
				tran_mat_m2s_best = tran_mat_m2s;
			}
			heading_d += heading_step_d;
		}

		//reflect the model ( ---->   |   <---- )
		//deal with the issue of truning left or right
		if (with_reflection)
		{
			pcl::transformPointCloud(*ModelCloud, *reflect_model, reflect_mat);
			so.getCloudCenterPoint(*reflect_model, center_reflect_model);

			t_x = center_scene.x - center_reflect_model.x;
			t_y = center_scene.y - center_reflect_model.y;
			t_z = center_scene.z - center_reflect_model.z;

			heading_d = 0.0;

			while (heading_d < 360.0)
			{
				heading_rad = heading_d * M_PI / 180.0;

				tran_mat_m2s.setIdentity();
				temp_rot_z_mat.setIdentity();
				temp_rot_z_mat(0, 0) = cos(heading_rad);
				temp_rot_z_mat(0, 1) = sin(heading_rad);
				temp_rot_z_mat(1, 0) = -sin(heading_rad);
				temp_rot_z_mat(1, 1) = cos(heading_rad);
				temp_rot_z_mat(0, 3) = t_x;
				temp_rot_z_mat(1, 3) = t_y;
				temp_rot_z_mat(2, 3) = t_z;

				temp_fitness = icp_reg(reflect_model, SceneCloud, temp_rot_z_mat, tran_mat_m2s,
					max_iter_num, dis_thre);

				if (temp_fitness < current_best_fitness)
				{
					current_best_fitness = temp_fitness;
					current_best_heading_d = heading_d;
					tran_mat_m2s_best = tran_mat_m2s * reflect_mat;
				}
				heading_d += heading_step_d;
			}
		}
		return current_best_fitness;
	}
}
