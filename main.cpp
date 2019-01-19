#include "utility.h"
#include "dataIo.h"
#include "voxelFilter.h"
#include "ground_extraction.h"
#include "pointcloudprocess.h"
#include "imageprocess.h"
#include "modelmatch.h"

#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <array>
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace utility;
using namespace groundExtraction;
using namespace PCprocess;
using namespace image;
using namespace std;

int main()
{
	/*-------------------------------------------------Readme----------------------------------------------------*/
	//C++ Program [Road Markings Extraction]
	//Compile with VS2013 x64 Release or Debug
	//Dependent 3rd Party Libs:  PCL 1.8, OpenCV 2 , LibLas, VTK, LibBoost, DXFLib
	//Point Cloud Acquisition Platform: MLS, TLS, ALS (Developing)
	//Application Scenarios: Highway, City Road
	//Author：Yue Pan et al. @ WHU

	//Consuming time : For MLS point cloud with about 10 million points
	//                 20 seconds with ground segmentation (recommended) 
	//                  8 seconds without ground segmentation(some road markings may not be detected)
	/*-----------------------------------------------------------------------------------------------------------*/
	
	/*-------------------------------------------------Welcome---------------------------------------------------*/
	cout << "Road Markings Extraction Program" << endl;
	cout << "By Yue Pan et al. @ WHU" << endl;
	cout << "-------------------------------------------------------------" << endl;
	/*-----------------------------------------------------------------------------------------------------------*/
	
	/*-----------------------------------------------Declaration-------------------------------------------------*/
	clock_t t1, t2, t3, t4, t5;       // Timing
	//Step 1
	DataIo io;                        // Data IO class
	string filename, paralistfile;    // Point cloud file name ; Parameter list file name
	int datatype, roadtype, is_road_extracted, density;  // datatype: (1.MLS/TLS 2.ALS) ;  roadtype: (1.Highway 2.City Road) ; density: approximate point number per m^2
	float resolution;                 // Projection Image's pixel size (m) 
	pcXYZIPtr cloud(new pcXYZI());    // Original point cloud
	vector<double> CloudBoundingBox;  // Xmin,Ymin,Zmin,Xmax,Ymax,Zmax after translation for Input point cloud;
	double X_origin, Y_origin;        // X,Y origin before translation;
	//Step 2
	pcXYZIPtr fcloud(new pcXYZI());   // Sampled point cloud
	//Step 3
	Csegmentation seg;                // Cloud Processing class 
	pcXYZIPtr ngcloud(new pcXYZI());  // Non-ground point cloud
	pcXYZIPtr gcloud(new pcXYZI());   // Ground point cloud
	//Step 4
	Imageprocess ip;                  // Image Processing class
	Mat imgI, imgZ, imgD;             // imgI:Intensity Projection Image; imgZ: Elevation Projection Image ; imgD: Density Projection Image
	//Step 5
	Mat imgImf, imgIgradient, imgIgradientroad, imgIbinary, imgZgradient, imgZbinary, imgDbinary, imgIbfilter, labelImg, colorLabelImg, imgFilled, Timg, dilateImg, closeImg, corner, cornerwithimg;   // imgImf: Intensity Projection Image after Median Filter ; imgIgradient: Intensity Gradient Image ; imgIgradientroad: Road's Intensity Gradient Image ; imgIbinary: Road's Intensity Binary Image ; imgZgradient: Slope (dZ) projection image ; imgZbinary: Slope binary image ; imgDbinary: Point density binary image ; imgIbfilter: Intensity Binary Image after filtering (Based on Connected Region's area) ; labelImg: Labeled Image based on Connected Conponent Analysis (CCA) ; colorLabelImg :  Labeled Image based on Connected Conponent Analysis (CCA) render by random colors ; imgFilled : Filled Image (the closed geometry are all filled) ; Timg: Truncated Image of Interested area (For Saving computing time) ; dilateImg : Image after dilation morphology calculation ; closeImg : Image after close morphology calculation ; corner : Image corner pixels ; cornerwithimg: Image with its corner pixels
	//Step 6
	pcXYZIPtr outcloud(new pcXYZI()); //Road Marking Cloud (All in one)
	vector <pcXYZI> outclouds, outcloud_otsu_sor, outcloud_otsu_sor_n, boundaryclouds, cornerclouds;  //Road Marking Clouds (Segmentation); Filter: Otsu+SOR ; Filter: Point Number ; Boundary Clouds ; Corner Clouds
	//Step 7
	vector <RoadMarking> roadmarkings, sideline_roadmarkings;  // Unit of Road markings (Category + Polyline)
	vector<vector<pcl::PointXYZI>> boundingdatas, modeldatas;  // Unit of Road markings' bounding box and model datas
	vector<BoundingFeature> boundingfeatures;                  // Unit of Road markings' bounding box feature    
	vector<bool> is_rights;                                    // Unit of Arrow Road markings' direction   
	//Step 8

	/*-----------------------------------------------------------------------------------------------------------*/

	/*--------------------------------------------Key Parameters-------------------------------------------------*/
	// Default value
	// float intensity_scale = 8;
	// float keypoint_nonmax_radius = 0.2;
	// float geometric_consistency_tolerant = 0.3;
	// float search_radius = 0.4;
	// float tolerantMinOverlap_ratio = 0.7;
	// float sideline_vector_distance = 3.5;                   
	// For parameter details, please refer to the Read-me file
	/*-----------------------------------------------------------------------------------------------------------*/

	/*-------------------------------------------------Execution-------------------------------------------------*/
	//Step 1. Data Import
	cout << "Input or the filename (las)" << endl << "Or you can simply drag the file here"<<endl;  
	cin >> filename;
	
	cout << "Input parameter list file" << endl << "Or you can just enter any character to use the default value"<<endl;
	cin >> paralistfile;
	io.readParalist(paralistfile);

	cout << "Input data type: 1. MLS or TLS, 2. ALS , Please Enter 1 or 2" << endl;
	cin >> datatype;
	
	cout << "Input road type: 1. Highway, 2. City road , Please Enter 1 or 2" << endl;
	cin >> roadtype;

	cout << "Has the road already been extracted: 1. No, 2. Yes , Please Enter 1 or 2" << endl;
	cin >> is_road_extracted;

	cout << "Input the approximate point density (number per m^2)" << endl;
	if (datatype == 1) cout << "For MLS, the value is usually 400 - 1500" << endl;
	if (datatype == 2) cout << "For ALS, the value is usually 10 - 100" << endl;
	cin >> density;

	if (datatype == 1) resolution = 2.5 * 1 / (sqrt(density));
	if (datatype == 2) resolution = 1.5 * 1 / (sqrt(density));
	cout << "Approximate resolution is " << setprecision(3) <<resolution<< " m ." << endl;

	io.displayparameter(datatype, roadtype, is_road_extracted);
	
	t1 = clock();
	
	io.readLasFile(filename, *cloud, CloudBoundingBox,X_origin,Y_origin);  // To import the field of intensity, you need to use las file.
	cout << "the number of raw point clouds: " << cloud->size() << endl;

	
	//Step 2. Voxel Down-sampling (Optional)
	/*float voxelSize = 0.02;
	VoxelFilter vfilter(voxelSize);
	fcloud = vfilter.filter(cloud);
	io.writePcdFile("filteredcloud.pcd", fcloud);
	cout << "the number of filtered point clouds: " << fcloud->size() << endl;*/
	
	if (is_road_extracted == 1) // The road hasn't been extracted. We need to use ground segmentation and multi-condition filtering to extract the road
	{
			//Step 3. Ground Filter and projection (Optional)
			//The Ground Segmentation can help you find the road more precisely. However, this process would take more time.
			// Method 1: Ronggang Filter (Fast)
			// Parameter Setting: grid_resolution_ = 0.5, min_pt_num_in_grid_ = 30, max_height_difference_ = 0.2
			Ground_Extraction ground;
			StructOperator so;
			Bounds bounds;
			CenterPoint center;
			so.getBoundAndCenter(*cloud, bounds, center);
			ground.ExtractGroundPoint(cloud, gcloud, ngcloud, bounds, center);

			// Method 2: PMF (Slow)
			//seg.GroundFilter_PMF(cloud, gcloud, ngcloud);

			t2 = clock();

			//Preprocessing: Segmentation and Fitting (Optional)

			/*Csegmentation seg;
			//RANSAC plane segmentation
			pcXYZIPtr fitcloud(new pcXYZI());
			float td = 1;
			fitcloud = seg.planesegRansac(gcloud, td);
			io.writePcdFile("ransac ground cloud.pcd", fitcloud);

			//Projection
			pcXYZIPtr pgcloud(new pcXYZI());
			pgcloud=seg.groundprojection(gcloud);
			io.writePcdFile("projected groundcloud.pcd", pgcloud);*/

			//Step 4. 3D->2D projection, Generating Projection Image
			Imageprocess ip;
			ip.savepcgrid(CloudBoundingBox, resolution, cloud, gcloud, ngcloud); //get image size and save point indices in each pixel
			//For better efficiency (without ground segmentation), you can replace gcloud and ngcloud with cloud
			//[0:Original Cloud, 1 : Ground Cloud, 2 : Non - ground Cloud]
			ip.pc2imgI(gcloud, 1, imgI, io.paralist.intensity_scale);
			ip.pc2imgZ(ngcloud, 2, imgZ);
			if (datatype == 1){ // For MLS
				float expectedmaxnum;
				expectedmaxnum = 1.0 * density * resolution * resolution;  // expectedmaxnum: expected max point number in a pixel 
				ip.pc2imgD(gcloud, 1, imgD, expectedmaxnum);
			}

			//Step 5. Image Processing 
			//5.1.1 Median filter  (Optional)
			medianBlur(imgI, imgImf, 3); // Remove the salt and pepper noise

			//5.1.2 Sobel gradient calculation and boundary extraction
			imgIgradient = ip.Sobelboundary(imgImf);
			imgZgradient = ip.Sobelboundary(imgZ);

			//5.2.1. Image Thresholding using Max Entropy Segmentation (All self-adaptive)
			imgZbinary = ip.maxEntropySegMentation(imgZgradient);
			//int maxroadslope = 50;  // Setting the threshold for Surface Roughness, 50/255 here
			//threshold(imgZgradient, imgZbinary, maxroadslope, 1, CV_THRESH_BINARY);
			if (datatype == 1) {
				imgDbinary = ip.maxEntropySegMentation(imgD);  // for MLS
				imgIgradientroad = ip.ExtractRoadPixelIZD(imgIgradient, imgZbinary, imgDbinary); //Use the slope and point density as the criterion
			}
			if (datatype == 2) imgIgradientroad = ip.ExtractRoadPixelIZ(imgIgradient, imgZbinary); //for ALS
			imgIbinary = ip.maxEntropySegMentation(imgIgradientroad);
			//However, Max Entropy Method is not efficient enough

			//5.3. Intensity Image Connected Component Analysis (CCA) and Labeling
			float smallregion;
			if (datatype == 1)  smallregion = 0.3 / (resolution*resolution);    //Pixel Number Threshold: 0.6 m^2 as the threshold for small connected region. Manually tune parameter (0.5) here.
			if (datatype == 2)  smallregion = 0.8 / (resolution*resolution);
			ip.RemoveSmallRegion(imgIbinary, imgIbfilter, smallregion); //CCA Filtering (Problem: It's inefficient to run CCA twice. Improve the codes latter.)
			ip.Truncate(imgIbfilter, Timg);
			ip.ImgFilling(Timg, imgFilled);  //Filling the holes inside the markings (Optional)

			//5.3.2. Morphological Operations: Dilation and Closing (Optional)
			/*Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
			morphologyEx(imgIbfilter, closeImg, MORPH_CLOSE, element);
			//dilate(imgIbfilter, dilateImg, Mat(),element);
			//erode(dilateImg, closeImg, Mat());  // ->closeImg*/

			ip.CcaBySeedFill(imgFilled, labelImg);    //CCA: Seed filling method with 8 neighbor   
			//ip.CcaByTwoPass(imgFilled, labelImg);   //CCA: Two pass method with 4 neighbor (Alternative)
			ip.LabelColor(labelImg, colorLabelImg);   //CCA Labeling and Truncation

			//5.4 Harris / Shi-Tomasi Corner Detection (Optional)
			/*//ip.DetectCornerHarris(imgIbfilter, colorLabelImg, corner, cornerwithimg,150);  //Using Harris Detector
			float minvertexdis = 2.0 / resolution;  //threshold 2 meter
			double mincornerscore = 0.05; //0.01 original
			ip.DetectCornerShiTomasi(imgFilled, colorLabelImg, cornerwithimg, minvertexdis, mincornerscore);  //Using Shi-Tomasi Detector*/

			t3 = clock();

			//Step 6. 2D->3D back to point cloud
			//ip.img2pc_g(colorLabelImg, gcloud, outcloud);                  //Ground Road Marking Points (All in one)
			ip.img2pclabel_g(labelImg, gcloud, outclouds, resolution / 3);   //Ground Road Marking Points (Segmentation) //Elevation filter: the last parameter is set as the dZ threshold for single pixel

			//Use Otsu (Intensity) Method and Statistics Outlier Remover to filter the point cloud
			seg.cloudFilter(outclouds, outcloud_otsu_sor, 256, 15, 1.5); // Three parameters: the first is for the histogram level of Otsu Thresholding , the second is for SOR neighbor number and the third is for SOR std threshold

			//Delete point clouds whose point number is less than a threshold 
			if (datatype == 1) seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, density / 5);  //the last parameter is set as the point number threshold for each cloud
			if (datatype == 2) seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, 10);

			t4 = clock();
	}

	else if (is_road_extracted == 2)  // The road has already been extracted. There's no need to do any ground segmentation
	{
			//Step 3. Ground Filter and projection (Optional)
			t2 = clock();

			//Step 4. 3D->2D projection, Generating Projection Image
			ip.savepcgrid(CloudBoundingBox, resolution, cloud); //get image size and save point indices in each pixel
			//[0:Original Cloud, 1 : Ground Cloud, 2 : Non - ground Cloud]
			ip.pc2imgI(cloud, 0, imgI, io.paralist.intensity_scale);

			//Step 5. Image Processing 
			//5.1.1 Median filter  (Optional)
			medianBlur(imgI, imgImf, 3); // Remove the salt and pepper noise

			//5.1.2 Sobel gradient calculation and boundary extraction
			imgIgradient = ip.Sobelboundary(imgImf);

			//5.2.1. Image Thresholding using Max Entropy Segmentation (All self-adaptive)
			imgIbinary = ip.maxEntropySegMentation(imgIgradient);
			//imgIbinary =
			//5.3. Intensity Image Connected Component Analysis (CCA) and Labeling
			float smallregion;
			if (datatype == 1)  smallregion = 0.3 / (resolution*resolution);    //Pixel Number Threshold: 0.25 m^2 as the threshold for small connected region. Manually tune parameter (0.5) here.
			if (datatype == 2)  smallregion = 0.8 / (resolution*resolution);
			ip.RemoveSmallRegion(imgIbinary, imgIbfilter, smallregion); //CCA Filtering (Problem: It's inefficient to run CCA twice. Improve the codes latter.)
			ip.Truncate(imgIbfilter, Timg);
			//ip.ImgFilling(Timg, imgFilled);  //Filling the holes inside the markings (Optional)

			//5.3.2. Morphological Operations: Dilation and Closing (Optional)
			/*Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
			morphologyEx(imgIbfilter, closeImg, MORPH_CLOSE, element);
			//dilate(imgIbfilter, dilateImg, Mat(),element);
			//erode(dilateImg, closeImg, Mat());  // ->closeImg*/

			ip.CcaBySeedFill(Timg, labelImg);    //CCA: Seed filling method with 8 neighbor   
			//ip.CcaByTwoPass(imgFilled, labelImg);   //CCA: Two pass method with 4 neighbor (Alternative)
			ip.LabelColor(labelImg, colorLabelImg);   //CCA Labeling and Truncation

			//5.4 Harris / Shi-Tomasi Corner Detection (Optional)
			/*//ip.DetectCornerHarris(imgIbfilter, colorLabelImg, corner, cornerwithimg,150);  //Using Harris Detector
			float minvertexdis = 2.0 / resolution;  //threshold 2 meter
			double mincornerscore = 0.05; //0.01 original
			ip.DetectCornerShiTomasi(imgFilled, colorLabelImg, cornerwithimg, minvertexdis, mincornerscore);  //Using Shi-Tomasi Detector*/

			t3 = clock();

			//Step 6. 2D->3D back to point cloud
			ip.img2pclabel_c(labelImg, cloud, outclouds, resolution / 3);   //Ground Road Marking Points (Segmentation) //Elevation filter: the last parameter is set as the dZ threshold for single pixel
			
			//Use Otsu (Intensity) Method and Statistics Outlier Remover to filter the point cloud
			seg.cloudFilter(outclouds, outcloud_otsu_sor, 256, 15, 1.5); // Three parameters: the first is for the histogram level of Otsu Thresholding , the second is for SOR neighbor number and the third is for SOR std threshold

			//Delete point clouds whose point number is less than a threshold 
			if (datatype == 1) seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, density / 5);  //the last parameter is set as the point number threshold for each cloud
			if (datatype == 2) seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, 10);

			t4 = clock();
	}
		
	/*-------------------------------------------------Highway-------------------------------------------------*/
	if (roadtype == 1) 
	{
			//Step 7. Object Recognition and Classification based on Model Matching and Geometric Information

			//7.1 Boundary and Corner Extraction (Optional) 
			// Boundary Extraction: Alpha-Shape Concave Hull Generation
			seg.BoundaryExtraction(outcloud_otsu_sor_n, boundaryclouds);
			// Corner Extraction: Neighborhood Processing
			// seg.CornerExtraction(boundaryclouds,cornerclouds,1,8,0.1,0.02,0.95); // Parameters: 1/0 Use Radius Search or KNN, 8, KNN's K, 0.15 search radius , 0.02 distance threshold, 0.94 maxcos 

			//7.2 Road Markings Rough Classification based on Bounding Box Feature
			seg.BoundingInformation(outcloud_otsu_sor_n, boundingdatas);
			seg.BoundingFeatureCalculation(boundingdatas, boundingfeatures);
			seg.CategoryJudgementBox_highway(boundingfeatures, roadmarkings);

			//7.3 Road Markings (Arrow) Classification based on Model Matching (Relatively Slow, You may try other faster 2D feature and matching strategy)
			ModelMatch mm = ModelMatch(io.paralist.keypoint_nonmax_radius, io.paralist.geometric_consistency_tolerant, 0.15, io.paralist.search_radius, io.paralist.tolerantMinOverlap_ratio);  // Parameter List: KeypointDetection's RadiusNonMax,tolerantEuclideanDis,tolerantHammingDis, Overlapping Ratio Calculation's Search Radius, tolerantMinOverlap ratio;
			mm.model_match(boundaryclouds, roadmarkings, modeldatas, is_rights, roadtype);

			//Step 8. Vectorization
			seg.MarkingVectorization_highway(outcloud_otsu_sor_n, boundingdatas, modeldatas, roadmarkings,is_rights,io.paralist.sideline_vector_distance, 0.2); // Vectorization // the parameters are set as the linear sample distance and ambiguous ratio for long side line vectorization
			seg.CombineSideLines(roadmarkings, io.paralist.sideline_vector_distance, sideline_roadmarkings); //Side lines Combination

			t5 = clock();
	}

	/*------------------------------------------------City Road-------------------------------------------------*/
	else if (roadtype == 2)
	{
			//Step 7. Object Recognition and Classification based on Model Matching and Geometric Information

			//7.1 Boundary and Corner Extraction (Optional) 
			// Boundary Extraction: Alpha-Shape Concave Hull Generation
			seg.BoundaryExtraction(outcloud_otsu_sor_n, boundaryclouds);
			// Corner Extraction: Neighborhood Processing
			// seg.CornerExtraction(boundaryclouds,cornerclouds,1,8,0.1,0.02,0.95); // Parameters: 1/0 Use Radius Search or KNN, 8, KNN's K, 0.15 search radius , 0.02 distance threshold, 0.94 maxcos 

			//7.2 Road Markings Rough Classification based on Bounding Box Feature
			seg.BoundingInformation(outcloud_otsu_sor_n, boundingdatas);
			seg.BoundingFeatureCalculation(boundingdatas, boundingfeatures);
			seg.CategoryJudgementBox_cityroad(boundingfeatures, roadmarkings);

			//7.3 Road Markings (Arrow) Classification based on Model Matching (Relatively Slow, You may try other faster 2D feature and matching strategy)
			ModelMatch mm = ModelMatch(io.paralist.keypoint_nonmax_radius, io.paralist.geometric_consistency_tolerant, 0.15, io.paralist.search_radius, io.paralist.tolerantMinOverlap_ratio);  // Parameter List: KeypointDetection's RadiusNonMax, tolerantEuclideanDis, tolerantHammingDis, Overlapping Ratio Calculation's Search Radius, tolerantMinOverlap ratio;
			mm.model_match(boundaryclouds, roadmarkings, modeldatas,is_rights, roadtype);

			//Step 8. Vectorization
			seg.MarkingVectorization_cityroad(outcloud_otsu_sor_n, boundingdatas, modeldatas, roadmarkings,is_rights, io.paralist.sideline_vector_distance, 0.15); // Vectorization // the parameters are set as the linear sample distance and ambiguous ratio for long side line vectorization
			seg.CombineSideLines(roadmarkings, io.paralist.sideline_vector_distance, sideline_roadmarkings); //Side lines Combination 

			t5 = clock();
	}
	
	/*------------------------------------------------Output----------------------------------------------------*/
	//Step 9. Output Result
	//Output Timing Statistics
	cout << "Time for Data Import and Ground Segmentation: " << (t2 - t1) *1.0 / 1000 << " s" << endl
         << "Time for Image Processing: " << (t3 - t2) *1.0 / 1000 << " s" << endl
		 << "Time for Cloud Processing: " << (t4 - t3) *1.0 / 1000 << " s" << endl
		 << "Time for Object Recognition: " << (t5 - t4) *1.0 / 1000 << " s" << endl
		 << "Total Consuming Time: " << (t5 - t1) *1.0 / 1000 << " s" << endl;

	//Output Images
	//ip.saveimg(imgI, imgZ, imgD, imgImf, imgIgradient, imgZgradient, imgZbinary, imgDbinary, imgIgradientroad, imgIbinary, imgFilled, colorLabelImg, cornerwithimg);//Saving the Images
	//ip.saveimg(imgI, imgZ, imgD, imgImf, imgIgradient, imgZgradient, imgZbinary, imgDbinary, imgIgradientroad, imgIbinary, imgFilled, colorLabelImg);
	ip.saveimg(imgI, imgImf, imgIgradient, imgIbinary, Timg, colorLabelImg);


	//Output Point Clouds
	//io.writePcdFile("groundcloud.pcd", gcloud);
	//io.writePcdFile("ungroundcloud.pcd", ngcloud);
	//io.writePcdFile("All Road Markings.pcd", outcloud);
	//io.writePcdAll("Labeled Clouds", "Labeled Cloud.pcd", outclouds);
	//io.writePcdAll("Filtered Labeled Clouds", "Filtered Labeled Cloud.pcd", outcloud_otsu_sor_n);
	//io.writePcdAll("Boundary Clouds", "Boundary Cloud.pcd", boundaryclouds);
	//io.writePcdAll("Classified Road Markings with Road Cloud", "Classified Road Markings.pcd", ngcloud, gcloud, outcloud_otsu_sor_n, roadmarkings);
	io.writePcdAll("Classified Road Markings", "Classified Road Markings.pcd", outcloud_otsu_sor_n, roadmarkings);
	io.writemarkVectDXF(roadmarkings, sideline_roadmarkings, X_origin, Y_origin);
	//io.writePcdAll("Corner Clouds", "Corner Cloud.pcd", cornerclouds);
	
	//Display Results
	if (is_road_extracted==1) io.displayroad(ngcloud, gcloud);                    //Display non-ground and ground cloud
	io.displaymark(outcloud_otsu_sor_n);                                          //Display road markings point clouds
	//io.displaymarkwithng(outcloud_otsu_sor_n, ngcloud);                         //Display road markings point clouds with non-ground cloud
	io.displaymarkbycategory(outcloud_otsu_sor_n, roadmarkings);                  //Display road markings point clouds rendered by category
	io.displaymarkVect(roadmarkings,sideline_roadmarkings);                       //Display vectorized road markings
	
	waitKey(0);
	return 1;

	/*-----------------------------------------------------------------------------------------------------------*/


	//任务表;
	//提供轨迹文件处理接口，有轨迹的话找路面点就容易多了;
	//模板制作;
	//提供las 输出，ply输入等;
	//文件夹批处理;

}