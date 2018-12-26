#include "utility.h"
#include "dataIo.h"
#include "voxelFilter.h"
#include "ground_extraction.h"
#include "segmentation.h"
#include "imageprocess.h"
//#include "binaryFeatureExtraction.h"
//#include "kMeansCluster.h"
//#include "bagOfWords.h"
//#include "imageprocess.h"
//#include "GibbsLDAEstimate.h"
//#include "topicAnalysis.h"
//#include "multipleSeg.h"
//#include "euclideanCluster.h"
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <array>
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace utility;
using namespace groundExtraction;
using namespace segmentation;
using namespace image;
using namespace cv;
//using namespace euclideanCluster;
//using namespace visualWord;
//using namespace kMeans;
//using namespace bagWords;
//using namespace gibbsEstimate;
//using namespace multiseg;


int main()
{
	//C++ Program [Road Markings Extraction]
	//Compile with VS2013 x64 Release or Debug
	//Dependent 3rd Party Libs:  PCL 1.8, OpenCV 2 , LibLas
	//Application Scenarios: MLS£¬ALS
	//Author£ºYue Pan et al. @ WHU

	//Consuming time : For MLS point cloud with about 10 million points
	//                 20 seconds with ground segmentation (recommended) 
	//                  8 seconds without ground segmentation(some road markings may not be detected)

	cout << "Road Markings Extraction Program" << endl;
	cout << "By Yue Pan et al. @ WHU" << endl;
	cout << "---------------------------------------------------" << endl;

	clock_t t1, t2, t3, t4;  //Timing
	
	//Step 1. Data Import
	cout << "Input the filename (las)" << endl;  
	string filename;
	cin >> filename;
	cout << "Input the resolution of projection img (m)" << endl
		 << "For MLS, the recommended value is 0.08 - 0.1" << endl; 
	float resolution;
	cin >> resolution;
	pcXYZIPtr cloud(new pcXYZI());
	vector<double> CloudBoundingBox; //Xmin,Ymin,Zmin,Xmax,Ymax,Zmax
	DataIo io;
	t1 = clock();
	io.readLasFile(filename, *cloud, CloudBoundingBox);  // To import the field of intensity, you need to use las file.
	cout << "the number of raw point clouds: " << cloud->size() << endl;

	//Step 2. Voxel Down-sampling (Optional)
	/*float voxelSize = 0.02;
	VoxelFilter vfilter(voxelSize);
	pcXYZIPtr fcloud(new pcXYZI());
	fcloud = vfilter.filter(cloud);
	io.writePcdFile("filteredcloud.pcd", fcloud);
	cout << "the number of filtered point clouds: " << fcloud->size() << endl;*/

	//Step 3. Ground Filter and projection (Optional)
	//The Ground Segmentation can help you find the road more precisely. However, this process would take more time.
	//Parameter Setting: grid_resolution_ = 0.5, min_pt_num_in_grid_ = 40, max_height_difference_ = 0.2
	pcXYZIPtr ngcloud(new pcXYZI());
	pcXYZIPtr gcloud(new pcXYZI());
	CGroundExtraction ground;
	StructOperator so;
	Bounds bounds;
	CenterPoint center;
	so.getBoundAndCenter(*cloud, bounds, center);
	ground.ExtractGroundPoint(cloud, gcloud, ngcloud, bounds, center);
	t2 = clock();
	//io.writePcdFile("groundcloud.pcd", gcloud);
	//io.writePcdFile("ungroundcloud.pcd", ngcloud);
	//io.displayroad(ngcloud, gcloud);
	
	//Preprocessing: Segmentation and Fitting (Optional)
	Csegmentation seg;
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
	Mat imgI, imgZ, imgD; //Intensity Projection Image, Elevation Projection Image and Density Projection Image
	ip.savepcgrid(CloudBoundingBox,resolution,cloud ,gcloud ,ngcloud); //get image size and save point indices in each pixel
	//For better efficiency (without ground segmentation), you can replace gcloud and ngcloud with cloud
	//[0:Original Cloud, 1 : Ground Cloud, 2 : Non - ground Cloud]
	ip.pc2imgI(gcloud, 1, imgI);  
	ip.pc2imgZ(ngcloud, 2, imgZ);
	ip.pc2imgD(gcloud, 1, imgD);
	
	//Step 5. Image Processing 
	//5.1.1 Median filter 
	Mat imgImf;
	medianBlur(imgI, imgImf, 3); // Remove the salt and pepper noise

	//5.1.2 Sobel gradient calculation and boundary extraction
	Mat imgIgradient, imgIgradientroad, imgIbinary, imgZgradient, imgZbinary, imgDbinary;
	imgIgradient = ip.Sobelboundary(imgImf);
	imgZgradient = ip.Sobelboundary(imgZ);

	//5.2.1. Image Thresholding using Max Entropy Segmentation (All self-adaptive)
	imgZbinary = ip.maxEntropySegMentation(imgZgradient);
	//int maxroadslope = 50;  // Setting the threshold for Surface Roughness, 50/255 here
	//threshold(imgZgradient, imgZbinary, maxroadslope, 1, CV_THRESH_BINARY);
	imgDbinary = ip.maxEntropySegMentation(imgD);
	imgIgradientroad = ip.ExtractRoadPixel(imgIgradient, imgZbinary, imgDbinary); //Use the slope and point density as the criterion	
	imgIbinary = ip.maxEntropySegMentation(imgIgradientroad);
	//However, max entropy is not efficient enough

	//5.3. Intensity Image Connected Component Analysis (CCA) and Labeling
	Mat imgIbfilter, labelImg,colorLabelImg, imgFilled,Timg;
	float smallregion = 0.3 / (resolution*resolution);    //0.6 m^2 as the threshold for small connected region. Manually tune parameter (1.0) here.
	ip.RemoveSmallRegion(imgIbinary, imgIbfilter, smallregion); //CCA Filtering (Problem: It's inefficient to run CCA twice. Improve the codes latter.)
	ip.Truncate(imgIbfilter,Timg);
	ip.ImgFilling(Timg, imgFilled);  //Filling the holes inside the markings (Optional)

	//5.3.2. Morphological Operations: Dilation and Closing (Optional)
	/*Mat dilateImg, closeImg;
	Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
	morphologyEx(imgIbfilter, closeImg, MORPH_CLOSE, element);
	//dilate(imgIbfilter, dilateImg, Mat(),element);
	//erode(dilateImg, closeImg, Mat());  // ->closeImg*/

	ip.CcaBySeedFill(imgFilled, labelImg);    //CCA: Seed filling method with 8 neighbor   
	//ip.CcaByTwoPass(imgFilled, labelImg);   //CCA: Two pass method with 4 neighbor (Alternative)
	ip.LabelColor(labelImg, colorLabelImg);  //CCA Labeling and Truncation
	//5.4. Boundary Fitting and Vectorization
	//5.4.1 Harris / Shi-Tomasi Corner Detection (Optional)
	//Mat corner, cornerwithimg;
	//ip.DetectCornerHarris(imgIbfilter, colorLabelImg, corner, cornerwithimg,150);  //Using Harris Detector
	//float minvertexdis = 2.5 / resolution;  //threshold 2 meter
	//double mincornerscore = 0.05; //0.01 original
	//ip.DetectCornerShiTomasi(imgIbfilter, colorLabelImg, cornerwithimg, minvertexdis, mincornerscore);  //Using Shi-Tomasi Detector
	ip.saveimg(imgI, imgZ, imgD, imgImf, imgIgradient, imgZgradient, imgZbinary, imgDbinary, imgIgradientroad, imgIbinary, imgFilled, colorLabelImg /* cornerwithimg*/);//Saving the Images
	t3 = clock();

	//Step 6. 2D->3D back to point cloud
	pcXYZIPtr outcloud(new pcXYZI());
	vector <pcXYZI> outclouds; // please use the cloud instead of the ptr
	vector <pcXYZI> outcloud_otsu_sor;
	vector <pcXYZI> outcloud_otsu_sor_n;
	ip.img2pc(colorLabelImg, gcloud, outcloud);        //Ground Road Marking Points
	ip.img2pclabel(labelImg, gcloud, outclouds,resolution/4);   //Ground Road Marking Points //the last parameter is set as the dZ threshold for single pixel
	
	seg.cloudFilter(outclouds, outcloud_otsu_sor, 256, 15, 1.5); // Three parameters: the first is for the histogram level of Otsu Thresholding , the second is for SOR neighbor number and the third is for SOR std threshold
	seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, 200);  //the last parameter is set as the point number threshold for each cloud
	vector <RoadMarking> roadmarkings;
	seg.MarkingVectorization(outcloud_otsu_sor_n, roadmarkings, 6.0); // Vectorization // the last parameter is set as the linear sample distance
	t4 = clock();
	
	io.writePcdFile("All Road Markings.pcd", outcloud);
	io.writePcdAll("Labeled Clouds", "Labeled Cloud.pcd", outclouds);
	io.writePcdAll("Filtered Labeled Clouds", "Filtered Labeled Cloud.pcd", outcloud_otsu_sor_n);
	
	cout << "Time for Data Import and Ground Segmentation: " << (t2 - t1) *1.0 / 1000 << " s" << endl
		 << "Time for Image Processing: " << (t3 - t2) *1.0 / 1000 << " s" << endl
		 << "Time for Cloud Processing: " << (t4 - t3) *1.0 / 1000 << " s" << endl
		 << "Total Consuming Time: " << (t4 - t1) *1.0 / 1000 << " s" << endl;

	io.displayroad(ngcloud, gcloud);                    //Display non-ground and ground cloud
	io.displaymark(outcloud_otsu_sor_n);                //Display road markings point clouds
	io.displaymarkwithng(outcloud_otsu_sor_n, ngcloud); //Display road markings point clouds with non-ground cloud
	io.displaymarkVect(roadmarkings);                   //Display vectorized road markings
	
	waitKey(0);
	return 1;

}