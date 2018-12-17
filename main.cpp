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

	cout << "Road Markings Extraction Program" << endl;
	cout << "By Yue Pan et al. @ WHU" << endl;
	cout << "-------------------------------------------" << endl;

	//Step 1. Data Import
	cout << "Input the filename (las)" << endl;  
	string filename;
	cin >> filename;
	cout << "Input the resolution of projection img (m)" << endl;
	float resolution;
	cin >> resolution;
	pcXYZIPtr cloud(new pcXYZI());
	DataIo io;
	io.readLasFile(filename, *cloud);  // To import the field of intensity, you need to use las file.
	cout << "the number of raw point clouds: " << cloud->size() << endl;
	cout << "Import OK" << endl;

	//Step 2. Voxel Down-sampling (Optional)
	/*float voxelSize = 0.02;
	VoxelFilter vfilter(voxelSize);
	pcXYZIPtr fcloud(new pcXYZI());
	fcloud = vfilter.filter(cloud);
	io.writePcdFile("filteredcloud.pcd", fcloud);
	cout << "the number of filtered point clouds: " << fcloud->size() << endl;*/

	//Step 3. Ground Filter and projection
	pcXYZIPtr ngcloud(new pcXYZI());
	pcXYZIPtr gcloud(new pcXYZI());
	CGroundExtraction ground;
	StructOperator so;
	Bounds bounds;
	CenterPoint center;
	so.getBoundAndCenter(*cloud, bounds, center);
	ground.ExtractGroundPoint(cloud, gcloud, ngcloud, bounds, center);
	io.writePcdFile("groundcloud.pcd", gcloud);
	io.writePcdFile("ungroundcloud.pcd", ngcloud);
	io.display(ngcloud, gcloud);
	
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
	int nx, ny;
	ip.pcgrid(gcloud, resolution, nx, ny);
	Mat imgI = Mat::zeros(nx, ny, CV_8UC3); //Intensity Projection Image
	Mat imgZ = Mat::zeros(nx, ny, CV_8UC3); //Elevation Projection Image
	ip.pc2imgI(gcloud, resolution, imgI);
	ip.pc2imgZ(gcloud, resolution, imgZ);
	imshow("Projection Image View (Intensity)", imgI);
	imshow("Projection Image View (Elevation)", imgZ);

	//Step 5. Image Processing 
	//5.1. Sobel gradient calculation and boundary extraction
	Mat imgIgradient,imgIbinary0,imgIbinary,imgZgradient,imgZbinary;
	imgIgradient = ip.Sobelboundary(imgI);
	imshow("Intensity Gradient", imgIgradient);
	imgZgradient = ip.Sobelboundary(imgZ);
	imshow("Elevation Gradient", imgZgradient);

	//5.2.1. Image Thresholding using Max Entropy Segmentation
	imgIbinary0 = ip.maxEntropySegMentation(imgIgradient);
	imshow("Intensity Binary Image", 255 * imgIbinary0);
	imgZbinary = ip.maxEntropySegMentation(imgZgradient);
	imshow("Elevation Binary Image", 255 * imgZbinary);
	imgIbinary = ip.ExtractRoadPixel(imgIbinary0, imgZbinary); //Use the slope as the criterion
	imshow("Configured Intensity Binary Image", 255 * imgIbinary);

	//5.2.2. Morphological Operations: Dilation and Closing (Optional)

	//5.3. Intensity Image Connected Component Analysis (CCA) and Labeling
	Mat imgIbfilter, labelImg, grayLabelImg, colorLabelImg0, colorLabelImg;
	float smallregion = 1.0 / (resolution*resolution);    //1.0 m^2 as the threshold for small connected region. Manually tune parameter (1.0) here.
	ip.RemoveSmallRegion(imgIbinary, imgIbfilter, smallregion); //CCA Filtering (Problem: It's inefficient to run CCA twice. Improve the codes latter.)
	ip.CcaBySeedFill(imgIbfilter, labelImg);  //CCA: Seed filling method with 8 neighbor
	//ip.CcaByTwoPass(imgbound, labelImg);   //CCA: Two pass method with 4 neighbor (Alternative)
	ip.LabelColor(labelImg, colorLabelImg0);  //CCA Labeling and Truncation
	imshow("CCA Labeling", colorLabelImg0);
	//colorLabelImg = ip.Truncate(colorLabelImg0);       //Truncate the Image
	//imshow("Truncated CCA Labeling", colorLabelImg);
	ip.saveimg(imgI, imgZ, imgIgradient, imgZgradient, imgIbinary0, imgZbinary, imgIbinary, colorLabelImg0); //Saving the Images

	//5.4. Boundary Fitting and Vectorization
	waitKey(0);
	return 1;
}