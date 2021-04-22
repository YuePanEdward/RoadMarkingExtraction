#include "data_io.h"
#include "utility.h"
#include "ground_extraction.h"
#include "pointcloudprocess.h"
#include "imageprocess.h"
#include "modelmatch.h"

#include <chrono>

using namespace std;
using namespace roadmarking;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        printf("Syntax is: %s input_file.las output_folder [model_pool] [config_file]\n", argv[0]);
        return (-1);
    }
    std::string inputFilePath = argv[1];
    std::string outputFolderPath = argv[2];
    std::string model_path = "./model_pool/models_urban_example/";
    std::string parm_file = "./config/parameter_urban_example.txt";

    if (argc == 3)
        printf("Model pool path and configuration file are not specified, use the default model pool and parameters.\n");
    if (argc == 4)
    {
        model_path = argv[3];
        printf("Configuration file is not specified, use the default parameters.\n");
    }
    if (argc >= 5)
    {
        model_path = argv[3];
        parm_file = argv[4];
    }

    /*-------------------------------------------------Readme----------------------------------------------------*/
    //[Road Markings Extraction, Classification and Vectorization]
    //Brief: Roadmarking extraction, classification (long sidelines, short dash lines, all kinds of arrows,
    //signs, etc.) and vectorization
    //Basic method:
    //1. ground segmentation and generate geo-referneced images
    //2. Image processing -> get roadmarking pixels
    //3. retrieve roadmarking point clouds from the pixels and apply filtering
    //4. classify the roadmarkings into 9 categories based on bounding box geometric feature and template
    //(model) matching
    //5. combine or delete some of the roadmarkings and then do vectorization based on bounding box and template
    //localization information
    //Implementation:
    //Dependent 3rd Party Libs: Eigen 3, PCL 1.7, OpenCV 2, Liblas, GDAL(optional), LASLib(optional)
    //Point Cloud Acquisition Platform: mainly MLS
    //Application Scenarios: Highway, Urban Road
    //Author: Yue Pan et al. @ WHU
    /*-----------------------------------------------------------------------------------------------------------*/

    cout << "-------------------------------------------------------------" << endl;
    cout << "Road Markings Extraction Module" << endl;

    /*-----------------------------------------------Declaration-------------------------------------------------*/
    clock_t t1, t2; // Timing
    //Step 1
    DataIo io; // Data IO class
    //string foldername, paralistfile;    // Las Data folder name ; Parameter list file name
    string filename = inputFilePath.substr(inputFilePath.rfind('/'));
    int datatype, roadtype, is_road_extracted; // datatype: (1.MLS/TLS 2.ALS) ;  roadtype: (1.Highway 2.City Road) ; density: approximate point number per m^2
    float resolution;                          // Projection Image's pixel size (m)

    datatype = 1;          //MLS
    is_road_extracted = 0; //not extracted

    io.readParalist(parm_file);

    roadtype = io.paralist.road_type;
    float density = io.paralist.expected_point_num_per_m_square;
    resolution = io.paralist.grid_resolution;
    if (io.paralist.visualization_on)
        cout << "Visualization on" << endl;

    io.displayparameter(datatype, roadtype, is_road_extracted);

    t1 = clock();

    cout << "-------------------------------------------------------------" << endl;
    cout << "Processing File : " << inputFilePath << endl;

    //Timing

    //Step 1
    pcXYZIPtr cloud(new pcXYZI());   // Original point clouds
    vector<double> CloudBoundingBox; // Xmin,Ymin,Zmin,Xmax,Ymax,Zmax after translation for Input point cloud;
    Bounds bound_3d_temp;
    double X_origin =0.0, Y_origin=0.0; // X,Y origin before translation;
    //Step 2,3
    pcXYZIPtr fcloud(new pcXYZI());  // Sampled point cloud
    pcXYZIPtr ngcloud(new pcXYZI()); // Non-ground point cloud
    pcXYZIPtr gcloud(new pcXYZI());  // Ground point cloud
    //Step 4
    Mat imgI, imgZ, imgD; // imgI:Intensity Projection Image; imgZ: Elevation Projection Image ; imgD: Density Projection Image
    //Step 5
    Mat imgImf, imgIgradient, imgIgradientroad, imgIbinary, imgZgradient, imgZbinary, imgDbinary, imgIbfilter, labelImg, colorLabelImg, imgFilled, Timg, dilateImg, closeImg, corner, cornerwithimg; // imgImf: Intensity Projection Image after Median Filter ; imgIgradient: Intensity Gradient Image ; imgIgradientroad: Road's Intensity Gradient Image ; imgIbinary: Road's Intensity Binary Image ; imgZgradient: Slope (dZ) projection image ; imgZbinary: Slope binary image ; imgDbinary: Point density binary image ; imgIbfilter: Intensity Binary Image after filtering (Based on Connected Region's area) ; labelImg: Labeled Image based on Connected Conponent Analysis (CCA) ; colorLabelImg :  Labeled Image based on Connected Conponent Analysis (CCA) render by random colors ; imgFilled : Filled Image (the closed geometry are all filled) ; Timg: Truncated Image of Interested area (For Saving computing time) ; dilateImg : Image after dilation morphology calculation ; closeImg : Image after close morphology calculation ; corner : Image corner pixels ; cornerwithimg: Image with its corner pixels
    //Step 6
    //pcXYZIPtr outcloud(new pcXYZI()); //Road Marking Cloud (All in one)
    vector<pcXYZI> outclouds, outcloud_otsu_sor, outcloud_otsu_sor_n, boundaryclouds; //Road Marking Clouds (Segmentation); Filter: Otsu+SOR ; Filter: Point Number ; Boundary Clouds ; Corner Clouds
    //Step 7
    RoadMarkings roadmarkings, sideline_roadmarkings, roadmarkings_vect; // Unit of Road markings (Category + Polyline)
    vector<vector<pcl::PointXYZI>> boundingdatas, modeldatas;            // Unit of Road markings' bounding box and model datas
    vector<BoundingFeature> boundingfeatures;                            // Unit of Road markings' bounding box feature
    vector<bool> is_rights;                                              // Unit of Arrow Road markings' direction

    //Step 1. Data Import
    string extension = inputFilePath.substr(inputFilePath.find_last_of('.') + 1); //Get the suffix of the file;
    if (!strcmp(extension.c_str(), "las"))
        io.readLasFile(inputFilePath, *cloud, bound_3d_temp);
    else if (!strcmp(extension.c_str(), "pcd"))
        io.readPcdFile(inputFilePath, cloud, bound_3d_temp);
    else 
        printf("Unrecognized data format. Please use *.pcd or *.las format point cloud.\n");
    
    X_origin = 0.5 * (bound_3d_temp.min_x + bound_3d_temp.max_x);
    Y_origin = 0.5 * (bound_3d_temp.min_y + bound_3d_temp.max_y); 
    cout << "Import [" << cloud->size() << "] points." << endl;
    printf("Global shift (X: %8.3f m, Y: %8.3f m)\n", -X_origin, -Y_origin);

    std::chrono::steady_clock::time_point tic = std::chrono::steady_clock::now();

    //Step 2. Voxel Down-sampling (Optional)
    /*float voxelSize = 0.02;
		VoxelFilter vfilter(voxelSize);
		fcloud= vfilter.filter(cloud);
		//cout << "the number of filtered point clouds in file " << i << " : " << fcloud->size() << endl;
		}*/

    //Step 3. Ground Filter and projection (Optional)
    // The Ground Segmentation can help you find the road more precisely. However, this process would take more time.
    // Method 1: Dual-threshold grid based filter (Fast)
    // Parameter Setting: grid_res_ = 0.5, min_pt_num_grid_ = 30, max_height_diff_ = 0.2
    Ground_Extraction ground;
    StructOperator so;
    Bounds bounds;
    CenterPoint center;
    so.getBoundAndCenter(*cloud, bounds, center);
    ground.Extract_ground_pts(cloud, gcloud, ngcloud, bounds, center);
    // cout << "Ground: " << gcloud->points.size() << ", "
    //      << "Non-Ground: " << ngcloud->points.size() << endl;
    cout << "Ground Segmentation done.\n";

    //Method 2: PMF (Slow)
    //seg.GroundFilter_PMF(cloud[i], gcloud, ngcloud);

    Csegmentation seg(resolution);
    //Preprocessing: Segmentation and Fitting (Optional)
    //RANSAC plane segmentation
    /*pcXYZIPtr fitcloud(new pcXYZI());
	  float td = 1;
	  fitcloud = seg.planesegRansac(gcloud, td);
	  io.writePcdFile("ransac ground cloud.pcd", fitcloud);
	  //Projection
	  pcXYZIPtr pgcloud(new pcXYZI());
      pgcloud = seg.groundprojection(gcloud);
	  io.writePcdFile("projected groundcloud.pcd", pgcloud);*/

    //Step 4. 3D->2D projection, Generating Projection Image
    Imageprocess ip;
    ip.savepcgrid(bound_3d_temp, resolution, cloud, gcloud, ngcloud); //get image size and save point indices in each pixel

    //For better efficiency (without ground segmentation), you can replace gcloud and ngcloud with cloud
    //[0:Original Cloud, 1 : Ground Cloud, 2 : Non - ground Cloud]
    // Image 1
    ip.pc2imgI(gcloud, 1, imgI, io.paralist.intensity_scale);
    // Image 2
    ip.pc2imgZ(ngcloud, 2, imgZ);
    // Image 3
    if (datatype == 1)
    { // For MLS
        float expectedmaxnum;
        expectedmaxnum = io.paralist.density_threshold * density * resolution * resolution; // expectedmaxnum: expected max point number in a pixel
        ip.pc2imgD(gcloud, 1, imgD, expectedmaxnum);
    }
    cout << "Point Cloud --> Geo-referneced Image done\n";

    //Step 5. Image Processing
    //5.1.1 Median filter  (Optional)
    // Image 4
    if (datatype == 1)               // For MLS
        medianBlur(imgI, imgImf, 3); // Remove the salt and pepper noise
    else if (datatype == 2)
        imgImf = imgI; // For ALS

    //5.1.2 Sobel gradient calculation and boundary extraction
    // Image 5
    imgIgradient = ip.Sobelboundary(imgImf);
    // Image 6
    imgZgradient = ip.Sobelboundary(imgZ);

    //5.2.1. Image Thresholding using Max Entropy Segmentation (All self-adaptive)
    // Image 7
    imgZbinary = ip.maxEntropySegMentation(imgZgradient);
    //int maxroadslope = 50;  // Setting the threshold for Surface Roughness, 50/255 here
    //threshold(imgZgradient, imgZbinary, maxroadslope, 1, CV_THRESH_BINARY);
    if (datatype == 1)
    {
        // Image 8
        imgDbinary = ip.maxEntropySegMentation(imgD); // for MLS
        // Image 9
        imgIgradientroad = ip.ExtractRoadPixelIZD(imgIgradient, imgZbinary, imgDbinary); //Use the slope and point density as the criterion
    }
    else if (datatype == 2)
        imgIgradientroad = ip.ExtractRoadPixelIZ(imgIgradient, imgZbinary); //for ALS
    // Image 10
    imgIbinary = ip.maxEntropySegMentation(imgIgradientroad);
    //However, Max Entropy Method is not efficient enough

    //5.3. Intensity Image Connected Component Analysis (CCA) and Labeling
    float smallregion;
    if (datatype == 1)
        smallregion = 0.4 / (resolution * resolution); //Pixel Number Threshold: 0.6 m^2 as the threshold for small connected region. Manually tune parameter (0.5) here.
    else if (datatype == 2)
        smallregion = 0.5 / (resolution * resolution);
    ip.RemoveSmallRegion(imgIbinary, imgIbfilter, smallregion); //CCA Filtering (Problem: It's inefficient to run CCA twice. Improve the codes latter.)

    // filling the hole
    Timg = imgIbfilter;
    // Timg = imgIbinary;
    //ip.Truncate(imgIbfilter, Timg); //may encounter some problem (OpenCV Error: Assertion failed)
    ip.ImgFilling(Timg, imgFilled); //Filling the holes inside the markings (Optional)

    //5.3.2. Morphological Operations: Dilation and Closing (Optional)
    /*Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
		morphologyEx(imgIbfilter, closeImg, MORPH_CLOSE, element);
		//dilate(imgIbfilter, dilateImg, Mat(),element);
		//erode(dilateImg, closeImg, Mat());  // ->closeImg*/

    // Image 11
    //ip.CcaBySeedFill(Timg, labelImg);
    ip.CcaBySeedFill(imgFilled, labelImg); //CCA: Seed filling method with 8 neighbor
    //ip.CcaByTwoPass(imgFilled, labelImg);   //CCA: Two pass method with 4 neighbor (Alternative)

    // Image 12
    ip.LabelColor(labelImg, colorLabelImg); //CCA Labeling and Truncation

    //5.4 Harris / Shi-Tomasi Corner Detection (Optional)
    /*ip.DetectCornerHarris(imgIbfilter, colorLabelImg, corner, cornerwithimg,150);  //Using Harris Detector
	  float minvertexdis = 2.0 / resolution;  //threshold 2 meter
	  double mincornerscore = 0.05; //0.01 original
	  ip.DetectCornerShiTomasi(imgFilled, colorLabelImg, cornerwithimg, minvertexdis, mincornerscore);  //Using Shi-Tomasi Detector*/
    cout << "Roadmarking pixels extraction done.\n";

    //Step 6. 2D->3D back to point cloud
    //ip.img2pc_g(colorLabelImg, gcloud, outcloud);                //Ground Road Marking Points (All in one)
    ip.img2pclabel_g(labelImg, gcloud, outclouds, resolution / 5); //Ground Road Marking Points (Segmentation) //Elevation filter: the last parameter is set as the dZ threshold for single pixel

    //Use Otsu (Intensity) Method and Statistics Outlier Remover to filter the point cloud
    seg.cloudFilter(outclouds, outcloud_otsu_sor, 256, 10, 2.5); // Three parameters: the first is for the histogram level of Otsu Thresholding , the second is for SOR neighbor number and the third is for SOR std threshold

    //Delete point clouds whose point number is less than a threshold
    if (datatype == 1)
        seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, density / 15); //the last parameter is set as the point number threshold for each cloud
    else
        seg.NFilter(outcloud_otsu_sor, outcloud_otsu_sor_n, 10);

    cout << "Roadmarking pixels --> roadmarking point cloud done.\n";

    //Step 7. Object Recognition and Classification based on Model Matching and Geometric Information
    //7.1 Boundary and Corner Extraction (Optional)
    // Boundary Extraction: Alpha-Shape Concave Hull Generation
    seg.BoundaryExtraction(outcloud_otsu_sor_n, boundaryclouds, 2, 0.7);
    // Corner Extraction: Neighborhood Processing
    // seg.CornerExtraction(boundaryclouds,cornerclouds,1,8,0.1,0.02,0.95); // Parameters: 1/0 Use Radius Search or KNN, 8, KNN's K, 0.15 search radius , 0.02 distance threshold, 0.94 maxcos
    cout << "Find [" << boundaryclouds.size() << "] candidate roadmarkings\n";

    //7.2 Road Markings Rough Classification based on Bounding Box Feature
    seg.BoundingInformation(outcloud_otsu_sor_n, boundingdatas);
    seg.BoundingFeatureCalculation(boundingdatas, boundingfeatures);

    if (roadtype == 1)
        seg.CategoryJudgementBox_highway(boundingfeatures, roadmarkings);
    else if (roadtype == 2)
        seg.CategoryJudgementBox_cityroad(boundingfeatures, roadmarkings);
    cout << "Roadmarking rough classification done\n";

    //7.3 Road Markings (Arrow) Classification based on Model Matching (Relatively Slow, You may try other faster 2D feature and matching strategy)
    cout << "Begin model matching.\n";
    ModelMatch mm = ModelMatch(10, io.paralist.model_matching_correct_match_fitness_thre, io.paralist.model_matching_heading_increment,
                               io.paralist.model_matching_overlapping_dist_thre, io.paralist.model_matching_overlapping_ratio_thre);
    mm.model_match(model_path, boundaryclouds, roadmarkings);

    cout << "Irregular roadmarking classification by model matching done.\n";

    cout << "Roadmarking extraction done, [" << roadmarkings.size() << "] in total.\n";

    //Step 8. Vectorization
    // the parameters are set as the linear sample distance and ambiguous ratio for long side line vectorization
    if (roadtype == 1)
        seg.MarkingVectorization_highway(boundaryclouds, boundingdatas, roadmarkings, io.paralist.sideline_vector_distance, 0.2);
    else if (roadtype == 2)
        seg.MarkingVectorization_cityroad(boundaryclouds, boundingdatas, roadmarkings, io.paralist.sideline_vector_distance, 0.15);

    seg.CombineSideLines(roadmarkings, io.paralist.sideline_vector_distance, sideline_roadmarkings); //Side lines Combination
    seg.GetRoadmarkingsForVect(roadmarkings, sideline_roadmarkings, roadmarkings_vect);

    cout << "Roadmarking vectorization done, [" << roadmarkings_vect.size() << "] in total\n";

    std::chrono::steady_clock::time_point toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);

    /*------------------------------------------------Output----------------------------------------------------*/
    //Step 9. Output Result

    if (!boost::filesystem::exists(outputFolderPath))
        boost::filesystem::create_directory(outputFolderPath);

    string output_sub_folder = outputFolderPath + filename + "_out";

    if (!boost::filesystem::exists(output_sub_folder))
        boost::filesystem::create_directory(output_sub_folder);

    // Output Images
    ip.saveimg(output_sub_folder + "/Geo-referenced_Image", 0, imgI, imgZ, imgD, imgImf, imgIgradient, imgZgradient, imgZbinary, imgDbinary, imgIgradientroad, imgIbinary, imgFilled, colorLabelImg); //Saving the Images

    // Output pointcloud
    // You can view the results in CloudCompare or AutoDesk Recap
    // io.writePcdAll(outputFilePath+"/Filtered_Labeled_Clouds", "filtered_labeled.pcd", outcloud_otsu_sor_n);
    // io.writePcdAll(output_sub_folder+"/Boundary_Clouds", "boundary.pcd", boundaryclouds);
    io.writeLasAll(0, output_sub_folder + "/Classified_Road_Markings", outcloud_otsu_sor_n, roadmarkings, X_origin, Y_origin);

    //Output vectorization results
    //You can view the results on https://beta.sharecad.org/ or in AutoCAD
    io.writemarkVectDXF(0, output_sub_folder + "/Vectorized_Road_Markings", roadmarkings, sideline_roadmarkings, X_origin, Y_origin); //*.dxf format
    //io.writeRoadmarkingShpWithOffset(outputFilePath, roadmarkings_vect, i, X_origin, Y_origin); //*.shp format

    //timing
    std::cout << "Process file [" << inputFilePath << "] done in [" << time_used.count() << "] s.\n";

    //Display Results
    if (io.paralist.visualization_on)
    {
        //io.displayroad(ngcloud, gcloud);                                              //Display non-ground and ground cloud
        io.displaymarkwithng(outcloud_otsu_sor_n, ngcloud);          //Display road markings point clouds with non-ground cloud
        io.displaymarkbycategory(outcloud_otsu_sor_n, roadmarkings); //Display road markings point clouds rendered by category
                                                                     //io.displaymarkVect(roadmarkings, sideline_roadmarkings);                        //Display vectorized road markings
    }

    return 1;
}
