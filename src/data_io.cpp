#include "data_io.h"
#include "utility.h"

// pcl io
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// *.shp vectorization
// #include <gdal.h>
// #include <gdal_alg.h>
// #include <gdal_priv.h>
// #include <ogrsf_frmts.h>

// *.las io
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

// *dxf vectorization
#include <dl_dxf.h>
#include <dl_entities.h>

#include <iosfwd>
#include <fstream>

//#if GDAL_VERSION_MAJOR >= 2
//#define GDAL2
//#endif

using namespace std;
using namespace boost::filesystem;

namespace roadmarking
{
	void DataIo::readParalist(string paralistfile)
	{
		std::ifstream infile;
		infile.open(paralistfile, ios::in);
		if (!infile.is_open()) cout << "Failed to open the parameter file, use default parameters." << endl;
		infile >> paralist.road_type;
		infile >> paralist.expected_point_num_per_m_square;
		infile >> paralist.grid_resolution;

		infile >> paralist.intensity_scale;
		infile >> paralist.density_threshold;

		infile >> paralist.model_matching_heading_increment;
		infile >> paralist.model_matching_correct_match_fitness_thre;
		infile >> paralist.model_matching_overlapping_dist_thre;
		infile >> paralist.model_matching_overlapping_ratio_thre;

		infile >> paralist.sideline_vector_distance;
		infile >> paralist.visualization_on;
	}

	void DataIo::displayparameter(int datatype, int roadtype, int is_road_extracted) {
		cout << "-------------------------------------------------------------" << endl;
		cout << "Parameter Specification:" << endl;

		if (datatype == 1) cout << "MLS data\t";
		else cout << "ALS data\t";

		if (roadtype == 1)cout << "Highway\t";
		else if (roadtype == 2) cout << "Urban road\t";

		/*if (is_road_extracted == 1) cout << "Road unextracted\n";
		else cout << "Road extracted\n";*/
    
		cout << "\nExpected point density:\t" << paralist.expected_point_num_per_m_square << " /m^2" << endl;
		cout << "Grid resolution (m):\t" << paralist.grid_resolution << endl;
		cout << "Point cloud intensity scale:\t" << paralist.intensity_scale << endl;
		cout << "Point cloud density threshold:\t" << paralist.density_threshold << endl;
		cout << "Model matching rotation anagle increment step (deg):\t" << paralist.model_matching_heading_increment << endl;
		cout << "Model matching fitness score threhold for candidate matching:\t" << paralist.model_matching_correct_match_fitness_thre << endl;
		cout << "Model matching overlapping search distance threshold (m):\t" << paralist.model_matching_overlapping_dist_thre << endl;
		cout << "Model matching tolerant min overlapping ratio (%):\t" << paralist.model_matching_overlapping_ratio_thre * 100.0 << " %" << endl;
		cout << "Sideline vectorization distance (m):\t" << paralist.sideline_vector_distance << endl;
		cout << "-------------------------------------------------------------" << endl;

	}

	bool DataIo::writeRoadmarkingShpWithOffset(const std::string &base_folder_name,
		RoadMarkings & roadmarkings, int file_index,
		float d_x, float d_y, float d_z)
	{
		// string outputFolder = base_folder_name + "\\Vectorized Road Markings (shp)"; // Windows
		string outputFolder = base_folder_name + "/Vectorized Road Markings (shp)"; // Ubuntu

		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}

		std::string filename;
		ostringstream oss;
		oss << "\\Vectorized Road Markings_" << file_index << ".shp";
		filename = outputFolder + oss.str();

		//const char *pszDriverName = "ESRI Shapefile";
		//GDALAllRegister();
		//OGRRegisterAll();
		//CPLSetConfigOption("GDAL_FILENAME_IS_UTF8", "NO");

		//OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(pszDriverName);
		////GDALDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(pszDriverName);

		//if (poDriver == NULL) {
		//	std::cout << "ESRI Shapefile : This driver is not available!" << std::endl;
		//	//exit(1);
		//}

		//if ((_access(filename.c_str(), 0)) != -1) {
		//	std::cout << "Exist this shp file." << std::endl;
		//	remove(filename.c_str());
		//}

		//OGRDataSource *poDS;
		////GDALDataset *poDS;

		//poDS = poDriver->CreateDataSource(filename.c_str(), NULL);
		////poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL);

		//if (poDS == NULL) {
		//	std::cout << "Failed in creating Dataset" << std::endl;
		//	//exit(1);
		//}
		//else
		//	std::cout << "Creating dataset done" << std::endl;

		////// create layers:
		//OGRLayer *poLayer;
		//poLayer = poDS->CreateLayer("roadmarking", NULL, wkbMultiLineString25D, NULL);
		//if (poLayer == NULL) {
		//	std::cout << "Layer creation failed!" << std::endl;
		//	exit(1);
		//}
		//else
		//	std::cout << "Creating layer done" << std::endl;

		//// create the attributes tables:
		//OGRFieldDefn poFieldID("ID", OFTInteger);
		//poFieldID.SetWidth(40);

		//OGRFieldDefn poFieldType("Type", OFTInteger);
		//poFieldID.SetWidth(40);

		//if (poLayer->CreateField(&poFieldID) != OGRERR_NONE) {
		//	std::cout << "ID field creation failed!" << std::endl;
		//	exit(1);
		//}
		//if (poLayer->CreateField(&poFieldType) != OGRERR_NONE) {
		//	std::cout << "Type field creation failed!" << std::endl;
		//	exit(1);
		//}

		//// create lines
		//int line_N = roadmarkings.size();

		//for (int i = 0; i < line_N; ++i) {

		//	if (roadmarkings[i].category > 0 && roadmarkings[i].polyline.size() > 1)
		//	{
		//		//cout << "Output vector " << i << " begin \n";
		//		int pt_N = roadmarkings[i].polyline.size();

		//		OGRFeature *poFeature;
		//		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
		//		poFeature->SetField("ID", i);
		//		poFeature->SetField("Type", roadmarkings[i].category);
		//		//OGRwkbGeometryType temp_line(wkbLineString25D);

		//		OGRLineString line;
		//		for (int j = 0; j < pt_N; ++j)
		//		{
		//			line.addPoint(roadmarkings[i].polyline[j].x + d_x,
		//				roadmarkings[i].polyline[j].y + d_y,
		//				roadmarkings[i].polyline[j].z + d_z);
		//		}
		//		if (roadmarkings[i].category > 1)
		//			line.addPoint(roadmarkings[i].polyline[0].x + d_x,
		//				roadmarkings[i].polyline[0].y + d_y,
		//				roadmarkings[i].polyline[0].z + d_z); // close the polyline

		//		poFeature->SetGeometry(&line);

		//		if (poLayer->CreateFeature(poFeature) != OGRERR_NONE) {
		//			std::cout << "Failed to create feature in shapefile." << std::endl;
		//		}
		//		OGRFeature::DestroyFeature(poFeature);

		//		//cout << "Output vector " << i << " done \n";
		//	}
		//}

		//OGRDataSource::DestroyDataSource(poDS);

		cout << "Output vectorized roadmarking done \n";

		return 1;
	}



	// bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
	// {
	// 	Bounds bound;
	// 	GetBoundaryOfPointCloud(pointCloud, bound);
	
	// 	ofstream ofs;
	// 	ofs.open(fileName, std::ios::out | std::ios::binary);
	// 	if (ofs.is_open())
	// 	{
	// 		liblas::Header header;
	// 		header.SetDataFormatId(liblas::ePointFormat2);
	// 		header.SetVersionMajor(1);
	// 		header.SetVersionMinor(2);
	// 		header.SetMin(bound.min_x, bound.min_y, bound.min_z);
	// 		header.SetMax(bound.max_x, bound.max_y, bound.max_z);
	// 		header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
	
	// 		header.SetScale(0.01, 0.01, 0.01);
	// 		header.SetPointRecordsCount(pointCloud.points.size());
	
	// 		liblas::Writer writer(ofs, header);
	// 		liblas::Point pt(&header);
	
	// 		for (int i = 0; i < pointCloud.points.size(); i++)
	// 		{
	// 			pt.SetCoordinates(pointCloud.points[i].x, pointCloud.points[i].y, pointCloud.points[i].z);
	// 			pt.SetIntensity(pointCloud.points[i].intensity);
	// 			writer.WritePoint(pt);
	// 		}
	// 		ofs.flush();
	// 		ofs.close();
	// 	}
	
	// 	return 1;
	// }
	
	// bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, liblas::Color lasColor)
	// {
	// 	Bounds bound;
	// 	GetBoundaryOfPointCloud(pointCloud, bound);
	
	// 	ofstream ofs;
	// 	ofs.open(fileName, std::ios::out | std::ios::binary);
	// 	if (ofs.is_open())
	// 	{
	// 		liblas::Header header;
	// 		header.SetDataFormatId(liblas::ePointFormat2);
	// 		header.SetVersionMajor(1);
	// 		header.SetVersionMinor(2);
	// 		header.SetMin(bound.min_x, bound.min_y, bound.min_z);
	// 		header.SetMax(bound.max_x, bound.max_y, bound.max_z);
	// 		header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
	
	// 		header.SetScale(0.01, 0.01, 0.01);
	// 		header.SetPointRecordsCount(pointCloud.points.size());
	
	// 		liblas::Writer writer(ofs, header);
	// 		liblas::Point pt(&header);
	
	// 		for (int i = 0; i < pointCloud.points.size(); i++)
	// 		{
	// 			pt.SetCoordinates(pointCloud.points[i].x, pointCloud.points[i].y, pointCloud.points[i].z);
	// 			pt.SetIntensity(pointCloud.points[i].intensity);
	// 			pt.SetColor(lasColor);
	// 			writer.WritePoint(pt);
	// 		}
	// 		ofs.flush();
	// 		ofs.close();
	// 	}
	
	// 	return 1;
	// }
	
	bool DataIo::writeLasFile(const std::string &fileName,  pcl::PointCloud<pcl::PointXYZI> &pointCloud, liblas::Color lasColor, double min_X, double min_Y)
	{
		Bounds bound;
		GetBoundaryOfPointCloud(pointCloud, bound);
		// hyj
		std::ofstream ofs;
		ofs.open(fileName, std::ios::out | std::ios::binary);
		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetMin(bound.min_x + min_X, bound.min_y + min_Y, bound.min_z);
			header.SetMax(bound.max_x + min_X, bound.max_y + min_Y, bound.max_z);
			header.SetOffset(double(bound.min_x + bound.max_x) / 2.0 + min_X, double(bound.min_y + bound.max_y) / 2.0 + min_Y, double(bound.min_z + bound.max_z) / 2.0);
			header.SetScale(0.01, 0.01, 0.01);
			header.SetPointRecordsCount(pointCloud.points.size());
	
			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);
	
			for (int i = 0; i < pointCloud.points.size(); i++)
			{
				pt.SetCoordinates(pointCloud.points[i].x+min_X, pointCloud.points[i].y+min_Y, pointCloud.points[i].z);
				pt.SetIntensity(pointCloud.points[i].intensity);
				pt.SetColor(lasColor);
				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}
	
		return 1;
	}

	// bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
	// {
	// 	Bounds bound;
	// 	GetBoundaryOfPointCloud(pointCloud, bound);
	
	// 	ofstream ofs;
	// 	ofs.open(fileName, std::ios::out | std::ios::binary);
	// 	if (ofs.is_open())
	// 	{
	// 		liblas::Header header;
	// 		header.SetDataFormatId(liblas::ePointFormat2);
	// 		header.SetVersionMajor(1);
	// 		header.SetVersionMinor(2);
	// 		header.SetMin(bound.min_x, bound.min_y, bound.min_z);
	// 		header.SetMax(bound.max_x, bound.max_y, bound.max_z);
	// 		header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
	
	// 		header.SetScale(0.0000001, 0.0000001, 0.0000001);
	// 		header.SetPointRecordsCount(pointCloud.points.size());
	
	// 		liblas::Writer writer(ofs, header);
	// 		liblas::Point pt(&header);
	
	// 		for (int i = 0; i < pointCloud.points.size(); i++)
	// 		{
	// 			if (bound.max_x < 10.0&&bound.max_y < 10.0&&bound.max_z < 10.0)
	// 			{
	// 				pt.SetCoordinates((double)(pointCloud.points[i].x*1000.0),
	// 					(double)(pointCloud.points[i].y*1000.0),
	// 					(double)(pointCloud.points[i].z*1000.0));
	// 			}
	// 			else
	// 			{
	// 				pt.SetCoordinates((double)pointCloud.points[i].x, (double)pointCloud.points[i].y,
	// 					(double)pointCloud.points[i].z);
	// 			}
	
	// 			uint8_t red = pointCloud.points[i].r;
	// 			uint8_t green = pointCloud.points[i].g;
	// 			uint8_t blue = pointCloud.points[i].b;
	// 			pt.SetColor(liblas::Color(red, green, blue));
	// 			writer.WritePoint(pt);
	// 		}
	// 		ofs.flush();
	// 		ofs.close();
	// 	}
	
	// 	return 1;
	// }
	
	bool DataIo::writeLasAll(int file_index, const string &outputFolder,  vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds,
		const RoadMarkings &roadmarkings, double minX, double minY)
	{
	
		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}
	
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss_id;
			//oss_id << setw(4) << setfill('0') << file_index <<  "_" << setw(4) << setfill('0') << i << ".las";

            oss_id << setw(4) << setfill('0') << i << "_" << setw(2) << setfill('0') << roadmarkings[i].category << ".las";

			// outputFileName = outputFolder + "\\" + oss.str(); // Windows
			outputFileName = outputFolder + "/" + oss_id.str(); // Ubuntu
			
			liblas::Color lasColor;
			
			switch (roadmarkings[i].category)
			{
			case 1:
				lasColor.SetRed(0); 
				lasColor.SetGreen(255);
				lasColor.SetBlue(0);
				break;
			case 2:
				lasColor.SetRed(255);
				lasColor.SetGreen(0);
				lasColor.SetBlue(0);
				break;
			case 3:
				lasColor.SetRed(0);
				lasColor.SetGreen(0);
				lasColor.SetBlue(255);
				break;
			case 4:
				lasColor.SetRed(0);
				lasColor.SetGreen(255);
				lasColor.SetBlue(255);
				break;
			case 5:
				lasColor.SetRed(255);
				lasColor.SetGreen(0);
				lasColor.SetBlue(255);
				break;
			case 6:
				lasColor.SetRed(255);
				lasColor.SetGreen(255);
				lasColor.SetBlue(0);
				break;
			case 8:
				lasColor.SetRed(255);
				lasColor.SetGreen(170);
				lasColor.SetBlue(0);
				break;
			default:
				lasColor.SetRed(255);
				lasColor.SetGreen(255);
				lasColor.SetBlue(255);
				break;
			}
	
			writeLasFile(outputFileName, pointClouds[i],lasColor,minX,minY);
		}
		cout << "Output Classified Road Markings Point Cloud Done" << endl;
		return true;
	}

	bool DataIo::writeLasAll(const std::string &folderName, int file_index, const string &fileName,  vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds,
		const RoadMarkings &roadmarkings, double minX, double minY)
	{
	
		string outputFolder;
	
		ostringstream oss;
		oss << "Classified Road Markings_" << file_index;
		
		// outputFolder = folderName + "Classified Road Markings (las)\\" + oss.str(); // Windows
		// outputFolder = folderName + "Classified Road Markings (las)/" + oss.str(); // Ubuntu
		outputFolder = folderName + "Classified Road Markings (las)/"; // Ubuntu
	
		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}
	
	
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss;
			oss << i << "_" << fileName;
			// outputFileName = outputFolder + "\\" + oss.str() + ".las"; // Windows
			outputFileName = outputFolder + "/" + oss.str() + ".las"; // Windows
			
			liblas::Color lasColor;
			
			switch (roadmarkings[i].category)
			{
			case 1:
				lasColor.SetRed(0); 
				lasColor.SetGreen(255);
				lasColor.SetBlue(0);
				break;
			case 2:
				lasColor.SetRed(255);
				lasColor.SetGreen(0);
				lasColor.SetBlue(0);
				break;
			case 3:
				lasColor.SetRed(0);
				lasColor.SetGreen(0);
				lasColor.SetBlue(255);
				break;
			case 4:
				lasColor.SetRed(0);
				lasColor.SetGreen(255);
				lasColor.SetBlue(255);
				break;
			case 5:
				lasColor.SetRed(255);
				lasColor.SetGreen(0);
				lasColor.SetBlue(255);
				break;
			case 6:
				lasColor.SetRed(255);
				lasColor.SetGreen(255);
				lasColor.SetBlue(0);
				break;
			case 8:
				lasColor.SetRed(255);
				lasColor.SetGreen(170);
				lasColor.SetBlue(0);
				break;
			default:
				lasColor.SetRed(255);
				lasColor.SetGreen(255);
				lasColor.SetBlue(255);
				break;
			}
	
			writeLasFile(outputFileName, pointClouds[i],lasColor,minX,minY);
		}
		cout << "Output Classified Road Markings Point Cloud Done" << endl;
		return true;
	}
	
	// bool DataIo::batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames,
	// 	std::vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds)
	// {
	// 	path folderPath(folderName);
	
	// 	//�����ļ���;
	// 	if (!exists(folderName))
	// 	{
	// 		create_directory(folderName);
	// 	}
	
	// 	if (fileNames.empty())
	// 	{
	// 		return 0;
	// 	}
	// 	else
	// 	{
	// 		for (size_t i = 0; i < fileNames.size(); ++i)
	// 		{
	// 			path dir(fileNames[i]);
	// 			string fileName;
	// 			//��ȡ������·���ͺ�׺���ļ���;
	// 			fileName = dir.stem().string();
	// 			fileName = folderName + "\\" + fileName + ".las";
	// 			writeLasFile(fileName, pointClouds[i]);
	// 		}
	// 	}
	
	// 	return 1;
	// }
	//
	//bool DataIo::batchWriteLasFile(const std::string &folderName, const std::vector<std::string> &fileNames,
	//	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &pointClouds)
	//{
	//	path folderPath(folderName);
	//
	//	//�����ļ���;
	//	if (!exists(folderName))
	//	{
	//		create_directory(folderName);
	//	}
	//
	//	if (fileNames.empty())
	//	{
	//		return 0;
	//	}
	//	else
	//	{
	//		for (size_t i = 0; i < fileNames.size(); ++i)
	//		{
	//			path dir(fileNames[i]);
	//			string fileName;
	//			//��ȡ������·���ͺ�׺���ļ���;
	//			fileName = dir.stem().string();
	//			fileName = folderName + "\\" + fileName + ".las";
	//			writeLasFile(fileName, pointClouds[i]);
	//		}
	//	}
	//
	//	return 1;
	//}

	bool DataIo::batchReadFileNamesInFolders(const std::string &folderName, const std::string & extension,
		std::vector<std::string> &fileNames)
	{
		//����boost�����ļ����ڵ��ļ�;
		if (!exists(folderName))
		{
			return 0;
		}
		else
		{
			//�������ļ����µ�����Ϊextension���ļ�,�����浽fileNames��;
			directory_iterator end_iter;
			for (directory_iterator iter(folderName); iter != end_iter; ++iter)
			{
				if (is_regular_file(iter->status()))
				{
					string fileName;
					fileName = iter->path().string();

					path dir(fileName);

					if (!dir.extension().string().empty())
					{
						if (!fileName.substr(fileName.rfind('.')).compare(extension))
						{
							fileNames.push_back(fileName);
						}
					}
				}
			}
		}
		return 1;
	}

	bool DataIo::batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName,//�ļ���·��;
		const std::string & extension,//Ŀ���ļ�����չ��;
		std::vector<std::string> &fileNames)//����Ŀ���ļ���·��+�ļ���+��չ��;
	{
		boost::filesystem::path  fullpath(folderName);

		if (!exists(fullpath))
		{
			return false;
		}

		recursive_directory_iterator end_iter;
		for (recursive_directory_iterator iter(fullpath); iter != end_iter; iter++)
		{
			try
			{
				if (is_directory(*iter))
				{
				}
				else
				{
					std::string sFileName = iter->path().string();
					path dir(sFileName);

					if (!dir.extension().string().empty())
					{
						if (!sFileName.substr(sFileName.rfind('.')).compare(extension))
						{
							fileNames.push_back(sFileName);
						}
					}

				}
			}
			catch (const std::exception & ex)
			{
				std::cerr << ex.what() << std::endl;
				continue;
			}
		}
		return true;
	}

	bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
	{
		pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS); //Ban pcl warnings
		
		if (pcl::io::loadPCDFile(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}

		return true;
	}

	bool DataIo::writePcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
	{
		pointCloud->width = pointCloud->points.size ();
		pointCloud->height = 1;
		if (pcl::io::savePCDFileBinary<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		return true;
	}

	bool DataIo::writePcdFile(const string &fileName, const pcXYZRGBPtr &pointCloud)
	{
		pointCloud->width = pointCloud->points.size ();
		pointCloud->height = 1;
		if (pcl::io::savePCDFileBinary<pcl::PointXYZRGB>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		return true;
	}

	bool DataIo::writePcdAll(const string &foldername, const string &fileName, const pcXYZIPtr &NGCloud, const pcXYZIPtr &GCloud, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings)
	{
		vector<pcXYZRGB> pointCloudsRGB;
		pointCloudsRGB.resize(pointClouds.size());

		//�����ɫ;
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			int R, G, B;

			switch (roadmarkings[i].category)
			{
			case 1:
				R = 255;
				G = 0;
				B = 0;
				break;
			case 2:
				R = 0;
				G = 255;
				B = 0;
				break;
			case 3:
				R = 0;
				G = 0;
				B = 255;
				break;
			case 4:
				R = 255;
				G = 0;
				B = 255;
				break;
			case 5:
				R = 0;
				G = 255;
				B = 255;
				break;
			default:
				R = 255;
				G = 255;
				B = 0;
				break;
			}
			for (size_t j = 0; j < pointClouds[i].size(); j++)
			{
				pcl::PointXYZRGB pt;
				pt.x = pointClouds[i].points[j].x;
				pt.y = pointClouds[i].points[j].y;
				pt.z = pointClouds[i].points[j].z;
				pt.r = R;
				pt.g = G;
				pt.b = B;
				pointCloudsRGB[i].push_back(pt);
			}
		}

		if (!boost::filesystem::exists(foldername))
		{
			boost::filesystem::create_directory(foldername);
		}

		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss;
			oss << i << "_" << fileName;
			// outputFileName = foldername + "\\" + oss.str(); // Windows
			outputFileName = foldername + "/" + oss.str(); // Windows

			writePcdFile(outputFileName, pointCloudsRGB[i].makeShared());
		}
		string NGCloud_Filename, GCloud_Filename;

		// NGCloud_Filename = foldername + "\\Non_Ground_Cloud.pcd"; // Windows
		// GCloud_Filename = foldername + "\\Ground_Cloud.pcd"; // Windows
		NGCloud_Filename = foldername + "/Non_Ground_Cloud.pcd"; // Windows
		GCloud_Filename = foldername + "/Ground_Cloud.pcd"; // Windows

		writePcdFile(NGCloud_Filename, NGCloud);
		writePcdFile(GCloud_Filename, GCloud);

		//cout << "Output Done" << endl;
		return true;
	}


	bool DataIo::writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings, double minX, double minY)
	{
		vector<pcXYZRGB> pointCloudsRGB;
		pointCloudsRGB.resize(pointClouds.size());

		//�����ɫ;
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			int R, G, B;

			switch (roadmarkings[i].category)
			{
			case 1:
				R = 255;
				G = 0;
				B = 0;
				break;
			case 2:
				R = 0;
				G = 255;
				B = 0;
				break;
			case 3:
				R = 0;
				G = 0;
				B = 255;
				break;
			case 4:
				R = 255;
				G = 0;
				B = 255;
				break;
			case 5:
				R = 0;
				G = 255;
				B = 255;
				break;
			default:
				R = 255;
				G = 255;
				B = 0;
				break;
			}
			for (size_t j = 0; j < pointClouds[i].size(); j++)
			{
				pcl::PointXYZRGB pt;
				pt.x = pointClouds[i].points[j].x + minX;
				pt.y = pointClouds[i].points[j].y + minY;
				pt.z = pointClouds[i].points[j].z;
				pt.r = R;
				pt.g = G;
				pt.b = B;
				pointCloudsRGB[i].push_back(pt);
			}
		}

		if (!boost::filesystem::exists(foldername))
		{
			boost::filesystem::create_directory(foldername);
		}

		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss;
			oss << i << "_" << fileName;
			outputFileName = foldername + "\\" + oss.str();

			writePcdFile(outputFileName, pointCloudsRGB[i].makeShared());
		}

		//cout << "Output Done" << endl;
		return true;
	}

	bool DataIo::writePcdAll(const string &foldername, const string &fileName, const vector<pcXYZI> &pointClouds, const RoadMarkings &roadmarkings)
	{
		vector<pcXYZRGB> pointCloudsRGB;
		pointCloudsRGB.resize(pointClouds.size());

		//�����ɫ;
		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			int R, G, B;

			switch (roadmarkings[i].category)
			{
			case 1:
				R = 0;
				G = 255;
				B = 0;
				break;
			case 2:
				R = 255;
				G = 0;
				B = 0;
				break;
			case 3:
				R = 0;
				G = 0;
				B = 255;
				break;
			case 4:
				R = 255;
				G = 0;
				B = 255;
				break;
			case 5:
				R = 0;
				G = 255;
				B = 255;
				break;
			default:
				R = 255;
				G = 255;
				B = 0;
				break;
			}
			for (size_t j = 0; j < pointClouds[i].size(); j++)
			{
				pcl::PointXYZRGB pt;
				pt.x = pointClouds[i].points[j].x;
				pt.y = pointClouds[i].points[j].y;
				pt.z = pointClouds[i].points[j].z;
				pt.r = R;
				pt.g = G;
				pt.b = B;
				pointCloudsRGB[i].push_back(pt);
			}
		}

		if (!boost::filesystem::exists(foldername))
		{
			boost::filesystem::create_directory(foldername);
		}

		for (size_t i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss;
			oss << i << "_" << fileName;
			outputFileName = foldername + "\\" + oss.str();

			writePcdFile(outputFileName, pointCloudsRGB[i].makeShared());
		}

		//cout << "Output Done" << endl;
		return true;
	}

	bool DataIo::writePcdAll(const string &folderName, const string &fileName, const vector<pcXYZI> &pointClouds) {

		if (!boost::filesystem::exists(folderName))
		{
			boost::filesystem::create_directory(folderName);
		}

		for (int i = 0; i < pointClouds.size(); i++)
		{
			string  outputFileName;
			ostringstream oss;
			oss << setw(4) << setfill('0') << i << "_" << fileName;
			// outputFileName = folderName + "\\" + oss.str(); // Windows
			outputFileName = folderName + "/" + oss.str(); // Ubuntu
			if (pointClouds[i].makeShared()->points.size() < 100) {
				//cout << "Warning: the" << i << "th point cloud contain point less than the threshold. Do not save it." << endl;
				continue;
			}
			writePcdFile(outputFileName, pointClouds[i].makeShared());
		}

		//cout << "Output Done" << endl;
		return true;

	}

	// ============================ LIBLAS =============================
	void DataIo::displayroad(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloud)
	{	
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(255, 255, 255);
		char t[256];
		string s;
		int n = 0;
		float maxi,maxvi,mini,max_z,min_z;
		maxi = -FLT_MAX;
		mini = FLT_MAX;
		max_z = -FLT_MAX;
		min_z = FLT_MAX;
	
		int startcolor[3];
		int endcolor[3];
		int rr, gg, bb;
		float kk;
	
		//cout << "Input the color scale that you want to render the point cloud." << endl;
		//cout << "Start Color R,G,B:" << endl;
	
		//cin >> startcolor[0] >> startcolor[1] >> startcolor[2];
	
		//cout << "End Color R,G,B:" << endl;
		//cin >> endcolor[0] >> endcolor[1] >> endcolor[2];
		
	
		startcolor[0] = 255; startcolor[1] = 0; startcolor[2] = 0;
		endcolor[0] = 0; endcolor[1] = 255; endcolor[2] = 0;
	
		rr = startcolor[0] - endcolor[0];
		gg = startcolor[1] - endcolor[1];
		bb = startcolor[2] - endcolor[2];
	
		//maxvi = 10000;
		pcXYZRGBPtr GC(new pcXYZRGB());
		pcXYZRGBPtr NGC(new pcXYZRGB());
	
		for (size_t i = 0; i < ngcloud->points.size(); ++i)
		{
			if (ngcloud->points[i].z > max_z) max_z = ngcloud->points[i].z;
			if (ngcloud->points[i].z < min_z) min_z = ngcloud->points[i].z;
		}
	
	
		for (size_t i = 0; i < gcloud->points.size(); ++i)
		{
			if (gcloud->points[i].intensity > maxi) maxi = gcloud->points[i].intensity;
			if (gcloud->points[i].intensity < mini) mini = gcloud->points[i].intensity;
		}
	
	
		//cout << "max intensity: " << maxi << "\tmin intensity: " << mini<<endl;
	
		for (size_t i = 0; i < ngcloud->points.size(); ++i)
		{
			pcl::PointXYZRGB pt;
			pt.x = ngcloud->points[i].x;
			pt.y = ngcloud->points[i].y;
			pt.z = ngcloud->points[i].z;
			
			kk = (ngcloud->points[i].z - min_z) / (max_z - min_z);
			pt.r = endcolor[0] + rr * kk;
			pt.g = endcolor[1] + gg * kk;
			pt.b = endcolor[2] + bb * kk;
			NGC->points.push_back(pt);
		}
		
		viewer->addPointCloud(NGC, "Non-ground");
	
		// Ground points are rendered in intensity
		for (size_t i = 0; i < gcloud->points.size(); ++i)
		{
			pcl::PointXYZRGB pt;
			pt.x = gcloud->points[i].x;
			pt.y = gcloud->points[i].y;
			pt.z = gcloud->points[i].z;
			/*if (gcloud->points[i].intensity<maxvi)
			{
				pt.r = 255;
				pt.g = 255;
				pt.b = 255;
			}
			else{
				pt.r = 0;
				pt.g = 0;
				pt.b = 0;
			}*/
			//pt.r = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);
			//pt.g = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);
			//pt.b = (int)((min(gcloud->points[i].intensity, maxvi) - mini) / (maxvi - mini) * 255);
	
			pt.r = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
			pt.g = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
			pt.b = 255 * (gcloud->points[i].intensity - mini) / (maxi - mini);
			GC->points.push_back(pt);
		}
	
		viewer->addPointCloud(GC, "Ground");
	
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
	
	void DataIo::displaymark(const vector<pcXYZI> &clouds){
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(255, 255, 255);
		char t[256];
		string s;
		int n = 0;
	
		vector<pcXYZRGB>  colorclouds;
		colorclouds.resize(clouds.size());
		for (int i = 0; i <clouds.size(); ++i)
		{
			int r = 255 * (rand() / (1.0 + RAND_MAX));
			int g = 255 * (rand() / (1.0 + RAND_MAX));
			int b = 255 * (rand() / (1.0 + RAND_MAX));
	
			for (size_t j = 0; j < clouds[i].points.size(); ++j)
			{
				pcl::PointXYZRGB pt;
				pt.x = clouds[i].points[j].x;
				pt.y = clouds[i].points[j].y;
				pt.z = clouds[i].points[j].z; 
				//�����ɫ
				
			    pt.r = r;
				pt.g = g;
				pt.b = b;
				
				colorclouds[i].points.push_back(pt);
	
			}
			string  colorcloud;
			ostringstream oss;
			oss << i <<  "_colorcloud";
			colorcloud = oss.str();
	
			viewer->addPointCloud(colorclouds[i].makeShared(),colorcloud);
	
		}	
	
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
	
	void DataIo::displaymarkwithng(const vector<pcXYZI> &clouds, const pcXYZIPtr &ngcloud)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(255, 255, 255);
		char t[256];
		string s;
		int n = 0;
	
		vector<pcXYZRGB>  colorclouds;
		colorclouds.resize(clouds.size());
		for (int i = 0; i < clouds.size(); ++i)
		{
			int r = 255 * (rand() / (1.0 + RAND_MAX));
			int g = 255 * (rand() / (1.0 + RAND_MAX));
			int b = 255 * (rand() / (1.0 + RAND_MAX));
	
			for (size_t j = 0; j < clouds[i].points.size(); ++j)
			{
				pcl::PointXYZRGB pt;
				pt.x = clouds[i].points[j].x;
				pt.y = clouds[i].points[j].y;
				pt.z = clouds[i].points[j].z;
				//�����ɫ
	
				pt.r = r;
				pt.g = g;
				pt.b = b;
	
				colorclouds[i].points.push_back(pt);
	
			}
			string  colorcloud;
			ostringstream oss;
			oss << i << "_colorcloud";
			colorcloud = oss.str();
	
			viewer->addPointCloud(colorclouds[i].makeShared(), colorcloud);
	
		}
	
		float  max_z, min_z;
		max_z = -FLT_MAX;
		min_z = FLT_MAX;
	
		int startcolor[3];
		int endcolor[3];
		int rr, gg, bb;
		float kk;
	
		startcolor[0] = 255; startcolor[1] = 0; startcolor[2] = 0;
		endcolor[0] = 0; endcolor[1] = 255; endcolor[2] = 0;
	
		rr = startcolor[0] - endcolor[0];
		gg = startcolor[1] - endcolor[1];
		bb = startcolor[2] - endcolor[2];
	
		pcXYZRGBPtr NGC(new pcXYZRGB());
	
		for (size_t i = 0; i < ngcloud->points.size(); ++i)
		{
			if (ngcloud->points[i].z > max_z) max_z = ngcloud->points[i].z;
			if (ngcloud->points[i].z < min_z) min_z = ngcloud->points[i].z;
		}
	
		for (size_t i = 0; i < ngcloud->points.size(); ++i)
		{
			pcl::PointXYZRGB pt;
			pt.x = ngcloud->points[i].x;
			pt.y = ngcloud->points[i].y;
			pt.z = ngcloud->points[i].z;
	
			kk = (ngcloud->points[i].z - min_z) / (max_z - min_z);
			pt.r = endcolor[0] + rr * kk;
			pt.g = endcolor[1] + gg * kk;
			pt.b = endcolor[2] + bb * kk;
			NGC->points.push_back(pt);
		}
	
		viewer->addPointCloud(NGC, "Non-ground");
	
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
	
	void DataIo::displaymarkbycategory(const vector<pcXYZI> &clouds, const RoadMarkings &roadmarkings)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		char t[256];
		string s;
		int n = 0;
	
		vector<pcXYZRGB>  colorclouds;
		colorclouds.resize(clouds.size());
		for (int i = 0; i < clouds.size(); ++i)
		{
			int r, g, b;
			switch (roadmarkings[i].category)
			{
			case 1:  //������    //��    //ǳ�� 135 206 250
				r = 0;
				g = 255;
				b = 0;
				break;
			
			case 2:  //������   //��    //õ��� 255 48 48
				r = 255;
				g = 0;
				b = 0;
				break;
	
			case 3:  //ǰ���ͷ  //��     //��ɫ 238 130 238
				r = 0;
				g = 0;
				b = 255;
				break;
			
			case 4:  //ǰ�����ͷ  //��   //ǳ�� 193 255 193
				r = 0;
				g = 255;
				b = 255;
				break;
	
			case 5:  //�����ͷ   //��   //�ۺ� 255 193 193
				r = 255;
				g = 0;
				b = 255;
				break;
	
			case 6: //��ͷ��ͷ  //��
				r = 255;
				g = 255;
				b = 0;
				break;
	
			case 8: //���α�־ //��
				r = 255;
				g = 170;
				b = 0;
				break;
	
			default:
				r = 255;
				g = 250;
				b = 250;
				break;
	
			}
			   
			for (size_t j = 0; j < clouds[i].points.size(); ++j)
			{
				pcl::PointXYZRGB pt;
				pt.x = clouds[i].points[j].x;
				pt.y = clouds[i].points[j].y;
				pt.z = clouds[i].points[j].z;
	
				pt.r = r;
				pt.g = g;
				pt.b = b;
	
				colorclouds[i].points.push_back(pt);
	
			}
			
			string  colorcloud;
			ostringstream oss;
			oss << i << "_colorcloud";
			colorcloud = oss.str();
	
			viewer->addPointCloud(colorclouds[i].makeShared(), colorcloud);
		}
	
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
	void DataIo::displaymarkVect(const RoadMarkings &roadmarkings)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vectorization Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		char t[256];
		string s;
		int n = 0;
	
		for (int i = 0; i < roadmarkings.size(); i++)
		{
			switch (roadmarkings[i].category)
			{
			case 1:  //������
				for (int j = 0; j < roadmarkings[i].polyline.size()-1; j++)
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 0.0, 1.0, 0.0, s);
					n++;
	
					pcl::PointXYZ pt2;
			        pt2.x = roadmarkings[i].polyline[j + 1].x;
				    pt2.y = roadmarkings[i].polyline[j + 1].y;
				    pt2.z = roadmarkings[i].polyline[j + 1].z;
				
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 0.0, 1.0, 0.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 0.0, 1.0, 0.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 2:
				for (int j = 0; j < 4; j++) //�������
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 0.0, 0.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 3){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 0.0, 0.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
	
			case 3:
				for (int j = 0; j < 7; j++) //ֱ��ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 0.0, 0.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 6){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 0.0, 0.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 0.0, 0.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 4:
				for (int j = 0; j < 14; j++) //ֱ�Ҽ�ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 0.0, 1.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 13){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 0.0, 1.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 0.0, 1.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 5:
				for (int j = 0; j < 9; j++) //�Ҽ�ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 0.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 8){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 0.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 0.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 8:
				for (int j = 0; j < 4; j++) //���α���
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 1.0, 0.0, s); //��
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 3){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 1.0, 0.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 1.0, 0.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			default:
				break;
	
			}
		}
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	
	}
	
	void DataIo::displaymarkVect(const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vectorization Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		char t[256];
		string s;
		int n = 0;
	
		for (int i = 0; i < sideline_markings.size(); i++)//������
		{
			for (int j = 0; j <sideline_markings[i].polyline.size() - 1; j++)
			{
				pcl::PointXYZ pt1;
				pt1.x = sideline_markings[i].polyline[j].x;
				pt1.y = sideline_markings[i].polyline[j].y;
				pt1.z = sideline_markings[i].polyline[j].z;
				sprintf(t, "%d", n);
				s = t;
				viewer->addSphere(pt1, 0.1, 0.0, 1.0, 0.0, s);
				n++;
	
				pcl::PointXYZ pt2;
				pt2.x = sideline_markings[i].polyline[j + 1].x;
				pt2.y = sideline_markings[i].polyline[j + 1].y;
				pt2.z = sideline_markings[i].polyline[j + 1].z;
	
				sprintf(t, "%d", n);
				s = t;
				viewer->addSphere(pt2, 0.1, 0.0, 1.0, 0.0, s);
				n++;
	
				sprintf(t, "%d", n);
				s = t;
				viewer->addLine(pt1, pt2, 0.0, 1.0, 0.0, s);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
				n++;
			}
		}
	
		for (int i = 0; i < roadmarkings.size(); i++)
		{
			switch (roadmarkings[i].category)
			{
			case 2:
				for (int j = 0; j < 4; j++) //�������
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 0.0, 0.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 3){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 0.0, 0.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
	
			case 3:
				for (int j = 0; j < 7; j++) //ֱ��ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 0.0, 0.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 6){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 0.0, 0.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 0.0, 0.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 4:
				for (int j = 0; j < 14; j++) //ֱ�Ҽ�ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 0.0, 1.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 13){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 0.0, 1.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 0.0, 1.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 5:
				for (int j = 0; j < 9; j++) //�Ҽ�ͷ
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 0.0, 1.0, s);
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 8){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 0.0, 1.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 0.0, 1.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
			case 8:
				for (int j = 0; j < 4; j++) //���α���
				{
					pcl::PointXYZ pt1;
					pt1.x = roadmarkings[i].polyline[j].x;
					pt1.y = roadmarkings[i].polyline[j].y;
					pt1.z = roadmarkings[i].polyline[j].z;
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt1, 0.1, 1.0, 1.0, 0.0, s); //��
					n++;
	
	
					pcl::PointXYZ pt2;
					if (j == 3){
						pt2.x = roadmarkings[i].polyline[0].x;
						pt2.y = roadmarkings[i].polyline[0].y;
						pt2.z = roadmarkings[i].polyline[0].z;
					}
					else{
						pt2.x = roadmarkings[i].polyline[j + 1].x;
						pt2.y = roadmarkings[i].polyline[j + 1].y;
						pt2.z = roadmarkings[i].polyline[j + 1].z;
					}
					sprintf(t, "%d", n);
					s = t;
					viewer->addSphere(pt2, 0.1, 1.0, 1.0, 0.0, s);
					n++;
	
					sprintf(t, "%d", n);
					s = t;
					viewer->addLine(pt1, pt2, 1.0, 1.0, 0.0, s);
					viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
					n++;
				}
	
				break;
	
			default:
				break;
	
	
			}
		}
		cout << "Click X(close) to continue..." << endl;
		while (!viewer->wasStopped())
		{
			viewer->spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}

	bool DataIo::readLasFileHeader(const std::string &fileName, liblas::Header& header)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}
		else
		{
			std::ifstream ifs;
			ifs.open(fileName, std::ios::in | std::ios::binary);
			if (ifs.bad())
			{
				return 0;
			}
	
			liblas::ReaderFactory f;
			liblas::Reader reader = f.CreateWithStream(ifs);
	
			header = reader.GetHeader();
		}
	
		return 1;
	}

	bool DataIo::writemarkVectDXF(const RoadMarkings &roadmarkings)
	{
		
		DL_Dxf dxf;
		DL_WriterA* dw = dxf.out("DXFTest.dxf", DL_Codes::AC1015);
	
		// section header:
		dxf.writeHeader(*dw);
		dw->sectionEnd();
	
		// section tables:
		dw->sectionTables();
	
		// VPORT:
		dxf.writeVPort(*dw);
	
		// LTYPE:
		dw->tableLinetypes(3);
		dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
		dw->tableEnd();
	
		// LAYER:
		dw->tableLayers(5);
		
		dxf.writeLayer(
			*dw,
			DL_LayerData("0", 0),
			DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS")//15 �߿�  ��ɫ  ��׼ͼ��
			);
		
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Side Lines", 0),
			DL_Attributes("", 1, 0x0000ff00, 15, "CONTINUOUS") //15 �߿�  0x00RRGGBB  ��ɫ  ��16���ƣ� 0x0000ff00Ϊ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Rectangle Road Markings", 0),
			DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS") // ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Arrows", 0),
			DL_Attributes("", 1, 0x000000ff, 15, "CONTINUOUS") // ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Others", 0),
			DL_Attributes("", 1, 0x00ffff00, 15, "CONTINUOUS") // ��ɫ
			);
	
		dw->tableEnd();
	
		// STYLE:
		dw->tableStyle(1);
		DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
		style.bold = false;
		style.italic = false;
		dxf.writeStyle(*dw, style);
		dw->tableEnd();
	
		// VIEW:
		dxf.writeView(*dw);
	
		// UCS:
		dxf.writeUcs(*dw);
	
		// APPID:
		dw->tableAppid(1);
		dxf.writeAppid(*dw, "ACAD");
		dw->tableEnd();
	
		// DIMSTYLE:
		dxf.writeDimStyle(*dw, 2.5, 0.625, 0.625, 0.625, 2.5);
	
		// BLOCK_RECORD:
		dxf.writeBlockRecord(*dw);
		dw->tableEnd();
	
		dw->sectionEnd();
	
		// BLOCK:
		dw->sectionBlocks();
		dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Model_Space");
		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space");
		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space0");
		dw->sectionEnd();
	
		// ENTITIES:
		dw->sectionEntities();
	
		DL_Attributes attributes("0", 256, 0x00000000, 5, "BYLAYER");
		DL_Attributes attributes_side_line("Side Lines", 256, 0x0000ff00, 5, "BYLAYER"); //��
		DL_Attributes attributes_rectangle("Rectangle Road Markings", 256, 0x00ff0000, 5, "BYLAYER");  //��
		DL_Attributes attributes_arrow("Arrows", 256, 0x000000ff, 5, "BYLAYER");  //��
		DL_Attributes attributes_others("Others", 256, 0x00ffff00, 5, "BYLAYER");  //��
		/*
		// LINE:
		DL_LineData lineData(10, 5, 0, 30, 5, 0);
		dxf.writeLine(*dw, lineData, attributes);
	
		// CIRCLE
		DL_CircleData circleData(10, 10, 0, 4);
		dxf.writeCircle(*dw, circleData, attributes);
		*/
		bool plineGen = true;
		//bool Isclosed;
		for (int i = 0; i < roadmarkings.size(); i++)
		{
			if (roadmarkings[i].category >= 1){
	
				int count = roadmarkings[i].polyline.size();
	
				//�Ƿ�պϣ�1�೤���߲��ñպ�
				switch (roadmarkings[i].category)
				{
				case 1:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						false * 0x1 + plineGen * 0x80),
						attributes_side_line);
					break;
				case 2:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_rectangle);
					break;
				case 3:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 4:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 5:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 6:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 8:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_others);
					break;
				
				default:
					break;
				}
				
				
				//���긳ֵ
				for (int j = 0; j < count; j++) {
				
					double bulge = 0;
					dxf.writeVertex(*dw, DL_VertexData(roadmarkings[i].polyline[j].x, roadmarkings[i].polyline[j].y, roadmarkings[i].polyline[j].z, bulge));
				}
	
				dxf.writePolylineEnd(*dw);
			}
		}
		
		// end section ENTITIES:
		dw->sectionEnd();
		dxf.writeObjects(*dw, "MY_OBJECTS");
		dxf.writeObjectsEnd(*dw);
	
		dw->dxfEOF();
		dw->close();
		delete dw;
	
	cout << "Output DXF done" << endl;
	return true;
	}
	
	bool DataIo::writemarkVectDXF(int file_index, const std::string & foldername, const RoadMarkings &roadmarkings, const RoadMarkings &sideline_markings, 
	     double minX, double minY)
	{
		std::string dxf_filename = foldername + ".dxf";
	
	
		DL_Dxf dxf;
		DL_WriterA* dw = dxf.out(dxf_filename.data(), DL_Codes::AC1015);
	
		// section header:
		dxf.writeHeader(*dw);
		dw->sectionEnd();
	
		// section tables:
		dw->sectionTables();
	
		// VPORT:
		dxf.writeVPort(*dw);
	
		// LTYPE:
		dw->tableLinetypes(3);
		dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
		dw->tableEnd();
	
		// LAYER:
		dw->tableLayers(5);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("0", 0),
			DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS")//15 �߿�  ��ɫ  ��׼ͼ��
			);
	
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Side Lines", 0),
			DL_Attributes("", 1, 0x0000ff00, 15, "CONTINUOUS") //15 �߿�  0x00RRGGBB  ��ɫ  ��16���ƣ� 0x0000ff00Ϊ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Rectangle Road Markings", 0),
			DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS") // ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Arrows", 0),
			DL_Attributes("", 1, 0x000000ff, 15, "CONTINUOUS") // ��ɫ
			);
	
		dxf.writeLayer(
			*dw,
			DL_LayerData("Others", 0),
			DL_Attributes("", 1, 0x00ffff00, 15, "CONTINUOUS") // ��ɫ
			);
		dw->tableEnd();
	
		// STYLE:
		dw->tableStyle(1);
		DL_StyleData style("Standard", 0, 0.0, 1.0, 0.0, 0, 2.5, "txt", "");
		style.bold = false;
		style.italic = false;
		dxf.writeStyle(*dw, style);
		dw->tableEnd();
	
		// VIEW:
		dxf.writeView(*dw);
	
		// UCS:
		dxf.writeUcs(*dw);
	
		// APPID:
		dw->tableAppid(1);
		dxf.writeAppid(*dw, "ACAD");
		dw->tableEnd();
	
		// DIMSTYLE:
		dxf.writeDimStyle(*dw, 2.5, 0.625, 0.625, 0.625, 2.5);
	
		// BLOCK_RECORD:
		dxf.writeBlockRecord(*dw);
		dw->tableEnd();
	
		dw->sectionEnd();
	
		// BLOCK:
		dw->sectionBlocks();
		dxf.writeBlock(*dw, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Model_Space");
		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space");
		dxf.writeBlock(*dw, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
		dxf.writeEndBlock(*dw, "*Paper_Space0");
		dw->sectionEnd();
	
		// ENTITIES:
		dw->sectionEntities();
	
		DL_Attributes attributes("0", 256, 0x00000000, 5, "BYLAYER");
		DL_Attributes attributes_side_line("Side Lines", 256, 0x0000ff00, 5, "BYLAYER"); //��
		DL_Attributes attributes_rectangle("Rectangle Road Markings", 256, 0x00ff0000, 5, "BYLAYER");  //��
		DL_Attributes attributes_arrow("Arrows", 256, 0x000000ff, 5, "BYLAYER");  //��
		DL_Attributes attributes_others("Others", 256, 0x00ffff00, 5, "BYLAYER");  //��
	
		/*
		// LINE:
		DL_LineData lineData(10, 5, 0, 30, 5, 0);
		dxf.writeLine(*dw, lineData, attributes);
	
		// CIRCLE
		DL_CircleData circleData(10, 10, 0, 4);
		dxf.writeCircle(*dw, circleData, attributes);
		*/
		bool plineGen = true;
		//bool Isclosed;
		
		for (int i = 0; i < sideline_markings.size(); i++)
		{
			
			dxf.writePolyline(
				*dw,
				DL_PolylineData(sideline_markings[i].polyline.size(), 0, 0, false * 0x1 + plineGen * 0x80),
				attributes_side_line);
	
	
			for (int j = 0; j < sideline_markings[i].polyline.size(); j++) {
	
				double bulge = 0;
				dxf.writeVertex(*dw, DL_VertexData(sideline_markings[i].polyline[j].x + minX, sideline_markings[i].polyline[j].y + minY, sideline_markings[i].polyline[j].z, bulge));
			}
	
			dxf.writePolylineEnd(*dw);
		}
	
		for (int i = 0; i < roadmarkings.size(); i++)
		{
			if (roadmarkings[i].category >= 2){
	
				int count = roadmarkings[i].polyline.size();
	
				//�Ƿ�պϣ�1�೤���߲��ñպ�
				switch (roadmarkings[i].category)
				{
				case 2:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_rectangle);
					break;
				case 3:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 4:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 5:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 6:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_arrow);
					break;
				case 8:
					dxf.writePolyline(
						*dw,
						DL_PolylineData(count,
						0, 0,
						true * 0x1 + plineGen * 0x80),
						attributes_others);
					break;
				default:
					break;
				}
	
	
				//���긳ֵ
				for (int j = 0; j < count; j++) {
	
					double bulge = 0;
					dxf.writeVertex(*dw, DL_VertexData(roadmarkings[i].polyline[j].x + minX, roadmarkings[i].polyline[j].y + minY, roadmarkings[i].polyline[j].z, bulge));
				}
	
				dxf.writePolylineEnd(*dw);
			}
		}
	
		// end section ENTITIES:
		dw->sectionEnd();
		dxf.writeObjects(*dw, "MY_OBJECTS");
		dxf.writeObjectsEnd(*dw);
	
		dw->dxfEOF();
		dw->close();
		delete dw;
		cout << "Output DXF file done" << endl;
		return true;
	
	}


	bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}
	
		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "δ����ƥ����" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const& header = reader.GetHeader();
	
		//header�����ֱ����bounding box ����
		double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();
	
		//int i = 0;
		/*whileѭ���б������еĵ�;*/
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			//i++;
			pcl::PointXYZI  pt;
			/*�����Ļ���������ǿ��ֵ��ֵ��PCL�еĵ�;*/
			/*��һ��ƽ�ƣ�������WGS84 �µĵ�����̫���ˣ�����ɾ�����ʧ�� ��Ϊlas�Ķ�ȡ��������double�ģ���pcd��int��*/
			pt.x = p.GetX() - Xmin;
			pt.y = p.GetY() - Ymin;
			pt.z = p.GetZ();
			pt.intensity = p.GetIntensity();
			//if (i % 100 == 0) { cout << p.GetPointSourceID() << "\t" << p.GetTime() << endl; } //��ЩPointSourceID �� Time ��Ϣ������ScanLine Profile ����ɨ���ߣ��������·�棬����Ч���е㿰��
			pointCloud.points.push_back(pt);
		}
	
		return 1;
	}
	
	// bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, vector<double> & boundingbox,double & X_min,double & Y_min)
	bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds & bound_3d)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}
	
		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "δ����ƥ����" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const& header = reader.GetHeader();
		
		//header�����ֱ����bounding box ����
		double Xmin, Ymin, Zmin, Xmax, Ymax,Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();

		bound_3d.min_x = Xmin;bound_3d.min_y = Ymin;bound_3d.min_z = Zmin;
		bound_3d.max_x = Xmax;bound_3d.max_y = Ymax;bound_3d.max_z = Zmax;

	    float center_x = bound_3d.min_x / 2.0f + bound_3d.max_x / 2.0f;
	    float center_y = bound_3d.min_y / 2.0f + bound_3d.max_y / 2.0f;
	    float center_z = bound_3d.min_z / 2.0f + bound_3d.max_z / 2.0f;
	
		/*whileѭ���б������еĵ�;*/
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			//i++;
			pcl::PointXYZI  pt;
			/*�����Ļ���������ǿ��ֵ��ֵ��PCL�еĵ�;*/
			/*��һ��ƽ�ƣ�������WGS84 �µĵ�����̫���ˣ�����ɾ�����ʧ�� ��Ϊlas�Ķ�ȡ��������double�ģ���pcd��int��*/
			pt.x = p.GetX()-center_x;
			pt.y = p.GetY()-center_y;  
			pt.z = p.GetZ();
			pt.intensity = p.GetIntensity();

			// print intentisy every 1 000 000
			//if (i % 1000000 == 0) { cout << pt.intensity << endl; } //��ЩPointSourceID �� Time ��Ϣ������ScanLine Profile ����ɨ���ߣ��������·�棬����Ч���е㿰��
			pointCloud.points.push_back(pt);
		}

		return 1;
	}

	void DataIo::getCloudBound(const pcXYZIPtr &cloud, Bounds & bound)
	{
		double min_x = cloud->points[0].x;
		double min_y = cloud->points[0].y;
		double min_z = cloud->points[0].z;
		double max_x = cloud->points[0].x;
		double max_y = cloud->points[0].y;
		double max_z = cloud->points[0].z;

		for (int i = 0; i < cloud->size(); i++)
		{
			//��ȡ�߽�
			if (min_x > cloud->points[i].x)
				min_x = cloud->points[i].x;
			if (min_y > cloud->points[i].y)
				min_y = cloud->points[i].y;
			if (min_z > cloud->points[i].z)
				min_z = cloud->points[i].z;
			if (max_x < cloud->points[i].x)
				max_x = cloud->points[i].x;
			if (max_y < cloud->points[i].y)
				max_y = cloud->points[i].y;
			if (max_z < cloud->points[i].z)
				max_z = cloud->points[i].z;
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;
	}

		bool DataIo::GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZI> &pointCloud, Bounds &bound)
	{
		if (pointCloud.points.empty())
		{
			return 0;
		}
		else
		{
			double min_x = pointCloud.points[0].x;
			double min_y = pointCloud.points[0].y;
			double min_z = pointCloud.points[0].z;
			double max_x = pointCloud.points[0].x;
			double max_y = pointCloud.points[0].y;
			double max_z = pointCloud.points[0].z;

			for (int i = 0; i < pointCloud.points.size(); i++)
			{
				//��ȡ�߽�
				if (min_x > pointCloud.points[i].x)
					min_x = pointCloud.points[i].x;
				if (min_y > pointCloud.points[i].y)
					min_y = pointCloud.points[i].y;
				if (min_z > pointCloud.points[i].z)
					min_z = pointCloud.points[i].z;
				if (max_x < pointCloud.points[i].x)
					max_x = pointCloud.points[i].x;
				if (max_y < pointCloud.points[i].y)
					max_y = pointCloud.points[i].y;
				if (max_z < pointCloud.points[i].z)
					max_z = pointCloud.points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}
		return 1;
	}


	bool DataIo::GetBoundaryOfPointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, Bounds &bound)

	{
		if (pointCloud.points.empty())
		{
			return 0;
		}
		else
		{
			double min_x = pointCloud.points[0].x;
			double min_y = pointCloud.points[0].y;
			double min_z = pointCloud.points[0].z;
			double max_x = pointCloud.points[0].x;
			double max_y = pointCloud.points[0].y;
			double max_z = pointCloud.points[0].z;

			for (int i = 0; i < pointCloud.points.size(); i++)
			{
				//��ȡ�߽�
				if (min_x > pointCloud.points[i].x)
					min_x = pointCloud.points[i].x;
				if (min_y > pointCloud.points[i].y)
					min_y = pointCloud.points[i].y;
				if (min_z > pointCloud.points[i].z)
					min_z = pointCloud.points[i].z;
				if (max_x < pointCloud.points[i].x)
					max_x = pointCloud.points[i].x;
				if (max_y < pointCloud.points[i].y)
					max_y = pointCloud.points[i].y;
				if (max_z < pointCloud.points[i].z)
					max_z = pointCloud.points[i].z;
			}
			bound.min_x = min_x;
			bound.max_x = max_x;
			bound.min_y = min_y;
			bound.max_y = max_y;
			bound.min_z = min_z;
			bound.max_z = max_z;
		}
		return 1;
	}

#if 0
	// ============================ LASLIB IO=============================
	// LASLIB is similar to LibLas
	// You need to install LASLIB instead of LibLas for these functions.

	// bool DataIo::readPointCloudFromLasFile(pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud, std::string &filename, Bounds &bound_3d)
	// {
	// 	LASreadOpener lasreadopener;
	// 	lasreadopener.set_file_name(filename.c_str());

	// 	// validation:
	// 	if (!lasreadopener.active())
	// 	{
	// 		std::cerr << "ERROR: no input specified\n";
	// 	}
	// 	LASreader *lasreader = lasreadopener.open();
	// 	if (lasreader == 0)
	// 	{
	// 		std::cerr << "ERROR: could not open lasreader\n";
	// 	}

	// 	// header information
	// 	size_t points_num = lasreader->header.number_of_point_records;
	// 	size_t record_length = lasreader->header.point_data_record_length;
	// 	double cloud_x_offset = lasreader->header.x_offset;
	// 	double cloud_y_offset = lasreader->header.y_offset;
	// 	double cloud_z_offset = lasreader->header.z_offset;

	// 	bound_3d.min_x = lasreader->header.min_x;
	// 	bound_3d.min_y = lasreader->header.min_y;
	// 	bound_3d.min_z = lasreader->header.min_z;
	// 	bound_3d.max_x = lasreader->header.max_x;
	// 	bound_3d.max_y = lasreader->header.max_y;
	// 	bound_3d.max_z = lasreader->header.max_z;
	// 	double center_x, center_y, center_z;
	// 	center_x = bound_3d.min_x / 2.0f + bound_3d.max_x / 2.0f;
	// 	center_y = bound_3d.min_y / 2.0f + bound_3d.max_y / 2.0f;
	// 	center_z = bound_3d.min_z / 2.0f + bound_3d.max_z / 2.0f;

	// 	//cout << "dx:" << cloud_x_offset << "," << "min_x:" << bound_3d.min_x << endl;

	// 	// read to pcl PointCloud
	// 	while (lasreader->read_point())
	// 	{
	// 		pcl::PointXYZI pt;

	// 		pt.x = lasreader->get_x() - center_x;
	// 		pt.y = lasreader->get_y() - center_y;
	// 		pt.z = lasreader->get_z() - center_z;
	// 		pt.intensity = lasreader->point.get_intensity();
	// 		pt.data[3] = lasreader->point.get_gps_time();
	// 		//std::cout << "gps time is " << lasreader->point.get_gps_time() << std::endl;
	// 		pointCloud->push_back(pt);
	// 	}

	// 	//cout << "x:" << pointCloud->points[0].x << endl;

	// 	// zero the pointers of the other header so they don't get deallocated twice
	// 	lasreader->header.unlink();

	// 	lasreader->close();
	// 	delete lasreader;

	// 	//std::cout << "------Finish Read Las File by Laslib.------" << std::endl;
	// 	return true;
	// }

	// bool DataIo::writecolorLASPointCloud(const std::string &fileName,
	// 	const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud,
	// 	int R, int G, int B, double Min_x, double Min_y, double Min_z)
	// {
	// 	Bounds bound3d;
	// 	getCloudBound(pointCloud, bound3d);

	// 	LASwriteOpener laswriteopener;
	// 	laswriteopener.set_file_name(fileName.c_str());

	// 	//header file for output
	// 	LASheader header;

	// 	header.point_data_format = 3;  // if with color, set as 3; if only intensity, set as 1
	// 	header.point_data_record_length = 34;  // if with color, set as 34; if only intensity, set as 28
	// 	header.version_minor = 2;
	// 	header.number_of_point_records = pointCloud->points.size();
	// 	header.x_scale_factor = 0.01;
	// 	header.y_scale_factor = 0.01;
	// 	header.z_scale_factor = 0.1;
	// 	header.min_x = bound3d.min_x + Min_x;
	// 	header.min_y = bound3d.min_y + Min_y;
	// 	header.min_z = bound3d.min_z + Min_z;
	// 	header.max_x = bound3d.max_x + Min_x;
	// 	header.max_y = bound3d.max_y + Min_y;
	// 	header.max_z = bound3d.max_z + Min_z;
	// 	//header.set_bounding_box(bound3d.min_x, bound3d.min_y, bound3d.min_z, bound3d.max_x, bound3d.max_y, bound3d.max_z);
	// 	double center_x, center_y, center_z;
	// 	center_x = bound3d.min_x / 2.0f + bound3d.max_x / 2.0f + Min_x;
	// 	center_y = bound3d.min_y / 2.0f + bound3d.max_y / 2.0f + Min_y;
	// 	center_z = bound3d.min_z / 2.0f + bound3d.max_z / 2.0f + Min_z;
	// 	header.x_offset = 0.0;
	// 	header.y_offset = 0.0;
	// 	header.z_offset = 0.0;

	// 	LASwriter* laswriter = laswriteopener.open(&header);
	// 	if (laswriter == 0)
	// 	{
	// 		std::cerr << "ERROR: could not open laswriter\n";
	// 	}

	// 	LASpoint *point = new LASpoint();
	// 	point->init(&header, header.point_data_format, header.point_data_record_length, 0);

	// 	for (int i = 0; i < pointCloud->points.size(); ++i)
	// 	{
	// 		// copy the point;
	// 		point->set_X(static_cast<double>((pointCloud->points[i].x + Min_x) / header.x_scale_factor));
	// 		point->set_Y(static_cast<double>((pointCloud->points[i].y + Min_y) / header.y_scale_factor));
	// 		point->set_Z(static_cast<double>((pointCloud->points[i].z + Min_z) / header.z_scale_factor));
	// 		point->set_intensity(pointCloud->points[i].intensity);
	// 		point->set_R(R);
	// 		point->set_G(G);
	// 		point->set_B(B);
	// 		// write the modified point
	// 		laswriter->write_point(point);
	// 		laswriter->update_inventory(point);
	// 	}

	// 	laswriter->update_header(&header, TRUE);

	// 	laswriter->close();
	// 	delete laswriter;

	// 	return true;
	// }


	// bool DataIo::writeLasAll(const string &base_folder_name, int file_index, vector<pcl::PointCloud<pcl::PointXYZI>> &pointClouds,
	// 	const RoadMarkings &roadmarkings, double minX, double minY, double minZ)
	// {
	// 	string outputFolder;

	// 	ostringstream oss;
	// 	oss << "\\Classified Road Markings_" << file_index;

	// 	outputFolder = base_folder_name + "\\Classified Road Markings (las)";

	// 	if (!boost::filesystem::exists(outputFolder))
	// 	{
	// 		boost::filesystem::create_directory(outputFolder);
	// 	}

	// 	outputFolder = outputFolder + oss.str();

	// 	if (!boost::filesystem::exists(outputFolder))
	// 	{
	// 		boost::filesystem::create_directory(outputFolder);
	// 	}

	// 	for (size_t i = 0; i < pointClouds.size(); i++)
	// 	{
	// 		string  outputFileName;
	// 		ostringstream oss;
	// 		oss << i << "_Roadmarking.las";
	// 		outputFileName = outputFolder + "\\" + oss.str();

	// 		int R, G, B;

	// 		switch (roadmarkings[i].category)
	// 		{
	// 		case 1:
	// 			R = 0;
	// 			G = 255;
	// 			B = 0;
	// 			break;
	// 		case 2:
	// 			R = 255;
	// 			G = 0;
	// 			B = 0;
	// 			break;
	// 		case 3:
	// 			R = 0;
	// 			G = 0;
	// 			B = 255;
	// 			break;
	// 		case 4:
	// 			R = 0;
	// 			G = 255;
	// 			B = 255;
	// 			break;
	// 		case 5:
	// 			R = 255;
	// 			G = 0;
	// 			B = 255;
	// 			break;
	// 		case 6:
	// 			R = 255;
	// 			G = 255;
	// 			B = 0;
	// 			break;
	// 		case 7: //orange
	// 			R = 255;
	// 			G = 128;
	// 			B = 64;
	// 			break;
	// 		case 8:  //pink
	// 			R = 255;
	// 			G = 192;
	// 			B = 203;
	// 			break;
	// 		default:
	// 			R = 255;
	// 			G = 255;
	// 			B = 255;
	// 			break;
	// 		}

	// 		writecolorLASPointCloud(outputFileName, pointClouds[i].makeShared(),
	// 			R, G, B, minX, minY, minZ);
	// 	}
	// 	cout << "Output Classified Road Markings Point Cloud Done" << endl;
	// 	return true;
	// }
# endif

}
