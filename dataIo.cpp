#include "dataIo.h"
#include "utility.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>
#include <pcl/visualization/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iosfwd>
#include <fstream>
#include <intsafe.h>

using namespace  std;
using namespace utility;

void DataIo::readPointXYZLC(const string &fileName, vector<PointXYZLC> &ptlcs)
{
	ifstream ifs(fileName);

	if (ifs.fail())
	{
		cout << "can not open the file." << endl;
		return;
	}

	PointXYZLC ptlc;
	size_t ptIndex = 0;
	size_t i = 0;
	while (!ifs.eof())
	{
		//if (i % 10 == 0){

			ifs >> ptlc.pt[0] >> ptlc.pt[1] >> ptlc.pt[2] >> ptlc.objectLabel >> ptlc.objectClass;  //注意ptlc.pt的格式，是Vector型，不是点啊
			ptlc.ptIndex = ptIndex;
			ptlcs.push_back(ptlc);

			ptIndex++;
			//i++;
			//cout << ptIndex << endl;
			ifs.get();
			if (ifs.peek() == EOF)
				break;
		//}
	}
}
bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::loadPCDFile(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}

bool DataIo::writePcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}
bool DataIo::writePcdAll(const string &folderName, const string &fileName, const vector<pcXYZI> &pointClouds){

	//创建文件夹;
	/*if (boost::filesystem::exists(folderName))
	{
		directory_iterator end_iter;
		for (directory_iterator iter(folderName); iter != end_iter; ++iter)
		{
			if (is_regular_file(iter->status()))
			{
				string fileName;
				fileName = iter->path().string();
				boost::filesystem::remove(fileName);
			}
		}
	}*/

	if (!boost::filesystem::exists(folderName))
	{
		boost::filesystem::create_directory(folderName);
	}

	for (size_t i = 0; i < pointClouds.size(); i++)
	{
		string  outputFileName;
		ostringstream oss;
		oss << i << "_"<<fileName;	
		outputFileName = folderName + "\\" + oss.str();
		
		writePcdFile(outputFileName, pointClouds[i].makeShared());
	}

	cout << "Output Done" << endl;
	return true;

}

void DataIo::displayroad(const pcXYZIPtr &ngcloud, const pcXYZIPtr &gcloud)
{

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;
	float maxi,maxvi,mini,maxz,minz;
	maxi = -FLT_MAX;
	mini = FLT_MAX;
	maxz = -FLT_MAX;
	minz = FLT_MAX;

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
		if (ngcloud->points[i].z > maxz) maxz = ngcloud->points[i].z;
		if (ngcloud->points[i].z < minz) minz = ngcloud->points[i].z;
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
		
		kk = (ngcloud->points[i].z - minz) / (maxz - minz);
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
			//随机赋色
			
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
			//随机赋色

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

	float  maxz, minz;
	maxz = -FLT_MAX;
	minz = FLT_MAX;

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
		if (ngcloud->points[i].z > maxz) maxz = ngcloud->points[i].z;
		if (ngcloud->points[i].z < minz) minz = ngcloud->points[i].z;
	}

	for (size_t i = 0; i < ngcloud->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = ngcloud->points[i].x;
		pt.y = ngcloud->points[i].y;
		pt.z = ngcloud->points[i].z;

		kk = (ngcloud->points[i].z - minz) / (maxz - minz);
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
void DataIo::displaymarkVect(const vector<RoadMarking> &roadmarkings)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Vectorization Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	for (int i = 0; i < roadmarkings.size(); i++)
	{
		switch (roadmarkings[i].category)
		{
		case 1:
			for (int j = 0; j < 4; j++)
			{
				pcl::PointXYZ pt1;
				pt1.x = roadmarkings[i].polyline[j].x;
				pt1.y = roadmarkings[i].polyline[j].y;
				pt1.z = roadmarkings[i].polyline[j].z;
				sprintf(t, "%d", n);
				s = t;
				viewer->addSphere(pt1, 0.05, 0.0, 0.0, 0.0, s);
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
				viewer->addSphere(pt2, 0.05, 0.0, 0.0, 0.0, s);
				n++;

				sprintf(t, "%d", n);
				s = t;
				viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, s);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
				n++;
			}

			break;


		case 2:
			for (int j = 0; j < roadmarkings[i].polyline.size()-1; j++)
			{
				pcl::PointXYZ pt1;
				pt1.x = roadmarkings[i].polyline[j].x;
				pt1.y = roadmarkings[i].polyline[j].y;
				pt1.z = roadmarkings[i].polyline[j].z;
				sprintf(t, "%d", n);
				s = t;
				viewer->addSphere(pt1, 0.05, 0.0, 0.0, 0.0, s);
				n++;

				pcl::PointXYZ pt2;
		        pt2.x = roadmarkings[i].polyline[j + 1].x;
			    pt2.y = roadmarkings[i].polyline[j + 1].y;
			    pt2.z = roadmarkings[i].polyline[j + 1].z;
			
				sprintf(t, "%d", n);
				s = t;
				viewer->addSphere(pt2, 0.05, 0.0, 0.0, 0.0, s);
				n++;

				sprintf(t, "%d", n);
				s = t;
				viewer->addLine(pt1, pt2, 0.0, 1.0, 0.0, s);
				viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, s);
				n++;
			}

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

bool DataIo::writemarkVectDXF(const vector<RoadMarking> &roadmarkings)
{
	//dxflib
		/*
		//输出dxf分块信息
		bool CBlockSegment::OutDxfSegInfo()
	{
		string OutDxfPath = srcPrjRootPath + "\\segInfo.dxf";
		dw = dxf.out(OutDxfPath.c_str(), DL_Codes::AC1015);
		if (dw == NULL)
		{
			printf("无法打开创建分块信息dxf文件");
			return false;
		}
		// section header:
		dxf.writeHeader(*dw);
		dw->sectionEnd();

		// section tables:
		dw->sectionTables();

		// VPORT:
		dxf.writeVPort(*dw);

		// LTYPE:
		dw->tableLinetypes(1);
		dxf.writeLinetype(*dw, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
		dxf.writeLinetype(*dw, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
		dw->tableEnd();

		// LAYER:
		dw->tableLayers(1);
		dxf.writeLayer(
			*dw,
			DL_LayerData("0", 0),
			DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS")
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

		DL_Attributes attributes("0", 256, 0x00ffff00, 5, "BYLAYER");
		string BlockName = "";
		Vector3d TextPos;
		vector<Vector3d> LineVertexs;

		for (int i = 1; i < gridRowNum + 1; i++)
		{
			for (int j = 1; j < gridColNum + 1; j++)
			{
				//字段名称
				std::stringstream ss;
				ss << inttoFormatStr(i, 3) << inttoFormatStr(j, 3);
				ss >> BlockName;
				//字段位置
				double nTextX = gridLowCorner[0] + j*m_gridSize - 0.5*m_gridSize;
				double nTextY = gridLowCorner[1] + i*m_gridSize - 0.5*m_gridSize;
				TextPos[0] = nTextX; TextPos[1] = nTextY; TextPos[2] = 0;
				WriteDxfText(BlockName.c_str(), TextPos, attributes);
				LineVertexs.clear();
				//线框顶点
				Vector3d v1(gridLowCorner[0] + j*m_gridSize, gridLowCorner[1] + i*m_gridSize, 0);
				Vector3d v2(gridLowCorner[0] + (j - 1)*m_gridSize, gridLowCorner[1] + i*m_gridSize, 0);
				Vector3d v3(gridLowCorner[0] + (j - 1)*m_gridSize, gridLowCorner[1] + (i - 1)*m_gridSize, 0);
				Vector3d v4(gridLowCorner[0] + j*m_gridSize, gridLowCorner[1] + (i - 1)*m_gridSize, 0);
				LineVertexs.push_back(v1);
				LineVertexs.push_back(v2);
				LineVertexs.push_back(v3);
				LineVertexs.push_back(v4);
				WriteDxfPolyline(LineVertexs, attributes, true);
			}
		}
		//轨迹
		DL_Attributes attributes1("0", 128, 0x00ff0000, 5, "BYLAYER");
		WriteDxfPos(posData, attributes1, false);
		// end section ENTITIES:
		dw->sectionEnd();
		dxf.writeObjects(*dw, "MY_OBJECTS");
		dxf.writeObjectsEnd(*dw);

		dw->dxfEOF();
		dw->close();
		delete dw;
		return true;
	}

	//写字符串到dxf
	void CBlockSegment::WriteDxfText(const char* TextName, Vector3d& textPos, DL_Attributes& attributes)
	{
		int TextSize = m_gridSize*0.1;
		const char* m_stylename = "textstyle0";
		dxf.writeMText(
			*dw,
			DL_MTextData(
			textPos[0],
			textPos[1],
			0,
			0.0,
			0.0,
			0.0,
			TextSize,
			TextSize,
			1,
			1,
			2,
			1,
			TextName,
			m_stylename,
			0),
			attributes);

	}
	//写多段线到dxf
	void CBlockSegment::WriteDxfPolyline(vector<Vector3d>& lineVecs, DL_Attributes& attributes, bool bIsClosed)
	{
		int count = lineVecs.size();
		bool plineGen = true;

		dxf.writePolyline(
			*dw,
			DL_PolylineData(count,
			0, 0,
			bIsClosed * 0x1 + plineGen * 0x80),
			attributes
			);

		for (int i = 0; i < count; i++) {
			Vector3d v = lineVecs[i];
			double bulge = 0;
			dxf.writeVertex(*dw, DL_VertexData(v[0], v[1], 0.0, bulge));
		}

		dxf.writePolylineEnd(*dw);
	}
	//写轨迹信息到dxf
	void CBlockSegment::WriteDxfPos(vector<vector<posXYZ>>& pxyz, DL_Attributes& attributes, bool bIsClosed)
	{
		for (int m = 0; m < pxyz.size(); m++)
		{
			if (pxyz[m].empty())
			{
				continue;
			}
			int count = pxyz[m].size();
			bool plineGen = true;

			dxf.writePolyline(
				*dw,
				DL_PolylineData(count,
				0, 0,
				bIsClosed * 0x1 + plineGen * 0x80),
				attributes
				);

			for (int i = 0; i < count; i++) {
				posXYZ v = pxyz[m][i];
				double bulge = 0;
				dxf.writeVertex(*dw, DL_VertexData(v.x, v.y, 0.0, bulge));
			}
			dxf.writePolylineEnd(*dw);
		}

	}*/
return true;

}

bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud, vector<double> & boundingbox)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();
	
	//header里可以直接提bounding box 出来
	double Xmin, Ymin, Zmin, Xmax, Ymax,Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	

	//int i = 0;
	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		//i++;
		pcl::PointXYZI  pt;
		/*将重心化后的坐标和强度值赋值给PCL中的点;*/
		/*做一个平移，否则在WGS84 下的点坐标太大了，会造成精度损失的 因为las的读取点数据是double的，而pcd是int的*/
		pt.x = p.GetX()-Xmin;
		pt.y = p.GetY()-Ymin;  
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		//if (i % 100 == 0) { cout << p.GetPointSourceID() << "\t" << p.GetTime() << endl; } //这些PointSourceID 和 Time 信息可用于ScanLine Profile 法（扫描线）法来提道路面，不过效率有点堪忧
		pointCloud.points.push_back(pt);
	}

	//Bounding Box after translation 
	boundingbox.push_back(0);
	boundingbox.push_back(0);
	boundingbox.push_back(Zmin);
	boundingbox.push_back(Xmax-Xmin);
	boundingbox.push_back(Ymax-Ymin);
	boundingbox.push_back(Zmax);

	return 1;
}

