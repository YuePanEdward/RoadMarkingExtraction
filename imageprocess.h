#ifndef IMAGE_H
#define IMAGE_H
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h> 
#include <array>
#include <algorithm>
#include "utility.h"

using namespace utility;
using namespace cv;

namespace image{

	class Imageprocess
	{
	  public:
		  void pcgrid(vector<double> &boundingbox, float resolution);                   //投影点云格网化(无需输入点云）
		  void savepcgrid(vector<double> &boundingbox, float resolution, const pcXYZIPtr &c, const pcXYZIPtr &gc, const pcXYZIPtr &ngc);   //投影点云格网化并记录点号
		  void pc2imgI(const pcXYZIPtr &cloud, int whatcloud,  Mat &img);               //点云转反射强度图像  whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		  void pc2imgZ(const pcXYZIPtr &cloud, int whatcloud,  Mat &img);               //点云转高程图像      whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		  void pc2imgD(const pcXYZIPtr &cloud, int whatcloud,  Mat &img);               //点云转点密度图像    whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		 
		  void img2pc(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  //标线图像转原始无过滤整体点云
		  void img2pclabel(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ);   //分割标线图像转分割点云

		  Mat Sobelboundary(Mat img0);                                                  //Sobel 图像边缘提取
		  
		  float caculateCurrentEntropy(Mat hist, int threshold);                        //计算当前阈值的前后景熵
		  Mat maxEntropySegMentation(Mat inputImage);                                   //最大熵阈值分割
		  Mat ExtractRoadPixel(const Mat & _binI,const Mat & _binZ,const Mat & _binD);  //用高程梯度滤除非道路像素

		  void RemoveSmallRegion(const Mat &_binImg, Mat &_binfilter, int AreaLimit);   //滤除像素数过少的连通域 (这样有点低效吧）
		  void CcaByTwoPass(const Mat & _binfilterImg, Mat & _labelImg);                //两次扫描法连通成分分析（4邻域）
		  void CcaBySeedFill(const Mat& _binfilterImg, Mat & _lableImg);                //种子填充法连通成分分析（8邻域）
		  
		  void ImgReverse(const Mat &img, Mat &img_reverse);                            //二值化图像反色
		  void ImgFilling(const Mat &img, Mat &img_fill);                               //空洞填充

		  void LabelColor(const Mat & _labelImg, Mat & _colorImg);                      //连通域着色
		  Scalar GetRandomColor();                                                      //随机取色器

		  void DetectCornerHarris(const Mat & src, const Mat & colorlabel, Mat & cornershow, Mat & cornerwithimg, int threshold);                                //Harris角点检测
		  void DetectCornerShiTomasi(const Mat & src, const Mat & colorlabel, Mat & cornerwithimg, int minDistance, double mincorenerscore);                     //Shi-Tomasi角点检测

		  void Truncate(const Mat & Img, Mat & TImg);                                    //图像非0区域截取

		  //Save img
		  void saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label/*const Mat &Corner*/);
	  protected:
	
	  private:
		  int nx, ny; //pixel number
		  int timin, tjmin; //truncated pixel no.
		  float minX, minY, minZ;  //bounding box minimum value
		  float res;  //resolution
		  vector<vector<vector<int>>> CMatrixIndice;    //cloud
		  vector<vector<vector<int>>> GCMatrixIndice;   //ground cloud
		  vector<vector<vector<int>>> NGCMatrixIndice;  //non-ground cloud
		  int totallabel; //过滤后的总连通块数
		  vector<vector <int>>  labelx;   //存各label的像素X坐标
		  vector<vector <int>>  labely;   //存各label的像素Y坐标

	};

}

#endif
