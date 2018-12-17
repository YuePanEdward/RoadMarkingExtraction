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
		  void pcgrid(const pcXYZIPtr &cloud, float resolution, int &nx, int &ny);      //投影点云格网化
		  void pc2imgI(const pcXYZIPtr &cloud, float resolution, Mat &img);             //点云转反射强度图像
		  void pc2imgZ(const pcXYZIPtr &cloud, float resolution, Mat &img);             //点云转高程图像

		  Mat Sobelboundary(Mat img0);                                                  //Sobel 图像边缘提取
		  
		  float caculateCurrentEntropy(Mat hist, int threshold);                        //计算当前阈值的前后景熵
		  Mat maxEntropySegMentation(Mat inputImage);                                   //最大熵阈值分割
		  Mat ExtractRoadPixel(const Mat & _binI, const Mat & _binZ);                   //用高程梯度滤除非道路像素

		  void RemoveSmallRegion(const Mat &_binImg, Mat &_binfilter, int AreaLimit);   //滤除像素数过少的连通域 (这样有点低效吧）
		  void CcaByTwoPass(const Mat & _binfilterImg, Mat & _labelImg);                //两次扫描法连通成分分析（4邻域）
		  void CcaBySeedFill(const Mat& _binfilterImg, Mat & _lableImg);                //种子填充法连通成分分析（8邻域）
		  
		  void LabelColor(const Mat & _labelImg, Mat & _colorImg);                      //连通域着色
		  Scalar GetRandomColor();                                                      //随机取色器

		  Mat Truncate(const Mat & Img);                                  //图像非0区域截取

		  //Save img
		  void saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &GI, const Mat &GZ, const Mat &BI, const Mat &BZ, const Mat &BI2, const Mat &Label);
	  protected:
	
	  private:
	};
}

#endif
