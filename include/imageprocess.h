#ifndef INCLUDE_IMAGE_H
#define INCLUDE_IMAGE_H


#include "utility.h"

//pcl
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h> 

//opencv
#include <opencv2/opencv.hpp>   
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

namespace roadmarking
{
	class Imageprocess
	{
	  public:

		  void pcgrid(Bounds &boundingbox, float resolution);                   //ͶӰ���Ƹ�����(����������ƣ�
		  
		  void savepcgrid(Bounds &boundingbox, float resolution, const pcXYZIPtr &c);   //ͶӰ���Ƹ���������¼���
		  void savepcgrid(Bounds &boundingbox, float resolution,  pcXYZIPtr &c,  pcXYZIPtr &gc, pcXYZIPtr &ngc);   //ͶӰ���Ƹ���������¼���, ���أ�����ͷǵ����
		  void pc2imgI(const pcXYZIPtr &cloud, int whatcloud,  Mat &img, float times_std);               //����ת����ǿ��ͼ��  whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]; //times_std: maxI = meanI+ times_std*stdI
		  void pc2imgZ(const pcXYZIPtr &cloud, int whatcloud,  Mat &img);               //����ת�߳�ͼ��      whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud]
		  void pc2imgD(const pcXYZIPtr &cloud, int whatcloud,  Mat &img ,float k);      //����ת���ܶ�ͼ��    whatcloud[ 0: Original Cloud , 1: Ground Cloud , 2: Non-ground Cloud] k:expected max point number in a pixel
		 
		  void img2pc_g(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  //����ͼ��תԭʼ�޹���������� (����㣩
		  void img2pc_c(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud);  //����ͼ��תԭʼ�޹���������ƣ����е㣩

		  void img2pclabel_g(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ);   //�ָ����ͼ��ת�ָ����(����㣩
		  void img2pclabel_c(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ);   //�ָ����ͼ��ת�ָ���ƣ����е㣩

		  Mat Sobelboundary(Mat img0);                                                  //Sobel ͼ���Ե��ȡ
		  
		  float caculateCurrentEntropy(Mat hist, int threshold);                        //���㵱ǰ��ֵ��ǰ����
		  Mat maxEntropySegMentation(Mat inputImage);                                   //�������ֵ�ָ�
		  Mat ExtractRoadPixelIZD(const Mat & _binI,const Mat & _binZ,const Mat & _binD); //�ø߳��ݶȼ����ܶ��˳��ǵ�·���� (for MLS)
		  Mat ExtractRoadPixelIZ (const Mat & _binI,const Mat & _binZ);                   //�ø߳��ݶ��˳��ǵ�·���� (for ALS)

		  void RemoveSmallRegion(const Mat &_binImg, Mat &_binfilter, int AreaLimit);   //�˳����������ٵ���ͨ�� (�����е��Ч�ɣ�
		  void CcaByTwoPass(const Mat & _binfilterImg, Mat & _labelImg);                //����ɨ�跨��ͨ�ɷַ�����4����
		  void CcaBySeedFill(const Mat& _binfilterImg, Mat & _lableImg);                //������䷨��ͨ�ɷַ�����8����
		  
		  void ImgReverse(const Mat &img, Mat &img_reverse);                            //��ֵ��ͼ��ɫ
		  void ImgFilling(const Mat &img, Mat &img_fill);                               //�ն����

		  void LabelColor(const Mat & _labelImg, Mat & _colorImg);                      //��ͨ����ɫ
		  Scalar GetRandomColor();                                                      //���ȡɫ��

		  void DetectCornerHarris(const Mat & src, const Mat & colorlabel, Mat & cornershow, Mat & cornerwithimg, int threshold);                                //Harris�ǵ���
		  void DetectCornerShiTomasi(const Mat & src, const Mat & colorlabel, Mat & cornerwithimg, int minDistance, double mincorenerscore);                     //Shi-Tomasi�ǵ���

		  void Truncate( Mat & Img, Mat & TImg);                                    //ͼ�����ػҶ�ֵ���������ȡ

		  //Save images
		  void saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label, const Mat &Corner);
		  
		  //hyj0728 without density
		  void saveimg(std::string base_folder, int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label);
		  
		  void saveimg(std::string base_folder, int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label); // ���� 1
		  void saveimg(const Mat &ProjI, const Mat &ProjImf, const Mat &GI, const Mat &BI, const Mat &BIF, const Mat &Label); //���� 2

		  int nx, ny; //pixel number
		  int timin, tjmin; //truncated pixel no.
		  float minX, minY, minZ;  //bounding box minimum value
		  float res;  //resolution
		  vector<vector<vector<int>>> CMatrixIndice;    //cloud
		  vector<vector<vector<int>>> GCMatrixIndice;   //ground cloud
		  vector<vector<vector<int>>> NGCMatrixIndice;  //non-ground cloud
		  int totallabel; //���˺������ͨ����
		  vector<vector <int>>  labelx;   //���label������X����
		  vector<vector <int>>  labely;   //���label������Y����


	protected:
	
	  private:
		 
	};

}

#endif
