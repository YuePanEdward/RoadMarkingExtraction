#include "imageprocess.h"

using namespace std;
using namespace cv;

namespace roadmarking
{

	void Imageprocess::pcgrid(Bounds &boundingbox, float resolution)
	{
		//boundingbox order: Xmin,Ymin,Zmin,Xmax,Ymax,Zmax
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		res = resolution;

		cout << "Image Size: " << nx << " * " << ny << endl;
	}

	void Imageprocess::savepcgrid(Bounds &boundingbox, float resolution, pcXYZIPtr &c, pcXYZIPtr &gc, pcXYZIPtr &ngc)
	{
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		// cout << lx << "," << ly << "," << lz << endl;
		// cout << boundingbox.max_x << "," << boundingbox.max_y << "," << boundingbox.max_z << endl;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		cout << "Image Size: " << nx << " x " << ny << endl;

		if(nx * ny > 2e7)
			cout<<"Warning: Too much grid map, the system may run out of memory."<<endl;

		CMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));

		//Saving point indices 
		for (int i = 0; i < c->points.size(); i++)
		{
			ix = (int)((c->points[i].x - minX) / resolution);
			iy = (int)((c->points[i].y - minY) / resolution);
			CMatrixIndice[ix][iy].push_back(i);
		}
        c.reset(new pcXYZI()); //free the memory
        
	    GCMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));
		NGCMatrixIndice.resize(nx, vector<vector<int>>(ny, vector<int>(0)));

		for (int i = 0; i < gc->points.size(); i++)
		{
			ix = (int)((gc->points[i].x - minX) / resolution);
			iy = (int)((gc->points[i].y - minY) / resolution);
			GCMatrixIndice[ix][iy].push_back(i);
		}
		for (int i = 0; i < ngc->points.size(); i++)
		{
			ix = (int)((ngc->points[i].x - minX) / resolution);
			iy = (int)((ngc->points[i].y - minY) / resolution);
			NGCMatrixIndice[ix][iy].push_back(i);
		}
	}

	void Imageprocess::savepcgrid(Bounds &boundingbox, float resolution, const pcXYZIPtr &c)
	{
		float lx, ly, lz;
		int ix, iy;
		lx = boundingbox.max_x - boundingbox.min_x;
		ly = boundingbox.max_y - boundingbox.min_y;
		lz = boundingbox.max_z - boundingbox.min_z;

		minX = -0.5 * lx;
		minY = -0.5 * ly;
		minZ = -0.5 * lz;

		nx = lx / resolution + 1;
		ny = ly / resolution + 1;

		res = resolution;

		cout << "Image Size: " << nx << " * " << ny << endl;

		CMatrixIndice.resize(nx);

		for (size_t i = 0; i < nx; i++)
		{
			CMatrixIndice[i].resize(ny);
		}

		//Saving point indices 
		for (size_t i = 0; i < c->points.size(); i++)
		{
			ix = (c->points[i].x - minX) / res;
			iy = (c->points[i].y - minY) / res;
			CMatrixIndice[ix][iy].push_back(i);
		}
	}

	void Imageprocess::pc2imgI(const pcXYZIPtr &cloud, int whatcloud, Mat &img, float times_std)
	{

		double mini, maxi;	//min and max Intensity
		double meani, stdi; //mean and standard deviation of Intensity
		int Number_non_zero_pixel;

		meani = 0;
		stdi = 0;
		Number_non_zero_pixel = 0;

		img.create(nx, ny, CV_8UC1); 

		vector<vector<vector<float>>> matrixi;
		vector<vector<float>> ave;

		matrixi.resize(nx);
		ave.resize(nx);

		for (int i = 0; i < nx; i++)
		{
			matrixi[i].resize(ny);
			ave[i].resize(ny);
			for (int j = 0; j < ny; j++)
			{
				matrixi[i][j].push_back(0);
			}
		}

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k)
						matrixi[i][j].push_back(cloud->points[*k].intensity);
				}
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k) 
						matrixi[i][j].push_back(cloud->points[*k].intensity);
				}
			}
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) 
						matrixi[i][j].push_back(cloud->points[*k].intensity);
				}
			}
			break;
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				ave[i][j] = (0.001 + accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0)) / matrixi[i][j].size();
			}
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				if (ave[i][j] > 1)
				{
					meani += ave[i][j];
					Number_non_zero_pixel++;
				}
			}
		}
		meani /= Number_non_zero_pixel;

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				if (ave[i][j] > 1)
				{
					stdi += (ave[i][j] - meani) * (ave[i][j] - meani);
				}
			}
		}

		stdi /= Number_non_zero_pixel;
		stdi = sqrt(stdi);
		maxi = meani + times_std * stdi; 
		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				if (ave[i][j] > maxi)
					ave[i][j] = maxi;
				img.at<uchar>(i, j) = 255 * ave[i][j] / maxi ;
			}
		}
	}

	void Imageprocess::pc2imgZ(const pcXYZIPtr &cloud, int whatcloud, Mat &img)
	{

		float minaz, maxaz;			 
		img.create(nx, ny, CV_8UC1); 

		minaz = FLT_MAX;
		maxaz = -FLT_MAX;

		vector<vector<vector<float>>> matrixz;
		vector<vector<float>> ave;
		matrixz.resize(nx);
		ave.resize(nx);

		for (int i = 0; i < nx; i++)
		{
			matrixz[i].resize(ny);
			ave[i].resize(ny);
			for (int j = 0; j < ny; j++)
			{
				matrixz[i][j].push_back(minZ); 
			}
		}

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k) 
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
				{
					for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) 
						matrixz[i][j].push_back(cloud->points[*k].z);
				}
			}
			break;
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
				ave[i][j] = (0.1 + accumulate(begin(matrixz[i][j]), end(matrixz[i][j]), 0.0)) / matrixz[i][j].size();
		}

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
			{
				maxaz = max(maxaz, ave[i][j]);
				minaz = min(minaz, ave[i][j]);
			}
		} 

		for (int i = 0; i < nx; i++)
		{
			for (int j = 0; j < ny; j++)
				img.at<uchar>(i, j) = 255 * (ave[i][j] - minaz) / (maxaz - minaz);
		}
	}

	void Imageprocess::pc2imgD(const pcXYZIPtr &cloud, int whatcloud, Mat &img, float expected_max_point_num_in_a_pixel)
	{

		img.create(nx, ny, CV_8UC1); 
		Eigen::MatrixXi Matrixnum;
		int maxnum, maxelement;
		Matrixnum.resize(nx, ny);
		Matrixnum = Eigen::MatrixXi::Zero(nx, ny);

		switch (whatcloud)
		{
		case 0: //Original Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = CMatrixIndice[i][j].size();
			}
			break;
		case 1: //Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = GCMatrixIndice[i][j].size();
			}
			break;
		case 2: //Non-Ground Point Cloud
			for (int i = 0; i < nx; i++)
			{
				for (int j = 0; j < ny; j++)
					Matrixnum(i, j) = NGCMatrixIndice[i][j].size();
			}
			break;
		}

		//maxelement = Matrixnum.maxCoeff();
		//if (maxelement < expectedmaxnum) maxnum = maxelement;
		//else maxnum = expectedmaxnum;
		maxnum = expected_max_point_num_in_a_pixel;

		//cout << "max point number"<<maxnum<<endl;
		for (size_t i = 0; i < nx; i++)
		{
			for (size_t j = 0; j < ny; j++)
			{
				int out;
				if (Matrixnum(i, j) < maxnum)
					out = 255 * Matrixnum(i, j) / maxnum;
				else
					out = 255;
				img.at<uchar>(i, j) = out;
			}
		}
	}

	void Imageprocess::img2pc_g(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr &outcloud)
	{

		Mat grayImage, binImage;
		cvtColor(img, grayImage, COLOR_BGR2GRAY);
		threshold(grayImage, binImage, 0, 1, CV_THRESH_BINARY);

		for (size_t i = timin; i < timin + img.rows; i++)
		{
			for (size_t j = tjmin; j < tjmin + img.cols; j++)
			{
				if (binImage.at<uchar>(i - timin, j - tjmin) == 1)
				{
					for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
						outcloud->points.push_back(incloud->points[*k]);
				}
			}
		}
	}

	void Imageprocess::img2pc_c(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr &outcloud)
	{

		Mat grayImage, binImage;
		cvtColor(img, grayImage, COLOR_BGR2GRAY);
		threshold(grayImage, binImage, 0, 1, CV_THRESH_BINARY);

		for (size_t i = timin; i < timin + img.rows; i++)
		{
			for (size_t j = tjmin; j < tjmin + img.cols; j++)
			{
				if (binImage.at<uchar>(i - timin, j - tjmin) == 1)
				{
					for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k)
						outcloud->points.push_back(incloud->points[*k]);
				}
			}
		}
	}

	void Imageprocess::img2pclabel_g(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ)
	{
		int classNo = 0;

		outclouds.resize(totallabel - 1);

		timin = 0;
		tjmin = 0;

		for (int i = timin; i < timin + img.rows; i++)
		{
			const int *data_src = (int *)img.ptr<int>(i - timin);
			for (int j = tjmin; j < tjmin + img.cols; j++)
			{
				int pixelValue = data_src[j - tjmin];

				if (pixelValue > 1)
				{
					double max_z, min_z, disz;
					max_z = -DBL_MAX;
					min_z = DBL_MAX;

					for (int k = 0; k < GCMatrixIndice[i][j].size(); k++)
					{

						if (max_z < incloud->points[GCMatrixIndice[i][j][k]].z)
							max_z = incloud->points[GCMatrixIndice[i][j][k]].z;
						if (min_z > incloud->points[GCMatrixIndice[i][j][k]].z)
							min_z = incloud->points[GCMatrixIndice[i][j][k]].z;
					}
					disz = max_z - min_z;
					if (disz < dZ)
					{
						for (auto g = GCMatrixIndice[i][j].begin(); g != GCMatrixIndice[i][j].end(); ++g)
							outclouds[pixelValue - 2].points.push_back(incloud->points[*g]);
					}
					else
					{
						if (outclouds[pixelValue - 2].size() == 0)
							outclouds[pixelValue - 2].points.push_back(incloud->points[GCMatrixIndice[i][j][0]]); 
					}
				}
			}
		}

		for (int k = 0; k <= totallabel - 2; k++)
		{
			if (outclouds[k].size() > 0)
				classNo++;
		}
		//cout << "Cloud Number: " << classNo << endl;
	}

	void Imageprocess::img2pclabel_c(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds, double dZ)
	{
		int classNo = 0;

		outclouds.resize(totallabel - 1);

		for (int i = timin; i < timin + img.rows; i++)
		{
			const int *data_src = (int *)img.ptr<int>(i - timin);
			for (int j = tjmin; j < tjmin + img.cols; j++)
			{
				int pixelValue = data_src[j - tjmin];

				if (pixelValue > 1)
				{
					double max_z, min_z, disz;
					max_z = -DBL_MAX;
					min_z = DBL_MAX;

					for (int k = 0; k < CMatrixIndice[i][j].size(); k++)
					{

						if (max_z < incloud->points[CMatrixIndice[i][j][k]].z)
							max_z = incloud->points[CMatrixIndice[i][j][k]].z;
						if (min_z > incloud->points[CMatrixIndice[i][j][k]].z)
							min_z = incloud->points[CMatrixIndice[i][j][k]].z;
					}
					disz = max_z - min_z;
					if (disz < dZ)
					{
						for (auto g = CMatrixIndice[i][j].begin(); g != CMatrixIndice[i][j].end(); ++g)
							outclouds[pixelValue - 2].points.push_back(incloud->points[*g]);
					}
					else
					{
						if (outclouds[pixelValue - 2].size() == 0)
							outclouds[pixelValue - 2].points.push_back(incloud->points[CMatrixIndice[i][j][0]]); //������ֿյ�����֮���ٰ������˳�
					}
				}
			}
		}

		for (int k = 0; k <= totallabel - 2; k++)
		{
			if (outclouds[k].size() > 0)
				classNo++;
		}
		cout << "Cloud Number: " << classNo << endl;
	}

	Mat Imageprocess::Sobelboundary(Mat img0)
	{
		//Using Sobel Operation
		Mat grad_xg, grad_yg, abs_grad_xg, abs_grad_yg, dstg;

		Sobel(img0, grad_xg, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
		convertScaleAbs(grad_xg, abs_grad_xg);

		Sobel(img0, grad_yg, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
		convertScaleAbs(grad_yg, abs_grad_yg);

		addWeighted(abs_grad_xg, 0.5, abs_grad_yg, 0.5, 0, dstg);
		return dstg;
	}

	float Imageprocess::caculateCurrentEntropy(Mat hist, int threshold)
	{
		float BackgroundSum = 0, targetSum = 0;
		const float *pDataHist = (float *)hist.ptr<float>(0);
		for (int i = 0; i < 256; i++)
		{
			if (i < threshold)
				BackgroundSum += pDataHist[i];
			else
				targetSum += pDataHist[i];
		}
		float BackgroundEntropy = 0, targetEntropy = 0;
		for (int i = 0; i < 256; i++)
		{
			if (i < threshold)
			{
				if (pDataHist[i] == 0)
					continue;
				float ratio1 = pDataHist[i] / BackgroundSum;
				BackgroundEntropy += -ratio1 * logf(ratio1);
			}
			else
			{
				if (pDataHist[i] == 0)
					continue;
				float ratio2 = pDataHist[i] / targetSum;
				targetEntropy += -ratio2 * logf(ratio2);
			}
		}
		return (targetEntropy + BackgroundEntropy); 
	}

	Mat Imageprocess::maxEntropySegMentation(Mat inputImage)
	{
		// Max Entropy Binarization
		// Using the distribution of histogram to calculate the threshold leading to the max entropy.
		const int channels[1] = {0};
		const int histSize[1] = {256};
		float pranges[2] = {0, 256};
		const float *ranges[1] = {pranges};
		MatND hist;
		calcHist(&inputImage, 1, channels, Mat(), hist, 1, histSize, ranges);
		float maxentropy = 0;
		int max_index = 0;
		Mat result;
		for (int i = 0; i < 256; i++) 
		{
			float cur_entropy = caculateCurrentEntropy(hist, i);
			if (cur_entropy > maxentropy)
			{
				maxentropy = cur_entropy;
				max_index = i;
			}
		}
		threshold(inputImage, result, max_index, 1, CV_THRESH_BINARY); // > max_index assign as 1   < max_index assign as 0
		return result;
	}

	Mat Imageprocess::ExtractRoadPixelIZD(const Mat &_imgI, const Mat &_binZ, const Mat &_binD)
	{
		Mat result;
		_imgI.convertTo(result, CV_8UC1);
		//int mini, minj, maxi, maxj;
		//vector <int> arrayi, arrayj;
		for (int i = 0; i < _imgI.rows; i++)
		{
			for (int j = 0; j < _imgI.cols; j++)
			{
				if (_binZ.at<uchar>(i, j) == 1 || _binD.at<uchar>(i, j) == 0)
				{
					result.at<uchar>(i, j) = 0;
				}
				else
				{
					result.at<uchar>(i, j) = _imgI.at<uchar>(i, j);
				}
			}
		}
		return result;
	}
	Mat Imageprocess::ExtractRoadPixelIZ(const Mat &_imgI, const Mat &_binZ)
	{
		Mat result;
		_imgI.convertTo(result, CV_8UC1);
		for (int i = 0; i < _imgI.rows; i++)
		{
			for (int j = 0; j < _imgI.cols; j++)
			{
				if (_binZ.at<uchar>(i, j) == 1)
				{
					result.at<uchar>(i, j) = 0;
				}
				else
				{
					result.at<uchar>(i, j) = _imgI.at<uchar>(i, j);
				}
			}
		}
		return result;
	}
	void Imageprocess::CcaByTwoPass(const Mat &_binImg, Mat &_labelImg)
	{
		// connected component analysis (8-component)
		// use two-pass algorithm 
		// 1. first pass: label each foreground pixel with a label
		// 2. second pass: visit each labeled pixel and merge neighbor labels
		//
		// foreground pixel: _binImg(x,y) = 1
		// background pixel: _binImg(x,y) = 0

		if (_binImg.empty() ||
			_binImg.type() != CV_8UC1)
		{
			return;
		}

		// 1. first pass

		_labelImg.release();
		_binImg.convertTo(_labelImg, CV_32SC1); // _labelImg -> _binImg  32 Signed 

		int label = 1;		   // start by 2
		vector<int> labelSet;  
		labelSet.push_back(0); // background: 0
		labelSet.push_back(1); // foreground: 1

		int rows = _binImg.rows - 1;
		int cols = _binImg.cols - 1;
		for (int i = 1; i < rows; i++) 
		{
			int *data_preRow = _labelImg.ptr<int>(i - 1); 
			int *data_curRow = _labelImg.ptr<int>(i);	  
			for (int j = 1; j < cols; j++)				  
			{
				if (data_curRow[j] == 1) 
				{
					vector<int> neighborLabels;			
					neighborLabels.reserve(2);			
					int leftPixel = data_curRow[j - 1]; 
					int upPixel = data_preRow[j];		
					//int leftupPixel = data_preRow[j - 1];                 

					if (leftPixel > 1) //
					{
						neighborLabels.push_back(leftPixel); 
					}
					if (upPixel > 1) 
					{
						neighborLabels.push_back(upPixel); 
					}

					if (neighborLabels.empty()) //
					{
						labelSet.push_back(++label); // assign to a new label
						data_curRow[j] = label;	
						labelSet[label] = label;
					}
					else
					{
						sort(neighborLabels.begin(), neighborLabels.end());
						int smallestLabel = neighborLabels[0];
						data_curRow[j] = smallestLabel; 

						// save equivalence
						for (size_t k = 1; k < neighborLabels.size(); k++) 
						{
							int tempLabel = neighborLabels[k]; //
							int &oldSmallestLabel = labelSet[tempLabel];
							if (oldSmallestLabel > smallestLabel)
							{
								labelSet[oldSmallestLabel] = smallestLabel;
								oldSmallestLabel = smallestLabel;
							}
							else if (oldSmallestLabel < smallestLabel)
							{
								labelSet[smallestLabel] = oldSmallestLabel;
							}
						}
					}
				}
			}
		}

		// update equivalent labels
		// assigned with the smallest label in each equivalent label set
		for (size_t i = 2; i < labelSet.size(); i++) 
		{
			int curLabel = labelSet[i];
			int preLabel = labelSet[curLabel];
			while (preLabel != curLabel)
			{
				curLabel = preLabel;
				preLabel = labelSet[preLabel];
			}
			labelSet[i] = curLabel;
		}

		// 2. second pass
		for (int i = 0; i < rows; i++)
		{
			int *data = _labelImg.ptr<int>(i);
			for (int j = 0; j < cols; j++)
			{
				int &pixelLabel = data[j];
				pixelLabel = labelSet[pixelLabel];
			}
		}
		totallabel = label;
		//cout << "Number label: " << totallabel << endl;
	}

	void Imageprocess::CcaBySeedFill(const Mat &_binImg, Mat &_lableImg)
	{
		// connected component analysis (8-component)
		// use seed filling algorithm
		// 1. begin with a foreground pixel and push its foreground neighbors into a stack;
		// 2. pop the top pixel on the stack and label it with the same label until the stack is empty
		//
		// foreground pixel: _binImg(x,y) = 1
		// background pixel: _binImg(x,y) = 0

		if (_binImg.empty() ||
			_binImg.type() != CV_8UC1)
		{
			cout << "Wrong type" << endl;
			return;
		}

		_lableImg.release();
		_binImg.convertTo(_lableImg, CV_32SC1); 

		int label = 1; // start by 2
		//vector<vector<pair<int, int>>> labeledPixel;
		//labeledPixel.resize(10000);

		int rows = _binImg.rows;
		int cols = _binImg.cols;
		for (int i = 1; i < rows - 1; i++)
		{
			int *data = _lableImg.ptr<int>(i);
			for (int j = 1; j < cols - 1; j++)
			{
				if (data[j] == 1)
				{
					std::stack<std::pair<int, int>> neighborPixels;
					neighborPixels.push(std::pair<int, int>(i, j)); // pixel position: <i,j>
					++label;										// begin with a new label
					while (!neighborPixels.empty())
					{
						// get the top pixel on the stack and label it with the same label
						std::pair<int, int> curPixel = neighborPixels.top();
						int curX = curPixel.first;
						int curY = curPixel.second;
						_lableImg.at<int>(curX, curY) = label;

						//pair<int, int> pixelcor(curX, curY);
						//labeledPixel[label].push_back(pixelcor);

						// pop the top pixel
						neighborPixels.pop();

						// push the 8-neighbors (foreground pixels)
						if (_lableImg.at<int>(curX, curY - 1) == 1)
						{ // left pixel
							neighborPixels.push(std::pair<int, int>(curX, curY - 1));
						}
						if (_lableImg.at<int>(curX, curY + 1) == 1)
						{ // right pixel
							neighborPixels.push(std::pair<int, int>(curX, curY + 1));
						}
						if (_lableImg.at<int>(curX - 1, curY) == 1)
						{ // up pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY));
						}
						if (_lableImg.at<int>(curX + 1, curY) == 1)
						{ // down pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY));
						}
						if (_lableImg.at<int>(curX - 1, curY - 1) == 1)
						{ // left up pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY - 1));
						}
						if (_lableImg.at<int>(curX - 1, curY + 1) == 1)
						{ // left down pixel
							neighborPixels.push(std::pair<int, int>(curX - 1, curY + 1));
						}
						if (_lableImg.at<int>(curX + 1, curY - 1) == 1)
						{ // right up pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY - 1));
						}
						if (_lableImg.at<int>(curX + 1, curY + 1) == 1)
						{ // right down pixel
							neighborPixels.push(std::pair<int, int>(curX + 1, curY + 1));
						}
					}
				}
			}
		}
		totallabel = label;
	}

	void Imageprocess::RemoveSmallRegion(const Mat &Src, Mat &Dst, int AreaLimit)
	{
		int RemoveCount = 0;
		Src.convertTo(Dst, CV_8UC1);
		int CheckMode = 1;	 
		int NeihborMode = 1; 
		Mat PointLabel = Mat::zeros(Src.size(), CV_8UC1);
		if (CheckMode == 1) 
		{
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) < 1)
						PointLabel.at<uchar>(i, j) = 3; 
				}
			}
		}
		else 
		{
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) > 10)
						PointLabel.at<uchar>(i, j) = 3; 
				}
			}
		}

		vector<Point2i> NeihborPos;
		NeihborPos.push_back(Point2i(-1, 0));
		NeihborPos.push_back(Point2i(1, 0));
		NeihborPos.push_back(Point2i(0, -1));
		NeihborPos.push_back(Point2i(0, 1));
		if (NeihborMode == 1)
		{
			NeihborPos.push_back(Point2i(-1, -1));
			NeihborPos.push_back(Point2i(-1, 1));
			NeihborPos.push_back(Point2i(1, -1));
			NeihborPos.push_back(Point2i(1, 1));
		}
		else
			int a = 0; 
		int NeihborCount = 4 + 4 * NeihborMode;
		int CurrX = 0, CurrY = 0;
		for (int i = 0; i < Src.rows; i++)
		{
			for (int j = 0; j < Src.cols; j++)
			{
				if (PointLabel.at<uchar>(i, j) == 0) 
				{									
					vector<Point2i> GrowBuffer;		 
					GrowBuffer.push_back(Point2i(j, i));
					PointLabel.at<uchar>(i, j) = 1;
					int CheckResult = 0;

					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						for (int q = 0; q < NeihborCount; q++)
						{
							CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
							CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
							if (CurrX >= 0 && CurrX < Src.cols && CurrY >= 0 && CurrY < Src.rows) 
							{
								if (PointLabel.at<uchar>(CurrY, CurrX) == 0)
								{
									GrowBuffer.push_back(Point2i(CurrX, CurrY)); 
									PointLabel.at<uchar>(CurrY, CurrX) = 1;		
								}
							}
						}
					}
					if (GrowBuffer.size() > AreaLimit)
						CheckResult = 2;
					else
					{
						CheckResult = 1;
						RemoveCount++; 
					}
					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						CurrX = GrowBuffer.at(z).x;
						CurrY = GrowBuffer.at(z).y;
						PointLabel.at<uchar>(CurrY, CurrX) += CheckResult; 
					}
				}
			}
		}
		CheckMode = 255 * (1 - CheckMode);
		for (int i = 0; i < Src.rows; ++i)
		{
			for (int j = 0; j < Src.cols; ++j)
			{
				if (PointLabel.at<uchar>(i, j) == 2)
				{
					Dst.at<uchar>(i, j) = 0;
				}
				else
				{
					Dst.at<uchar>(i, j) = Src.at<uchar>(i, j);
				}
			}
		}
	}

	Scalar Imageprocess::GetRandomColor()
	{
		uchar r = 255 * (rand() / (1.0 + RAND_MAX)); // rand() / (1.0+ RAND_MAX) : a random float number between 0 and 1 (can't be equal to 1)
		uchar g = 255 * (rand() / (1.0 + RAND_MAX)); 
		uchar b = 255 * (rand() / (1.0 + RAND_MAX));
		return Scalar(b, g, r);
	}

	void Imageprocess::LabelColor(const Mat &_labelImg, Mat &_colorLabelImg)
	{
		if (_labelImg.empty() ||
			_labelImg.type() != CV_32SC1)
		{
			return;
		}

		std::map<int, Scalar> colors;

		int rows = _labelImg.rows;
		int cols = _labelImg.cols;

		_colorLabelImg.release();
		_colorLabelImg.create(rows, cols, CV_8UC3);
		_colorLabelImg = Scalar::all(0);

		for (int i = 0; i < rows; i++)
		{
			const int *data_src = (int *)_labelImg.ptr<int>(i);
			uchar *data_dst = _colorLabelImg.ptr<uchar>(i);
			for (int j = 0; j < cols; j++)
			{
				int pixelValue = data_src[j];
				if (pixelValue > 1)
				{
					//if(j%100==0) cout << pixelValue << endl;
					//labelx[pixelValue - 2].push_back(i);
					//labely[pixelValue - 2].push_back(j);

					if (colors.count(pixelValue) <= 0)
					{
						colors[pixelValue] = GetRandomColor();
					}
					Scalar color = colors[pixelValue];
					*data_dst++ = color[0];
					*data_dst++ = color[1];
					*data_dst++ = color[2];
				}
				else
				{
					data_dst++;
					data_dst++;
					data_dst++;
				}
			}
		}
	}
	void Imageprocess::Truncate(Mat &Img, Mat &TruncatedImg)
	{
		int mini, minj, maxi, maxj, di, dj;
		mini = INT_MAX;
		minj = INT_MAX;
		maxi = 0;
		maxj = 0;

		for (int i = 0; i < Img.rows; i++)
		{
			Img.at<uchar>(i, 0) = 0;
			Img.at<uchar>(i, Img.cols - 1) = 0;
		}
		for (int j = 0; j < Img.cols; j++)
		{
			Img.at<uchar>(0, j) = 0;
			Img.at<uchar>(Img.rows - 1, j) = 0;
		}

		for (int i = 0; i < Img.rows; i++)
		{
			for (int j = 0; j < Img.cols; j++)
			{
				if (Img.at<uchar>(i, j) != 0)
				{
					if (i < mini)
						mini = i;
					if (i > maxi)
						maxi = i;
					if (j < minj)
						minj = j;
					if (j > maxj)
						maxj = j;
				}
			}
		}

		timin = mini - 1;
		tjmin = minj - 1;
		di = maxi - mini + 3;
		dj = maxj - minj + 3;
		cv::Rect rect(tjmin, timin, dj, di);
		TruncatedImg = Img(rect);

		/*TruncatedImg.create(di, dj, CV_8UC1);
	for (int i = 0; i < di; i++)
	{
		for (int j = 0; j < dj; j++)
		{
			TruncatedImg.at<uchar>(i, j) = Img.at<uchar>(timin + i, tjmin + j);
		}
	}*/

		//cout << "X: " << tjmin << "  Y: " << timin << "  dX: " << dj << "  dY: " << di<< endl;

		//imshow("Truncated", 255*TruncatedImg);
	}

	void Imageprocess::DetectCornerHarris(const Mat &src, const Mat &colorlabel, Mat &cornershow, Mat &cornerwithimg, int threshold)
	{
		Mat corner, corner8u, imageGray;
		src.convertTo(imageGray, CV_8UC1);
		corner = Mat::zeros(src.size(), CV_32FC1);
		cornerHarris(imageGray, corner, 3, 3, 0.04, BORDER_DEFAULT);
		normalize(corner, corner8u, 0, 255, CV_MINMAX); 
		convertScaleAbs(corner8u, cornershow);
		cornerwithimg = colorlabel.clone();
		for (int i = 0; i < src.rows; i++)
		{
			for (int j = 0; j < src.cols; j++)
			{
				if (cornershow.at<uchar>(i, j) > threshold)
				{
					circle(cornerwithimg, Point(j, i), 2, Scalar(255, 255, 255), 2); 
				}
			}
		}
	}
	void Imageprocess::ImgReverse(const Mat &img, Mat &img_reverse)
	{
		img.convertTo(img_reverse, CV_8UC1);

		for (int i = 1; i < img.rows - 1; i++)
		{
			for (int j = 1; j < img.cols - 1; j++)
			{
				if (img.at<uchar>(i, j) == 1)
					img_reverse.at<uchar>(i, j) = 0;
				if (img.at<uchar>(i, j) == 0)
					img_reverse.at<uchar>(i, j) = 1;
			}
		}
		//imshow("imgReverse", 255*img_reverse);
	}
	void Imageprocess::ImgFilling(const Mat &img, Mat &img_fill)
	{
		img.convertTo(img_fill, CV_8UC1);

		Mat img_reverse, img_reverse_label;
		//threshold(img, img_reverse, 1, 1, CV_THRESH_BINARY);
		ImgReverse(img, img_reverse);
		CcaBySeedFill(img_reverse, img_reverse_label);

		for (int i = 1; i < img.rows - 1; i++)
		{
			for (int j = 1; j < img.cols - 1; j++)
			{
				if (img_reverse_label.at<int>(i, j) == 2)
					img_reverse.at<uchar>(i, j) = 0;
				img_fill.at<uchar>(i, j) = img.at<uchar>(i, j) + img_reverse.at<uchar>(i, j);
			}
		}

		//imshow("imgfill", 255 * img_fill);
	}

	void Imageprocess::DetectCornerShiTomasi(const Mat &src, const Mat &colorlabel, Mat &cornerwithimg, int minDistance, double qualityLevel)
	{
		Mat imageGray;
		src.convertTo(imageGray, CV_8UC1);
		vector<Point2f> corners;
		int maxCorners = INT_MAX;
		int blockSize = 3;
		bool useHarrisDetector = false;
		double k = 0.04;

		cornerwithimg = colorlabel.clone();
		/// Apply corner detection :Determines strong corners on an image.
		goodFeaturesToTrack(imageGray, corners, maxCorners, qualityLevel, minDistance, Mat(), blockSize, useHarrisDetector, k);

		// Draw corners detected
		for (int i = 0; i < corners.size(); i++)
		{
			circle(cornerwithimg, corners[i], 3, Scalar(255, 255, 255), 1, 8, 0);
		}
	}

	void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label, const Mat &Corner)
	{

		imwrite("1_Intensity Projection Image.jpg", ProjI);
		imwrite("2_Elevation Projection Image.jpg", ProjZ);
		imwrite("3_Density Projection Image.jpg", ProjD);
		imwrite("4_Intensity Projection Image after Median Filter.jpg", ProjImf);
		imwrite("5_Intensity Gradient Image.jpg", GI);
		imwrite("6_Slope Image.jpg", GZ);
		imwrite("7_Slope Binary Image.jpg", 255 * BZ);
		imwrite("8_Density Binary Image.jpg", 255 * BD);
		imwrite("9_Road Intensity Gradient Image.jpg", GIR);
		imwrite("10_Road Intensity Binary Image.jpg", 255 * BI);
		imwrite("11_CCA Filter Road Intensity Binary Image.jpg", 255 * BIF);
		imwrite("12_RoadMarkings.jpg", Label);
		imwrite("13_Marking Corners.jpg", Corner);

		//cout << "Image Output Done." << endl;
	}

	void Imageprocess::saveimg(std::string outputFolder,
							   int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		
		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}

		string img1, img2, img3, img4, img5, img6, img7, img8, img9, img10, img11, img12;

		img1 = outputFolder + "/" + "1_Intensity Projection Image.jpg";
		img2 = outputFolder + "/" + "2_Elevation Projection Image.jpg";
		img3 = outputFolder + "/" + "3_Density Projection Image.jpg";
		img4 = outputFolder + "/" + "4_Intensity Projection Image after Median Filter.jpg";
		img5 = outputFolder + "/" + "5_Intensity Gradient Image.jpg";
		img6 = outputFolder + "/" + "6_Slope Image.jpg";
		img7 = outputFolder + "/" + "7_Slope Binary Image.jpg";
		img8 = outputFolder + "/" + "8_Density Binary Image.jpg";
		img9 = outputFolder + "/" + "9_Road Intensity Gradient Image.jpg";
		img10 = outputFolder + "/" + "10_Road Intensity Binary Image.jpg";
		img11 = outputFolder + "/" + "11_CCA Filter Road Intensity Binary Image.jpg";
		img12 = outputFolder + "/" + "12_RoadMarkings.jpg";

		imwrite(img1, ProjI);
		imwrite(img2, ProjZ);
		imwrite(img3, ProjD);
		imwrite(img4, ProjImf);
		imwrite(img5, GI);
		imwrite(img6, GZ);
		imwrite(img7, 255 * BZ);
		imwrite(img8, 255 * BD);
		imwrite(img9, GIR);
		imwrite(img10, 255 * BI);
		imwrite(img11, 255 * BIF);
		imwrite(img12, Label);

		cout << "Image Output Done." << endl;
	}

	//hyj0728 without density
	void Imageprocess::saveimg(std::string outputFolder_base,
							   int file_index, const Mat &ProjI, const Mat &ProjZ, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		std::string outputFolder;

		ostringstream oss;
		// oss << "\\Images_"<< file_index;
		oss << "/Images_" << setw(4) << setfill('0') << file_index;
		// outputFolder = outputFolder_base + "\\Geo-reference Images";

		if (!boost::filesystem::exists(outputFolder_base))
		{
			boost::filesystem::create_directory(outputFolder_base);
		}

		outputFolder = outputFolder_base + oss.str();

		if (!boost::filesystem::exists(outputFolder))
		{
			boost::filesystem::create_directory(outputFolder);
		}

		string img1, img2, img3, img4, img5, img6, img7, img8, img9, img10;

		img1 = outputFolder + "/" + "1_Intensity Projection Image.jpg";
		img2 = outputFolder + "/" + "2_Elevation Projection Image.jpg";
		// img3 = outputFolder + "/" + "3_Density Projection Image.jpg";
		img3 = outputFolder + "/" + "3_Intensity Projection Image after Median Filter.jpg";
		img4 = outputFolder + "/" + "4_Intensity Gradient Image.jpg";
		img5 = outputFolder + "/" + "5_Slope Image.jpg";
		img6 = outputFolder + "/" + "6_Slope Binary Image.jpg";
		// img8 = outputFolder + "/" + "8_Density Binary Image.jpg";
		img7 = outputFolder + "/" + "7_Road Intensity Gradient Image.jpg";
		img8 = outputFolder + "/" + "8_Road Intensity Binary Image.jpg";
		img9 = outputFolder + "/" + "9_CCA Filter Road Intensity Binary Image.jpg";
		img10 = outputFolder + "/" + "10_RoadMarkings.jpg";

		imwrite(img1, ProjI);
		imwrite(img2, ProjZ);
		// imwrite(img3, ProjD);
		imwrite(img3, ProjImf);
		imwrite(img4, GI);
		imwrite(img5, GZ);
		imwrite(img6, 255 * BZ);
		// imwrite(img8, 255 * BD);
		imwrite(img7, GIR);
		imwrite(img8, 255 * BI);
		imwrite(img9, 255 * BIF);
		imwrite(img10, Label);

		cout << "Image Output Done." << endl;
	}

	void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjImf, const Mat &GI, const Mat &BI, const Mat &BIF, const Mat &Label)
	{
		imwrite("1_Intensity Projection Image.jpg", ProjI);
		imwrite("2_Intensity Projection Image after Median Filter.jpg", ProjImf);
		imwrite("3_Intensity Gradient Image.jpg", GI);
		imwrite("4_Road Intensity Binary Image.jpg", 255 * BI);
		imwrite("5_CCA Filter Road Intensity Binary Image.jpg", 255 * BIF);
		imwrite("6_RoadMarkings.jpg", Label);

		//cout << "Image Output Done." << endl;
	}

}