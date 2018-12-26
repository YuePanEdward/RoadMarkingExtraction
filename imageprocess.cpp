#include "imageprocess.h" 
#include <numeric>


using namespace image;
using namespace std;

void Imageprocess::pcgrid(vector<double> &boundingbox, float resolution){
	//boundingbox order: Xmin,Ymin,Zmin,Xmax,Ymax,Zmax
	float lx, ly;

	lx = boundingbox[3] - boundingbox[0];
	ly = boundingbox[4] - boundingbox[1];

	minX = boundingbox[0];
	minY = boundingbox[1];
	minZ = boundingbox[2];

	nx = lx / resolution + 1;
	ny = ly / resolution + 1;

	res = resolution;

	cout << "Image Size: " << nx << " * " << ny << endl;
}


void Imageprocess::savepcgrid(vector<double> &boundingbox, float resolution, const pcXYZIPtr &c, const pcXYZIPtr &gc, const pcXYZIPtr &ngc)
{   
	float lx, ly;
	int ix, iy;
	lx = boundingbox[3] - boundingbox[0];
	ly = boundingbox[4] - boundingbox[1];

	minX = boundingbox[0];
	minY = boundingbox[1];
	minZ = boundingbox[2];

	nx = lx / resolution + 1;
	ny = ly / resolution + 1;

	res = resolution;

	cout << "Image Size: " << nx << " * " << ny << endl;
	
	CMatrixIndice.resize(nx);
	GCMatrixIndice.resize(nx);
	NGCMatrixIndice.resize(nx);

	for (size_t i = 0; i < nx; i++){
		CMatrixIndice[i].resize(ny);
		GCMatrixIndice[i].resize(ny);
		NGCMatrixIndice[i].resize(ny);
	}

	//Saving point indices  (遍历一次点云 [耗时],想办法地面分割的时候存点序号，这样就不用再遍历两遍了）
	for (size_t i = 0; i < c->points.size(); i++){
		ix = (c->points[i].x - minX) / res;
		iy = (c->points[i].y - minY) / res;
		CMatrixIndice[ix][iy].push_back(i);
	}
	for (size_t i = 0; i < gc->points.size(); i++){
		ix = (gc->points[i].x - minX) / res;
		iy = (gc->points[i].y - minY) / res;
		GCMatrixIndice[ix][iy].push_back(i);
	}
	for (size_t i = 0; i < ngc->points.size(); i++){
		ix = (ngc->points[i].x - minX) / res;
		iy = (ngc->points[i].y - minY) / res;
		NGCMatrixIndice[ix][iy].push_back(i);
	}

}
void Imageprocess::pc2imgI(const pcXYZIPtr &cloud, int whatcloud, Mat &img ){
	

	float mini,maxi;  //min and max Intensity

	img.create(nx, ny, CV_8UC1); //分配内存
    mini = FLT_MAX;
    maxi = -FLT_MAX;

	vector<vector<vector<float>>> matrixi;	
	vector<vector<float>> ave;
	
	matrixi.resize(nx);
	ave.resize(nx);

	for (int i = 0; i < nx; i++){
		matrixi[i].resize(ny);
		ave[i].resize(ny);
		for (int j = 0; j < ny; j++){
			matrixi[i][j].push_back(0);
		}
	}

	switch (whatcloud){
	   case 0:  //Original Point Cloud
		   for (int i = 0; i < nx; i++){
			   for (int j = 0; j < ny; j++){
				  for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					matrixi[i][j].push_back(cloud->points[*k].intensity);
			   }
		    }
		   break;
	   case 1:  //Ground Point Cloud
		   for (int i = 0; i < nx; i++){
			   for (int j = 0; j < ny; j++){
				   for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					   matrixi[i][j].push_back(cloud->points[*k].intensity);
			   }
		   }
		   break;
	   case 2:  //Non-Ground Point Cloud
		   for (int i = 0; i < nx; i++){
			   for (int j = 0; j < ny; j++){
				   for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					   matrixi[i][j].push_back(cloud->points[*k].intensity);
			   }
		   }
		   break;
	}

	for (int i = 0; i < nx; i++){
		for (int j = 0; j < ny; j++){
			ave[i][j] = (0.1 + accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0)) / matrixi[i][j].size();
			//格网里点再取平均
		}
	}

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			maxi = max(maxi, ave[i][j]);
			mini = min(mini, ave[i][j]);
		}
	}// 求取最大，最小格网平均反射强度


	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			img.at<uchar>(i, j) = 255 * (ave[i][j] - mini) / (maxi - mini);
	         //pixel 赋值
		}
	}
}

void Imageprocess::pc2imgZ(const pcXYZIPtr &cloud, int whatcloud,  Mat &img){

	float minaz, maxaz;   //maxaz,minaz 是图片所代表的max和minz（是像素内的均值）
	img.create(nx, ny, CV_8UC1); //分配内存

	minaz =  FLT_MAX;
	maxaz = -FLT_MAX;

	vector<vector<vector<float>>> matrixz;
	vector<vector<float>> ave;
	matrixz.resize(nx);
	ave.resize(nx);

	for (int i = 0; i < nx; i++){
		matrixz[i].resize(ny);
		ave[i].resize(ny);
		for (int j = 0; j < ny; j++){
			matrixz[i][j].push_back(minZ); //用最低点高程补空
		}
	}
	
	switch (whatcloud){
	case 0:  //Original Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				for (auto k = CMatrixIndice[i][j].begin(); k != CMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					matrixz[i][j].push_back(cloud->points[*k].z);
			}
		}
		break;
	case 1:  //Ground Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					matrixz[i][j].push_back(cloud->points[*k].z);
			}
		}
		break;
	case 2:  //Non-Ground Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				for (auto k = NGCMatrixIndice[i][j].begin(); k != NGCMatrixIndice[i][j].end(); ++k) //迭代器，以后多用迭代器，少用下标
					matrixz[i][j].push_back(cloud->points[*k].z);
			}
		}
		break;
	}

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			ave[i][j] = (0.1 + accumulate(begin(matrixz[i][j]), end(matrixz[i][j]), 0.0)) / matrixz[i][j].size();
			//格网里点再取平均
		}
	}

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			maxaz = max(maxaz, ave[i][j]);
			minaz = min(minaz, ave[i][j]);
		}
	}// 求取最大，最小格网平均高程

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			img.at<uchar>(i, j) = 255 * (ave[i][j] - minaz) / (maxaz - minaz);
			//pixel 赋值
		}
	}
}

void Imageprocess::pc2imgD(const pcXYZIPtr &cloud, int whatcloud, Mat &img){

	img.create(nx, ny, CV_8UC1); //分配内存
	Eigen::MatrixXi Matrixnum;
	int maxnum, maxelement;
	float expectedmaxnum;
	Matrixnum.resize(nx, ny);
	Matrixnum = Eigen::MatrixXi::Zero(nx, ny);
	
	switch (whatcloud){
	case 0:  //Original Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				Matrixnum(i,j) = CMatrixIndice[i][j].size();
			}
		}
		break;
	case 1:  //Ground Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				Matrixnum(i,j) = GCMatrixIndice[i][j].size();
			}
		}
		break;
	case 2:  //Non-Ground Point Cloud
		for (int i = 0; i < nx; i++){
			for (int j = 0; j < ny; j++){
				Matrixnum(i,j) = NGCMatrixIndice[i][j].size();
			}
		}
		break;
	}

	expectedmaxnum = res / 0.008;  // 0.01 is set empirically.  Point Spacing
	//maxelement = Matrixnum.maxCoeff();
	//if (maxelement < expectedmaxnum) maxnum = maxelement;
	//else maxnum = expectedmaxnum;
	maxnum = expectedmaxnum;

	//cout << "max point number"<<maxnum<<endl;
	for (size_t i = 0; i < nx; i++)
	{
		for (size_t j = 0; j < ny; j++)
		{
			int out;
			if (Matrixnum(i, j) < maxnum) out = 255 * Matrixnum(i, j) / maxnum;
			else out = 255;
			img.at<uchar>(i, j) = out;
			//pixel 赋值
		}
	}
}

void Imageprocess::img2pc(const Mat &img, const pcXYZIPtr &incloud, pcXYZIPtr & outcloud)
{
	/*timin = 0;
	tjmin = 0;*/
	
	Mat grayImage,binImage;
	cvtColor(img, grayImage, COLOR_BGR2GRAY);
	threshold(grayImage, binImage, 0 , 1, CV_THRESH_BINARY);
	
	for (size_t i = timin; i <timin+ img.rows; i++)
	{
		for (size_t j = tjmin; j < tjmin+img.cols; j++)
		{
			if (binImage.at<uchar>(i-timin, j-tjmin) == 1)
			{
				for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
					outcloud->points.push_back(incloud->points[*k]);
			}
		
		}
	}

}

void Imageprocess::img2pclabel(const Mat &img, const pcXYZIPtr &incloud, vector<pcXYZI> &outclouds , double dZ)
{
	/*timin = 0;
	tjmin = 0;*/
	int classNo = 0;
	
	outclouds.resize(totallabel-1);
	
	for (int i = timin; i < timin+img.rows; i++)
	{
		const int* data_src = (int*)img.ptr<int>(i-timin);
		for (int j = tjmin; j < tjmin+img.cols; j++)
		{
			int pixelValue = data_src[j-tjmin];
			
			//在这儿就滤dZ比较好，一个个像素的滤，更安全
			if (pixelValue > 1)
			{
				double max_z, min_z, disz;
				max_z = -DBL_MAX;
				min_z = DBL_MAX;


				for (int k = 0; k < GCMatrixIndice[i][j].size(); k++){
				
					if (max_z < incloud->points[GCMatrixIndice[i][j][k]].z)  max_z = incloud->points[GCMatrixIndice[i][j][k]].z;
					if (min_z > incloud->points[GCMatrixIndice[i][j][k]].z)  min_z = incloud->points[GCMatrixIndice[i][j][k]].z;
			
				}
				disz = max_z - min_z;
				if (disz < dZ){
					for (auto g = GCMatrixIndice[i][j].begin(); g != GCMatrixIndice[i][j].end(); ++g)
						outclouds[pixelValue - 2].points.push_back(incloud->points[*g]);
				}
				else
				{
					if (outclouds[pixelValue - 2].size()==0)
					outclouds[pixelValue - 2].points.push_back(incloud->points[GCMatrixIndice[i][j][0]]);//避免出现空点现象，之后再按数量滤除
				}
			}
		}

	}

	/*for (size_t i = 0; i < nx; i++) //The same
	{
		for (size_t j = 0; j < ny; j++)
		{
			if (img.at<int>(i, j) > 1) //注意输入是 32S的，不能再uchar了
			{
				for (auto k = GCMatrixIndice[i][j].begin(); k != GCMatrixIndice[i][j].end(); ++k)
					outclouds[img.at<int>(i, j)-2].points.push_back(incloud->points[*k]);
			}
		}
	}*/
	
	for (int k = 0; k <= totallabel - 2; k++)
	{
		if (outclouds[k].size()>0) classNo++;
	}
	cout << "Cloud Number: " << classNo << endl;
}

Mat Imageprocess::Sobelboundary(Mat img0)
{
	//Using Sobel Operation
	Mat grad_xg, grad_yg, abs_grad_xg, abs_grad_yg, dstg;

	//将原始图转化为灰度图
	//cvtColor(img0, grayImage, COLOR_BGR2GRAY);
	//求x方向梯度
	Sobel(img0, grad_xg, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_xg, abs_grad_xg);
	//求y方向梯度S
	Sobel(img0, grad_yg, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_yg, abs_grad_yg);
	//合并梯度
	addWeighted(abs_grad_xg, 0.5, abs_grad_yg, 0.5, 0, dstg);
	return dstg;
}

float Imageprocess::caculateCurrentEntropy(Mat hist, int threshold)
{
	float BackgroundSum = 0, targetSum = 0;
	const float* pDataHist = (float*)hist.ptr<float>(0);
	for (int i = 0; i < 256; i++)
	{
		//累计背景值
		if (i < threshold)
		{
			BackgroundSum += pDataHist[i];
		}
		//累计前景值
		else
		{
			targetSum += pDataHist[i];
		}
	}
	float BackgroundEntropy = 0, targetEntropy = 0;
	for (int i = 0; i < 256; i++)
	{
		//计算背景熵
		if (i < threshold)
		{
			if (pDataHist[i] == 0)
				continue;
			float ratio1 = pDataHist[i] / BackgroundSum;
			BackgroundEntropy += -ratio1*logf(ratio1);
		}
		else  //计算前景熵
		{
			if (pDataHist[i] == 0)
				continue;
			float ratio2 = pDataHist[i] / targetSum;
			targetEntropy += -ratio2*logf(ratio2);
		}
	}
	return (targetEntropy + BackgroundEntropy);  //加和，得到当前阈值的熵
}


Mat Imageprocess::maxEntropySegMentation(Mat inputImage)
{   
	// Max Entropy Binarization 
	// Using the distribution of histogram to calculate the threshold leading to the max entropy.
	const int channels[1] = { 0 };
	const int histSize[1] = { 256 };
	float pranges[2] = { 0, 256 };
	const float* ranges[1] = { pranges };
	MatND hist;
	calcHist(&inputImage, 1, channels, Mat(), hist, 1, histSize, ranges);
	float maxentropy = 0;
	int max_index = 0;
	Mat result;
	for (int i = 0; i < 256; i++)  //遍历256个值作为阈值， 求取最大熵值
	{
		float cur_entropy = caculateCurrentEntropy(hist, i);
		if (cur_entropy > maxentropy)
		{
			maxentropy = cur_entropy;
			max_index = i;
		}
	}
	threshold(inputImage, result, max_index, 1, CV_THRESH_BINARY);  // > max_index assign as 1   < max_index assign as 0
	return result;
}

Mat Imageprocess::ExtractRoadPixel(const Mat & _imgI, const Mat & _binZ ,const Mat & _binD)
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
			else{ result.at<uchar>(i, j) = _imgI.at<uchar>(i, j); }
			
			/*if (result.at<uchar>(i, j) == 1)
			{
				arrayi.push_back(i);
				arrayj.push_back(j);
			}*/
		}
	}
	return result;

	 /*maxi = *(std::max_element(std::begin(arrayi), std::end(arrayi)));
	 mini = *(std::min_element(std::begin(arrayi), std::end(arrayi)));
	 maxj = *(std::max_element(std::begin(arrayj), std::end(arrayj)));
	 minj = *(std::min_element(std::begin(arrayj), std::end(arrayj)));

	 Rect rect(mini, minj, maxi-mini+1, maxj-minj+1);
	 Mat ROI = result(rect);
	 return ROI;*/
}
void Imageprocess::CcaByTwoPass(const Mat & _binImg, Mat & _labelImg)
{
	// connected component analysis (8-component)
	// use two-pass algorithm 两遍扫描法
	// 1. first pass: label each foreground pixel with a label
	// 2. second pass: visit each labeled pixel and merge neighbor labels
	// 
	// foreground pixel: _binImg(x,y) = 1
	// background pixel: _binImg(x,y) = 0

	//  reference: https://blog.csdn.net/icvpr/article/details/10259577 


	if (_binImg.empty() ||
		_binImg.type() != CV_8UC1)
	{
		return;
	}

	// 1. first pass

	_labelImg.release();
	_binImg.convertTo(_labelImg, CV_32SC1);                             // _labelImg -> _binImg  32 Signed 为了能分足够多类来label，所以用这种32位的

	int label = 1;                                                      // start by 2
	vector<int> labelSet;                                               // 用来存label
	labelSet.push_back(0);                                              // background: 0
	labelSet.push_back(1);                                              // foreground: 1

	int rows = _binImg.rows - 1;
	int cols = _binImg.cols - 1;
	for (int i = 1; i < rows; i++)                                      // 行遍历
	{
		int* data_preRow = _labelImg.ptr<int>(i - 1);                   // 指向上一行
		int* data_curRow = _labelImg.ptr<int>(i);                       // 指向这一行
		for (int j = 1; j < cols; j++)                                  // 列遍历
		{
			if (data_curRow[j] == 1)                                    // 若 当前行 该像素 是前景 （就是还没有label）
			{
				vector<int> neighborLabels;                             // 创建邻域label数组
				neighborLabels.reserve(2);                              // 预留空间
				int leftPixel = data_curRow[j - 1];                     // 存左邻像素
				int upPixel = data_preRow[j];                           // 存上邻像素
				//int leftupPixel = data_preRow[j - 1];                   // 存左上邻像素

				if (leftPixel > 1)                                      // 若左邻像素有自己的label
				{
					neighborLabels.push_back(leftPixel);                // 邻域label 里把左邻label加进去
				}
				if (upPixel > 1)                                        // 若上邻像素有自己的label
				{
					neighborLabels.push_back(upPixel);                  // 邻域label 里把上邻label加进去
				}
				/*if (leftupPixel > 1)                                    // 若左上邻像素有自己的label
				{
					neighborLabels.push_back(leftPixel);                // 邻域label 里把左上邻label加进去
				}*/

				if (neighborLabels.empty())                             // 若邻域都还没label 
				{
					labelSet.push_back(++label);                        // assign to a new label
					data_curRow[j] = label;                             // 当前像素标为该label
					labelSet[label] = label;                            
				}
				else                                                    // 不然 （即邻域存在label了）
				{ 
					sort(neighborLabels.begin(), neighborLabels.end()); // 邻域label排序 从小到大来
					int smallestLabel = neighborLabels[0];              
					data_curRow[j] = smallestLabel;                     // 当前像素赋最小的label

					// save equivalence
					for (size_t k = 1; k < neighborLabels.size(); k++)  // 遍历邻域label们
					{
						int tempLabel = neighborLabels[k];              // 
						int& oldSmallestLabel = labelSet[tempLabel];
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
	// 记录Neighbors中各个值（label）之间的相等关系，即这些值（label）同属同一个连通区域；  
	for (size_t i = 2; i < labelSet.size(); i++)  // 0,1 不算
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
		int* data = _labelImg.ptr<int>(i);
		for (int j = 0; j < cols; j++)
		{
			int& pixelLabel = data[j];                                    
			pixelLabel = labelSet[pixelLabel];
		}
	}
	totallabel = label;
	//cout << "Number label: " << totallabel << endl;
}


void Imageprocess:: CcaBySeedFill(const Mat& _binImg, Mat& _lableImg)
{
	// connected component analysis (8-component)
	// use seed filling algorithm
	// 1. begin with a foreground pixel and push its foreground neighbors into a stack;
	// 2. pop the top pixel on the stack and label it with the same label until the stack is empty
	// 
	// foreground pixel: _binImg(x,y) = 1
	// background pixel: _binImg(x,y) = 0

	// 注意种子填充法要求边缘一个像素都得为0，为1的话就下标越界了

	if (_binImg.empty() ||
		_binImg.type() != CV_8UC1)
	{
		cout << "Wrong type" << endl;
		return;
	}

	_lableImg.release();
	_binImg.convertTo(_lableImg, CV_32SC1);    //_labelImg 是 CV_32SC1的

	int label = 1;  // start by 2
	//vector<vector<pair<int, int>>> labeledPixel;
	//labeledPixel.resize(10000); // 这样写不太严谨，认为最多10000个类

	int rows = _binImg.rows;
	int cols = _binImg.cols;
	for (int i = 1; i < rows - 1; i++)
	{
		int* data = _lableImg.ptr<int>(i);
		for (int j = 1; j < cols - 1; j++)
		{
			if (data[j] == 1)
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(i, j));     // pixel position: <i,j>
				++label;  // begin with a new label
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
					{// left pixel
						neighborPixels.push(std::pair<int, int>(curX, curY - 1));
					}
					if (_lableImg.at<int>(curX, curY + 1) == 1)
					{// right pixel
						neighborPixels.push(std::pair<int, int>(curX, curY + 1));
					}
					if (_lableImg.at<int>(curX - 1, curY) == 1)
					{// up pixel
						neighborPixels.push(std::pair<int, int>(curX - 1, curY));
					}
					if (_lableImg.at<int>(curX + 1, curY) == 1)
					{// down pixel
						neighborPixels.push(std::pair<int, int>(curX + 1, curY));
					}
					if (_lableImg.at<int>(curX - 1, curY - 1) == 1)
					{// left up pixel
						neighborPixels.push(std::pair<int, int>(curX -1, curY - 1));
					}
					if (_lableImg.at<int>(curX - 1, curY + 1) == 1)
					{// left down pixel
						neighborPixels.push(std::pair<int, int>(curX - 1, curY + 1));
					}
					if (_lableImg.at<int>(curX + 1, curY - 1) == 1)
					{// right up pixel
						neighborPixels.push(std::pair<int, int>(curX + 1, curY - 1));
					}
					if (_lableImg.at<int>(curX + 1, curY + 1) == 1)
					{// right down pixel
						neighborPixels.push(std::pair<int, int>(curX + 1, curY + 1));
					}
				}
			}
		}
	}
	/*int labelnumber=label-1;
	for (int m = 2; m <= label; m++){
		if (labeledPixel[m].size() < K){
			for (int n = 0; n < labeledPixel[m].size(); n++){
				int del_i = labeledPixel[m][n].first;
				int del_j = labeledPixel[m][n].second;
				_lableImg.at<int>(del_i, del_j) = 0;
			}
			labelnumber--;
		}
	}*/
	totallabel = label;

	//cout << "Number label: " << totallabel << endl;
}


void Imageprocess::RemoveSmallRegion(const Mat &Src, Mat &Dst, int AreaLimit)
	{
		int RemoveCount = 0;
		//新建一幅标签图像初始化为0像素点，为了记录每个像素点检验状态的标签，0代表未检查，1代表正在检查,2代表检查不合格（需要反转颜色），3代表检查合格或不需检查   
		//初始化的图像全部为0，未检查  
		Src.convertTo(Dst, CV_8UC1);
		int CheckMode = 1; //白色视为连通域
		int NeihborMode = 1; //8邻域
		Mat PointLabel = Mat::zeros(Src.size(), CV_8UC1);
		if (CheckMode == 1)//去除小连通区域的白色点  
		{
			//cout << "去除小连通域.";
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) < 1)
					{
						PointLabel.at<uchar>(i, j) = 3;//将背景黑色点标记为合格，像素为3     
					}
				}
			}
		}
		else//去除孔洞，黑色点像素  
		{
			//cout << "去除孔洞";
			for (int i = 0; i < Src.rows; i++)
			{
				for (int j = 0; j < Src.cols; j++)
				{
					if (Src.at<uchar>(i, j) > 10)
					{
						PointLabel.at<uchar>(i, j) = 3;//如果原图是白色区域，标记为合格，像素为3  
					}
				}
			}
		}


		vector<Point2i>NeihborPos;//将邻域压进容器  
		NeihborPos.push_back(Point2i(-1, 0));
		NeihborPos.push_back(Point2i(1, 0));
		NeihborPos.push_back(Point2i(0, -1));
		NeihborPos.push_back(Point2i(0, 1));
		if (NeihborMode == 1)
		{
			//cout << "Neighbor mode: 8邻域." << endl;
			NeihborPos.push_back(Point2i(-1, -1));
			NeihborPos.push_back(Point2i(-1, 1));
			NeihborPos.push_back(Point2i(1, -1));
			NeihborPos.push_back(Point2i(1, 1));
		}
		else int a = 0;//cout << "Neighbor mode: 4邻域." << endl;
		int NeihborCount = 4 + 4 * NeihborMode;
		int CurrX = 0, CurrY = 0;
		//开始检测  
		for (int i = 0; i < Src.rows; i++)
		{
			for (int j = 0; j < Src.cols; j++)
			{
				if (PointLabel.at<uchar>(i, j) == 0)//标签图像像素点为0，表示还未检查的不合格点  
				{   //开始检查  
					vector<Point2i>GrowBuffer;//记录检查像素点的个数  
					GrowBuffer.push_back(Point2i(j, i));
					PointLabel.at<uchar>(i, j) = 1;//标记为正在检查  
					int CheckResult = 0;

					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						for (int q = 0; q < NeihborCount; q++)
						{
							CurrX = GrowBuffer.at(z).x + NeihborPos.at(q).x;
							CurrY = GrowBuffer.at(z).y + NeihborPos.at(q).y;
							if (CurrX >= 0 && CurrX < Src.cols&&CurrY >= 0 && CurrY < Src.rows)  //防止越界    
							{
								if (PointLabel.at<uchar>(CurrY, CurrX) == 0)
								{
									GrowBuffer.push_back(Point2i(CurrX, CurrY));      //邻域点加入buffer    
									PointLabel.at<uchar>(CurrY, CurrX) = 1;           //更新邻域点的检查标签，避免重复检查    
								}
							}
						}
					}
					if (GrowBuffer.size() > AreaLimit) //判断结果（是否超出限定的大小），1为未超出，2为超出    
						CheckResult = 2;
					else
					{
						CheckResult = 1;
						RemoveCount++;//记录有多少区域被去除  
					}

					for (int z = 0; z < GrowBuffer.size(); z++)
					{
						CurrX = GrowBuffer.at(z).x;
						CurrY = GrowBuffer.at(z).y;
						PointLabel.at<uchar>(CurrY, CurrX) += CheckResult;//标记不合格的像素点，像素值为2  
					}
					//********结束该点处的检查**********    
				}
			}
		}
		CheckMode = 255 * (1 - CheckMode);
		//开始反转面积过小的区域    
		for (int i = 0; i < Src.rows; ++i)
		{
			for (int j = 0; j < Src.cols; ++j)
			{
				if (PointLabel.at<uchar>(i, j) == 2)
				{
					Dst.at<uchar>(i, j) = 0;
				}
				//else if (PointLabel.at<uchar>(i, j) == 3)
				else
                {
					Dst.at<uchar>(i, j) = Src.at<uchar>(i, j);

				}
			}
		}
		//cout << RemoveCount << " objects removed." << endl;
}



Scalar Imageprocess::GetRandomColor()
{
	uchar r = 255 * (rand() / (1.0 + RAND_MAX));   // rand() / (1.0+ RAND_MAX) : a random float number between 0 and 1 (can't be equal to 1)
	uchar g = 255 * (rand() / (1.0 + RAND_MAX));   // rand() 0 ~ 0x7fff （32767）  RAND_MAX = 32767
	uchar b = 255 * (rand() / (1.0 + RAND_MAX));
	return Scalar(b, g, r);
}


void Imageprocess::LabelColor(const Mat & _labelImg, Mat & _colorLabelImg)
{
	if (_labelImg.empty() ||
		_labelImg.type() != CV_32SC1)
	{
		return;
	}

	std::map<int, Scalar> colors;  //映射

	//labelx.resize(totallabel - 1);
	//labely.resize(totallabel - 1);

	int rows = _labelImg.rows;
	int cols = _labelImg.cols;

	_colorLabelImg.release();
	_colorLabelImg.create(rows, cols, CV_8UC3);
	_colorLabelImg = Scalar::all(0);

	for (int i = 0; i < rows; i++)
	{
		const int* data_src = (int*)_labelImg.ptr<int>(i);
		uchar* data_dst = _colorLabelImg.ptr<uchar>(i);
		for (int j = 0; j < cols; j++)
		{
			int pixelValue = data_src[j];
			if (pixelValue > 1)
			{
				//if(j%100==0) cout << pixelValue << endl;
				//labelx[pixelValue - 2].push_back(i); // 确定各Label所占据的像素之x，y
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
void Imageprocess::Truncate(const Mat & Img, Mat & TruncatedImg)
{
	int mini, minj, maxi, maxj, di, dj;
	mini = INT_MAX; minj = INT_MAX;
	maxi = 0; maxj = 0;
	for (int i = 0; i < Img.rows; i++)
	{
		for (int j = 0; j < Img.cols; j++)
		{
			if (Img.at<uchar>(i, j) != 0)
			{
				if (i<mini)mini = i;
				if (i>maxi)maxi = i;
				if (j<minj)minj = j;
				if (j>maxj)maxj = j;
			}
		}
	}
   timin = mini-1;  
   tjmin = minj-1;

   
	di = maxi - mini+3;
	dj = maxj - minj+3;

	/*TruncatedImg.create(di, dj, CV_8UC1);
	for (int i = 0; i < di; i++)
	{
		for (int j = 0; j < dj; j++)
		{
			TruncatedImg.at<uchar>(i, j) = Img.at<uchar>(timin + i, tjmin + j);
		}
	}*/
	
	//cout << "X: " << tjmin << "  Y: " << timin << "  dX: " << dj << "  dY: " << di<< endl;
	Rect rect(tjmin, timin, dj, di);  
	TruncatedImg = Img(rect);
	//imshow("Truncated", 255*TruncatedImg);
}

void Imageprocess::DetectCornerHarris(const Mat & src, const Mat & colorlabel, Mat & cornershow, Mat & cornerwithimg, int threshold)
{
	Mat corner, corner8u, imageGray;
	src.convertTo(imageGray, CV_8UC1);
	corner = Mat::zeros(src.size(), CV_32FC1);
	cornerHarris(imageGray, corner, 3, 3, 0.04, BORDER_DEFAULT);
	normalize(corner, corner8u, 0, 255, CV_MINMAX);  //归一化
	convertScaleAbs(corner8u, cornershow);
	cornerwithimg = colorlabel.clone();
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			if (cornershow.at<uchar>(i, j)>threshold)  //阈值判断
			{
				circle(cornerwithimg, Point(j, i), 2, Scalar(255, 255, 255), 2); //标注角点
			}
		}
	}
}
//bug的根源在于CCA 种子填充对边缘像素的处理，下标越界
void Imageprocess::ImgReverse(const Mat &img, Mat &img_reverse){
	img.convertTo(img_reverse, CV_8UC1);

	for (int i = 1; i < img.rows-1; i++)
	{
		for (int j = 1; j < img.cols-1; j++)
		{
			if (img.at<uchar>(i, j) == 1)img_reverse.at<uchar>(i, j) = 0;
			if (img.at<uchar>(i, j) == 0)img_reverse.at<uchar>(i, j) = 1; 
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

	for (int i = 1; i < img.rows-1; i++)
	{
		for (int j = 1; j < img.cols-1; j++)
		{
			if (img_reverse_label.at<int>(i, j) == 2)
				img_reverse.at<uchar>(i, j) = 0;
			img_fill.at<uchar>(i, j) = img.at<uchar>(i, j) + img_reverse.at<uchar>(i, j);
		
		}
	}

	//imshow("imgfill", 255 * img_fill);
}

void Imageprocess::DetectCornerShiTomasi(const Mat & src, const Mat & colorlabel, Mat & cornerwithimg, int minDistance, double qualityLevel)
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
	for (int i = 0; i < corners.size(); i++){
		circle(cornerwithimg, corners[i], 3, Scalar(255, 255, 255), 1, 8, 0);
	}
}

void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &ProjD, const Mat &ProjImf, const Mat &GI, const Mat &GZ, const Mat &BZ, const Mat &BD, const Mat &GIR, const Mat &BI, const Mat &BIF, const Mat &Label/*const Mat &Corner*/)
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
//imwrite("12_Marking Corners.jpg",Corner);

cout << "Image Output Done." << endl;
}


