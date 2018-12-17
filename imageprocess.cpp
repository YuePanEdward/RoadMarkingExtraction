#include "imageprocess.h" 
#include <numeric>


using namespace image;
using namespace std;

void Imageprocess::pcgrid(const pcXYZIPtr &cloud, float resolution, int &nx, int &ny){ //nx,ny 传引用
	float minx, miny, maxx, maxy, lx, ly;
	int ix, iy;
	minx = miny = FLT_MAX;
	maxx = maxy = -FLT_MAX;
	for (size_t i = 0; i < cloud->points.size(); i++){
		if (minx > cloud->points[i].x) minx = cloud->points[i].x;
		if (miny > cloud->points[i].y) miny = cloud->points[i].y;
		if (maxx < cloud->points[i].x) maxx = cloud->points[i].x;
		if (maxy < cloud->points[i].y) maxy = cloud->points[i].y;
	}
	lx = maxx - minx;
	ly = maxy - miny;
	nx = lx / resolution + 1;
	ny = ly / resolution + 1;
	cout << "nx\t" << nx << "\tny\t" << ny << endl;
}

void Imageprocess:: pc2imgI(const pcXYZIPtr &cloud, float resolution, Mat &img){
	
	float minx, miny, maxx, maxy, maxi, mini, lx, ly;
	int ix, iy,nx,ny;
	minx = miny = mini = FLT_MAX;
	maxx = maxy = maxi = -FLT_MAX;

	for (size_t i = 0; i < cloud->points.size(); i++){
		if (minx > cloud->points[i].x) minx = cloud->points[i].x;
		if (miny > cloud->points[i].y) miny = cloud->points[i].y;
		if (maxx < cloud->points[i].x) maxx = cloud->points[i].x;
		if (maxy < cloud->points[i].y) maxy = cloud->points[i].y;
	}

	lx = maxx - minx;
	ly = maxy - miny;
	nx = lx / resolution + 1;
	ny = ly / resolution + 1;
	vector<vector<vector<float>>> matrixi;	
	vector<vector<float>> ave;
	matrixi.resize(nx);
	ave.resize(nx);

	for (size_t i = 0; i < nx; i++){
		matrixi[i].resize(ny);
		ave[i].resize(ny);
	}

	for (size_t i = 0; i < nx; i++){
		for (size_t j = 0; j < ny; j++){
			matrixi[i][j].push_back(0);
		}
	}
	
	for (size_t i = 0; i < cloud->points.size(); i++){
		ix = (cloud->points[i].x - minx) / resolution;
		iy = (cloud->points[i].y - miny) / resolution;
		matrixi[ix][iy].push_back(cloud->points[i].intensity);
	}
	//先把点扔到格网里去
	
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			ave[i][j] = (1 + accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0)) / matrixi[i][j].size();
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
	}

	for (int i = 0; i < nx; i++)
	{
		uchar *data = img.ptr<uchar>(i);
		for (int j = 0; j < ny; j++)
		{
			int out;
			if (ave[i][j] > mini){
				 out = (int)((ave[i][j] - mini) / (maxi - mini) * 255);
			}
			else{ out = 0; }

	         data[3 * j] = out;
	         data[3 * j + 1] = out;
	         data[3 * j + 2] = out;
	         //pixel 赋值
		}
	}

	//imshow("Projection Image", img);
	/*for (size_t i = 0; i < nx; i++){
		for (size_t j = 0; j < ny; j++)
		{

			img.at<uchar>(i, j) = 1;// accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0) / matrixi[i][j].size();

			//img.at<uchar>(i,j)    灰度图像
			//img.at<Vec3b>(i,j)    rgb 图像

	    }
	}*/

}

void Imageprocess::pc2imgZ(const pcXYZIPtr &cloud, float resolution, Mat &img){

	float minx, miny, maxx, maxy, maxz, minz, lx, ly;
	int ix, iy, nx, ny;
	minx = miny = minz = FLT_MAX;
	maxx = maxy = maxz = -FLT_MAX;

	for (size_t i = 0; i < cloud->points.size(); i++){
		if (minx > cloud->points[i].x) minx = cloud->points[i].x;
		if (miny > cloud->points[i].y) miny = cloud->points[i].y;
		if (maxx < cloud->points[i].x) maxx = cloud->points[i].x;
		if (maxy < cloud->points[i].y) maxy = cloud->points[i].y;
	}

	lx = maxx - minx;
	ly = maxy - miny;
	nx = lx / resolution + 1;
	ny = ly / resolution + 1;
	vector<vector<vector<float>>> matrixz;
	vector<vector<float>> ave;
	matrixz.resize(nx);
	ave.resize(nx);

	for (size_t i = 0; i < nx; i++){
		matrixz[i].resize(ny);
		ave[i].resize(ny);
	}

	for (size_t i = 0; i < nx; i++){
		for (size_t j = 0; j < ny; j++){
			matrixz[i][j].push_back(0);
		}
	}

	for (size_t i = 0; i < cloud->points.size(); i++){
		ix = (cloud->points[i].x - minx) / resolution;
		iy = (cloud->points[i].y - miny) / resolution;
		matrixz[ix][iy].push_back(cloud->points[i].z);
	}
	//先把点扔到格网里去

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			ave[i][j] = (1 + accumulate(begin(matrixz[i][j]), end(matrixz[i][j]), 0.0)) / matrixz[i][j].size();
			//格网里点再取平均

		}
	}

	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			maxz = max(maxz, ave[i][j]);
			minz = min(minz, ave[i][j]);
		}
	}

	for (int i = 0; i < nx; i++)
	{
		uchar *data = img.ptr<uchar>(i);
		for (int j = 0; j < ny; j++)
		{
			int out;
			if (ave[i][j] > minz){
				out = (int)((ave[i][j] - minz) / (maxz - minz) * 255);
			}
			else{ out = 0; }

			data[3 * j] = out;
			data[3 * j + 1] = out;
			data[3 * j + 2] = out;
			//pixel 赋值
		}
	}

	//imshow("Projection Image", img);
	/*for (size_t i = 0; i < nx; i++){
	for (size_t j = 0; j < ny; j++)
	{

	img.at<uchar>(i, j) = 1;// accumulate(begin(matrixi[i][j]), end(matrixi[i][j]), 0.0) / matrixi[i][j].size();

	//img.at<uchar>(i,j)    灰度图像
	//img.at<Vec3b>(i,j)    rgb 图像

	}
	}*/
}

Mat Imageprocess::Sobelboundary(Mat img0)
{
	//Using Sobel Operation
	Mat grayImage, grad_xg, grad_yg, abs_grad_xg, abs_grad_yg, dstg;

	//将原始图转化为灰度图
	cvtColor(img0, grayImage, COLOR_BGR2GRAY);
	//求x方向梯度
	Sobel(grayImage, grad_xg, CV_16S, 1, 0, 3, 1, 1, BORDER_DEFAULT);
	convertScaleAbs(grad_xg, abs_grad_xg);
	//求y方向梯度
	Sobel(grayImage, grad_yg, CV_16S, 0, 1, 3, 1, 1, BORDER_DEFAULT);
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

Mat Imageprocess::ExtractRoadPixel(const Mat & _binI, const Mat & _binZ)
{
	Mat result;
	_binI.convertTo(result, CV_8UC1);
	int mini, minj, maxi, maxj;
	vector <int> arrayi, arrayj;
	for (int i = 0; i < _binI.rows; i++)
	{
		for (int j = 0; j < _binI.cols; j++)
		{
			if (_binZ.at<uchar>(i, j) == 1)
			{
				result.at<uchar>(i, j) = 0;
			}
			else{ result.at<uchar>(i, j) = _binI.at<uchar>(i, j); }
			
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


	if (_binImg.empty() ||
		_binImg.type() != CV_8UC1)
	{
		return;
	}

	_lableImg.release();
	_binImg.convertTo(_lableImg, CV_32SC1);

	int label = 1;  // start by 2

	int rows = _binImg.rows - 1;
	int cols = _binImg.cols - 1;
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

	std::map<int, Scalar> colors;

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

Mat Imageprocess::Truncate(const Mat & Img)
{
	Mat result0;
	Mat result;
	int mini, minj, maxi, maxj;
	mini = INT_MAX; minj = INT_MAX;
	maxi = 0; maxj = 0;
	for (int i = 0; i < Img.rows; i++)
	{
		for (int j = 0; j < Img.cols; j++)
		{
			if (Img.at<uchar>(i, j) != 0)
			{
				mini = min(i, mini);
				minj = min(j, minj);
				maxi = max(i, maxi);
				maxj = max(i, maxj);
			}
		}
	}
	//cout << *maxj - *minj << endl << *maxi - *mini << endl;
	Rect rect(minj, mini, maxj - minj, maxi - mini);
	result = Img(rect);
	return result;
}

void Imageprocess::saveimg(const Mat &ProjI, const Mat &ProjZ, const Mat &GI, const Mat &GZ, const Mat &BI, const Mat &BZ, const Mat &BI2, const Mat &Label)
{
imwrite("1_Intensity Projection Image.jpg", ProjI);
imwrite("2_Elevation Projection Image.jpg", ProjZ);
imwrite("3_Intensity Gradient Image.jpg", GI);
imwrite("4_Slope Image.jpg", GZ);
imwrite("5_Intensity Binary Image.jpg", 255*BI);
imwrite("6_Slope Binary Image.jpg", 255*BZ);
imwrite("7_Road Intensity Binary Image.jpg", 255*BI2);
imwrite("8_RoadMarkings.jpg", Label);

cout << "Image Output Done." << endl;
}


