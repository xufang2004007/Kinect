//----------------------------【程序的功能】-----------------------------------------------------
// 1、用opencv和kinect sdk共同完成kinect彩色和深度图数据读取和处理工作；
// 2、将彩色图像和深度图像都转化成摄像头坐标下的3D位置信息，可以不用做内参标定
// 3、根据目标物的方位生成机器人路径规划；
// 4、最终生成机器人的路径规划多项式系数，并完成与TwinCAT的数据交互
//---------------------------------------------------------------------------------

//----------------------------【windows头文件】------------------------------------
#include "stdafx.h"
#include <iostream>						// 数据流
#include <cmath>
#include <vector>
#include <stdio.h>
#include "time.h"
//----------------------------【kinec相关t头文件】---------------------------------
#include "nuiapi.h"						// kinetc数据api
#include "Imagebasics.h"				// 深度图像基础定义
#include "kinect_depthdata.h"			// 深度图像基础数据
#include "Kinect_function.h"
//----------------------------【opencv头文件】-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// 包含所有的opencv2头文件
//----------------------------【霍夫检测算法头文件】-------------------------------------
#include "Ellipse.cpp"

//----------------------------【命名空间定义】-------------------------------------
using namespace std;
using namespace cv;
RNG g_rng(12345);

//--------------------------------【结构体定义】------------------------------------------
struct Data_Vector
{
	double Mean_Vector;
	double Std_Vecotr;
};

struct Data_ImageRemoveMess
{
	Mat				outputImage;
	Data_Vector		data_iImage2std;
};

//----------------------------【函数与全局变量声明】-----------------------------------------
BOOL CreateFirstConnected();					// Kinect连接并打开数据流
int Depth_Process(HANDLE h);					// 深度图处理程序
int Color_Process(HANDLE h);					// 彩色图处理程序
int Mapping_Color_To_Skeletion(void);			// 彩色图向摄像头坐标系的位置转换

int Target_Recoginition(Mat Color_Image);														// 目标识别函数
vector<Rect> on_Matching(Mat & srcImage, Mat & templateImage);									// 模板匹配函数
Mat Image_Segmentation(Mat & srcImage, vector<Rect> & Segmentation_Area, uchar threshould);		// 图像分割函数
Data_Vector Mean_square_deviation(vector<vector<double>> & inputVector, int Mode);				// 均方差求取函数
double Mode_Vector(vector<vector<double>> & inputVector);										// 众数求取函数
Data_ImageRemoveMess ImageRemoveMess(Mat & inputImage);											// 杂物去除函数
Mat ImageEdgeRemoveBackground(Mat & inputImage);												// 去除背景函数
Mat Image_Format_Conversion(Mat & srcImage, Mat & Refer_Image);									// 图片格式转换

int i = 0;										// 保存彩色图的序号
int PCD_Number = 0;								// 保存PCD点云的序号
Mat Color;
Color_Image_Pixel Color_Pixel_Data[640 * 480];	// 保存彩色图像信息
Data_Ellipse Ellipse_Data;						// 椭圆信息

//-------------------------------【主程序】--------------------------------------------
int main(int argc, char * argv[])
{
	system("color 3F");

	int connect_hr = CreateFirstConnected();							// 进行连接检测程序
	int update = 0;

	m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];


	if (connect_hr)
	{
		//PCLView();

		while (1)
		{
			Color_Process(m_pVideoStreamHandle);
			Depth_Process(m_pDepthStreamHandle);

		}
	}
	NuiShutdown();
	return 0;
}

//-------------------------------【Kinect连接程序】--------------------------------------------
BOOL CreateFirstConnected()
{
	INuiSensor * pNuiSensor = nullptr;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);							// 获取连接的kinect数量
	cout << "kinect num:" << iSensorCount << endl;
	if (FAILED(hr))
	{
		m_pNuiSensor = pNuiSensor;
		return FALSE;
	}

	// Look at each Kinect sensor 对每个kinect进行初始化
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		// 获取kinect对象 
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		// 查看kinect状态，有的设备没连接电源，也许有的设备有其他异常  
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)		// 如果有一台正常的，那我们这个程序的初始化就算完毕了，因为这个例子只用一个kinect而已
		{
			m_pNuiSensor = pNuiSensor;
			cout << "finding " << i + 1 << " kinect waiting for connecting" << endl;
			//break;
		}

		// This sensor wasn't OK, so release it since we're not using it 如果是不正常的设备，那么Release掉，免得内存泄露 
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)  // 如果pNuiSensor不为空，那表明找到某一个正常的kinect设备了
	{
		// 初始化kinect，用NUI_INITIALIZE_FLAG_USES_DEPTH表示要使用深度图 ，NUI_INITIALIZE_FLAG_USES_COLOR表示彩色图
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		if (SUCCEEDED(hr))
		{
			//  打开彩色图流
			HANDLE m_hNextVideoFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, colorResolution, 0, 2, m_hNextVideoFrameEvent, &m_pVideoStreamHandle);
			if (SUCCEEDED(hr)) { cout << "success open color stream video" << endl; }
			else { cout << "Could not open color stream video" << endl; return FALSE; }

			//  打开深度图流
			HANDLE m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,				// 表示要打开深度图流
				NUI_IMAGE_RESOLUTION_640x480,		// 深度图大小
				0,									// 帧设置，0表示无设置
				2,									// 缓存多少帧，最大为4
				m_hNextDepthFrameEvent,				// 用来通信的Event句柄
				&m_pDepthStreamHandle);				// 用来读取数据的流句柄，要从这里读取深度图数据
			if (SUCCEEDED(hr)) { cout << "success open depth stream video" << endl; }
			else { cout << "Could not open depth stream video" << endl; return FALSE; }

			cout << "Successful connected with Kinect" << endl;
			return TRUE;
		}
		else { cout << "Failed connected with Kinect" << endl; }
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << m_pNuiSensor << endl;
		return FALSE;
	}
	return SUCCEEDED(hr);
}

//-------------------------------【获取彩色图像数据，并进行显示】--------------------------------------------
int Color_Process(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame_Color;						// 定义彩色帧
	HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame_Color);
	if (FAILED(hr))
	{
		//cout << "GetColor Image Frame Failed" << endl;
		return -1;
	}

	INuiFrameTexture* pTexture_Color = pImageFrame_Color->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture_Color->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0)
	{
		BYTE* pBuffer = (BYTE*)LockedRect.pBits;					// 按位存储的彩色图片

		/*for (int i = 0; i<cColorHeight; i++)
		{
		uchar *pBuffer1 = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
		for (int j = 0; j<cColorWidth; j++)
		{
		src_cloud1->points[i * 640 + j].b = uint8_t(pBuffer1[4 * j]);
		src_cloud1->points[i * 640 + j].g = uint8_t(pBuffer1[4 * j + 1]);
		src_cloud1->points[i * 640 + j].r = uint8_t(pBuffer1[4 * j + 2]);
		}
		}*/

		//OpenCV显示彩色视频
		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);		// 定义画布，说明彩色图片是8位4通道，也就是一个像素占8*4/8=4个字节
		temp.copyTo(Color);
		//imshow("Color", temp);

	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Color);				// 释放图像帧
	return 0;
}

//-------------------------------【获取深度图像数据，并进行显示】-------------------------------------------- 
int Depth_Process(HANDLE h)
{
	NUI_IMAGE_FRAME  pImageFrame_Depth;								// 定义深度帧
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(h, 0, &pImageFrame_Depth);
	if (FAILED(hr))
	{
		return -1;
	}

	BOOL nearMode = FALSE;											// 关闭近景模式
	INuiFrameTexture* pTexture_Depth;								// 定义深度纹理

	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &pImageFrame_Depth, &nearMode, &pTexture_Depth);


	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;
	pTexture_Depth->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0)
	{
		//int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		//int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		//深度范围在800-4000范围内数据比较正常，建议使用在1200-3810范围内的深度值
		int minDepth = NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;		// 最小深度距离800
		int maxDepth = NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;		// 最高深度距离4000

		BYTE * rgbrun = m_depthRGBX;												// 将深度图首指针给rgbrun，方便按位保存图片

																					//const NUI_DEPTH_IMAGE_PIXEL * pixel = Pixel_Depth;

		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
		const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		while (pBufferRun < pBufferEnd)
		{
			//pixel = pBufferRun;
			int Index_Pixel = pBufferRun - pBufferStart;							// 明确像素点是第几个
			Pixel_Depth[Index_Pixel].depth = pBufferRun->depth;
			Pixel_Depth[Index_Pixel].playerIndex = pBufferRun->playerIndex;

			// 得到每个像素点的深度值
			USHORT depth = pBufferRun->depth;

			// 将每个像素点的深度值转化为0-255的RGB值
			BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);

			// Write out blue byte
			*(rgbrun++) = intensity;

			// Write out green byte
			*(rgbrun++) = intensity;

			// Write out red byte
			*(rgbrun++) = intensity;

			++rgbrun;

			// Increment our index into the Kinect's depth buffer
			++pBufferRun;
			//++pixel;
		}

		Mat depthTmp(cDepthHeight, cDepthWidth, CV_8UC4, m_depthRGBX);	// 定义画布，说明深度图片是8位4通道，也就是一个像素占8*4/8=4个字节

		//imshow("Depth", depthTmp);

		Mapping_Color_To_Skeletion();

	}
ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(h, &pImageFrame_Depth);
	return 0;
}

//-------------------------------【将彩色图中对应点转化为摄像头坐标系中的三维坐标】-------------------------------------------- 
int Mapping_Color_To_Skeletion(void)
{
	INuiCoordinateMapper*			pMapper;
	HRESULT hr = m_pNuiSensor->NuiGetCoordinateMapper(&pMapper);
	if (S_OK != hr)
	{
		cout << "Can not Get CoordinateMapper" << endl;
		return -1;
	}

	hr = pMapper->MapColorFrameToSkeletonFrame(NUI_IMAGE_TYPE_COLOR, colorResolution, depthResolution, 640 * 480, Pixel_Depth, 640 * 480, Color_Mapping_Skeletion_3D);

	if (S_OK == hr)
	{
		Data_Ellipse Ellipse;
		int q = Target_Recoginition(Color);
		if (q == 1) {
			Ellipse = Ellipse_Data;
		}
		else
		{
			cout << "读取图片错误" << endl;
		}
		
		cout << Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].x << "  "<< Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].y << "  " << Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].z << endl;
		//cout << "彩色图中心点在摄像头坐标系中的位置:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;

		//for (int k = 0; k < 640 * 480; k++)
		//{
		//	src_cloud->points[k].x = Color_Mapping_Skeletion_3D[k].x;
		//	src_cloud->points[k].y = Color_Mapping_Skeletion_3D[k].y;
		//	src_cloud->points[k].z = Color_Mapping_Skeletion_3D[k].z;
		//	src_cloud1->points[k].x = Color_Mapping_Skeletion_3D[k].x;
		//	src_cloud1->points[k].y = Color_Mapping_Skeletion_3D[k].y;
		//	src_cloud1->points[k].z = Color_Mapping_Skeletion_3D[k].z;
		//}

	}
	else
	{
		cout << "Can not Mapping" << endl;
		return -1;
	}

	return 0;
}

//--------------------------------【目标识别函数】------------------------------------------
int Target_Recoginition(Mat Color_Image)
{
	clock_t start, finish; double totaltime;
	start = clock();

	// 图像读取************************************************
	cout << "\n-----------开始图像读取--------------" << endl;

	Mat temp = imread("4_2.jpg");		// 读取模板

	// 图像格式转换，由CV_8UC4转换成CV_8UC3
	Mat Color_Image423 = Image_Format_Conversion(Color_Image, temp);

	Mat src;
	Color_Image423.copyTo(src);
	

	if (!src.data)
	{
		cout << "Read image error" << endl;
		return -1;
	}

	int rows = src.rows;
	int columns = src.cols;

	// 模板匹配************************************************
	vector<Rect> Matching_region = on_Matching(src, temp);

	//// 图像分割************************************************
	/*Mat imageSegmentation = Image_Segmentation(src, Matching_region,0);
	namedWindow("分割后图像", 1);
	imshow("分割后图像", imageSegmentation);*/

	// 去除杂物************************************************
	double stdImage2std[5];
	Mat imageRemoveMess;
	src.copyTo(imageRemoveMess);

	for (int i = 0; i < 5; i++)
	{
		Data_ImageRemoveMess dst = ImageRemoveMess(imageRemoveMess);
		imageRemoveMess = dst.outputImage;
		stdImage2std[i] = dst.data_iImage2std.Std_Vecotr;
		if (i >= 1)
		{
			double diff = stdImage2std[i] - stdImage2std[i - 1];
			cout << "	均方差的均方差的差值：" << diff << endl;
			if (stdImage2std[i] < dst.data_iImage2std.Mean_Vector || abs(diff) < 0.05)
			{
				break;
			}
		}
	}

	// 去除背景************************************************
	Mat Image_Gray;
	cvtColor(imageRemoveMess, Image_Gray, CV_RGB2GRAY);
	Mat imageGrayRemoveBackground = ImageEdgeRemoveBackground(Image_Gray);

	// 边缘检测************************************************
	Mat dst;
	imageGrayRemoveBackground.copyTo(dst);
	blur(dst, dst, Size(3, 3));
	Canny(dst, dst, 100, 120, 3, false);                  //Canny边缘检测

	// 图像膨胀，增加连通区域************************************************
	int g_nStructElementSize = 1; //结构元素(内核矩阵)的尺寸  

								  //获取自定义核  
	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(g_nStructElementSize + 1, g_nStructElementSize + 1),
		Point(-1, -1));
	dilate(dst, dst, element);

	// 图像分割************************************************
	dst = Image_Segmentation(dst, Matching_region, 10);

	// 提取轮廓************************************************
	vector<vector<Point>>  contours;
	vector<Vec4i> hierarchy;
	findContours(dst, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(-1, -1));
	Mat drawing = Mat::zeros(dst.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//任意值
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
	}

	//Hough变换************************************************
	int accumulate;   // Hough空间最大累积量
	Hough_Ellipse myellipse;
	//Mat result(src.size(), CV_8UC3, Scalar(0));

	cout << "-----------开始进行霍夫变换椭圆检测--------------" << endl;

	for (int i = 0; i < contours.size(); i++)
	{
		double ContourArea = contourArea(contours[i], true);
		if (contours[i].size() >= 10 && ContourArea >= 60)
		{
			cout << "	连通区域 [" << i << "]" << "中点的数目：" << contours[i].size() << endl;
			cout << "	连通区域 [" << i << "]" << "的面积：" << contourArea(contours[i], true) << endl;

			myellipse.Computer_axy(contours[i], dst.size());
			accumulate = myellipse.hough_ellipse(contours[i]);
			//cout<<"accumulate:"<<accumulate<<endl;
			//cout<<"contours[i].size():"<<contours[i].size()<<endl;

			if (accumulate >= contours[i].size()*0.3)    // 判断是否超过给定阈值，判断是否为椭圆
			{
				Ellipse_Data = myellipse.get_data_Ellipse();
				//result = myellipse.draw_Eliipse(src);
			}
			else
			{
				cout << "	Contours [" << i << "] is not an ellipse" << endl;
				cout << "	-------------------------------------------- " << endl;
				//break;
			}
		}
		else
		{
			cout << "	Contours [" << i << "] is not an vaild area" << endl;
			cout << "	-------------------------------------------- " << endl;
		}
	}

	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	cout << "\n此程序的运行时间为" << totaltime << "秒！" << endl;
	
	return 1;
}

//--------------------------------【模板匹配函数】------------------------------------------
vector<Rect> on_Matching(Mat & srcImage, Mat & templateImage)
{
	cout << "\n-----------开始进行模板匹配--------------" << endl;

	//【1】给局部变量初始化
	vector<Rect> Matching_region;

	Mat g_srcImage;
	srcImage.copyTo(g_srcImage);

	Mat g_templateImage;
	templateImage.copyTo(g_templateImage);

	Mat g_resultImage; 

	//【2】初始化用于结果输出的矩阵
	int resultImage_rows = g_srcImage.rows - g_templateImage.rows + 1;
	int resultImage_cols = g_srcImage.cols - g_templateImage.cols + 1;

	g_resultImage.create(resultImage_rows, resultImage_cols, CV_32FC1);


	//【3】进行匹配和标准化

	matchTemplate(g_srcImage, g_templateImage, g_resultImage, CV_TM_CCOEFF);

	normalize(g_resultImage, g_resultImage, 0, 1, NORM_MINMAX, -1, Mat());

	//【4】通过函数 minMaxLoc 定位最匹配的位置
	double minValue; double maxValue; Point minLocation; Point maxLocation;
	Point matchLocation;

	minMaxLoc(g_resultImage, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

	matchLocation = maxLocation;

	//【5】绘制出矩形，并显示最终结果
	//rectangle(g_srcImage, matchLocation, Point(matchLocation.x + g_templateImage.cols, matchLocation.y + g_templateImage.rows), Scalar(0, 0, 255), 2, 8, 0);
	//rectangle(g_resultImage, matchLocation, Point(matchLocation.x + g_templateImage.cols, matchLocation.y + g_templateImage.rows), Scalar(0, 0, 255), 2, 8, 0);

	Matching_region.resize(1);
	Matching_region[0].x = matchLocation.x;
	Matching_region[0].y = matchLocation.y;
	Matching_region[0].width = g_templateImage.cols;
	Matching_region[0].height = g_templateImage.rows;

	return Matching_region;
}

//--------------------------------【图像分割函数】------------------------------------------
Mat Image_Segmentation(Mat & srcImage, vector<Rect> & Segmentation_Area, uchar threshould)
{
	cout << "\n-----------开始进行图像分割--------------" << endl;

	Mat outputImage;
	srcImage.copyTo(outputImage);
	int ChannelNumner = outputImage.channels();

	cout << "通道数目：" << ChannelNumner << endl;

	//typedef uchar	Point_Segmentation;		// 默认是单通道的图像

	if (ChannelNumner == 1)
	{
		typedef		Vec3b	Point_Segmentation;
	}

	int rowNumber = srcImage.rows;	// 行数
	int colNumber = srcImage.cols;	// 列数

	int Number_Segmentation_Area = Segmentation_Area.size();

	for (int k = 0; k < Number_Segmentation_Area; k++)
	{
		for (int i = 0; i < rowNumber; i++)  //行循环
		{
			for (int j = 0; j < colNumber; j++)   //列循环
			{
				if (ChannelNumner == 1)
				{
					if ((i >= (Segmentation_Area[k].y - threshould) && i <= (Segmentation_Area[k].y + Segmentation_Area[k].height + threshould)) && (j >= (Segmentation_Area[k].x - threshould) && j <= (Segmentation_Area[k].x + Segmentation_Area[k].width + threshould)))
					{
						outputImage.at<uchar>(i, j) = outputImage.at<uchar>(i, j);
					}
					else
					{
						outputImage.at<uchar>(i, j) = 0;		// 无用区域标为黑色
					}
				}
				else if (ChannelNumner == 3)
				{
					if ((i >= Segmentation_Area[k].y && i <= Segmentation_Area[k].y + Segmentation_Area[k].height) && (j >= Segmentation_Area[k].x && j <= Segmentation_Area[k].x + Segmentation_Area[k].width))
					{
						outputImage.at<Vec3b>(i, j) = outputImage.at<Vec3b>(i, j);
					}
					else
					{
						outputImage.at<Vec3b>(i, j) = { 255,255,255 };		// 无用区域标为白色
					}
				}
			}
		}
	}

	return outputImage;
}

//--------------------------------【均方差求取函数】------------------------------------------
Data_Vector Mean_square_deviation(vector<vector<double>> & inputVector, int Mode)
{
	cout << "数组元素均值和均方差求解" << endl;
	Data_Vector dataVector;
	vector<vector<double>> v1 = inputVector;
	int rowNumber = inputVector.size();	// 行数
	int colNumber = inputVector[0].size();	// 列数，此公式适用于每行所对应的列相等
	int dataNumber = rowNumber * colNumber;	// 数据总数
	int actualNumber = 0;

	double sumVector = 0;	// 数据总和
	double meanVector = 0;	// 数据均值
	double stdVector = 0;	// 数据均方差

	switch (Mode)
	{
	case 1:
		actualNumber = dataNumber;
		for (int i = 0; i < rowNumber; i++)
		{
			for (int j = 0; j < colNumber; j++)
			{
				sumVector += v1[i][j];
			}
		}
		meanVector = sumVector / actualNumber;
		cout << "meanVector：" << meanVector << endl;
	case 2:
		actualNumber = dataNumber;
		for (int i = 0; i < rowNumber; i++)
		{
			for (int j = 0; j < colNumber; j++)
			{
				if (v1[i][j] == 255)
				{
					actualNumber = actualNumber - 1;
				}
				else
				{
					sumVector += v1[i][j];
				}
			}
		}
		meanVector = sumVector / actualNumber;
		cout << "meanVector：" << meanVector << endl;
	default:
		break;
	}

	for (int i = 0; i < rowNumber; i++)
	{
		for (int j = 0; j < colNumber; j++)
		{
			sumVector += pow((v1[i][j] - meanVector), 2);
		}
	}
	stdVector = sqrt(sumVector / dataNumber);

	dataVector.Mean_Vector = meanVector;
	dataVector.Std_Vecotr = stdVector;
	return dataVector;
}

//--------------------------------【众数求取函数】------------------------------------------
double Mode_Vector(vector<vector<double>> & inputVector)
{
	cout << "数组元素众数求取" << endl;
	vector<vector<double>> v1 = inputVector;
	int rowNumber = inputVector.size();	// 行数
	int colNumber = inputVector[0].size();	// 列数，此公式适用于每行所对应的列相等

	vector <vector<double> >::iterator	iter;		// 行迭代器
	vector< double >::iterator			it;			// 列迭代器
	vector< double >					histogram(rowNumber*colNumber, 0);

	for (iter = v1.begin(); iter != v1.end(); iter++)
	{
		for (it = (*iter).begin(); it != (*iter).end(); it++)
		{
			histogram[*it]++;
		}
	}

	double mode = max_element(histogram.begin(), histogram.end()) - histogram.begin();		// 返回数组中数目最多的数
	return mode;
}

//--------------------------------【去除杂物函数】------------------------------------------
Data_ImageRemoveMess ImageRemoveMess(Mat & inputImage)
{
	cout << "\n-----------开始进行杂物去除--------------" << endl;
	Data_ImageRemoveMess dataImageRemoveMess;
	vector<Mat> channelsRGB;
	int rowNumber = inputImage.rows;	// 行数
	int colNumber = inputImage.cols;	// 列数

	cout << "图像行列数：[" << rowNumber << "," << colNumber << "]" << endl;
	vector < vector<double>> meanImage(rowNumber, vector<double>(colNumber, 0));
	vector < vector<double>> iImage2std(rowNumber, vector<double>(colNumber, 0));

	Mat outputImage;
	inputImage.copyTo(outputImage);

	for (int i = 0; i < rowNumber; i++)  //行循环
	{
		for (int j = 0; j < colNumber; j++)   //列循环
		{
			meanImage[i][j] = double((inputImage.at<Vec3b>(i, j)[0] + inputImage.at<Vec3b>(i, j)[1] + inputImage.at<Vec3b>(i, j)[2]) / 3);
			iImage2std[i][j] = sqrt((pow(inputImage.at<Vec3b>(i, j)[0] - meanImage[i][j], 2) + pow(inputImage.at<Vec3b>(i, j)[1] - meanImage[i][j], 2) + pow(inputImage.at<Vec3b>(i, j)[2] - meanImage[i][j], 2)) / 3);
		}
	}

	double			mode_meanImage = Mode_Vector(meanImage);		// 求频数最多的数				

	Data_Vector		data_iImage2std = Mean_square_deviation(iImage2std, 1);	// 各像素RGB值均方差的中值和均方差

	cout << "	中值众数：" << mode_meanImage << endl;
	cout << "	均方差中值：" << data_iImage2std.Mean_Vector << endl;
	cout << "	均方差的均方差：" << data_iImage2std.Std_Vecotr << endl;

	for (int i = 0; i < rowNumber; i++)
	{
		for (int j = 0; j < colNumber; j++)
		{
			if ((iImage2std[i][j] > data_iImage2std.Mean_Vector + 6 * data_iImage2std.Std_Vecotr) || meanImage[i][j] > mode_meanImage)
			{
				outputImage.at<Vec3b>(i, j) = { uchar(mode_meanImage),uchar(mode_meanImage),uchar(mode_meanImage) };
			}
		}
	}

	dataImageRemoveMess.outputImage = outputImage;
	dataImageRemoveMess.data_iImage2std.Mean_Vector = data_iImage2std.Mean_Vector;
	dataImageRemoveMess.data_iImage2std.Std_Vecotr = data_iImage2std.Std_Vecotr;

	return dataImageRemoveMess;

}

//--------------------------------【去除背景函数】------------------------------------------
Mat ImageEdgeRemoveBackground(Mat & inputImage)
{
	cout << "\n-----------开始进行背景去除--------------" << endl;
	Mat outputImage;
	inputImage.copyTo(outputImage);
	int rowNumber = inputImage.rows;	// 行数
	int colNumber = inputImage.cols;	// 列数

	Mat tmp_m, tmp_sd;
	double m = 0, sd = 0;

	vector < vector<double>> data_inputImage(rowNumber, vector<double>(colNumber, 0));
	for (int i = 0; i < rowNumber; i++)
	{
		uchar* data = outputImage.ptr<uchar>(i);
		for (int j = 0; j < colNumber; j++)
		{
			data_inputImage[i][j] = data[j];
		}
	}

	Data_Vector		data_iImage2std = Mean_square_deviation(data_inputImage, 2);

	//meanStdDev(inputImage, tmp_m, tmp_sd);		// 中值和均方差
	//m = tmp_m.at<double>(0, 0);					// 中值
	//sd = tmp_sd.at<double>(0, 0);				// 均方差

	m = data_iImage2std.Mean_Vector;			// 中值
	sd = data_iImage2std.Std_Vecotr;			// 均方差

	cout << "	像素中值：" << m << endl;
	cout << "	像素均方差：" << sd << endl;

	Mat_<uchar>::iterator it = outputImage.begin<uchar>();
	Mat_<uchar>::iterator itend = outputImage.end<uchar>();

	for (; it != itend; it++)
	{
		if (*it >= m + 3 * sd)
			*it = 255;
		else
			*it = *it;
	}

	return outputImage;
}

//--------------------------------【图像格式转换】------------------------------------------
Mat Image_Format_Conversion(Mat & srcImage, Mat & Refer_Image)
{
	Mat Image_back;
	vector<Mat> channels_4;
	vector<Mat> channels_3;

	split(srcImage, channels_4);
	split(Refer_Image, channels_3);

	for (int i = 0; i < Refer_Image.channels(); i++)
	{
		channels_3.at(i) = channels_4.at(i);
	}

	merge(channels_3, Image_back);

	return Image_back;
}