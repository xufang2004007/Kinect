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
//----------------------------【kinec相关t头文件】---------------------------------
#include "nuiapi.h"						// kinetc数据api
#include "Imagebasics.h"				// 深度图像基础定义
#include "kinect_depthdata.h"			// 深度图像基础数据
#include "Kinect_function.h"
//----------------------------【opencv头文件】-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// 包含所有的opencv2头文件

//----------------------------【PCL头文件】-------------------------------------
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>

//----------------------------【命名空间定义】-------------------------------------
using namespace std;
using namespace cv;
using namespace pcl;

//----------------------------【函数与全局变量声明】-----------------------------------------
BOOL CreateFirstConnected();					// Kinect连接并打开数据流
int Depth_Process(HANDLE h);					// 深度图处理程序
int Color_Process(HANDLE h);					// 彩色图处理程序
int Mapping_Color_To_Skeletion(void);			// 彩色图向摄像头坐标系的位置转换
void Getting_Pixel_AfterMapping();				// 汇总深度图匹配到彩色图后的像素点位信息

int i = 0;										// 保存彩色图的序号
int PCD_Number = 0;								// 保存PCD点云的序号
Mat Copy_Color;

//-------------------------------主程序--------------------------------------------
int main(int argc, char * argv[]) 
{
	int connect_hr = CreateFirstConnected();							// 进行连接检测程序

	m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];

	if (connect_hr) 
	{
		while (1)
		{
			Color_Process(m_pVideoStreamHandle);
			Depth_Process(m_pDepthStreamHandle);
			if (PCD_Number >1) 
			{
				break;
			}
		}
	}
	NuiShutdown();
	return 0;
}

//-------------------------------Kinect连接程序--------------------------------------------
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
		else { cout << "Failed connected with Kinect" << endl;}
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << m_pNuiSensor << endl;
		return FALSE;
	}
	return SUCCEEDED(hr);
}

//-------------------------------获取彩色图像数据，并进行显示--------------------------------------------
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
		
		//OpenCV显示彩色视频
		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);		// 定义画布，说明彩色图片是8位4通道，也就是一个像素占8*4/8=4个字节
		temp.copyTo(Copy_Color);
		imshow("Color", temp);

		int c = waitKey(10);										// 等待键盘输入
		if (c == 'q' || c == 'Q')									// 若按键Q，则保存彩色图	
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);	// JPG格式图片的压缩级别    
			compression_params.push_back(95);						// 0到100的图片质量（CV_IMWRITE_JPEG_QUALITY），默认值为95

			imwrite(ImageName[i++], temp, compression_params);		// 保存图片，一共十张，名称再ImageBasics.h中定义

			cout << "get kincet " << i << " Color Image" << endl;
			
			if (i >= sizeof(ImageName)/sizeof(ImageName[0]))
			{
				i = 0;
			}
		}
	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Color);				// 释放图像帧
	return 0;
}

//-------------------------------获取深度图像数据，并进行显示-------------------------------------------- 
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
		
		imshow("Depth", depthTmp);

		int c = waitKey(1);//按下ESC结束  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);		// JPG格式图片的压缩级别    
			compression_params.push_back(95);
			imwrite("Depth.jpg", depthTmp, compression_params);
		}

		Mapping_Color_To_Skeletion();
		
	}
ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(h, &pImageFrame_Depth);
	return 0;
}

//-------------------------------将彩色图中对应点转化为摄像头坐标系中的三维坐标-------------------------------------------- 
int Mapping_Color_To_Skeletion(void)
{
	INuiCoordinateMapper*			pMapper;
	HRESULT hr = m_pNuiSensor->NuiGetCoordinateMapper(&pMapper);
	if (S_OK != hr)
	{
		cout << "Can not Get CoordinateMapper" << endl;
		return -1;
	}

	hr = pMapper->MapColorFrameToSkeletonFrame(NUI_IMAGE_TYPE_COLOR,colorResolution,depthResolution, 640 * 480, Pixel_Depth, 640 * 480, Color_Mapping_Skeletion_3D);
	
	if (S_OK == hr)
	{
		cout << "彩色图中心点在摄像头坐标系中的位置:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;
		
		PointCloud<pcl::PointXYZ> cloud;
		cloud.width = 640*480;
		cloud.height = 1;
		cloud.is_dense = FALSE;
		cloud.points.resize(cloud.width * cloud.height);

		for (int  k = 0; k < 640*480; k++)
		{
			cloud.points[k].x = Color_Mapping_Skeletion_3D[k].x;
			cloud.points[k].y = Color_Mapping_Skeletion_3D[k].y;
			cloud.points[k].z = Color_Mapping_Skeletion_3D[k].z;
		}
		io::savePCDFileASCII(PCDFile[PCD_Number], cloud);
		cout << "Get [ " << PCD_Number + 1 << " ] PCD File" << endl;
		PCD_Number++;
	}
	else
	{
		cout << "Can not Mapping" << endl;
		return -1;
	}
	
	//cout << "匹配前240行320列处的深度：" << Pixel_Depth[239 * 640 + 319].depth << endl;
	//cout << size(Color_Mapping_Skeletion_3D) << endl;
	//cout << "彩色图中心点在摄像头坐标系中的位置:  [ "<< 1000*Color_Mapping_Skeletion_3D[100*640+319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[100 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[100 * 640 + 319].z << "mm ]" << endl;
	//Getting_Pixel_AfterMapping();
	return 0;
}


void Getting_Pixel_AfterMapping()
{
	for (int j = 0; j < size(Pixel_Depth); j++)
	{
		Depth_Mapping_Color_3D[j].X = Depth_Mapping_Color_2D[j].x;
		Depth_Mapping_Color_3D[j].Y = Depth_Mapping_Color_2D[j].y;
		Depth_Mapping_Color_3D[j].depth = Pixel_Depth[j].depth;
	}
	Mat jshow;
	jshow.create(480, 640, CV_8UC4);
	jshow = 0;

	for (int i = 0; i < 480; i++)
	{
		for (int j = 0; j < 640; j++)
		{
			//在内存中偏移量
			long index = i * 640 + j;
			//从保存了映射坐标的数组中获取点
			Depth_Mapping_Color_Pixel depthPointAtIndex = Depth_Mapping_Color_3D[index];

			uchar *prt_rgb = Copy_Color.ptr(depthPointAtIndex.X);
			uchar *prt_show = jshow.ptr(i);

			//边界判断
			if (depthPointAtIndex.X >= 0 && depthPointAtIndex.X < 480)
			{
				//深度判断，在MIN_DISTANCE与MAX_DISTANCE之间的当成前景，显示出来
				//这个使用也很重要，当使用真正的深度像素点再在深度图像中获取深度值来判断的时候，会出错

				if (depthPointAtIndex.Y * 3 >=0 && depthPointAtIndex.Y * 3 + 2 < 640)
				{
					prt_show[3 * j] = prt_rgb[depthPointAtIndex.Y * 4];
					prt_show[3 * j + 1] = prt_rgb[depthPointAtIndex.Y * 4 + 1];
					prt_show[3 * j + 2] = prt_rgb[depthPointAtIndex.Y * 4 + 2];
					prt_show[3 * j + 3] = 0;
				}
			}
		}
	}
	imshow("show", jshow);
}
	//cout << "匹配后240行320列处的深度："<< Depth_Mapping_Color_3D[239 * 640 + 319].depth << endl;
