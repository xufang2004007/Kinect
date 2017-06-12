//---------------------------------------------------------------------------------
	// 该程序用opencv和kinect sdk共同完成kinect数据读取和处理工作
	// 最终生成机器人的路径规划多项式系数，并完成与TwinCAT的数据交互
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
#include <opencv2/opencv.hpp>

//----------------------------【命名空间定义】-------------------------------------
using namespace std;
using namespace cv;

//----------------------------【函数与全局变量声明】-----------------------------------------
BOOL CreateFirstConnected();					// Kinect连接并打开数据流
int Depth_Process(HANDLE h);					// 深度图处理程序
int Color_Process(HANDLE h);					// 彩色图处理程序
DWORD WINAPI KinectDataThread(LPVOID pParam);	// kinect读取数据流线程

BOOL process_on;

//-------------------------------主程序--------------------------------------------
int main(int argc, char * argv[]) 
{
	int connect_hr = CreateFirstConnected();							// 进行连接检测程序
	if (connect_hr) 
	{
		m_hEvNuiProcessStop = CreateEvent(NULL, TRUE, FALSE, NULL);		//用于结束的事件对象
		//																 
		//HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0); //开启一个线程---用于读取彩色、深度数据； 
		//while (m_hEvNuiProcessStop != NULL)
		//{
		//	WaitForSingleObject(m_hProcesss, INFINITE);
		//	CloseHandle(m_hProcesss);
		//	m_hProcesss = NULL;
		//}
		process_on = TRUE;
		while (process_on)
		{
			Color_Process(m_pVideoStreamHandle);
			Depth_Process(m_pDepthStreamHandle);
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

//获取彩色图像数据，并进行显示  
int Color_Process(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame_Color = NULL;
	HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame_Color);
	if (FAILED(hr))
	{
		//cout << "GetColor Image Frame Failed" << endl;
		return-1;
	}
	INuiFrameTexture* pTexture_Color = pImageFrame_Color->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture_Color->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		BYTE* pBuffer = (BYTE*)LockedRect.pBits;
		//OpenCV显示彩色视频

		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);
		imshow("Color", temp);

		int c = waitKey(1);//按下ESC结束  
						   //如果在视频界面按下ESC,q,Q都会导致整个程序退出  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //PNG格式图片的压缩级别    
			compression_params.push_back(95);
			imwrite("a.jpg", temp, compression_params);
		}
		//if (c == 27)
		//{
		//	process_on = FALSE;
		//}
	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Color);
	return 0;
}

//获取深度图像数据，并进行显示  
int Depth_Process(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame_Depth = NULL;
	HRESULT hr = NuiImageStreamGetNextFrame(h, 0, &pImageFrame_Depth);
	if (FAILED(hr))
	{
		//cout << "GetDepth Image Frame Failed" << endl;
		return-1;
	}
	INuiFrameTexture* pTexture_Depth = pImageFrame_Depth->pFrameTexture;
	NUI_LOCKED_RECT LockedRect;
	pTexture_Depth->LockRect(0, &LockedRect, NULL, 0);
	if (LockedRect.Pitch != 0)
	{
		BYTE* pBuff = (BYTE*)LockedRect.pBits;
		//OpenCV显示深度视频

		Mat depthTmp(cDepthHeight, cDepthWidth, CV_16U, pBuff);
		imshow("Depth", depthTmp);

		int c = waitKey(1);//按下ESC结束  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //PNG格式图片的压缩级别    
			compression_params.push_back(95);
			imwrite("depth.jpg", depthTmp, compression_params);
		}
		//if (c == 27) 
		//{
		//	process_on = FALSE;
		//}
	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Depth);
	return 0;
}

//DWORD WINAPI KinectDataThread(LPVOID pParam)
//{
//	cout << "进入进程" << endl;
//	HANDLE hEvents[3] = { m_hEvNuiProcessStop,m_hNextVideoFrameEvent,
//		m_hNextDepthFrameEvent};
//	while (1)
//	{
//		cout << "1" << endl;
//		int nEventIdx;
//		nEventIdx = WaitForMultipleObjects(sizeof(hEvents) / sizeof(hEvents[0]),
//			hEvents, FALSE, 100);
//		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hEvNuiProcessStop, 0))
//		{
//			cout << "2" << endl;
//			break;
//		}
//		//Process signal events  
//		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextVideoFrameEvent, 0))
//		{
//			Color_Process(m_pVideoStreamHandle);
//		}
//		if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
//		{
//			Depth_Process(m_pDepthStreamHandle);
//		}
//	}
//	CloseHandle(m_hEvNuiProcessStop);
//	m_hEvNuiProcessStop = NULL;
//	CloseHandle(m_hNextDepthFrameEvent);
//	CloseHandle(m_hNextVideoFrameEvent);
//	return 0;
//}