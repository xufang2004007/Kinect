// kinect_OpenCV_ImageProcess.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "NuiApi.h"						// Kinetc数据API
#include "DepthBasics.h"				// 深度图像基础定义
#include "Kinect_DepthData.h"			// 深度图像基础数据
#include <iostream>						// 数据流
#include <cmath>

using namespace std;

Data_Pixel Data_In_Pixel[480][640];		// 定义存放像素点信息的数组，有480行，680列
										//Data_Pixel * ptrData_In_Pixel = (Data_Pixel*)Data_In_Pixel;
INuiCoordinateMapper* DepthCoordinate = nullptr;

BOOL CreateFirstConnected()
{
	INuiSensor * pNuiSensor = nullptr;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);  // 获取连接的kinect数量
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
		// Initialize the Kinect and specify that we'll be using depth 初始化kinect，用NUI_INITIALIZE_FLAG_USES_DEPTH表示要使用深度图 
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
		if (SUCCEEDED(hr))
		{
			HANDLE m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames  打开深度图流，用来接收图像 
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,				// 表示要打开深度图流
				NUI_IMAGE_RESOLUTION_640x480,		// 深度图大小
				0,									// 帧设置，0表示无设置
				2,									// 缓存多少帧，最大为4
				m_hNextDepthFrameEvent,				// 用来通信的Event句柄
				&m_pDepthStreamHandle);				// 用来读取数据的流句柄，要从这里读取深度图数据

			cout << "Successful connected with Kinect" << endl;
			return TRUE;
		}
		else
		{
			cout << "Failed connected with Kinect" << endl;
		}
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << m_pNuiSensor << endl;
		return FALSE;
	}
	return SUCCEEDED(hr);
}

void ProcessDepth()
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	// 通过kinect对象，从m_pDepthStreamHandle中获取图像数据，还记得m_pDepthStreamHandle么，是在初始化kinect设备时创建的深度图流  
	// 在这里调用这个代码的意义是：将一帧深度图，保存在imageFrame中
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 100, &imageFrame);
	if (FAILED(hr))
	{
		cout << "Image getting failed" << endl;
		return;
	}

	BOOL nearMode = false;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture 通过imageFrame把数据转化成纹理 
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it 锁定数据
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		int i = 0;
		//Vector4 *pSkeletonPoint[640*480];
		//NUI_DEPTH_IMAGE_POINT *Data_In_Pixel[640*480];

		// Get the min and max reliable depth for the current frame
		int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		// 将m_depthRGBX的首地址保存在rgbrun，方便赋值
		BYTE * rgbrun = m_depthRGBX;

		NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast< NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);  // 去掉了const声明

		const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;

		// end pixel is start + width*height - 1
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		//对m_depthRGBX也就是rgbrun赋值
		while (pBufferRun < pBufferEnd)
		{
			// 定义像素点的位置信息
			int Index_Pixel = pBufferRun - pBufferStart;							// 明确像素点是第几个
																					//cout << Index_Pixel << endl;

																					// 第X行，第Y列的像素点
			int X_Pixel = Index_Pixel / cDepthWidth;
			int Y_Pixel = Index_Pixel % cDepthWidth;
			//Data_In_Pixel[Index_Pixel]->x = X_Pixel; Data_In_Pixel[Index_Pixel]->y = Y_Pixel;		// 存入x,y的值

			// discard the portion of the depth that contains only the player index
			USHORT depth = pBufferRun->depth;

			Data_In_Pixel[X_Pixel][X_Pixel].Depth = depth;			// 保存深度值
																	//ptrData_In_Pixel->Depth = depth;						// 保存深度值

																	//hr = DepthCoordinate->MapDepthPointToSkeletonPoint(NUI_IMAGE_RESOLUTION_640x480, Data_In_Pixel[X_Pixel * 640 + Y_Pixel], pSkeletonPoint[X_Pixel * 640 + Y_Pixel]);
																	//
																	//if (S_OK == hr)
																	//{
																	//	cout << pSkeletonPoint[X_Pixel * 640 + Y_Pixel]->x << pSkeletonPoint[X_Pixel * 640 + Y_Pixel]->y << pSkeletonPoint[X_Pixel * 640 + Y_Pixel]->z << endl;
																	//}

																	// To convert to a byte, we're discarding the most-significant
																	// rather than least-significant bits.
																	// We're preserving detail, although the intensity will "wrap."
																	// Values outside the reliable depth range are mapped to 0 (black).

																	// Note: Using conditionals in this loop could degrade performance.
																	// Consider using a lookup table instead when writing production code.
			BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);

			Data_In_Pixel[X_Pixel][Y_Pixel].intensity = intensity;	// 保存色素信息
																	//ptrData_In_Pixel->Depth = intensity;

			cout << X_Pixel << " " << Y_Pixel << " " << depth << " " << int(intensity) << endl;
			//cout << depth << " " << intensity << endl;

			// Write out blue byte	写蓝色素位
			*(rgbrun++) = intensity;

			// Write out green byte 写绿色素位
			*(rgbrun++) = intensity;

			// Write out red byte   写红色素位
			*(rgbrun++) = intensity;

			// We're outputting BGR, the last byte in the 32 bits is unused so skip it
			// If we were outputting BGRA, we would write alpha here.
			++rgbrun;

			// Increment our index into the Kinect's depth buffer
			++pBufferRun;
		}
		//delete pSkeletonPoint;
		//delete Data_In_Pixel;
	}
	//cout << Data_In_Pixel[240][320].Depth << "    " << Data_In_Pixel[240][320].intensity << endl;  // 用来测试数据流是否正确
	// We're done with the texture so unlock it 解锁和释放纹理
	pTexture->UnlockRect(0);

	pTexture->Release();

ReleaseFrame:
	// Release the frame 释放帧
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
}

int main()
{
	bool connect_hr = CreateFirstConnected();		// 进行连接检测程序
	while (connect_hr)										// while的循环条件可以用作控制
	{
		ProcessDepth();								// 完成主程序过程
	}
}

