//----------------------------������Ĺ��ܡ�-----------------------------------------------------
	// 1����opencv��kinect sdk��ͬ���kinect��ɫ�����ͼ���ݶ�ȡ�ʹ�������
	// 2��������ͼ�����ɫͼ��Ķ��룬����������ʶ��Ŀ���
	// 3������Ŀ����ķ�λ���ɻ�����·���滮��
	// 4���������ɻ����˵�·���滮����ʽϵ�����������TwinCAT�����ݽ���
//---------------------------------------------------------------------------------

//----------------------------��windowsͷ�ļ���------------------------------------
#include "stdafx.h"
#include <iostream>						// ������
#include <cmath>
#include <vector>
#include <stdio.h>
//----------------------------��kinec���tͷ�ļ���---------------------------------
#include "nuiapi.h"						// kinetc����api
#include "Imagebasics.h"				// ���ͼ���������
#include "kinect_depthdata.h"			// ���ͼ���������
#include "Kinect_function.h"
//----------------------------��opencvͷ�ļ���-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// �������е�opencv2ͷ�ļ�

//----------------------------�������ռ䶨�塿-------------------------------------
using namespace std;
using namespace cv;

//----------------------------��������ȫ�ֱ���������-----------------------------------------
BOOL CreateFirstConnected();					// Kinect���Ӳ���������
int Depth_Process(HANDLE h);					// ���ͼ�������
int Color_Process(HANDLE h);					// ��ɫͼ�������
int i = 0;										// �����ɫͼ�����

//-------------------------------������--------------------------------------------
int main(int argc, char * argv[]) 
{
	int connect_hr = CreateFirstConnected();							// �������Ӽ�����

	m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];

	if (connect_hr) 
	{
		while (1)
		{
			Color_Process(m_pVideoStreamHandle);
			Depth_Process(m_pDepthStreamHandle);
			int m = waitKey(1);//����ESC����
		}
	}
	NuiShutdown();
	return 0;
}

//-------------------------------Kinect���ӳ���--------------------------------------------
BOOL CreateFirstConnected()
{
	INuiSensor * pNuiSensor = nullptr;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);							// ��ȡ���ӵ�kinect����
	cout << "kinect num:" << iSensorCount << endl;
	if (FAILED(hr))
	{
		m_pNuiSensor = pNuiSensor;
		return FALSE;
	}

	// Look at each Kinect sensor ��ÿ��kinect���г�ʼ��
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		// ��ȡkinect���� 
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		// �鿴kinect״̬���е��豸û���ӵ�Դ��Ҳ���е��豸�������쳣  
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)		// �����һ̨�����ģ��������������ĳ�ʼ����������ˣ���Ϊ�������ֻ��һ��kinect����
		{
			m_pNuiSensor = pNuiSensor;
			cout << "finding " << i + 1 << " kinect waiting for connecting" << endl;
			//break;
		}

		// This sensor wasn't OK, so release it since we're not using it ����ǲ��������豸����ôRelease��������ڴ�й¶ 
		pNuiSensor->Release();
	}

	if (NULL != m_pNuiSensor)  // ���pNuiSensor��Ϊ�գ��Ǳ����ҵ�ĳһ��������kinect�豸��
	{
		// ��ʼ��kinect����NUI_INITIALIZE_FLAG_USES_DEPTH��ʾҪʹ�����ͼ ��NUI_INITIALIZE_FLAG_USES_COLOR��ʾ��ɫͼ
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
		if (SUCCEEDED(hr))
		{
			//  �򿪲�ɫͼ��
			HANDLE m_hNextVideoFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, colorResolution, 0, 2, m_hNextVideoFrameEvent, &m_pVideoStreamHandle);
			if (SUCCEEDED(hr)) { cout << "success open color stream video" << endl; }
			else { cout << "Could not open color stream video" << endl; return FALSE; }
			
			//  �����ͼ��
			HANDLE m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,				// ��ʾҪ�����ͼ��
				NUI_IMAGE_RESOLUTION_640x480,		// ���ͼ��С
				0,									// ֡���ã�0��ʾ������
				2,									// �������֡�����Ϊ4
				m_hNextDepthFrameEvent,				// ����ͨ�ŵ�Event���
				&m_pDepthStreamHandle);				// ������ȡ���ݵ��������Ҫ�������ȡ���ͼ����
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

//��ȡ��ɫͼ�����ݣ���������ʾ  
int Color_Process(HANDLE h)
{
	const NUI_IMAGE_FRAME * pImageFrame_Color;						// �����ɫ֡
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
		BYTE* pBuffer = (BYTE*)LockedRect.pBits;					// ��λ�洢�Ĳ�ɫͼƬ
		
		//OpenCV��ʾ��ɫ��Ƶ
		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);		// ���廭����˵����ɫͼƬ��8λ4ͨ����Ҳ����һ������ռ8*4/8=4���ֽ�
		imshow("Color", temp);

		int c = waitKey(10);										// �ȴ���������
		if (c == 'q' || c == 'Q')									// ������Q���򱣴��ɫͼ	
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);	// JPG��ʽͼƬ��ѹ������    
			compression_params.push_back(95);						// 0��100��ͼƬ������CV_IMWRITE_JPEG_QUALITY����Ĭ��ֵΪ95

			imwrite(ImageName[i++], temp, compression_params);		// ����ͼƬ��һ��ʮ�ţ�������ImageBasics.h�ж���

			cout << "get kincet " << i << " Color Image" << endl;
			
			if (i >= sizeof(ImageName)/sizeof(ImageName[0]))
			{
				i = 0;
			}
		}
	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Color);				// �ͷ�ͼ��֡
	return 0;
}

//��ȡ���ͼ�����ݣ���������ʾ  
int Depth_Process(HANDLE h)
{
	NUI_IMAGE_FRAME  pImageFrame_Depth;								// �������֡
	HRESULT hr = m_pNuiSensor->NuiImageStreamGetNextFrame(h, 0, &pImageFrame_Depth);
	if (FAILED(hr))
	{
		return -1;
	}
	
	BOOL nearMode = FALSE;											// �رս���ģʽ
	INuiFrameTexture* pTexture_Depth;								// �����������

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
		//��ȷ�Χ��800-4000��Χ�����ݱȽ�����������ʹ����1200-3810��Χ�ڵ����ֵ
		int minDepth = NUI_IMAGE_DEPTH_MINIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;		// ��С��Ⱦ���800
		int maxDepth = NUI_IMAGE_DEPTH_MAXIMUM >> NUI_IMAGE_PLAYER_INDEX_SHIFT;		// �����Ⱦ���4000

		BYTE * rgbrun = m_depthRGBX;												// �����ͼ��ָ���rgbrun�����㰴λ����ͼƬ
		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		while (pBufferRun < pBufferEnd)
		{
			// �õ�ÿ�����ص�����ֵ
			USHORT depth = pBufferRun->depth;

			// ��ÿ�����ص�����ֵת��Ϊ0-255��RGBֵ
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
		}

		Mat depthTmp(cDepthHeight, cDepthWidth, CV_8UC4, m_depthRGBX);	// ���廭����˵�����ͼƬ��8λ4ͨ����Ҳ����һ������ռ8*4/8=4���ֽ�

		imshow("Depth", depthTmp);

		int c = waitKey(1);//����ESC����  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);		// JPG��ʽͼƬ��ѹ������    
			compression_params.push_back(95);
			imwrite("Depth.jpg", depthTmp, compression_params);
		}
	}
ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(h, &pImageFrame_Depth);
	return 0;
}

