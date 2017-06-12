//---------------------------------------------------------------------------------
	// �ó�����opencv��kinect sdk��ͬ���kinect���ݶ�ȡ�ʹ�����
	// �������ɻ����˵�·���滮����ʽϵ�����������TwinCAT�����ݽ���
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
#include <opencv2/opencv.hpp>

//----------------------------�������ռ䶨�塿-------------------------------------
using namespace std;
using namespace cv;

//----------------------------��������ȫ�ֱ���������-----------------------------------------
BOOL CreateFirstConnected();					// Kinect���Ӳ���������
int Depth_Process(HANDLE h);					// ���ͼ�������
int Color_Process(HANDLE h);					// ��ɫͼ�������
DWORD WINAPI KinectDataThread(LPVOID pParam);	// kinect��ȡ�������߳�

BOOL process_on;

//-------------------------------������--------------------------------------------
int main(int argc, char * argv[]) 
{
	int connect_hr = CreateFirstConnected();							// �������Ӽ�����
	if (connect_hr) 
	{
		m_hEvNuiProcessStop = CreateEvent(NULL, TRUE, FALSE, NULL);		//���ڽ������¼�����
		//																 
		//HANDLE m_hProcesss = CreateThread(NULL, 0, KinectDataThread, 0, 0, 0); //����һ���߳�---���ڶ�ȡ��ɫ��������ݣ� 
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
		//OpenCV��ʾ��ɫ��Ƶ

		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);
		imshow("Color", temp);

		int c = waitKey(1);//����ESC����  
						   //�������Ƶ���水��ESC,q,Q���ᵼ�����������˳�  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //PNG��ʽͼƬ��ѹ������    
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

//��ȡ���ͼ�����ݣ���������ʾ  
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
		//OpenCV��ʾ�����Ƶ

		Mat depthTmp(cDepthHeight, cDepthWidth, CV_16U, pBuff);
		imshow("Depth", depthTmp);

		int c = waitKey(1);//����ESC����  
		if (c == 'q' || c == 'Q')
		{
			vector<int>compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY); //PNG��ʽͼƬ��ѹ������    
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
//	cout << "�������" << endl;
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