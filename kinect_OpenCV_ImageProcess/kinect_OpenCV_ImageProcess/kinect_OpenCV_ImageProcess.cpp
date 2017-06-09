// kinect_OpenCV_ImageProcess.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include "NuiApi.h"						// Kinetc����API
#include "DepthBasics.h"				// ���ͼ���������
#include "Kinect_DepthData.h"			// ���ͼ���������
#include <iostream>						// ������
#include <cmath>

using namespace std;

Data_Pixel Data_In_Pixel[480][640];		// ���������ص���Ϣ�����飬��480�У�680��
										//Data_Pixel * ptrData_In_Pixel = (Data_Pixel*)Data_In_Pixel;
INuiCoordinateMapper* DepthCoordinate = nullptr;

BOOL CreateFirstConnected()
{
	INuiSensor * pNuiSensor = nullptr;
	HRESULT hr;

	int iSensorCount = 0;
	hr = NuiGetSensorCount(&iSensorCount);  // ��ȡ���ӵ�kinect����
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
		// Initialize the Kinect and specify that we'll be using depth ��ʼ��kinect����NUI_INITIALIZE_FLAG_USES_DEPTH��ʾҪʹ�����ͼ 
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);
		if (SUCCEEDED(hr))
		{
			HANDLE m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a depth image stream to receive depth frames  �����ͼ������������ͼ�� 
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,				// ��ʾҪ�����ͼ��
				NUI_IMAGE_RESOLUTION_640x480,		// ���ͼ��С
				0,									// ֡���ã�0��ʾ������
				2,									// �������֡�����Ϊ4
				m_hNextDepthFrameEvent,				// ����ͨ�ŵ�Event���
				&m_pDepthStreamHandle);				// ������ȡ���ݵ��������Ҫ�������ȡ���ͼ����

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
	// ͨ��kinect���󣬴�m_pDepthStreamHandle�л�ȡͼ�����ݣ����ǵ�m_pDepthStreamHandleô�����ڳ�ʼ��kinect�豸ʱ���������ͼ��  
	// ����������������������ǣ���һ֡���ͼ��������imageFrame��
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 100, &imageFrame);
	if (FAILED(hr))
	{
		cout << "Image getting failed" << endl;
		return;
	}

	BOOL nearMode = false;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture ͨ��imageFrame������ת�������� 
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it ��������
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

		// ��m_depthRGBX���׵�ַ������rgbrun�����㸳ֵ
		BYTE * rgbrun = m_depthRGBX;

		NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast< NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);  // ȥ����const����

		const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;

		// end pixel is start + width*height - 1
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		//��m_depthRGBXҲ����rgbrun��ֵ
		while (pBufferRun < pBufferEnd)
		{
			// �������ص��λ����Ϣ
			int Index_Pixel = pBufferRun - pBufferStart;							// ��ȷ���ص��ǵڼ���
																					//cout << Index_Pixel << endl;

																					// ��X�У���Y�е����ص�
			int X_Pixel = Index_Pixel / cDepthWidth;
			int Y_Pixel = Index_Pixel % cDepthWidth;
			//Data_In_Pixel[Index_Pixel]->x = X_Pixel; Data_In_Pixel[Index_Pixel]->y = Y_Pixel;		// ����x,y��ֵ

			// discard the portion of the depth that contains only the player index
			USHORT depth = pBufferRun->depth;

			Data_In_Pixel[X_Pixel][X_Pixel].Depth = depth;			// �������ֵ
																	//ptrData_In_Pixel->Depth = depth;						// �������ֵ

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

			Data_In_Pixel[X_Pixel][Y_Pixel].intensity = intensity;	// ����ɫ����Ϣ
																	//ptrData_In_Pixel->Depth = intensity;

			cout << X_Pixel << " " << Y_Pixel << " " << depth << " " << int(intensity) << endl;
			//cout << depth << " " << intensity << endl;

			// Write out blue byte	д��ɫ��λ
			*(rgbrun++) = intensity;

			// Write out green byte д��ɫ��λ
			*(rgbrun++) = intensity;

			// Write out red byte   д��ɫ��λ
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
	//cout << Data_In_Pixel[240][320].Depth << "    " << Data_In_Pixel[240][320].intensity << endl;  // ���������������Ƿ���ȷ
	// We're done with the texture so unlock it �������ͷ�����
	pTexture->UnlockRect(0);

	pTexture->Release();

ReleaseFrame:
	// Release the frame �ͷ�֡
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
}

int main()
{
	bool connect_hr = CreateFirstConnected();		// �������Ӽ�����
	while (connect_hr)										// while��ѭ������������������
	{
		ProcessDepth();								// ������������
	}
}

