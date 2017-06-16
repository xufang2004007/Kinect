//----------------------------������Ĺ��ܡ�-----------------------------------------------------
	// 1����opencv��kinect sdk��ͬ���kinect��ɫ�����ͼ���ݶ�ȡ�ʹ�������
	// 2������ɫͼ������ͼ��ת��������ͷ�����µ�3Dλ����Ϣ�����Բ������ڲα궨
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

//----------------------------��PCLͷ�ļ���-------------------------------------
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>

//----------------------------�������ռ䶨�塿-------------------------------------
using namespace std;
using namespace cv;
using namespace pcl;

//----------------------------��������ȫ�ֱ���������-----------------------------------------
BOOL CreateFirstConnected();					// Kinect���Ӳ���������
int Depth_Process(HANDLE h);					// ���ͼ�������
int Color_Process(HANDLE h);					// ��ɫͼ�������
int Mapping_Color_To_Skeletion(void);			// ��ɫͼ������ͷ����ϵ��λ��ת��
void Getting_Pixel_AfterMapping();				// �������ͼƥ�䵽��ɫͼ������ص�λ��Ϣ

int i = 0;										// �����ɫͼ�����
int PCD_Number = 0;								// ����PCD���Ƶ����
Mat Copy_Color;

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
			if (PCD_Number >1) 
			{
				break;
			}
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

//-------------------------------��ȡ��ɫͼ�����ݣ���������ʾ--------------------------------------------
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
		temp.copyTo(Copy_Color);
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

//-------------------------------��ȡ���ͼ�����ݣ���������ʾ-------------------------------------------- 
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

		//const NUI_DEPTH_IMAGE_PIXEL * pixel = Pixel_Depth;

		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
		const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		while (pBufferRun < pBufferEnd)
		{
			//pixel = pBufferRun;
			int Index_Pixel = pBufferRun - pBufferStart;							// ��ȷ���ص��ǵڼ���
			Pixel_Depth[Index_Pixel].depth = pBufferRun->depth;
			Pixel_Depth[Index_Pixel].playerIndex = pBufferRun->playerIndex;

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
			//++pixel;
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

		Mapping_Color_To_Skeletion();
		
	}
ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(h, &pImageFrame_Depth);
	return 0;
}

//-------------------------------����ɫͼ�ж�Ӧ��ת��Ϊ����ͷ����ϵ�е���ά����-------------------------------------------- 
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
		cout << "��ɫͼ���ĵ�������ͷ����ϵ�е�λ��:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;
		
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
	
	//cout << "ƥ��ǰ240��320�д�����ȣ�" << Pixel_Depth[239 * 640 + 319].depth << endl;
	//cout << size(Color_Mapping_Skeletion_3D) << endl;
	//cout << "��ɫͼ���ĵ�������ͷ����ϵ�е�λ��:  [ "<< 1000*Color_Mapping_Skeletion_3D[100*640+319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[100 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[100 * 640 + 319].z << "mm ]" << endl;
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
			//���ڴ���ƫ����
			long index = i * 640 + j;
			//�ӱ�����ӳ������������л�ȡ��
			Depth_Mapping_Color_Pixel depthPointAtIndex = Depth_Mapping_Color_3D[index];

			uchar *prt_rgb = Copy_Color.ptr(depthPointAtIndex.X);
			uchar *prt_show = jshow.ptr(i);

			//�߽��ж�
			if (depthPointAtIndex.X >= 0 && depthPointAtIndex.X < 480)
			{
				//����жϣ���MIN_DISTANCE��MAX_DISTANCE֮��ĵ���ǰ������ʾ����
				//���ʹ��Ҳ����Ҫ����ʹ��������������ص��������ͼ���л�ȡ���ֵ���жϵ�ʱ�򣬻����

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
	//cout << "ƥ���240��320�д�����ȣ�"<< Depth_Mapping_Color_3D[239 * 640 + 319].depth << endl;
