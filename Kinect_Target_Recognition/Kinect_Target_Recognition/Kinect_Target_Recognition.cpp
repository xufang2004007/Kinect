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
#include "time.h"
//----------------------------��kinec���tͷ�ļ���---------------------------------
#include "nuiapi.h"						// kinetc����api
#include "Imagebasics.h"				// ���ͼ���������
#include "kinect_depthdata.h"			// ���ͼ���������
#include "Kinect_function.h"
//----------------------------��opencvͷ�ļ���-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// �������е�opencv2ͷ�ļ�
//----------------------------���������㷨ͷ�ļ���-------------------------------------
#include "Ellipse.cpp"

//----------------------------�������ռ䶨�塿-------------------------------------
using namespace std;
using namespace cv;
RNG g_rng(12345);

//--------------------------------���ṹ�嶨�塿------------------------------------------
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

//----------------------------��������ȫ�ֱ���������-----------------------------------------
BOOL CreateFirstConnected();					// Kinect���Ӳ���������
int Depth_Process(HANDLE h);					// ���ͼ�������
int Color_Process(HANDLE h);					// ��ɫͼ�������
int Mapping_Color_To_Skeletion(void);			// ��ɫͼ������ͷ����ϵ��λ��ת��

int Target_Recoginition(Mat Color_Image);														// Ŀ��ʶ����
vector<Rect> on_Matching(Mat & srcImage, Mat & templateImage);									// ģ��ƥ�亯��
Mat Image_Segmentation(Mat & srcImage, vector<Rect> & Segmentation_Area, uchar threshould);		// ͼ��ָ��
Data_Vector Mean_square_deviation(vector<vector<double>> & inputVector, int Mode);				// ��������ȡ����
double Mode_Vector(vector<vector<double>> & inputVector);										// ������ȡ����
Data_ImageRemoveMess ImageRemoveMess(Mat & inputImage);											// ����ȥ������
Mat ImageEdgeRemoveBackground(Mat & inputImage);												// ȥ����������
Mat Image_Format_Conversion(Mat & srcImage, Mat & Refer_Image);									// ͼƬ��ʽת��

int i = 0;										// �����ɫͼ�����
int PCD_Number = 0;								// ����PCD���Ƶ����
Mat Color;
Color_Image_Pixel Color_Pixel_Data[640 * 480];	// �����ɫͼ����Ϣ
Data_Ellipse Ellipse_Data;						// ��Բ��Ϣ

//-------------------------------��������--------------------------------------------
int main(int argc, char * argv[])
{
	system("color 3F");

	int connect_hr = CreateFirstConnected();							// �������Ӽ�����
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

//-------------------------------��Kinect���ӳ���--------------------------------------------
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
		else { cout << "Failed connected with Kinect" << endl; }
	}
	if (NULL == m_pNuiSensor || FAILED(hr))
	{
		cout << "No ready Kinect found!" << m_pNuiSensor << endl;
		return FALSE;
	}
	return SUCCEEDED(hr);
}

//-------------------------------����ȡ��ɫͼ�����ݣ���������ʾ��--------------------------------------------
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

		//OpenCV��ʾ��ɫ��Ƶ
		Mat temp(cColorHeight, cColorWidth, CV_8UC4, pBuffer);		// ���廭����˵����ɫͼƬ��8λ4ͨ����Ҳ����һ������ռ8*4/8=4���ֽ�
		temp.copyTo(Color);
		//imshow("Color", temp);

	}
	NuiImageStreamReleaseFrame(h, pImageFrame_Color);				// �ͷ�ͼ��֡
	return 0;
}

//-------------------------------����ȡ���ͼ�����ݣ���������ʾ��-------------------------------------------- 
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

		//imshow("Depth", depthTmp);

		Mapping_Color_To_Skeletion();

	}
ReleaseFrame:
	m_pNuiSensor->NuiImageStreamReleaseFrame(h, &pImageFrame_Depth);
	return 0;
}

//-------------------------------������ɫͼ�ж�Ӧ��ת��Ϊ����ͷ����ϵ�е���ά���꡿-------------------------------------------- 
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
			cout << "��ȡͼƬ����" << endl;
		}
		
		cout << Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].x << "  "<< Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].y << "  " << Color_Mapping_Skeletion_3D[Ellipse.point_center.y * 640 + Ellipse.point_center.x].z << endl;
		//cout << "��ɫͼ���ĵ�������ͷ����ϵ�е�λ��:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;

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

//--------------------------------��Ŀ��ʶ������------------------------------------------
int Target_Recoginition(Mat Color_Image)
{
	clock_t start, finish; double totaltime;
	start = clock();

	// ͼ���ȡ************************************************
	cout << "\n-----------��ʼͼ���ȡ--------------" << endl;

	Mat temp = imread("4_2.jpg");		// ��ȡģ��

	// ͼ���ʽת������CV_8UC4ת����CV_8UC3
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

	// ģ��ƥ��************************************************
	vector<Rect> Matching_region = on_Matching(src, temp);

	//// ͼ��ָ�************************************************
	/*Mat imageSegmentation = Image_Segmentation(src, Matching_region,0);
	namedWindow("�ָ��ͼ��", 1);
	imshow("�ָ��ͼ��", imageSegmentation);*/

	// ȥ������************************************************
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
			cout << "	������ľ�����Ĳ�ֵ��" << diff << endl;
			if (stdImage2std[i] < dst.data_iImage2std.Mean_Vector || abs(diff) < 0.05)
			{
				break;
			}
		}
	}

	// ȥ������************************************************
	Mat Image_Gray;
	cvtColor(imageRemoveMess, Image_Gray, CV_RGB2GRAY);
	Mat imageGrayRemoveBackground = ImageEdgeRemoveBackground(Image_Gray);

	// ��Ե���************************************************
	Mat dst;
	imageGrayRemoveBackground.copyTo(dst);
	blur(dst, dst, Size(3, 3));
	Canny(dst, dst, 100, 120, 3, false);                  //Canny��Ե���

	// ͼ�����ͣ�������ͨ����************************************************
	int g_nStructElementSize = 1; //�ṹԪ��(�ں˾���)�ĳߴ�  

								  //��ȡ�Զ����  
	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(g_nStructElementSize + 1, g_nStructElementSize + 1),
		Point(-1, -1));
	dilate(dst, dst, element);

	// ͼ��ָ�************************************************
	dst = Image_Segmentation(dst, Matching_region, 10);

	// ��ȡ����************************************************
	vector<vector<Point>>  contours;
	vector<Vec4i> hierarchy;
	findContours(dst, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_SIMPLE, Point(-1, -1));
	Mat drawing = Mat::zeros(dst.size(), CV_8UC3);
	for (int i = 0; i< contours.size(); i++)
	{
		Scalar color = Scalar(g_rng.uniform(0, 255), g_rng.uniform(0, 255), g_rng.uniform(0, 255));//����ֵ
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
	}

	//Hough�任************************************************
	int accumulate;   // Hough�ռ�����ۻ���
	Hough_Ellipse myellipse;
	//Mat result(src.size(), CV_8UC3, Scalar(0));

	cout << "-----------��ʼ���л���任��Բ���--------------" << endl;

	for (int i = 0; i < contours.size(); i++)
	{
		double ContourArea = contourArea(contours[i], true);
		if (contours[i].size() >= 10 && ContourArea >= 60)
		{
			cout << "	��ͨ���� [" << i << "]" << "�е����Ŀ��" << contours[i].size() << endl;
			cout << "	��ͨ���� [" << i << "]" << "�������" << contourArea(contours[i], true) << endl;

			myellipse.Computer_axy(contours[i], dst.size());
			accumulate = myellipse.hough_ellipse(contours[i]);
			//cout<<"accumulate:"<<accumulate<<endl;
			//cout<<"contours[i].size():"<<contours[i].size()<<endl;

			if (accumulate >= contours[i].size()*0.3)    // �ж��Ƿ񳬹�������ֵ���ж��Ƿ�Ϊ��Բ
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
	cout << "\n�˳��������ʱ��Ϊ" << totaltime << "�룡" << endl;
	
	return 1;
}

//--------------------------------��ģ��ƥ�亯����------------------------------------------
vector<Rect> on_Matching(Mat & srcImage, Mat & templateImage)
{
	cout << "\n-----------��ʼ����ģ��ƥ��--------------" << endl;

	//��1�����ֲ�������ʼ��
	vector<Rect> Matching_region;

	Mat g_srcImage;
	srcImage.copyTo(g_srcImage);

	Mat g_templateImage;
	templateImage.copyTo(g_templateImage);

	Mat g_resultImage; 

	//��2����ʼ�����ڽ������ľ���
	int resultImage_rows = g_srcImage.rows - g_templateImage.rows + 1;
	int resultImage_cols = g_srcImage.cols - g_templateImage.cols + 1;

	g_resultImage.create(resultImage_rows, resultImage_cols, CV_32FC1);


	//��3������ƥ��ͱ�׼��

	matchTemplate(g_srcImage, g_templateImage, g_resultImage, CV_TM_CCOEFF);

	normalize(g_resultImage, g_resultImage, 0, 1, NORM_MINMAX, -1, Mat());

	//��4��ͨ������ minMaxLoc ��λ��ƥ���λ��
	double minValue; double maxValue; Point minLocation; Point maxLocation;
	Point matchLocation;

	minMaxLoc(g_resultImage, &minValue, &maxValue, &minLocation, &maxLocation, Mat());

	matchLocation = maxLocation;

	//��5�����Ƴ����Σ�����ʾ���ս��
	//rectangle(g_srcImage, matchLocation, Point(matchLocation.x + g_templateImage.cols, matchLocation.y + g_templateImage.rows), Scalar(0, 0, 255), 2, 8, 0);
	//rectangle(g_resultImage, matchLocation, Point(matchLocation.x + g_templateImage.cols, matchLocation.y + g_templateImage.rows), Scalar(0, 0, 255), 2, 8, 0);

	Matching_region.resize(1);
	Matching_region[0].x = matchLocation.x;
	Matching_region[0].y = matchLocation.y;
	Matching_region[0].width = g_templateImage.cols;
	Matching_region[0].height = g_templateImage.rows;

	return Matching_region;
}

//--------------------------------��ͼ��ָ����------------------------------------------
Mat Image_Segmentation(Mat & srcImage, vector<Rect> & Segmentation_Area, uchar threshould)
{
	cout << "\n-----------��ʼ����ͼ��ָ�--------------" << endl;

	Mat outputImage;
	srcImage.copyTo(outputImage);
	int ChannelNumner = outputImage.channels();

	cout << "ͨ����Ŀ��" << ChannelNumner << endl;

	//typedef uchar	Point_Segmentation;		// Ĭ���ǵ�ͨ����ͼ��

	if (ChannelNumner == 1)
	{
		typedef		Vec3b	Point_Segmentation;
	}

	int rowNumber = srcImage.rows;	// ����
	int colNumber = srcImage.cols;	// ����

	int Number_Segmentation_Area = Segmentation_Area.size();

	for (int k = 0; k < Number_Segmentation_Area; k++)
	{
		for (int i = 0; i < rowNumber; i++)  //��ѭ��
		{
			for (int j = 0; j < colNumber; j++)   //��ѭ��
			{
				if (ChannelNumner == 1)
				{
					if ((i >= (Segmentation_Area[k].y - threshould) && i <= (Segmentation_Area[k].y + Segmentation_Area[k].height + threshould)) && (j >= (Segmentation_Area[k].x - threshould) && j <= (Segmentation_Area[k].x + Segmentation_Area[k].width + threshould)))
					{
						outputImage.at<uchar>(i, j) = outputImage.at<uchar>(i, j);
					}
					else
					{
						outputImage.at<uchar>(i, j) = 0;		// ���������Ϊ��ɫ
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
						outputImage.at<Vec3b>(i, j) = { 255,255,255 };		// ���������Ϊ��ɫ
					}
				}
			}
		}
	}

	return outputImage;
}

//--------------------------------����������ȡ������------------------------------------------
Data_Vector Mean_square_deviation(vector<vector<double>> & inputVector, int Mode)
{
	cout << "����Ԫ�ؾ�ֵ�;��������" << endl;
	Data_Vector dataVector;
	vector<vector<double>> v1 = inputVector;
	int rowNumber = inputVector.size();	// ����
	int colNumber = inputVector[0].size();	// �������˹�ʽ������ÿ������Ӧ�������
	int dataNumber = rowNumber * colNumber;	// ��������
	int actualNumber = 0;

	double sumVector = 0;	// �����ܺ�
	double meanVector = 0;	// ���ݾ�ֵ
	double stdVector = 0;	// ���ݾ�����

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
		cout << "meanVector��" << meanVector << endl;
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
		cout << "meanVector��" << meanVector << endl;
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

//--------------------------------��������ȡ������------------------------------------------
double Mode_Vector(vector<vector<double>> & inputVector)
{
	cout << "����Ԫ��������ȡ" << endl;
	vector<vector<double>> v1 = inputVector;
	int rowNumber = inputVector.size();	// ����
	int colNumber = inputVector[0].size();	// �������˹�ʽ������ÿ������Ӧ�������

	vector <vector<double> >::iterator	iter;		// �е�����
	vector< double >::iterator			it;			// �е�����
	vector< double >					histogram(rowNumber*colNumber, 0);

	for (iter = v1.begin(); iter != v1.end(); iter++)
	{
		for (it = (*iter).begin(); it != (*iter).end(); it++)
		{
			histogram[*it]++;
		}
	}

	double mode = max_element(histogram.begin(), histogram.end()) - histogram.begin();		// ������������Ŀ������
	return mode;
}

//--------------------------------��ȥ�����ﺯ����------------------------------------------
Data_ImageRemoveMess ImageRemoveMess(Mat & inputImage)
{
	cout << "\n-----------��ʼ��������ȥ��--------------" << endl;
	Data_ImageRemoveMess dataImageRemoveMess;
	vector<Mat> channelsRGB;
	int rowNumber = inputImage.rows;	// ����
	int colNumber = inputImage.cols;	// ����

	cout << "ͼ����������[" << rowNumber << "," << colNumber << "]" << endl;
	vector < vector<double>> meanImage(rowNumber, vector<double>(colNumber, 0));
	vector < vector<double>> iImage2std(rowNumber, vector<double>(colNumber, 0));

	Mat outputImage;
	inputImage.copyTo(outputImage);

	for (int i = 0; i < rowNumber; i++)  //��ѭ��
	{
		for (int j = 0; j < colNumber; j++)   //��ѭ��
		{
			meanImage[i][j] = double((inputImage.at<Vec3b>(i, j)[0] + inputImage.at<Vec3b>(i, j)[1] + inputImage.at<Vec3b>(i, j)[2]) / 3);
			iImage2std[i][j] = sqrt((pow(inputImage.at<Vec3b>(i, j)[0] - meanImage[i][j], 2) + pow(inputImage.at<Vec3b>(i, j)[1] - meanImage[i][j], 2) + pow(inputImage.at<Vec3b>(i, j)[2] - meanImage[i][j], 2)) / 3);
		}
	}

	double			mode_meanImage = Mode_Vector(meanImage);		// ��Ƶ��������				

	Data_Vector		data_iImage2std = Mean_square_deviation(iImage2std, 1);	// ������RGBֵ���������ֵ�;�����

	cout << "	��ֵ������" << mode_meanImage << endl;
	cout << "	��������ֵ��" << data_iImage2std.Mean_Vector << endl;
	cout << "	������ľ����" << data_iImage2std.Std_Vecotr << endl;

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

//--------------------------------��ȥ������������------------------------------------------
Mat ImageEdgeRemoveBackground(Mat & inputImage)
{
	cout << "\n-----------��ʼ���б���ȥ��--------------" << endl;
	Mat outputImage;
	inputImage.copyTo(outputImage);
	int rowNumber = inputImage.rows;	// ����
	int colNumber = inputImage.cols;	// ����

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

	//meanStdDev(inputImage, tmp_m, tmp_sd);		// ��ֵ�;�����
	//m = tmp_m.at<double>(0, 0);					// ��ֵ
	//sd = tmp_sd.at<double>(0, 0);				// ������

	m = data_iImage2std.Mean_Vector;			// ��ֵ
	sd = data_iImage2std.Std_Vecotr;			// ������

	cout << "	������ֵ��" << m << endl;
	cout << "	���ؾ����" << sd << endl;

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

//--------------------------------��ͼ���ʽת����------------------------------------------
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