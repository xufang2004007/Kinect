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
#include <Shlobj.h>
#include "nuiapi.h"						// kinetc����api
#include "Imagebasics.h"				// ���ͼ���������
#include "kinect_depthdata.h"			// ���ͼ���������
//----------------------------��opencvͷ�ļ���-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// �������е�opencv2ͷ�ļ�

//----------------------------��PCLͷ�ļ���-------------------------------------
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// ���vtk������ֵĴ���
#include <vtkAutoInit.h>  
VTK_MODULE_INIT(vtkRenderingOpenGL);

//----------------------------�������ռ䶨�塿-------------------------------------
using namespace std;
using namespace cv;
using namespace pcl;


//----------------------------��������ȫ�ֱ���������-----------------------------------------
BOOL CreateFirstConnected();					// Kinect���Ӳ���������
int Color_Process(HANDLE h);					// ��ɫͼ�������
int Depth_Process(HANDLE h);					// ���ͼ�������
int Mapping_Color_To_Skeletion(void);			// ��ɫͼ������ͷ����ϵ��λ��ת��
void Normals_Process();							// �Ե��Ƶķ��߽��д���
void Plane_segmentation();						// �Ե��ƽ���ƽ��ָ�

int i = 0;										// �����ɫͼ�����
int PCD_Number = 0;								// ����PCD���Ƶ����
Mat Copy_Color;

PointCloud<pcl::PointXYZ>::Ptr					scr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud<pcl::PointXYZ>::Ptr					cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr				cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud<pcl::Normal>::Ptr					normals(new pcl::PointCloud<pcl::Normal>);
VoxelGrid<pcl::PointXYZ>						Sor_VoxelGrid;			//	�������͵������˲�����
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>	Sor_OutlinerRemoval;	// ����ȥ����Ⱥ����˲�����


//-------------------------------������--------------------------------------------
int main(int argc, char * argv[])
{
	
	int connect_hr = CreateFirstConnected();							// �������Ӽ�����
	int update = 0;
	
	scr_cloud->width = 640*480;
	scr_cloud->height = 1;
	scr_cloud->is_dense = FALSE;
	scr_cloud->points.resize(scr_cloud->width * scr_cloud->height);

	if (connect_hr)
	{
		while (1)
		{
			Color_Process(m_pVideoStreamHandle);
			Depth_Process(m_pDepthStreamHandle);
			//update++;
			//if (!viewer->wasStopped())
			//{
			//	sor.setInputCloud(scr_cloud);                      //������Ҫ���˵ĵ��Ƹ��˲�����
			//	sor.setLeafSize(0.03f, 0.03f, 0.03f);           //�����˲�ʱ���������ش�СΪ1cm������
			//	sor.filter(*cloud_filtered);                   //ִ���˲������洢���cloud_filtered

			//	viewer->updatePointCloud(cloud_filtered, "cloud");
			//	Sleep(10);
			//}

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
		else { cout << "Failed connected with Kinect" << endl; }
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

			++pBufferRun;
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

	hr = pMapper->MapColorFrameToSkeletonFrame(NUI_IMAGE_TYPE_COLOR, colorResolution, depthResolution, 640 * 480, Pixel_Depth, 640 * 480, Color_Mapping_Skeletion_3D);

	if (S_OK == hr)
	{
		//cout << "��ɫͼ���ĵ�������ͷ����ϵ�е�λ��:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;

		for (int k = 0; k < 640 * 480; k++)
		{
			scr_cloud->points[k].x = Color_Mapping_Skeletion_3D[k].x;
			scr_cloud->points[k].y = Color_Mapping_Skeletion_3D[k].y;
			scr_cloud->points[k].z = Color_Mapping_Skeletion_3D[k].z;
		}

		//Normals_Process();			// ���е��Ƶķ��ߴ���
		Plane_segmentation();		// ����ƽ�棬ƽ��ָ�
	}
	else
	{
		cout << "Can not Mapping" << endl;
		return -1;
	}

	return 0;


}

//-------------------------------���Ʋ������ֹ���-------------------------------------------- 
void Normals_Process()
{
	double Time = (double)cvGetTickCount();

	// --- ���͵��� ---
	Sor_VoxelGrid.setInputCloud(scr_cloud);						//������Ҫ���˵ĵ��Ƹ��˲�����
	Sor_VoxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);				//�����˲�ʱ���������ش�СΪ5cm������
	Sor_VoxelGrid.filter(*cloud_filtered);						//ִ���˲������洢���cloud_filtered

	cout << "���͵�����ĵ����е����Ŀ��" << cloud_filtered->size() << endl;	

	// -- ȥ����Ⱥ�� -- 
	Sor_OutlinerRemoval.setInputCloud(cloud_filtered);
	Sor_OutlinerRemoval.setMeanK(500);
	Sor_OutlinerRemoval.setStddevMulThresh(0.5);
	Sor_OutlinerRemoval.filter(*cloud_outliner);

	cout << "ȥ����Ⱥ�������е����Ŀ��" << cloud_outliner->size() << endl;

	// --- �ٴν��͵��� ---
	Sor_VoxelGrid.setInputCloud(cloud_outliner);				//������Ҫ���˵ĵ��Ƹ��˲�����
	Sor_VoxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);				//�����˲�ʱ���������ش�СΪ5cm������
	Sor_VoxelGrid.filter(*cloud_filtered);						//ִ���˲������洢���cloud_filtered

	cout << "�ٴν��͵�����ĵ����е����Ŀ��" << cloud_filtered->size() << endl;
	
	//�������߹��ƵĶ���
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_filtered);
	//����ÿһ���㶼�ð뾶Ϊ3cm�Ľ���������ʽ
	normalEstimation.setRadiusSearch(3);
	//Kd_tree��һ�����ݽṹ���ڹ�������Լ��������ƣ����߹��ƶ����ʹ�����ֽṹ���ҵ�����ڵ�
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	//���㷨��
	normalEstimation.compute(*normals);
	cout << "��" << normals->points.size() / 2 << "���㴦�ķ���ϵ��Ϊ��"<<"[" << normals->points[normals->points.size() / 2].normal_x << "," << normals->points[normals->points.size() / 2].normal_y << "," << normals->points[normals->points.size() / 2].normal_z << "]" << endl;
	cout << "��10���㴦�ķ���ϵ��Ϊ��" << "[" << normals->points[10].normal_x << "," << normals->points[10].normal_y << "," << normals->points[10].normal_z << "]" << endl;


	Time = (double)cvGetTickCount() - Time;
	cout << "���߹���ʱ��Ϊ��" << Time / (cvGetTickFrequency() * 1000000) << endl;//��

	////���ӻ�
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
	//viewer->addCoordinateSystem(1.0);
	//viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud");
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filtered, normals, 40, 1, "normals");

	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(1000000));
	//}

}

//-------------------------------����ƽ��ָ�-------------------------------------------- 
void Plane_segmentation()
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);		// ģ�Ͳ���
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);

	seg.setInputCloud(scr_cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		//return (-1);
	}

	cout << "The Point Number in PCL Cloud��"<< inliers->indices.size() << endl;

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
}