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
#include <Shlobj.h>
#include "nuiapi.h"						// kinetc数据api
#include "Imagebasics.h"				// 深度图像基础定义
#include "kinect_depthdata.h"			// 深度图像基础数据
//----------------------------【opencv头文件】-------------------------------------
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>			// 包含所有的opencv2头文件

//----------------------------【PCL头文件】-------------------------------------
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

// 解决vtk编译出现的错误
#include <vtkAutoInit.h>  
VTK_MODULE_INIT(vtkRenderingOpenGL);

//----------------------------【命名空间定义】-------------------------------------
using namespace std;
using namespace cv;
using namespace pcl;


//----------------------------【函数与全局变量声明】-----------------------------------------
BOOL CreateFirstConnected();					// Kinect连接并打开数据流
int Color_Process(HANDLE h);					// 彩色图处理程序
int Depth_Process(HANDLE h);					// 深度图处理程序
int Mapping_Color_To_Skeletion(void);			// 彩色图向摄像头坐标系的位置转换
void Normals_Process();							// 对点云的法线进行处理
void Plane_segmentation();						// 对点云进行平面分割

int i = 0;										// 保存彩色图的序号
int PCD_Number = 0;								// 保存PCD点云的序号
Mat Copy_Color;

PointCloud<pcl::PointXYZ>::Ptr					scr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud<pcl::PointXYZ>::Ptr					cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr				cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>);
PointCloud<pcl::Normal>::Ptr					normals(new pcl::PointCloud<pcl::Normal>);
VoxelGrid<pcl::PointXYZ>						Sor_VoxelGrid;			//	创建降低点数的滤波对象
pcl::StatisticalOutlierRemoval<pcl::PointXYZ>	Sor_OutlinerRemoval;	// 创建去除离群点的滤波对象


//-------------------------------主程序--------------------------------------------
int main(int argc, char * argv[])
{
	
	int connect_hr = CreateFirstConnected();							// 进行连接检测程序
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
			//	sor.setInputCloud(scr_cloud);                      //设置需要过滤的点云给滤波对象
			//	sor.setLeafSize(0.03f, 0.03f, 0.03f);           //设置滤波时创建的体素大小为1cm立方体
			//	sor.filter(*cloud_filtered);                   //执行滤波处理，存储输出cloud_filtered

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
		else { cout << "Failed connected with Kinect" << endl; }
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

			++pBufferRun;
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

	hr = pMapper->MapColorFrameToSkeletonFrame(NUI_IMAGE_TYPE_COLOR, colorResolution, depthResolution, 640 * 480, Pixel_Depth, 640 * 480, Color_Mapping_Skeletion_3D);

	if (S_OK == hr)
	{
		//cout << "彩色图中心点在摄像头坐标系中的位置:  [ " << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].x << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].y << "mm ," << 1000 * Color_Mapping_Skeletion_3D[239 * 640 + 319].z << "mm ]" << endl;

		for (int k = 0; k < 640 * 480; k++)
		{
			scr_cloud->points[k].x = Color_Mapping_Skeletion_3D[k].x;
			scr_cloud->points[k].y = Color_Mapping_Skeletion_3D[k].y;
			scr_cloud->points[k].z = Color_Mapping_Skeletion_3D[k].z;
		}

		//Normals_Process();			// 进行点云的法线处理
		Plane_segmentation();		// 割离平面，平面分割
	}
	else
	{
		cout << "Can not Mapping" << endl;
		return -1;
	}

	return 0;


}

//-------------------------------点云采样发现估计-------------------------------------------- 
void Normals_Process()
{
	double Time = (double)cvGetTickCount();

	// --- 降低点数 ---
	Sor_VoxelGrid.setInputCloud(scr_cloud);						//设置需要过滤的点云给滤波对象
	Sor_VoxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);				//设置滤波时创建的体素大小为5cm立方体
	Sor_VoxelGrid.filter(*cloud_filtered);						//执行滤波处理，存储输出cloud_filtered

	cout << "降低点数后的点云中点的数目：" << cloud_filtered->size() << endl;	

	// -- 去除离群点 -- 
	Sor_OutlinerRemoval.setInputCloud(cloud_filtered);
	Sor_OutlinerRemoval.setMeanK(500);
	Sor_OutlinerRemoval.setStddevMulThresh(0.5);
	Sor_OutlinerRemoval.filter(*cloud_outliner);

	cout << "去除离群点后点云中点的数目：" << cloud_outliner->size() << endl;

	// --- 再次降低点数 ---
	Sor_VoxelGrid.setInputCloud(cloud_outliner);				//设置需要过滤的点云给滤波对象
	Sor_VoxelGrid.setLeafSize(0.1f, 0.1f, 0.1f);				//设置滤波时创建的体素大小为5cm立方体
	Sor_VoxelGrid.filter(*cloud_filtered);						//执行滤波处理，存储输出cloud_filtered

	cout << "再次降低点数后的点云中点的数目：" << cloud_filtered->size() << endl;
	
	//创建法线估计的对象
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud_filtered);
	//对于每一个点都用半径为3cm的近邻搜索方式
	normalEstimation.setRadiusSearch(3);
	//Kd_tree是一种数据结构便于管理点云以及搜索点云，法线估计对象会使用这种结构来找到最近邻点
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);

	//计算法线
	normalEstimation.compute(*normals);
	cout << "第" << normals->points.size() / 2 << "个点处的法线系数为："<<"[" << normals->points[normals->points.size() / 2].normal_x << "," << normals->points[normals->points.size() / 2].normal_y << "," << normals->points[normals->points.size() / 2].normal_z << "]" << endl;
	cout << "第10个点处的法线系数为：" << "[" << normals->points[10].normal_x << "," << normals->points[10].normal_y << "," << normals->points[10].normal_z << "]" << endl;


	Time = (double)cvGetTickCount() - Time;
	cout << "法线估计时间为：" << Time / (cvGetTickFrequency() * 1000000) << endl;//秒

	////可视化
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

//-------------------------------点云平面分割-------------------------------------------- 
void Plane_segmentation()
{
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);		// 模型参数
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

	cout << "The Point Number in PCL Cloud："<< inliers->indices.size() << endl;

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;
}