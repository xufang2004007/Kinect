//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "NuiApi.h"
#include <string>
#include "kinect_depthdata.h"			// 深度图像基础数据

using namespace std;

//---图像分辨率------------------------------------------
static const int        cDepthWidth = 640;
static const int        cDepthHeight = 480;
static const int        cBytesPerPixel = 4;
static const int        cColorWidth = 640;
static const int        cColorHeight = 480;

static const int        cStatusMessageMaxLen = MAX_PATH * 2;


HWND                    m_hWnd;

bool                    m_bNearMode;

//---Image stream 分辨率-------------------------------
NUI_IMAGE_RESOLUTION	colorResolution = NUI_IMAGE_RESOLUTION_640x480;	//设置图像分辨率
NUI_IMAGE_RESOLUTION	depthResolution = NUI_IMAGE_RESOLUTION_640x480;

// Current Kinect
INuiSensor*             m_pNuiSensor;

//----各种内核事件和句柄----------------------------------------------------------------- 
HANDLE					m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE);
HANDLE					m_pDepthStreamHandle(INVALID_HANDLE_VALUE); 
HANDLE					m_hNextVideoFrameEvent(INVALID_HANDLE_VALUE);
HANDLE					m_pVideoStreamHandle(INVALID_HANDLE_VALUE);

//----深度图像信息-----------------------------------------------------------------
BYTE*						m_depthRGBX;
NUI_DEPTH_IMAGE_PIXEL		Pixel_Depth[cDepthWidth*cDepthHeight] = { 0,0 };								// 存储深度图像数据
NUI_COLOR_IMAGE_POINT		Depth_Mapping_Color_2D[cDepthWidth*cDepthHeight] = { 0,0,0 };					// 存放了深度图匹配到彩色图后的二维坐标位置
Depth_Mapping_Color_Pixel	Depth_Mapping_Color_3D[cDepthWidth*cDepthHeight] = { 0,0,0 };					// 存放了深度图匹配到彩色图后的三维坐标位置
Vector4						Color_Mapping_Skeletion_3D[cDepthWidth*cDepthHeight] = { 0,0,0 };					// 存放了从彩色图匹配到摄像头坐标系下的三维坐标


//---ColorImage保存名称数组-------------------------------
string ImageName[10] = { "Kinect1.jpg","Kinect2.jpg", "Kinect3.jpg", "Kinect4.jpg", "Kinect5.jpg", "Kinect6.jpg", "Kinect7.jpg", "Kinect8.jpg", "Kinect9.jpg", "Kinect10.jpg" };
string PCDFile[10] = { "PCD1.pcd","PCD2.pcd","PCD3.pcd" ,"PCD4.pcd" ,"PCD5.pcd" ,"PCD6.pcd" ,"PCD7.pcd" ,"PCD8.pcd" ,"PCD9.pcd" ,"PCD10.pcd"};
