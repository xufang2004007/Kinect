//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "NuiApi.h"
#include <string>

//struct Data_Pixel
//{
//	USHORT Depth;
//	BYTE intensity;
//};
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

//----深度图画布-----------------------------------------------------------------
BYTE*	                m_depthRGBX;

//---ColorImage保存名称数组-------------------------------
string ImageName[10] = { "Kinect1.jpg","Kinect2.jpg", "Kinect3.jpg", "Kinect4.jpg", "Kinect5.jpg", "Kinect6.jpg", "Kinect7.jpg", "Kinect8.jpg", "Kinect9.jpg", "Kinect10.jpg" };