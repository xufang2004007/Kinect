//------------------------------------------------------------------------------
// <copyright file="DepthBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once

#include "resource.h"
#include "NuiApi.h"

//struct Data_Pixel
//{
//	USHORT Depth;
//	BYTE intensity;
//};

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
HANDLE					m_hNextVideoFrameEvent;
HANDLE					m_pVideoStreamHandle;
HANDLE					m_hEvNuiProcessStop;							//用于结束的事件对象; 

BYTE	                m_depthRGBX[cDepthWidth*cDepthHeight*cBytesPerPixel];