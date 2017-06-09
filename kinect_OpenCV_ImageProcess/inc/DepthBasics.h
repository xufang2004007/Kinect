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
static const int        cDepthWidth = 640;
static const int        cDepthHeight = 480;
static const int        cBytesPerPixel = 4;

static const int        cStatusMessageMaxLen = MAX_PATH * 2;


HWND                    m_hWnd;

bool                    m_bNearMode;

// Current Kinect
INuiSensor*             m_pNuiSensor;

HANDLE					m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE);
HANDLE					m_pDepthStreamHandle(INVALID_HANDLE_VALUE);

BYTE	                m_depthRGBX[cDepthWidth*cDepthHeight*cBytesPerPixel];