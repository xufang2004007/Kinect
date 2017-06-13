//------------------------------------------------------------------------------
// <copyright file="DepthBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// 实现深度图像的提取
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <strsafe.h>
#include "DepthBasics.h"
#include "resource.h"
#include <iostream>
#include "stdio.h"
#include <fstream>

using namespace std;

struct Data_Pixel
{
	USHORT Depth;
	BYTE intensity;
};

/// <summary>
/// Entry point for the application
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="hPrevInstance">always 0</param>
/// <param name="lpCmdLine">command line arguments</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
/// <returns>status</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow)
{
    CDepthBasics application;
    application.Run(hInstance, nCmdShow);
}

/// <summary>
/// Constructor
/// </summary>
CDepthBasics::CDepthBasics() :
    m_pD2DFactory(NULL),
    m_pDrawDepth(NULL),
    m_hNextDepthFrameEvent(INVALID_HANDLE_VALUE),
    m_pDepthStreamHandle(INVALID_HANDLE_VALUE),
    m_bNearMode(false),
    m_pNuiSensor(NULL)
{
    // create heap storage for depth pixel data in RGBX format
    m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];
}

/// <summary>
/// Destructor
/// </summary>
CDepthBasics::~CDepthBasics()
{
    if (m_pNuiSensor)
    {
        m_pNuiSensor->NuiShutdown();
    }

    if (m_hNextDepthFrameEvent != INVALID_HANDLE_VALUE)
    {
        CloseHandle(m_hNextDepthFrameEvent);
    }

    // clean up Direct2D renderer
    delete m_pDrawDepth;
    m_pDrawDepth = NULL;

    // done with depth pixel data
    delete[] m_depthRGBX;

    // clean up Direct2D
    SafeRelease(m_pD2DFactory);

    SafeRelease(m_pNuiSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CDepthBasics::Run(HINSTANCE hInstance, int nCmdShow)
{
    MSG       msg = {0};
    WNDCLASS  wc;

    // Dialog custom window class win32创建窗口前的准备工作，构造窗口类结构
    ZeroMemory(&wc, sizeof(wc));
    wc.style         = CS_HREDRAW | CS_VREDRAW;							// 窗口类型
    wc.cbWndExtra    = DLGWINDOWEXTRA;									// 
    wc.hInstance     = hInstance;										// 当前实例句柄
    wc.hCursor       = LoadCursorW(NULL, IDC_ARROW);
    wc.hIcon         = LoadIconW(hInstance, MAKEINTRESOURCE(IDI_APP));	// 为窗口添加图标
    wc.lpfnWndProc   = DefDlgProcW;										// 定义窗口处理函数
    wc.lpszClassName = L"DepthBasicsAppDlgWndClass";					// 窗口类名

    if (!RegisterClassW(&wc))
    {
        return 0;
    }

    // Create main application window  用该窗口类创建对话框
    HWND hWndApp = CreateDialogParamW(
        hInstance,
        MAKEINTRESOURCE(IDD_APP),
        NULL,
        (DLGPROC)CDepthBasics::MessageRouter, 
        reinterpret_cast<LPARAM>(this));

    // Show window 显示对话框
    ShowWindow(hWndApp, nCmdShow);

	// 这个用来检测kinect消息的event 
    const int eventCount = 1;
    HANDLE hEvents[eventCount];

    // Main message loop  消息循环，在这个消息循环里，如果没有kinect，那就是简单的处理窗口消息即可
    while (WM_QUIT != msg.message)
    {
		// 为什么在这里赋值，相当于每次循环都赋值？因为这个句柄随着消息处理会变化
        hEvents[0] = m_hNextDepthFrameEvent;

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
		// 检查kinect事件，第一个参数1表示等待一个句柄，第二个参数是消息数组，第三个参数指示是不是要等数组里的所有消息，参数是false  
		// 第四个参数是等待多久，INFINITE表示永远，第五个参数呢，因为第四个参数说没有kinect消息这个函数就一直阻塞这里，那么它肯定可能影响正常的windows消息处理
		// 所以第五个参数表示说有些情况下也要打断这个等待，QS_ALLINPUT就表示在有windows消息时，该函数也不再阻塞的继续往下执行
        MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);

        // Explicitly check the Kinect frame event since MsgWaitForMultipleObjects
        // can return for other reasons even though it is signaled.
        Update();

		// 处理windows消息
        while (PeekMessageW(&msg, NULL, 0, 0, PM_REMOVE))
        {
            // If a dialog message will be taken care of by the dialog proc
            if ((hWndApp != NULL) && IsDialogMessageW(hWndApp, &msg))
            {
                continue;
            }

            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    return static_cast<int>(msg.wParam);
}

/// <summary>
/// Main processing function
/// </summary>
void CDepthBasics::Update()
{
    if (NULL == m_pNuiSensor)
    {
        return;
    }
	
	// 返回WAIT_OBJECT_0表示kinect有消息来，否则表示没消息
    if ( WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0) )
    {
        ProcessDepth();
    }
}

/// <summary>
/// Handles window messages, passes most to the class instance to handle
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    CDepthBasics* pThis = NULL;
    
    if (WM_INITDIALOG == uMsg)
    {
        pThis = reinterpret_cast<CDepthBasics*>(lParam);
        SetWindowLongPtr(hWnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(pThis));
    }
    else
    {
        pThis = reinterpret_cast<CDepthBasics*>(::GetWindowLongPtr(hWnd, GWLP_USERDATA));
    }

    if (pThis)
    {
        return pThis->DlgProc(hWnd, uMsg, wParam, lParam);
    }

    return 0;
}

/// <summary>
/// Handle windows messages for the class instance
/// </summary>
/// <param name="hWnd">window message is for</param>
/// <param name="uMsg">message</param>
/// <param name="wParam">message data</param>
/// <param name="lParam">additional message data</param>
/// <returns>result of message processing</returns>
LRESULT CALLBACK CDepthBasics::DlgProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
        case WM_INITDIALOG:
        {
            // Bind application window handle
            m_hWnd = hWnd;

            // Init Direct2D 初始化DirectX  
            D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &m_pD2DFactory);

            // Create and initialize a new Direct2D image renderer (take a look at ImageRenderer.h)
            // We'll use this to draw the data we receive from the Kinect to the screen 创建图片展示对象
            m_pDrawDepth = new ImageRenderer();
			// 初始化展示对象，这里用到了窗口，用到了DirectX对象，以及宽度高度参数
            HRESULT hr = m_pDrawDepth->Initialize(GetDlgItem(m_hWnd, IDC_VIDEOVIEW), m_pD2DFactory, cDepthWidth, cDepthHeight, cDepthWidth * sizeof(long));
            if (FAILED(hr))
            {
                SetStatusMessage(L"Failed to initialize the Direct2D draw device.");
            }

            // Look for a connected Kinect, and create it if found 连接kinect设备
            CreateFirstConnected();
        }
        break;

        // If the titlebar X is clicked, destroy app
        case WM_CLOSE:
            DestroyWindow(hWnd);
            break;

        case WM_DESTROY:
            // Quit the main message pump
            PostQuitMessage(0);
            break;

        // Handle button press
        case WM_COMMAND:
            // If it was for the near mode control and a clicked event, change near mode
            if (IDC_CHECK_NEARMODE == LOWORD(wParam) && BN_CLICKED == HIWORD(wParam))
            {
                // Toggle out internal state for near mode
                m_bNearMode = !m_bNearMode;

                if (NULL != m_pNuiSensor)
                {
                    // Set near mode based on our internal state
                    m_pNuiSensor->NuiImageStreamSetImageFrameFlags(m_pDepthStreamHandle, m_bNearMode ? NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE : 0);
                }
            }
            break;
    }

    return FALSE;
}

/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CDepthBasics::CreateFirstConnected()
{
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);  // 获取连接的kinect数量
    if (FAILED(hr))
    {
        return hr;
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
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it 如果是不正常的设备，那么Release掉，免得内存泄露 
        pNuiSensor->Release();
    }

    if (NULL != m_pNuiSensor)  // 如果m_pNuiSensor不为空，那表明找到某一个正常的kinect设备了
    {
        // Initialize the Kinect and specify that we'll be using depth 初始化kinect，用NUI_INITIALIZE_FLAG_USES_DEPTH表示要使用深度图 
        hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH); 
        if (SUCCEEDED(hr))
        {
            // Create an event that will be signaled when depth data is available
			// 创建这个Event，这个Event是kinect和应用程序通信的Event，当kinect有消息时，kinect SDK会通过SetEvent来通知应用程序  
			// 应用程序则通过WaitObject来等待这个Event，完成通信
            m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

            // Open a depth image stream to receive depth frames  打开深度图流，用来接收图像 
            hr = m_pNuiSensor->NuiImageStreamOpen(
                NUI_IMAGE_TYPE_DEPTH,				// 表示要打开深度图流
                NUI_IMAGE_RESOLUTION_640x480,		// 深度图大小
                0,									// 帧设置，0表示无设置
                2,									// 缓存多少帧，最大为4
                m_hNextDepthFrameEvent,				// 用来通信的Event句柄
                &m_pDepthStreamHandle);				// 用来读取数据的流句柄，要从这里读取深度图数据
        }
    }

    if (NULL == m_pNuiSensor || FAILED(hr))
    {
        SetStatusMessage(L"No ready Kinect found!");
        return E_FAIL;
    }

    return hr;
}

/// <summary>
/// Handle new depth data
/// </summary>
void CDepthBasics::ProcessDepth()
{
    HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

    // Attempt to get the depth frame
	// 通过kinect对象，从m_pDepthStreamHandle中获取图像数据，还记得m_pDepthStreamHandle么，是在初始化kinect设备时创建的深度图流  
	// 在这里调用这个代码的意义是：将一帧深度图，保存在imageFrame中
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
    if (FAILED(hr))
    {
        return;
    }

    BOOL nearMode;
    INuiFrameTexture* pTexture;

    // Get the depth image pixel texture 通过imageFrame把数据转化成纹理 
    hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture( m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
    
	if (FAILED(hr))
    {
        goto ReleaseFrame;
    }

    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it 锁定数据
    pTexture->LockRect(0, &LockedRect, NULL, 0);

	cout << LockedRect.Pitch << endl;

	Data_Pixel Data_In_Pixel[640][480];		// 定义存放像素点信息的数组
	Data_Pixel * ptrData_In_Pixel = (Data_Pixel*)Data_In_Pixel;
    // Make sure we've received valid data
    if (LockedRect.Pitch != 0)
    {
        // Get the min and max reliable depth for the current frame
        int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
        int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		// 将m_depthRGBX的首地址保存在rgbrun，方便赋值
		BYTE * rgbrun = m_depthRGBX;
        const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		const NUI_DEPTH_IMAGE_PIXEL * pBufferStart = pBufferRun;

        // end pixel is start + width*height - 1
        const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight); 

		// 对m_depthRGBX也就是rgbrun赋值 
        while ( pBufferRun < pBufferEnd )
        {
			// 定义像素点的位置信息
			//int Index_Pixel = pBufferRun - pBufferStart;		// 明确像素点是第几个
			//int X_Pixel = Index_Pixel / cDepthWidth;				// 定义像素点的横坐标
			//int Y_Pixel = Index_Pixel % cDepthWidth;				// 定义像素点的纵坐标

            // discard the portion of the depth that contains only the player index
            USHORT depth = pBufferRun->depth;
			
			ptrData_In_Pixel->Depth = depth;			// 保存深度值

            // To convert to a byte, we're discarding the most-significant
            // rather than least-significant bits.
            // We're preserving detail, although the intensity will "wrap."
            // Values outside the reliable depth range are mapped to 0 (black).

            // Note: Using conditionals in this loop could degrade performance.
            // Consider using a lookup table instead when writing production code.
            BYTE intensity = static_cast<BYTE>(depth >= minDepth && depth <= maxDepth ? depth % 256 : 0);

			ptrData_In_Pixel->intensity = intensity;	// 保存色素信息
            
			// Write out blue byte	写蓝色素位
            *(rgbrun++) = intensity;

            // Write out green byte 写绿色素位
            *(rgbrun++) = intensity;

            // Write out red byte   写红色素位
            *(rgbrun++) = intensity;

            // We're outputting BGR, the last byte in the 32 bits is unused so skip it
            // If we were outputting BGRA, we would write alpha here.
            ++rgbrun;

            // Increment our index into the Kinect's depth buffer
            ++pBufferRun;
			++ptrData_In_Pixel;
        }

        // Draw the data with Direct2D 最后将m_depthRGBX保存的图片，显示在窗口上
        m_pDrawDepth->Draw(m_depthRGBX, cDepthWidth * cDepthHeight * cBytesPerPixel);

		for (int i = 0; i < 640; i++)
		{
			for (int j = 0; j < 480; j++)
			{
				cout << Data_In_Pixel[i][j].Depth <<" ";
			}
			cout << endl;
		}
    }

    // We're done with the texture so unlock it 解锁和释放纹理
    pTexture->UnlockRect(0);

    pTexture->Release();

ReleaseFrame:
    // Release the frame 释放帧
    m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
void CDepthBasics::SetStatusMessage(WCHAR * szMessage)
{
    SendDlgItemMessageW(m_hWnd, IDC_STATUS, WM_SETTEXT, 0, (LPARAM)szMessage);
}