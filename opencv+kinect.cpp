#include <opencv2\opencv.hpp>  
#include<iostream>
//windows��ͷ�ļ�������Ҫ����ȻNuiApi.h�ò���
#include <Windows.h>
//Kinect for windows ��ͷ�ļ�
#include "NuiApi.h"
 
using namespace std;
using namespace cv;
 
#include <d3d11.h>
 
 
//��Զ����(mm)
const int MAX_DISTANCE = 3500;
//�������(mm)
const int MIN_DISTANCE = 200;
 
const LONG m_depthWidth = 640;
const LONG m_depthHeight = 480;
const LONG m_colorWidth = 640;
const LONG m_colorHeight = 480;
const LONG cBytesPerPixel = 4;
 
int main()
{
    //��ɫͼ��
    Mat image_rgb;
    //���ͼ��
    Mat image_depth;
 
    //����һ��MAT
    image_rgb.create(480,640,CV_8UC3);
    image_depth.create(480,640,CV_8UC1);
 
    //һ��KINECTʵ��ָ��
    INuiSensor*  m_pNuiSensor = NULL;
 
    if (m_pNuiSensor != NULL)
    {
        return 0;
    }
 
    //��¼��ǰ����KINECT��������Ϊ��������׼����
    int iSensorCount;
    //��õ�ǰKINECT������
    HRESULT hr = NuiGetSensorCount(&iSensorCount);
 
 
    //�������г�ʼ��KINETCʵ���������������һ��KINECT������û���õ�ѭ��
    hr = NuiCreateSensorByIndex(iSensorCount - 1, &m_pNuiSensor);
    //��ʼ����������Խ��ղ�ɫ�����������
    hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
 
    //�ж��Ƿ����
    if (FAILED(hr))
    {
        cout<<"NuiInitialize failed"<<endl;
        return hr;
    }
 
    //��ɫͼ���ȡ��һ֡�¼�
    HANDLE nextColorFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    //��ɫͼ���¼����
    HANDLE colorStreamHandle = NULL;
    //���ͼ���ȡ��һ֡�¼�
    HANDLE nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    //���ͼ���¼����
    HANDLE depthStreamHandle = NULL;
 
    //ʵ����������������NUI_IMAGE_TYPE_COLOR��ʾ��ɫͼ��
    hr = m_pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0,2,nextColorFrameEvent,&colorStreamHandle);
 
    if( FAILED( hr ) )//�ж��Ƿ���ȡ��ȷ
    {
        cout<<"Could not open color image stream video"<<endl;
        m_pNuiSensor->NuiShutdown();
        return hr;
    }
 
    //ʵ����������������NUI_IMAGE_TYPE_DEPTH��ʾ���ͼ��
    hr = m_pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0,2, nextDepthFrameEvent, &depthStreamHandle);
 
    if( FAILED( hr ) )//�ж��Ƿ���ȡ��ȷ
    {
        cout<<"Could not open color image stream video"<<endl;
        m_pNuiSensor->NuiShutdown();
        return hr;
    }
 
 
 
    cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
    moveWindow("depth",300,600);
    cv::namedWindow("colorImage",CV_WINDOW_AUTOSIZE);
    moveWindow("colorImage",0,200);
 
    while (1)
    {
        NUI_IMAGE_FRAME pImageFrame_rgb;
        NUI_IMAGE_FRAME pImageFrame_depth;
 
        //���޵ȴ��µĲ�ɫ���ݣ��ȵ��󷵻�
        if (WaitForSingleObject(nextColorFrameEvent, 0) == 0)
        {
            //�ӸղŴ���������������еõ���֡���ݣ���ȡ�������ݵ�ַ����pImageFrame
            hr = m_pNuiSensor->NuiImageStreamGetNextFrame(colorStreamHandle, 0, &pImageFrame_rgb);
            if (FAILED(hr))
            {
                cout<<"Could not get color image"<<endl;
                m_pNuiSensor->NuiShutdown();
                return -1;
            }
 
            INuiFrameTexture *pTexture = pImageFrame_rgb.pFrameTexture;
            NUI_LOCKED_RECT lockedRect;
 
            //��ȡ����֡��LockedRect���������������ݶ���pitchÿ���ֽ�����pBits��һ���ֽڵ�ַ
            //���������ݣ����������Ƕ����ݵ�ʱ��kinect�Ͳ���ȥ�޸���
 
 
            pTexture->LockRect(0, &lockedRect, NULL, 0);
            //ȷ�ϻ�õ������Ƿ���Ч
            if (lockedRect.Pitch != 0)
            {
                //������ת��ΪOpenCV��Mat��ʽ
                for (int i = 0; i < image_rgb.rows; i++)
                {
                    //��i�е�ָ��
                    uchar *prt = image_rgb.ptr(i);
 
                    //ÿ���ֽڴ���һ����ɫ��Ϣ��ֱ��ʹ��uchar
                    uchar *pBuffer = (uchar*)(lockedRect.pBits) + i * lockedRect.Pitch;
 
                    for (int j = 0; j < image_rgb.cols; j++)
                    {   
                        prt[3 * j] = pBuffer[4 * j];//�ڲ�������4���ֽڣ�0-1-2��BGR����4������δʹ��
                        prt[3 * j + 1] = pBuffer[4 * j + 1];
                        prt[3 * j + 2] = pBuffer[4 * j + 2];
                    }
                }
 
                imshow("colorImage",image_rgb);
                //�������
                pTexture->UnlockRect(0);
                //�ͷ�֡
                m_pNuiSensor->NuiImageStreamReleaseFrame(colorStreamHandle, &pImageFrame_rgb );
            }
            else
            {
                cout<<"Buffer length of received texture is bogus\r\n"<<endl;
            }
 
            BOOL nearMode;
            INuiFrameTexture* pColorToDepthTexture;  
 
 
            //���ͼ��Ĵ���
            if (WaitForSingleObject(nextDepthFrameEvent, INFINITE) == 0)
            {
 
                hr = m_pNuiSensor->NuiImageStreamGetNextFrame(depthStreamHandle, 0 , &pImageFrame_depth);
 
                if (FAILED(hr))
                {
                    cout<<"Could not get color image"<<endl;
                    NuiShutdown();
                    return -1;
                }
 
                hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(  
                    depthStreamHandle, &pImageFrame_depth, &nearMode, &pColorToDepthTexture);  
                INuiFrameTexture *pTexture = pImageFrame_depth.pFrameTexture;
                NUI_LOCKED_RECT lockedRect;
                NUI_LOCKED_RECT ColorToDepthLockRect;  
 
                pTexture->LockRect(0, &lockedRect, NULL, 0);
                pColorToDepthTexture->LockRect(0,&ColorToDepthLockRect,NULL,0);  
 
                //��һ��
                for (int i = 0; i < image_depth.rows; i++)
                {
                    uchar *prt = image_depth.ptr<uchar>(i);
 
                    uchar* pBuffer = (uchar*)(lockedRect.pBits) + i * lockedRect.Pitch;
                    //������Ҫת������Ϊÿ�����������2���ֽڣ�Ӧ��BYTEת��USHORT
                    USHORT *pBufferRun = (USHORT*)pBuffer;
 
                    for (int j = 0; j < image_depth.cols; j++)
                    {
                        //���򣬽����ݹ�һ����������Ⱦ�����300mm-3500mm��Χ�ڵ����أ�ӳ�䵽��0��255���ڣ�
                        //������Χ�ģ���ȥ���Ǳ�Ե����
                        if (pBufferRun[j] << 3 > MAX_DISTANCE) prt[j] = 255;
                        else if(pBufferRun[j] << 3 < MIN_DISTANCE) prt[j] = 0;
                        else prt[j] = (BYTE)(256 * (pBufferRun[j] << 3)/ MAX_DISTANCE);
                    }
                }
                imshow("depth", image_depth);
 
 
 
                //�������Ƕ��벿�֣���ǰ���ٳ���
 
                //�����ȵ�Ĳ���
                NUI_DEPTH_IMAGE_POINT* depthPoints = new NUI_DEPTH_IMAGE_POINT[640 * 480];
                if (ColorToDepthLockRect.Pitch != 0)  
                {  
                    HRESULT hrState = S_OK;  
                     
                    //һ�����ڲ�ͬ�ռ�����ת����ࣨ��������ȣ���ɫ��������
                    INuiCoordinateMapper* pMapper;  
 
                    //����KINECTʵ���Ŀռ�����ϵ
                    hrState = m_pNuiSensor->NuiGetCoordinateMapper(&pMapper);  
 
                    if (FAILED(hrState))  
                    {  
                        return hrState;  
                    }  
 
                    //��Ҫ��һ��������ɫ�ռ�ӳ�䵽��ȿռ䡣����˵����
                    //������1������ɫͼ�������
                    //������2������ɫͼ��ķֱ���
                    //������3�������ͼ��ķֱ���
                    //������4�������ͼ��ĸ���
                    //������5����������ص���
                    //������6����ȡ�ڴ�Ĵ�С������������ΪNUI_DEPTH_IMAGE_PIXEL
                    //������7�������ӳ������Ĳ���
                    hrState = pMapper->MapColorFrameToDepthFrame(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, NUI_IMAGE_RESOLUTION_640x480,  
                        640 * 480, (NUI_DEPTH_IMAGE_PIXEL*)ColorToDepthLockRect.pBits,640 * 480, depthPoints);  
 
                    if (FAILED(hrState))  
                    {  
                        return hrState;  
                    }  
 
 
                    //��ʾ��ͼ��
                    Mat show;
                    show.create(480,640,CV_8UC3);
                    show = 0;
 
                    for (int i = 0; i < image_rgb.rows; i++)
                    {
                        for (int j = 0; j < image_rgb.cols; j++)
                        {
                            uchar *prt_rgb = image_rgb.ptr(i);
                            uchar *prt_show = show.ptr(i);
                            //���ڴ���ƫ����
                            long index = i * 640 + j;  
                            //�ӱ�����ӳ������������л�ȡ��
                            NUI_DEPTH_IMAGE_POINT depthPointAtIndex = depthPoints[index]; 
 
                            //�߽��ж�
                            if (depthPointAtIndex.x >= 0 && depthPointAtIndex.x < image_depth.cols &&
                                depthPointAtIndex.y >=0 && depthPointAtIndex.y < image_depth.rows)
                            {
                                //����жϣ���MIN_DISTANCE��MAX_DISTANCE֮��ĵ���ǰ������ʾ����
                                //���ʹ��Ҳ����Ҫ����ʹ��������������ص��������ͼ���л�ȡ���ֵ���жϵ�ʱ�򣬻����
                                if (depthPointAtIndex.depth >= MIN_DISTANCE && depthPointAtIndex.depth <= MAX_DISTANCE)
                                {
                                    prt_show[3 * j]     = prt_rgb[j * 3];
                                    prt_show[3 * j + 1] = prt_rgb[j * 3 + 1];
                                    prt_show[3 * j + 2] = prt_rgb[j * 3 + 2];
                                }
                            }
                        }
                    }
                    imshow("show", show);
                }
 
                delete []depthPoints;
                 
                pTexture->UnlockRect(0);
                m_pNuiSensor->NuiImageStreamReleaseFrame(depthStreamHandle, &pImageFrame_depth);
            }
 
            else
            {
                cout<<"Buffer length of received texture is bogus\r\n"<<endl;
            }
        }
 
        if (cvWaitKey(20) == 27)
            break;
    }
    return 0;
}