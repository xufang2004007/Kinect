#include "stdafx.h"
#include"Ellipse.h"
#include"math.h"
#include <iostream>
using namespace std;

inline void Hough_Ellipse::Computer_axy(vector<Point> contour,Size imgsize )
{ 
   float Ly,Lx,LL;
   double maxVal;
   Mat distance(1,contour.size(),CV_32FC1) ;      //ÿһ�㵽���������о���
   Mat max_distance(imgsize,CV_32FC1,Scalar(0));  //ÿһ�㵽������������
  
   for(int i=0;i<max_distance.rows;i++)
   {
	   for(int j=0;j<max_distance.cols;j++)
	   {
         for(int n=0;n<contour.size();n++)
		 {
			Ly=(i-contour.at(n).y)*(i-contour.at(n).y);
			Lx=(j-contour.at(n).x)*(j-contour.at(n).x);
			LL=sqrt(Ly+Lx);
			distance.at<float>(n)=LL;
		 }
		 minMaxLoc(distance, NULL,&maxVal,NULL,NULL);
		 max_distance.at<float>(i,j)=maxVal;
	   }
   }
    double minVal = 0; //���ֵһ��Ҫ����ֵ����������ʱ�ᱨ��
    Point minLoc;
    minMaxLoc(max_distance,&minVal, NULL,  &minLoc, NULL);
	a=minVal;
	point_center=minLoc;
}

inline int Hough_Ellipse::hough_ellipse(vector<Point> contour)
{   
  double G,XX,YY;
  int B;
  Mat hough_space(floor(a+1),180,CV_8UC1,Scalar(0));  //�߶�:a�����180
  
  for(int k=0;k<contour.size();k++)
	{
	  for(int w=0;w<180;w++)
	  {
		  G = w*CV_PI/180; //�Ƕ�ת��Ϊ����
		  XX = pow(((contour.at(k).y-point_center.y)*cos(G)+(contour.at(k).x-point_center.x)*sin(G)),2)/(a*a);
		  YY = pow((-(contour.at(k).y-point_center.y)*sin(G)+(contour.at(k).x-point_center.x)*cos(G)),2);
		  
		  B = floor(sqrt(abs(YY/(1-XX)))+1);
		  if(B>0 && B<=a)
  		  {
			  hough_space.at<uchar>(B,w)+=1;
		  } 
	  }
	}
    double Circumference;
	double maxVal = 0; //���ֵһ��Ҫ����ֵ����������ʱ�ᱨ��
	Point maxLoc;
	minMaxLoc(hough_space, NULL,&maxVal, NULL,&maxLoc);
	b=maxLoc.y;
	theta=maxLoc.x;
	Circumference=2*CV_PI*b+4*(a-b);
	
	return maxVal;
}

inline Mat Hough_Ellipse::draw_Eliipse(Mat src )
{
	cout << "	�� ��Բ���� ����" << endl;
	cout << "		���᣺" << a << endl;
	cout << "		���᣺" << b << endl;
	cout << "		��Բ���ģ�" << point_center << endl;   
	cout << "		��ת�Ƕȣ�" << theta <<  endl;
	if ( round(a / b) >= 2.5) {
		cout << "		" << a / b << endl;
		cout << "	------------- ����Բ�ǹ�����Բ��--------------" << endl;
	}
	else
	{
		cout << "		" << a / b << endl;
		cout << "	------------------------------------- " << endl;
		ellipse(src, point_center, Size(b, a), theta, 0, 360, Scalar(0, 255, 0), 3);
	}
	return  src;
}

inline Data_Ellipse Hough_Ellipse::get_data_Ellipse()
{
	Data_Ellipse Data;
	Data.a = a;
	Data.b = b;
	Data.point_center = point_center;
	Data.theta = theta;
	return Data;
}