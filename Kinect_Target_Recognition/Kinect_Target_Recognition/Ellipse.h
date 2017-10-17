#include "opencv2/opencv.hpp"

using namespace cv;

struct Data_Ellipse
{
	double a, b, theta;
	Point point_center;
};

class Hough_Ellipse
{
  public:
	  void Computer_axy(vector<Point> contour,Size imgsize );  
	  int hough_ellipse(vector<Point> contour);
	  Mat draw_Eliipse(Mat);
	  Data_Ellipse get_data_Ellipse();
  private:
	  double a;    //半长轴
	  double b;    //短轴
	  double theta;   //椭圆的旋转角度
	  Point point_center;   //椭圆中心的坐标    
};

