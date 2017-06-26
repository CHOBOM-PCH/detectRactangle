#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <vector>
#include <math.h>
#include <iostream>

#ifdef _DEBUG
#pragma comment(lib,"opencv_core2413d.lib")
#pragma comment(lib,"opencv_highgui2413d.lib")
#pragma comment(lib,"opencv_imgproc2413d.lib")
#else
#pragma comment(lib,"opencv_core2413.lib")
#pragma comment(lib,"opencv_highgui2413.lib")
#pragma comment(lib,"opencv_imgproc2413.lib")
#endif

using namespace cv;
using namespace std;
long cnt=1;

int main(){

	Mat img;
	Mat gray_img;
	Mat edge_img;
	Mat blur_img;
	Mat threshOutput_img;
	Mat threshOutput2_img;
	Mat contour_img;

	int key;
	int i, k;
	double dp = 1;
	double min_dist = 30;
	double t, s;
	double angle12,angle23,angle34,angle41;
	int px1,py1,px2,py2,sx1,sy1,sx2,sy2,sx3,sy3,sx4,sy4,spx1,spy1,spx2,spy2;
	double len12,len23,len34,len41;
	double longlen;
	double abase, aheight;
	char buffer[25];
	cv::Point pt1, pt2, pt3, pt4;

	cv::VideoCapture cap(1);

	while(1){
		cap >> img;
		if(img.empty()) break;

		cv::cvtColor(img, gray_img, CV_BGR2GRAY);
		GaussianBlur(gray_img, blur_img, cv::Size(7,7), 3);
		adaptiveThreshold(blur_img, threshOutput_img, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 15, 2);
		threshold(blur_img, threshOutput2_img, 100, 255, THRESH_OTSU);
		Canny(threshOutput_img, edge_img, 100, 300);

		vector<vector<Point>> contour;
		findContours(edge_img, //외곽선 검출한후 저장
			contour, //외곽선 저장
			RETR_LIST, //모든 윤곽선 검색
			CHAIN_APPROX_SIMPLE);//양끝만 남김 제일위 제일아래 우측 좌측
		vector<Point> poly;
		for (int ii = 0; ii < contour.size(); ii++){
			cv::approxPolyDP(contour[ii], poly,contour[ii].size()*0.08, true);
			if (poly.size() == 4 && fabs(cv::contourArea(contour[ii])) > 1000){
				pt1 = poly[0];
				pt2 = poly[1];
				pt3 = poly[2];
				pt4 = poly[3];

				circle(img,pt1,3,Scalar(255, 0, 0),3);
				circle(img,pt2,3,Scalar(255, 0, 0),3);
				circle(img,pt3,3,Scalar(255, 0, 0),3);
				circle(img,pt4,3,Scalar(255, 0, 0),3);

				line(img, pt1, pt2, Scalar(0, 255, 0),2);
				line(img, pt1, pt4, Scalar(0, 255, 0),2);
				line(img, pt3, pt2, Scalar(0, 255, 0),2);
				line(img, pt4, pt3, Scalar(0, 255, 0),2);

				//len12 = sqrt(pow((pt1.x - pt2.x), 2.0) + pow((pt1.y - pt2.y), 2.0));
				//len23 = sqrt(pow((pt2.x - pt3.x), 2.0) + pow((pt2.y - pt3.y), 2.0));
				//len34 = sqrt(pow((pt3.x - pt4.x), 2.0) + pow((pt3.y - pt4.y), 2.0));
				//len41 = sqrt(pow((pt4.x - pt1.x), 2.0) + pow((pt4.y - pt1.y), 2.0));

				//longlen = len12;
				//px1 = pt1.x;
				//py1 = pt1.y;
				//px2 = pt2.x;
				//py2 = pt2.y;

				//sx1 = pt2.x;
				//sy1 = pt2.y;
				//sx2 = pt3.x;
				//sy2 = pt3.y;
				//sx3 = pt4.x;
				//sy3 = pt4.y;
				//sx4 = pt1.x;
				//sy4 = pt1.y;
				//if(len23 > longlen)
				//{
				//	longlen = len23;
				//	px1 = pt2.x;
				//	py1 = pt2.y;
				//	px2 = pt3.x;
				//	py2 = pt3.y;

				//	sx1 = pt3.x;
				//	sy1 = pt3.y;
				//	sx2 = pt4.x;
				//	sy2 = pt4.y;
				//	sx3 = pt1.x;
				//	sy3 = pt1.y;
				//	sx4 = pt2.x;
				//	sy4 = pt2.y;
				//}

				//if(len34 > longlen)
				//{
				//	longlen = len34;
				//	px1 = pt3.x;
				//	py1 = pt3.y;
				//	px2 = pt4.x;
				//	py2 = pt4.y;

				//	sx1 = pt4.x;
				//	sy1 = pt4.y;
				//	sx2 = pt1.x;
				//	sy2 = pt1.y;
				//	sx3 = pt2.x;
				//	sy3 = pt2.y;
				//	sx4 = pt3.x;
				//	sy4 = pt3.y;
				//}

				//if(len41 > longlen)
				//{
				//	longlen = len41;
				//	px1 = pt4.x;
				//	py1 = pt4.y;
				//	px2 = pt1.x;
				//	py2 = pt1.y;

				//	sx1 = pt1.x;
				//	sy1 = pt1.y;
				//	sx2 = pt2.x;
				//	sy2 = pt2.y;
				//	sx3 = pt3.x;
				//	sy3 = pt3.y;
				//	sx4 = pt4.x;
				//	sy4 = pt4.y;
				//}

				//spx1 = (sx1 + sx2) / 2;
				//spy1 = (sy1 + sy2) / 2;
				//spx2 = (sx3 + sx4) / 2;
				//spy2 = (sy3 + sy4) / 2;
				//line(img, Point(spx1,spy1), Point(spx2,spy2),Scalar(0,0,255),2);

				//px1 = spx1;
				//py1 = spy1;
				//px2 = spx2;
				//py2 = spy2;
				////find min x -> point2
				////point 1 - point2
				//abase = px2 - px1;
				//aheight = py2 - py1;
				//angle23 = atan(aheight/abase);
				//angle12 = angle23*180.0/3.141592;
				//sprintf(buffer,"ang= %02.2f",angle12);
				//cv::putText(img,buffer,Point(abs((px2+px1)/2),abs((py2+py1)/2)), 1, 1, Scalar(0,0, 255));
			}
		}
		imshow("image",img);
		imshow("edge", edge_img);
		imshow("thresh", threshOutput_img);
		key = cvWaitKey(1);
		if(key > 10) break;
		cnt++;
		if(cnt == 100)
		{
			cnt = 1;
			//total_fps = 0.0;
		}
	}
	return 0;
}
