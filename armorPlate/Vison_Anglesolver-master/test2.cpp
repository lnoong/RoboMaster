// test2.cpp : �������̨Ӧ�ó������ڵ㡣
//
#include "stdafx.h"
#include <opencv2\opencv.hpp>
#include <iostream>
#include <string>
#include "AngleSolver.h"

using namespace cv;
using namespace std;

Mat frame,org;  //����һ��Mat���������ڴ洢ÿһ֡��ͼ��  
double time0;
int exposure = -1;
double pitch_now = 0;
float camera_m[3][3] = { { 1028.4, 0, 643.9749 }, { 0, 1029.4, 660.8618 }, { 0, 0, 1 } };
float dist_c[1][5] = { -0.4214, 0.2319 ,  0, 0 ,0 };
Mat camera_matrix = Mat(3, 3, CV_32FC1, camera_m);
Mat dist_coeff = Mat(1, 5, CV_32FC1, dist_c);
AngleSolverFactory anglesolverfactory;
AngleSolver anglesolver = AngleSolver(camera_matrix, dist_coeff, 100, 100,1,0,4000);//��������ͷ�ڲλ�����Ŀ���Сȷ������

void on_mouse(int event, int x, int y, int flags, void *ustc)//event����¼����ţ�x,y������꣬flags��ק�ͼ��̲����Ĵ���  
{
	double angle_x = 0;
	double angle_y = 0;
	static Point pre_pt = (-1, -1);//��ʼ����  
	if (event == CV_EVENT_LBUTTONDOWN)// 
	{
		pre_pt = Point(x, y);
		//cout << "x=  " << x << "y=  " << y << endl;
		RotatedRect rect(Point2f(x, y), Size2f(100, 100), 0);
		//Point2f target2d[4];
		anglesolverfactory = AngleSolverFactory(&anglesolver);
		anglesolverfactory.getAngle(rect, anglesolverfactory.TARGET_SAMLL_ATMOR, angle_x, angle_y, 14, pitch_now, Point2f(0, 0));
		pitch_now = angle_y;
	}
}

int main(int argc, _TCHAR* argv[]){
	//cout << camera_matrix << endl;
	//cout << dist_coeff << endl;
	//��1��������ͷ������Ƶ  
	VideoCapture capture(0);
	if (false == capture.isOpened())
	{
		return -1;
	}
	exposure = capture.get(CV_CAP_PROP_EXPOSURE);
	//��ʾ�ع�ֵ  
	cout << ">����ǰ: �ع�ֵ= " << exposure << endl;
	//�����ع�ֵ  
	capture.set(CV_CAP_PROP_EXPOSURE, exposure);
	exposure = capture.get(CV_CAP_PROP_EXPOSURE);
	//��ʾ�ع�ֵ  
	cout << ">���ú�: �ع�ֵ= " << exposure << endl;
	double width = capture.get(CV_CAP_PROP_FRAME_WIDTH);
	double height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	//��ʾ�ߴ�  
	cout << ">��= " << width << ";  ��: =" << height << endl;
	//��2��ѭ����ʾÿһ֡
	cvNamedWindow("GGB", 1);
	while (1)
	{

		time0 = static_cast<double>(getTickCount());//��¼��ʼʱ��  
		capture >> frame;  //��ȡ��ǰ֡  
		//����Ƶ������ɣ��˳�ѭ��  
		if (frame.empty())
		{
			break;
		}
		imshow("GGB", frame);  //��ʾ��ǰ֡
		setMouseCallback("GGB",on_mouse, 0);
		//cvShowImage("src", frame);
		//��ʾ֡��  
		//cout << ">֡��= " << getTickFrequency() / (getTickCount() - time0) << endl;  reinterpret_cast<void*> (&frame)
		char c = (char)waitKey(10);
		if (c == 27)
			break;
	}
	return 0;
}

