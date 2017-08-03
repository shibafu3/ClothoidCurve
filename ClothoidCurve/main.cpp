#ifdef _DEBUG
//Debug���[�h�̏ꍇ
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300d.lib")            // opencv_core
#else
//Release���[�h�̏ꍇ
#pragma comment(lib,"C:\\opencv\\build\\x86\\vc12\\lib\\opencv_world300.lib") 
#endif

#include "clothoidcurve.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
/*���l�v�Z���C�u����*/
#define _USE_MATH_DEFINES
#include <math.h>
/*�ϒ����X�g���C�u����*/
#include <vector>
/*OpenCV���C�u����*/
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui_c.h"

using namespace std;
using namespace cv;


int main(){
	//SingleClothoidCurve curve1;
	//curve1.SetPoint(Point(0, 0), Point(10, 10));
	//curve1.SetAngle(M_PI / 2, M_PI / 2);
	//curve1.SetCurvature(-5, 0);
	//curve1.CreatePath();

	TripleClothoidCurve clothoid;
	clothoid.SetMaxLineLength(3);
	clothoid.SetMaxa12(100);
	clothoid.SetDrawStep(0.00001);
	clothoid.SetS1S2(0.25, 0.75);
	clothoid.SetPoint(Point2d(13.8179, 61.248), Point2d(120, 20));
	clothoid.SetAccuracy(0.1, 1, 0.01, 0.01);
	clothoid.SetAngle(-M_PI / 2, 0);
	clothoid.SetCurvature(0, 0);

	//	curve3.UIInPut();
	vector<Point2d> curve;
	clothoid.CreateCurve(curve);
	clothoid.GetCurve(curve);


	//�摜�o��
	Mat image(Size(1024, 1024), CV_8U, Scalar::all(0));
	line(image, Point(0, 511), Point(1023, 511), Scalar(127), 1, CV_AA);
	line(image, Point(511, 0), Point(511, 1023), Scalar(127), 1, CV_AA);
	for (vector<Point2d>::iterator itr = curve.begin(); itr < curve.end(); itr++){
		image.at<unsigned char>(Point2d(itr->x, -itr->y) + Point2d(511, 511)) = 255;
	}
	imshow("ClothoidCurve", image);


	//�t�@�C���o��
	ofstream ofs("C:\\Users\\0133752\\Desktop\\Clothoid.csv");
	for (int i = 0; i < curve.size(); ++i){
		ofs << setprecision(16) << curve[i].x << "," << setprecision(16) << curve[i].y << endl;
	}

	//	curve3.CorrectEndPoint();
//	clothoid.PrintCurve();
	clothoid.PrintParam();

	waitKey(0);
	return 0;
}