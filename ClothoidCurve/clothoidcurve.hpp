#ifndef CLOTHOIDCURVE_HPP
#define CLOTHOIDCURVE_HPP

/*OpenCVライブラリ*/
#include "opencv2/opencv.hpp"
/*数値計算ライブラリ*/
#define _USE_MATH_DEFINES
#include <math.h>
/*可変長リストライブラリ*/
#include <vector>
#include <iostream>


class SingleClothoidCurve{
	cv::Point point_start;
	cv::Point vector_start;
	double radius_start;
	double curvature_start;

	cv::Point point_goal;
	cv::Point vector_goal;
	double radius_goal;
	double curvature_goal;

	std::vector<cv::Point2d> path;


public:
	void SetPoint(cv::Point start, cv::Point goal){
		point_start = start;
		point_goal = goal;
	}
	void SetAngle(double start, double goal){
		radius_start = start;
		radius_goal = goal;
	}
	void SetCurvature(double start, double goal){
		curvature_start = start;
		//		curvature_goal = (radius_goal  * radius_goal) / 2;
	}
	void SetVector(cv::Point start, cv::Point goal){
		vector_start = start;
		vector_goal = goal;
	}
	void CulcVector(cv::Point start, cv::Point goal){
		vector_start = cv::Point(cos(radius_start), sin(radius_start));
		vector_goal = cv::Point(cos(radius_goal), sin(radius_goal));
	}
	cv::Point housenVectorLeft(cv::Point vector_in){
		cv::Point out;
		return out = cv::Point(vector_in.x * cos(M_PI / 2) - vector_in.y * sin(M_PI / 2), vector_in.x * sin(M_PI / 2) + vector_in.y * cos(M_PI / 2));
	}
	cv::Point housenVectorRight(cv::Point vector_in){
		cv::Point out;
		return out = cv::Point(vector_in.x * cos(-M_PI / 2) - vector_in.y * sin(-M_PI / 2), vector_in.x * sin(-M_PI / 2) + vector_in.y * cos(-M_PI / 2));
	}
	void CreatePath(){
		double S = 0;
		double phi = 0;
		double phi_0 = radius_start;
		double phi_1 = radius_goal;
		double phi_u = curvature_start;
		double phi_v = phi_1 - phi_0 - phi_u;
		double k = 0;
		double r;
		cv::Point2d n = point_start;
		double step = 0.01;

		cv::Mat image(cv::Size(1024, 1024), CV_8U, cv::Scalar::all(0));

		for (S = 0; S <= 1; S += step){
			std::cout << n << std::endl;
			phi = phi_0 + phi_u * S + phi_v * S * S;
			k = phi_u + 2 * phi_v * S;

			n = n + cv::Point2d(step * cos(phi), step * sin(phi));
			image.at<unsigned char>(n * 200 + cv::Point2d(500, 500)) = 255;


		}

		cv::imshow("sss", image);
		cv::waitKey(0);
	}

};



class TripleClothoidCurve{
	/////////////////////////////////////////////////
	//
	// Private Variable
	//
	cv::Point2d point_start;
	cv::Point2d vector_start;
	double radius_start;
	double curvature_start;

	cv::Point2d point_end;
	cv::Point2d vector_end;
	double radius_end;
	double curvature_end;

	cv::Point2d temp_point;

	std::vector<cv::Point2d> curve;

	double h;
	double r;

	double max_line_length;
	double max_a12;

	double S1, S2;
	double step_draw;
	double step_integral_sin;
	double step_integral_cos;
	double step_search_a;
	double step_search_h;
	double accuracy_width;
	double accuracy_length;

	double t11, t12, t13, t21, t22, t23;
	double a10, a11, a12, a20, a21, a22, a30, a31, a32;

	double theta0;

	double phi0;
	double phi1;

	double k0;
	double k1;

	double S;
	double phi;
	//
	// Private Variable
	//
	/////////////////////////////////////////////////

	/////////////////////////////////////////////////
	//
	// Private Function
	//
	void CalcParamA(){
		a22 = (a12*(t11*t23 - t13*t21) + h*(k0*(t23 - t13) + k1*t13) - t23*(phi1 - phi0)) / (t13*t22 - t12*t23);
		a32 = (a12*(t11*t22 - t12*t21) + h*(k0*(t22 - t12) + k1*t12) - t22*(phi1 - phi0)) / (t12*t23 - t13*t22);

		a10 = phi0 - theta0;
		a11 = k0 * h;

		a20 = a10 + (a11 * S1) + (a12 * S1 * S1);
		a21 = a11 + (2 * a12 * S1);

		a30 = a20 + a21 * (S2 - S1) + a22 * (S2 - S1) * (S2 - S1);
		a31 = a21 + 2 * a22 * (S2 - S1);
	}
	double SinPhi1(double x){
		return sin(a10 + a11 * x + a12 * x * x);
	}
	double SinPhi2(double x){
		return sin(a20 + a21 * (x - S1) + a22 * (x - S1) * (x - S1));
	}
	double SinPhi3(double x){
		return sin(a30 + a31 * (x - S2) + a32 * (x - S2) * (x - S2));
	}
	double CosPhi1(double x){
		return cos(a10 + a11 * x + a12 * x * x);
	}
	double CosPhi2(double x){
		return cos(a20 + a21 * (x - S1) + a22 * (x - S1) * (x - S1));
	}
	double CosPhi3(double x){
		return cos(a30 + a31 * (x - S2) + a32 * (x - S2) * (x - S2));
	}
	double IntegralSin(){
		double ans = 0;
		for (double S = 0; S < S1; S += step_integral_sin){
			ans += (SinPhi1(S) + SinPhi1(S + step_integral_sin)) * step_integral_sin / 2;
		}
		for (double S = S1; S < S2; S += step_integral_sin){
			ans += (SinPhi2(S) + SinPhi2(S + step_integral_sin)) * step_integral_sin / 2;
		}
		for (double S = S2; S < 1; S += step_integral_sin){
			ans += (SinPhi3(S) + SinPhi3(S + step_integral_sin)) * step_integral_sin / 2;
		}
		return ans;
	}
	double IntegralCos(){
		double ans = 0;
		for (double S = 0; S < S1; S += step_integral_cos){
			ans += (CosPhi1(S) + CosPhi1(S + step_integral_cos)) * step_integral_cos / 2;
		}
		for (double S = S1; S < S2; S += step_integral_cos){
			ans += (CosPhi2(S) + CosPhi2(S + step_integral_cos)) * step_integral_cos / 2;
		}
		for (double S = S2; S < 1; S += step_integral_cos){
			ans += (CosPhi3(S) + CosPhi3(S + step_integral_cos)) * step_integral_cos / 2;
		}
		return ans;
	}
	double GetSolustionSin(){ return IntegralSin(); }
	double GetSolustionCos(){ return h * IntegralCos() - r; }
	bool ZeroCross(double in1, double in2){
		if (in1 * in2 <= 0){
			return true;
		}
		return false;
	}
	int SearchNestOnece(double nest_initial_a, double nest_initial_h, double nest_max_a, double nest_max_h, double nest_step_a, double nest_step_h, std::vector<double> &list_a, std::vector<double> &list_h){
		double solution_sin_1;
		double solution_sin_2;
		double solution_sin_3;
		double solution_sin_4;

		double solution_cos_1;
		double solution_cos_2;
		double solution_cos_3;
		double solution_cos_4;

		int flag;
		int solution_counter = 0;

		double loop_a = nest_initial_a;
		double loop_h = nest_initial_h;

		////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////


		solution_counter = 0;
		for (loop_h = nest_initial_h; loop_h < nest_max_h; loop_h += nest_step_h){
			for (loop_a = nest_initial_a; loop_a < nest_max_a; loop_a += nest_step_a){
				if (list_a.size()){ return 0; }
				flag = 0;


				a12 = loop_a;
				h = loop_h;
				CalcParamA();
				solution_sin_1 = GetSolustionSin();
				solution_cos_1 = GetSolustionCos();


				a12 = loop_a + nest_step_a;
				h = loop_h;
				CalcParamA();
				solution_sin_2 = GetSolustionSin();
				solution_cos_2 = GetSolustionCos();


				a12 = loop_a;
				h = loop_h + nest_step_h;
				CalcParamA();
				solution_sin_3 = GetSolustionSin();
				solution_cos_3 = GetSolustionCos();


				a12 = loop_a + nest_step_a;
				h = loop_h + nest_step_h;
				CalcParamA();
				solution_sin_4 = GetSolustionSin();
				solution_cos_4 = GetSolustionCos();


				flag += ZeroCross(solution_sin_1, solution_sin_2);
				flag += ZeroCross(solution_sin_1, solution_sin_3);
				flag += ZeroCross(solution_sin_2, solution_sin_4);
				flag += ZeroCross(solution_sin_3, solution_sin_4);
				flag += ZeroCross(solution_cos_1, solution_cos_2);
				flag += ZeroCross(solution_cos_1, solution_cos_3);
				flag += ZeroCross(solution_cos_2, solution_cos_4);
				flag += ZeroCross(solution_cos_3, solution_cos_4);

				if (flag >= 4){
					if ((nest_step_h < accuracy_length) && (nest_step_a < accuracy_width)){
						list_a.push_back(loop_a);
						list_h.push_back(loop_h);
						return 0;
					}
					SearchNestOnece(loop_a, loop_h,
						loop_a + nest_step_a, loop_h + nest_step_h,
						nest_step_a / 10, nest_step_h / 10,
						list_a, list_h);
				}
			}
		}

		return 0;
	}
	int SearchParamAHNext(){
		std::vector<double> list_a;
		std::vector<double> list_h;

		////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////

		SearchNestOnece(-max_a12, r,
			max_a12, max_line_length * r,
			step_search_a, step_search_h,
			list_a, list_h);

		if (list_a.empty()){ return -1; }
		a12 = list_a[0];
		h = list_h[0];
		CalcParamA();
		return 0;
	}
	//
	// Private Function
	//
	/////////////////////////////////////////////////
public:

	/////////////////////////////////////////////////
	//
	// Set
	//
	void SetMaxLineLength(double length){
		max_line_length = length;
	}
	void SetMaxa12(double a12){
		max_a12 = a12;
	}
	void SetPoint(cv::Point2d start, cv::Point2d end){
		point_start = start;
		point_end = end;

		r = sqrt((point_end.x - point_start.x) * (point_end.x - point_start.x) + (point_end.y - point_start.y) * (point_end.y - point_start.y));
		theta0 = atan2(point_end.y - point_start.y, point_end.x - point_start.x);
	}
	void SetAngle(double start, double end){
		radius_start = start;
		radius_end = end;

		vector_start = cv::Point2d(cos(radius_start), sin(radius_start));
		vector_end = cv::Point2d(cos(radius_end), sin(radius_end));

		phi0 = radius_start;
		phi1 = radius_end;

	}
	void SetCurvature(double start, double end){
		curvature_start = start;
		curvature_end = end;

		k0 = curvature_start;
		k1 = curvature_end;
	}
	void SetAccuracy(double width, double length, double integral_sin, double integral_cos, double search_a, double search_h){
		accuracy_width = width;
		accuracy_length = length;
		step_integral_sin = integral_sin;
		step_integral_cos = integral_cos;
		step_search_a = search_a;
		step_search_h = search_h;
	}
	void SetAccuracy(double width, double length, double integral_sin, double integral_cos){
		accuracy_width = width;
		accuracy_length = length;
		step_integral_sin = integral_sin;
		step_integral_cos = integral_cos;
		step_search_a = max_a12 * 2 / 100;
		step_search_h = ((max_line_length * r) - r) / 100;
	}
	void SetAccuracy(double width, double length){
		accuracy_width = width;
		accuracy_length = length;
		step_integral_sin = 0.001;
		step_integral_cos = 0.001;
		step_search_a = max_a12 * 2 / 100;
		step_search_h = ((max_line_length * r) - r) / 100;
	}
	void SetDrawStep(double step){
		step_draw = step;
	}
	void SetS1S2(double in1, double in2){
		S1 = in1;
		S2 = in2;

		t11 = -S1 * S1 + 2 * S1;
		t12 = -S2 * S2 + 2 * S2 + S1 * S1 - 2 * S1;
		t13 = S2 * S2 - 2 * S2 + 1;
		t21 = 2 * S1;
		t22 = 2 * S2 - 2 * S1;
		t23 = 2 - 2 * S2;

		a12 = 0;
		h = 0;
	}
	//
	// Set
	//
	/////////////////////////////////////////////////////

	/////////////////////////////////////////////////////
	//
	// Create
	//
	int CreateCurve(std::vector<cv::Point2d> &list_curve){
		S = 0;
		phi = 0;
		double step = step_draw;
		temp_point = point_start;

		curve.clear();

		if (SearchParamAHNext()){ return -1; }
		curve.push_back(temp_point);
		for (double S = step; S < S1; S += step){
			phi = theta0 + a10 + a11 * S + a12 * S * S;

			temp_point += h * cv::Point2d(step * cos(phi), step * sin(phi));
			curve.push_back(temp_point);
		}
		for (double S = S1; S < S2; S += step){
			phi = theta0 + a20 + a21 * (S - S1) + a22 * (S - S1) * (S - S1);

			temp_point += h * cv::Point2d(step * cos(phi), step * sin(phi));
			curve.push_back(temp_point);
		}
		for (double S = S2; S < 1; S += step){
			phi = theta0 + a30 + a31 * (S - S2) + a32 * (S - S2) * (S - S2);

			temp_point += h * cv::Point2d(step * cos(phi), step * sin(phi));
			curve.push_back(temp_point);
		}
		phi = theta0 + a30 + a31 * (1 - S2) + a32 * (1 - S2) * (1 - S2);
		temp_point += h * cv::Point2d(step * cos(phi), step * sin(phi));
		curve.push_back(temp_point);

		list_curve = curve;

		return 0;
	}
	void CorrectEndPoint(){
		cv::Point2d difference = point_end - curve.back();

		for (size_t i = 0; i < curve.size(); i++){
			curve[i] = curve[i] + (difference  * (i / double(curve.size() - 1)));
		}

		//間違い
		//for (std::vector<cv::Point2d>::iterator itr = curve.begin(); itr < curve.end(); itr++){
		//	*itr = *itr - point_start;
		//}
		//for (std::vector<cv::Point2d>::iterator itr = curve.begin(); itr < curve.end(); itr++){
		//	*itr = cv::Point2d(itr->x * (point_end.x - point_start.x) / curve.back().x, itr->y * (point_end.y - point_start.y) / curve.back().y) + point_start;
		//}
	}
	//
	// Create
	//
	/////////////////////////////////////////////////////

	/////////////////////////////////////////////////////
	//
	// OutPut
	//
	void PrintParam(){
		std::cout << "S1  " << S1 << ", " << std::endl;
		std::cout << "S2  " << S2 << ", " << std::endl;
		std::cout << "a10 " << a10 << ", " << phi0 - theta0 << std::endl;
		std::cout << "a11 " << a11 << ", " << k0 * h << std::endl;
		std::cout << "a12 " << a12 << ", " << std::endl;
		std::cout << "a20 " << a20 << ", " << a10 + a11 * S1 + a12 * S1 * S1 << std::endl;
		std::cout << "a21 " << a21 << ", " << a11 + 2 * a12 * S1 << std::endl;
		std::cout << "a22 " << a22 << ", " << (a12 * (t11 * t23 - t13 * t21) + h * (k0 * (t23 - t13) + k1 * t13) - (phi1 - phi0) * t23) / (t13 * t22 - t12 * t23) << std::endl;
		std::cout << "a30 " << a30 << ", " << a20 + a21 * (S2 - S1) + a22 * (S2 - S1) * (S2 - S1) << std::endl;
		std::cout << "a31 " << a31 << ", " << a21 + 2 * a22 * (S2 - S1) << std::endl;
		std::cout << "a32 " << a32 << ", " << (a12 * (t11 * t22 - t12 * t21) + h * (k0 * (t22 - t12) + k1 * t12) - (phi1 - phi0) * t22) / (t12 * t23 - t13 * t22) << std::endl;
		std::cout << "phi0 " << phi0 << ", " << a10 << std::endl;
		std::cout << "phi1 " << phi1 << ", " << a30 + a31 * (1 - S2) + a32 * (1 - S2) * (1 - S2) << ", " << phi << std::endl;
		std::cout << "theta0 " << theta0 << std::endl;
		std::cout << "h   " << h << ", " << std::endl;
		std::cout << "r   " << r << ", " << std::endl;
		std::cout << "k0  " << k0 << ", " << a11 / h << std::endl;
		std::cout << "k1  " << k1 << ", " << (a31 + 2 * a32 * (1 - S2)) / h << std::endl;
		std::cout << "k0.1 " << (a11 + 2 * a12 * S1) / h << std::endl;
		std::cout << "k0.2 " << (a21 + 2 * a22 * (S2 - S1)) / h << std::endl;

		std::cout << "intesin " << IntegralSin() << std::endl;
		std::cout << "intecos " << IntegralCos() << std::endl;
		std::cout << "absh " << abs(r - h * IntegralCos()) << std::endl;
		std::cout << "absa " << abs(IntegralSin()) << std::endl;
		std::cout << "arival Point" << curve.back() << std::endl;
	}
	void GetParam(
		double &S1_out, double &S2_out,
		double &a10_out, double &a11_out, double &a12_out,
		double &a20_out, double &a21_out, double &a22_out,
		double &a30_out, double &a31_out, double &a32_out,
		double &phi0_out, double &phi1_out,
		double &theta0_out,
		double &h_out,
		double &r_out,
		double &k0_out, double &k1_out){

		S2_out = S2;
		S1_out = S1;
		a10_out = phi0 - theta0;
		a11_out = k0 * h;
		a12_out = a12;
		a20_out = a10 + a11 * S1 + a12 * S1 * S1;
		a21_out = a11 + 2 * a12 * S1;
		a22_out = (a12 * (t11 * t23 - t13 * t21) + h * (k0 * (t23 - t13) + k1 * t13) - (phi1 - phi0) * t23) / (t13 * t22 - t12 * t23);
		a30_out = a20 + a21 * (S2 - S1) + a22 * (S2 - S1) * (S2 - S1);
		a31_out = a21 + 2 * a22 * (S2 - S1);
		a32_out = (a12 * (t11 * t22 - t12 * t21) + h * (k0 * (t22 - t12) + k1 * t12) - (phi1 - phi0) * t22) / (t12 * t23 - t13 * t22);
		phi0_out = a10;
		phi1_out = a30 + a31 * (1 - S2) + a32 * (1 - S2) * (1 - S2);
		theta0_out = theta0;
		h_out = h;
		r_out = r;
		k0 = a11 / h;
		k1 = (a31 + 2 * a32 * (1 - S2)) / h;

		return;
	}
	void PrintCurve(){
		std::cout << curve << std::endl;
	}
	void GetCurve(std::vector<cv::Point2d> &out_curve){
		out_curve = curve;
	}
	//
	// OutPut
	//
	/////////////////////////////////////////////////////
};

#endif