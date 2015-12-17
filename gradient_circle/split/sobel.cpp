#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

#define PI 3.141592653
#define RRATE 1.544
#define SEARCHRADIUS 32
/** @function main */
int main(int argc, char** argv)
{

	Mat src, src_gray, grad;
	Mat grad_x, grad_y, magnitude, angle, grad_x_f, grad_y_f;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	/// Load an image
	//src = imread("14394948-2015-12-09-140936.bmp", 1);
	//src = imread("14394948-2015-12-09-140914.bmp", 1);
	//src = imread("14394948-2015-12-09-140717.bmp", 1);
	//src = imread("14394948-2015-12-15-112037.bmp", 1);
	//src = imread("14394948-2015-12-15-112107.bmp", 1);
	src = imread("14394948-2015-12-15-112125.bmp", 1);
	//src = imread("14394948-2015-12-15-112134.bmp", 1);
	//src = imread("14394948-2015-12-15-112148.bmp", 1);
	//src = imread("14394948-2015-12-15-112201.bmp", 1);
	if (!src.data)
	{
		return -1;
	}

	GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);

	/// Convert it to gray
	cvtColor(src, src_gray, CV_BGR2GRAY);


	/// Gradient X
	Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	//Sobel(src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);

	/// Gradient Y
	Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	//Sobel(src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	grad_x.convertTo(grad_x_f, CV_32FC1);
	grad_y.convertTo(grad_y_f, CV_32FC1);
	cartToPolar(grad_x_f, grad_y_f, magnitude, angle,true);
	/// Total Gradient (approximate)



	vector<Vec3f> circles;

	/// Apply the Hough Transform to find the circles
	//HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 188, 40, 40, 25, 30);
	HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 188, 40, 40, 45, 50);
	circle(src, Point(circles[0][0], circles[0][1]), circles[0][2], Scalar(0, 255, 0), 4, CV_AA);

	float R = circles[0][2] / RRATE;
	double radius1, radius2;



	radius1 = circles[0][2] * 3.5436;
	radius2 = circles[0][2] * 4.3456;



	//double angle2hole_1 = 82.56;
	double angle2hole_1 = 51.91;
	double angle2hole_2 = angle2hole_1 - 22.27;
	int hole_1_x = circles[0][0] + radius1 * cos(angle2hole_1/180*PI);
	int hole_1_y = circles[0][1] + radius1 * sin(angle2hole_1/180*PI);
	line(src, Point(circles[0][0], circles[0][1]), Point(circles[0][0] + (radius1 + 200) * cos(angle2hole_1 / 180 * PI), circles[0][1] + (radius1 + 200) * sin(angle2hole_1 / 180 * PI)), Scalar(0, 0, 255),3);
	line(src, Point(circles[0][0], circles[0][1]), Point(circles[0][0] + (radius1 + 200) * cos(angle2hole_2 / 180 * PI), circles[0][1] + (radius1 + 200) * sin(angle2hole_2 / 180 * PI)), Scalar(0, 0, 255),3);
	//int hole_1_x = 1004;
	//int hole_1_y = 708;
	circle(src, Point(hole_1_x, hole_1_y), 22, Scalar(0, 0, 255), 3);

	int hole_2_x = circles[0][0] + radius2 * cos(angle2hole_2 / 180 * PI);
	int hole_2_y = circles[0][1] + radius2 * sin(angle2hole_2 / 180 * PI);
	//int hole_1_x = 1004;
	//int hole_1_y = 708;
	circle(src, Point(hole_2_x, hole_2_y), 22, Scalar(0, 0, 255), 3);
	int flag_x, flag_y, max = 0;
	for (int a = 0; a < 360; a+=6)
	{
		for (int l = 0; l <SEARCHRADIUS; l++)
		{
			int C_x = cvRound(hole_1_x + (float)l*cos((float)a / 180 * PI));
			int C_y = cvRound(hole_1_y + (float)l*sin((float)a / 180 * PI));
			int count = 0;												  
			for (int n_a = 0; n_a < 360; n_a+=2)
			{
				int L_x = cvRound(C_x + (float)R * cos((float)n_a / 180 * PI));
				int L_y = cvRound(C_y + (float)R * sin((float)n_a / 180 * PI));
				float gradient_angle;
				if (angle.at<float>(L_y, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y, L_x) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y, L_x) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y-1, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y-1, L_x) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y-1, L_x) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y, L_x-1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y, L_x-1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y, L_x-1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y-1, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y-1, L_x-1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y-1, L_x-1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y-1, L_x+1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y-1, L_x+1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y-1, L_x+1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y+1, L_x-1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y+1, L_x-1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y+1, L_x-1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y+1, L_x+1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y+1, L_x+1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y+1, L_x+1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
			
			}
			if (count > max)
			{
				max = count;
				flag_x = C_x;
				flag_y = C_y;

			}

		}
	}
	circle(src, Point(flag_x, flag_y), R, Scalar(255, 0, 0), 3);










	//int hole_2_x = circles[0][0] + radius2 * cos(angle2hole_2 / 180 * PI);
	//int hole_2_y = circles[0][1] + radius2 * sin(angle2hole_2 / 180 * PI);
	//int hole_1_x = 1004;
	//int hole_1_y = 708;
	//circle(src, Point(hole_2_x, hole_2_y), 6, Scalar(0, 0, 255), 1);
	int flag_x_2, flag_y_2, max_ = 0;
	for (int a = 0; a < 360; a+=6)
	{
		for (int l = 0; l < SEARCHRADIUS; l++)
		{
			int C_x = cvRound(hole_2_x + (float)l*cos((float)a / 180 * PI));
			int C_y = cvRound(hole_2_y + (float)l*sin((float)a / 180 * PI));
			int count = 0;
			for (int n_a = 0; n_a < 360; n_a+=2)
			{
				int L_x = cvRound(C_x + (float)R * cos((float)n_a / 180 * PI));
				int L_y = cvRound(C_y + (float)R * sin((float)n_a / 180 * PI));
				float gradient_angle;
				if (angle.at<float>(L_y, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y, L_x) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y, L_x) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y - 1, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y, L_x - 1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y, L_x - 1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y, L_x - 1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y - 1, L_x)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x - 1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x - 1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y - 1, L_x + 1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x + 1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y - 1, L_x + 1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y + 1, L_x - 1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y + 1, L_x - 1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y + 1, L_x - 1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}
				if (angle.at<float>(L_y + 1, L_x + 1)  > 180.)
				{
					gradient_angle = angle.at<float>(L_y + 1, L_x + 1) - 180;

				}
				else
				{
					gradient_angle = angle.at<float>(L_y + 1, L_x + 1) + 180;

				}
				if (abs(gradient_angle - n_a) < 10.)
				{
					count++;
				}

			}
			if (count > max_)
			{
				max_ = count;
				flag_x_2 = C_x;
				flag_y_2 = C_y;

			}

		}
	}
	circle(src, Point(flag_x_2, flag_y_2), R, Scalar(255, 0, 0), 3);
	return 0;


	//src_gray.copyTo(filtered, mask);

	//HoughCircles(filtered, circles1, CV_HOUGH_GRADIENT, 1, src_gray.rows / 188, 12, 12, 15, 17);
	//vector<Point> arr_p;
	//for (size_t i = 0; i < circles1.size(); i++)
	//{
	//	Point center(cvRound(circles1[i][0]), cvRound(circles1[i][1]));
	//	int radius = cvRound(circles1[i][2]);
	//	


	//	// circle center
	//	//circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
	//	// circle outline
	//	double distance = sqrt((circles[0][0] - circles1[i][0])*(circles[0][0] - circles1[i][0]) + (circles[0][1] - circles1[i][1])*(circles[0][1] - circles1[i][1]));

	//	if (abs(distance - circles[0][2] * 3.5436) < 5 || abs(distance - circles[0][2] * 4.1456) < 5)
	//	{
	//		//circle(src, center, radius, Scalar(0, 0, 255), 1);
	//		arr_p.push_back(center);
	//	}
	//	//circle(src, center, radius1, Scalar(0, 0, 255));
	//	//circle(src, center, radius2, Scalar(0, 0, 255));
	//	//float x = r*cos(t) + h;
	//	//float y = r*sin(t) + k;
	//}
	//for (int i = 0; i < arr_p.size(); i++)
	//{
	//	for (int j = (i + 1); j < arr_p.size(); j++)
	//	{
	//		double distance = sqrt((arr_p[i].x - arr_p[j].x)*(arr_p[i].x - arr_p[j].x) + (arr_p[i].y - arr_p[j].y)*(arr_p[i].y - arr_p[j].y));
	//		if (40 < distance && 50 > distance)
	//		{
	//			circle(src, arr_p[i], 16, Scalar(0, 0, 255), 1);
	//			circle(src, arr_p[j], 16, Scalar(0, 0, 255), 1);
	//		}

	//	}
	//}
	//imshow(window_name, grad);

	//waitKey(0);

	
}