//#include <stdio.h>
//#include <iostream>
//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/features2d.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//
//using namespace cv;
//int edgeThresh = 1;
//Mat res,edge, cedge;
//
////static void onTrackbar(int, void*)
////{
////	blur(res, edge, Size(3, 3));
////
////	// Run the edge detector on grayscale9
////	Canny(edge, edge, edgeThresh, edgeThresh, 7);
////	cedge = Scalar::all(0);
////
////	vector<Vec4i> lines;
////	HoughLinesP(edge, lines, 1, CV_PI / 180, 500, 500, 100);
////	
////	img.copyTo(cedge, edge);
////	for (size_t i = 0; i < lines.size(); i++)
////	{
////		Vec4i l = lines[i];
////		line(cedge, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 255, 255), 3, CV_AA);
////	}
////
////	imshow("Edge map", cedge);
////}
//int main(int argc, char** argv)
//{
//	
//	Mat img_src = imread("14.jpg", 1);
//	Mat img;
//	cvtColor(img_src, img, CV_BGR2GRAY);
//	Mat templ = imread("xx.jpg",0);
//	Mat result;
//	threshold(img, img, 0, 255, THRESH_OTSU);
//	vector<Mat> mv;
//
//	split(img_src, mv);
//	Mat b, g, r, minus;
//	Mat _b = mv[0];
//	Mat _g = mv[1];
//	Mat _r = mv[2];
//	mv[0].convertTo(b, CV_32FC1);
//	mv[1].convertTo(g, CV_32FC1);
//	mv[2].convertTo(r, CV_32FC1);
//	Mat xxxx = Mat::zeros(r.size(), CV_8UC1);
//
//	minus = r.mul(g) - b.mul(b);
//	for (int i = 0; i < r.rows; i++)
//	{
//		for (int j = 0; j < r.cols; j++)
//		{
//			int temp = (int)_r.at<uchar>(i, j) - _g.at<uchar>(i, j);
//			if (abs(temp) < 2)
//				xxxx.at<uchar>(i, j) = 255;
//
//		}
//	}
//	normalize(minus, minus, 0, 255, NORM_MINMAX, -1, Mat());
//
//
//	minus.convertTo(res, CV_8UC1);
//	int result_cols = img.cols - templ.cols + 1;
//	int result_rows = img.rows - templ.rows + 1;
//
//	result.create(result_cols, result_rows, CV_32FC1);
//	int match_method = CV_TM_SQDIFF;
//	/// Do the Matching and Normalize
//	matchTemplate(img, templ, result, match_method);
//	normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
//
//	/// Localizing the best match with minMaxLoc
//	double minVal; double maxVal; Point minLoc; Point maxLoc;
//	Point matchLoc;
//
//	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
//
//
//	/// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
//	if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)
//	{
//		matchLoc = minLoc;
//	}
//	else
//	{
//		matchLoc = maxLoc;
//	}
//
//	/// Show me what you got
//	rectangle(img_src, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar(255,0,0), 2, 8, 0);
//	rectangle(result, matchLoc, Point(matchLoc.x + templ.cols, matchLoc.y + templ.rows), Scalar::all(0), 2, 8, 0);
//
//	//namedWindow("Edge map", 1);
//
//	// create a toolbar
//	//createTrackbar("Canny threshold", "Edge map", &edgeThresh, 100, onTrackbar);
//
//	// Show the image
//	//onTrackbar(0, 0);
//
//	// Wait for a key stroke; the same function arranges events processing
//	//waitKey(0);
//
//
//	return 0;
//}
//
