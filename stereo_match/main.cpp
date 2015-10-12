/*
 *  stereo_match.cpp
 *  calibration
 *
 *  Created by Victor  Eruhimov on 1/18/10.
 *  Copyright 2010 Argus Corp. All rights reserved.
 *
 */

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;


static void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}
static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}
static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners)
{
	corners.resize(0);

		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(j*squareSize),
			float(i*squareSize), 0));

}
static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

int main(int argc, char** argv)
{
	int flags = 0;
	const char* inputFilename1 = "left/image_list.xml";
	const char* inputFilename2 = "right/image_list.xml";
	Size boardSize(9,6), imageSize(640,480);
	float squareSize = 30.f;
	vector<vector<Point2f> > imagePoints1;
	vector<vector<Point2f> > imagePoints2;
	Mat cameraMatrix1 = Mat::eye(3, 3, CV_64F), distCoeffs1 = Mat::zeros(8, 1, CV_64F);
	Mat cameraMatrix2 = Mat::eye(3, 3, CV_64F), distCoeffs2 = Mat::zeros(8, 1, CV_64F);
	vector<Mat> rvecs1, tvecs1;
	vector<Mat> rvecs2, tvecs2;
	vector<vector<Point3f> > objectPoints1(1);
	vector<vector<Point3f> > objectPoints2(1);

	const char* img1_filename = "left/left01.jpg";
	const char* img2_filename = "right/right01.jpg";
	char  imageList1[4][20] = { "left/left01.jpg",
		"left/left03.jpg",
		"left/left04.jpg",
		"left/left09.jpg", };
		
		
		
	char  imageList2[4][20] = { "right/right01.jpg",
		"right/right03.jpg",
		"right/right04.jpg",
		"right/right09.jpg" };

	int nframes1 = 4;
	int nframes2 = 4;
    const char* intrinsic_filename = 0;
    const char* extrinsic_filename = 0;
    const char* disparity_filename = "out.jpg";
    const char* point_cloud_filename = "xyz.txt";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
    int alg = STEREO_SGBM;
    int SADWindowSize = 0, numberOfDisparities = 0;
    bool no_display = false;
    float scale = 1.f;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);


    int color_mode = alg == STEREO_BM ? 0 : -1;
    Mat img1 = imread(img1_filename, color_mode);
    Mat img2 = imread(img2_filename, color_mode);


    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

	//readStringList(inputFilename1, imageList1);
	//readStringList(inputFilename2, imageList2);


		Mat view, viewGray;
		vector<Point2f> pointbuf;
		view = imread("left/left01.jpg", 1);
		cvtColor(view, viewGray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners(view, boardSize, pointbuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		

		cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		imagePoints1.push_back(pointbuf);


		view = imread("right/right01.jpg", 1);
		cvtColor(view, viewGray, COLOR_BGR2GRAY);
		found = findChessboardCorners(view, boardSize, pointbuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		

		cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		imagePoints2.push_back(pointbuf);




	calcChessboardCorners(boardSize, squareSize, objectPoints1[0]);
	calcChessboardCorners(boardSize, squareSize, objectPoints2[0]);

	objectPoints1.resize(imagePoints1.size(), objectPoints1[0]);
	objectPoints2.resize(imagePoints2.size(), objectPoints2[0]);

	double rms1 = calibrateCamera(objectPoints1, imagePoints1, imageSize, cameraMatrix1,
		distCoeffs1, rvecs1, tvecs1, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	double rms2 = calibrateCamera(objectPoints2, imagePoints2, imageSize, cameraMatrix2,
		distCoeffs2, rvecs2, tvecs2, flags | CALIB_FIX_K4 | CALIB_FIX_K5);

	

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;



        Mat R, T, R1, P1, R2, P2, invR1, MR1, MR2;
		Rodrigues(rvecs1[0], MR1);
		Rodrigues(rvecs2[0], MR2);
		invert(MR1,invR1);
		R = MR2 * invR1;
		T = tvecs2[0] - MR2 * invR1 * tvecs1[0];

		stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

        Mat map11, map12, map21, map22;
		initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(alg == STEREO_HH ? StereoSGBM::MODE_HH : StereoSGBM::MODE_SGBM);

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        printf("press any key to continue...");
        fflush(stdout);
        waitKey();
        printf("\n");
    }

    if(disparity_filename)
        imwrite(disparity_filename, disp8);

    if(point_cloud_filename)
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename, xyz);
        printf("\n");
    }

    return 0;
}
