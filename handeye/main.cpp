#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";




const char* liveCaptureHelp =
"When the live video from camera is used as input, the following hot-keys may be used:\n"
"  <ESC>, 'q' - quit the program\n"
"  'g' - start capturing images\n"
"  'u' - switch undistortion on/off\n";

static void help()
{
	printf("This is a camera calibration sample.\n"
		"Usage: calibration\n"
		"     -w <board_width>         # the number of inner corners per one of board dimension\n"
		"     -h <board_height>        # the number of inner corners per another board dimension\n"
		"     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
		"     [-n <number_of_frames>]  # the number of frames to use for calibration\n"
		"                              # (if not specified, it will be set to the number\n"
		"                              #  of board views actually available)\n"
		"     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
		"                              # (used only for video capturing)\n"
		"     [-s <squareSize>]       # square size in some user-defined units (1 by default)\n"
		"     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
		"     [-op]                    # write detected feature points\n"
		"     [-oe]                    # write extrinsic parameters\n"
		"     [-zt]                    # assume zero tangential distortion\n"
		"     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
		"     [-p]                     # fix the principal point at the center\n"
		"     [-v]                     # flip the captured images around the horizontal axis\n"
		"     [-V]                     # use a video file, and not an image list, uses\n"
		"                              # [input_data] string for the video file name\n"
		"     [-su]                    # show undistorted images after calibration\n"
		"     [input_data]             # input data, one of the following:\n"
		"                              #  - text file with a list of the images of the board\n"
		"                              #    the text file can be generated with imagelist_creator\n"
		"                              #  - name of video file with a video of the board\n"
		"                              # if input_data not specified, a live view from the camera is used\n"
		"\n");
	printf("\n%s", usage);
	printf("\n%s", liveCaptureHelp);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float(j*squareSize),
			float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
			float(i*squareSize), 0));
		break;

	default:
		CV_Error(Error::StsBadArg, "Unknown pattern type\n");
	}
}

static bool runCalibration(vector<vector<Point2f> > imagePoints,
	Size imageSize, Size boardSize, Pattern patternType,
	float squareSize, float aspectRatio,
	int flags, Mat& cameraMatrix, Mat& distCoeffs,
	vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs,
	double& totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}


static void saveCameraParams(const string& filename,
	Size imageSize, Size boardSize,
	float squareSize, float aspectRatio, int flags,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs,
	const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf)-1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)
	{
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
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


static bool runAndSave(const string& outputFilename,
	const vector<vector<Point2f> >& imagePoints,
	Size imageSize, Size boardSize, Pattern patternType, float squareSize,
	float aspectRatio, int flags, Mat& cameraMatrix,
	Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
		aspectRatio, flags, cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if (ok)
		saveCameraParams(outputFilename, imageSize,
		boardSize, squareSize, aspectRatio,
		flags, cameraMatrix, distCoeffs,
		writeExtrinsics ? rvecs : vector<Mat>(),
		writeExtrinsics ? tvecs : vector<Mat>(),
		writeExtrinsics ? reprojErrs : vector<float>(),
		writePoints ? imagePoints : vector<vector<Point2f> >(),
		totalAvgErr);
	return ok;
}


//int main(int argc, char** argv)
//{
//	Size boardSize, imageSize;
//	float squareSize = 1.f, aspectRatio = 1.f;
//	Mat cameraMatrix, distCoeffs;
//	const char* outputFilename = "out_camera_data.yml";
//	const char* inputFilename = 0;
//
//	int i, nframes = 10;
//	bool writeExtrinsics = false, writePoints = false;
//	bool undistortImage = false;
//	int flags = 0;
//	VideoCapture capture;
//	bool flipVertical = false;
//	bool showUndistorted = false;
//	bool videofile = false;
//	int delay = 1000;
//	clock_t prevTimestamp = 0;
//	int mode = DETECTION;
//	int cameraId = 0;
//	vector<vector<Point2f> > imagePoints;
//	vector<string> imageList;
//	Pattern pattern = CHESSBOARD;
//
//	if (argc < 2)
//	{
//		help();
//		return 0;
//	}
//
//	for (i = 1; i < argc; i++)
//	{
//		const char* s = argv[i];
//		if (strcmp(s, "-w") == 0)
//		{
//			if (sscanf(argv[++i], "%u", &boardSize.width) != 1 || boardSize.width <= 0)
//				return fprintf(stderr, "Invalid board width\n"), -1;
//		}
//		else if (strcmp(s, "-h") == 0)
//		{
//			if (sscanf(argv[++i], "%u", &boardSize.height) != 1 || boardSize.height <= 0)
//				return fprintf(stderr, "Invalid board height\n"), -1;
//		}
//		else if (strcmp(s, "-pt") == 0)
//		{
//			i++;
//			if (!strcmp(argv[i], "circles"))
//				pattern = CIRCLES_GRID;
//			else if (!strcmp(argv[i], "acircles"))
//				pattern = ASYMMETRIC_CIRCLES_GRID;
//			else if (!strcmp(argv[i], "chessboard"))
//				pattern = CHESSBOARD;
//			else
//				return fprintf(stderr, "Invalid pattern type: must be chessboard or circles\n"), -1;
//		}
//		else if (strcmp(s, "-s") == 0)
//		{
//			if (sscanf(argv[++i], "%f", &squareSize) != 1 || squareSize <= 0)
//				return fprintf(stderr, "Invalid board square width\n"), -1;
//		}
//		else if (strcmp(s, "-n") == 0)
//		{
//			if (sscanf(argv[++i], "%u", &nframes) != 1 || nframes <= 3)
//				return printf("Invalid number of images\n"), -1;
//		}
//		else if (strcmp(s, "-a") == 0)
//		{
//			if (sscanf(argv[++i], "%f", &aspectRatio) != 1 || aspectRatio <= 0)
//				return printf("Invalid aspect ratio\n"), -1;
//			flags |= CALIB_FIX_ASPECT_RATIO;
//		}
//		else if (strcmp(s, "-d") == 0)
//		{
//			if (sscanf(argv[++i], "%u", &delay) != 1 || delay <= 0)
//				return printf("Invalid delay\n"), -1;
//		}
//		else if (strcmp(s, "-op") == 0)
//		{
//			writePoints = true;
//		}
//		else if (strcmp(s, "-oe") == 0)
//		{
//			writeExtrinsics = true;
//		}
//		else if (strcmp(s, "-zt") == 0)
//		{
//			flags |= CALIB_ZERO_TANGENT_DIST;
//		}
//		else if (strcmp(s, "-p") == 0)
//		{
//			flags |= CALIB_FIX_PRINCIPAL_POINT;
//		}
//		else if (strcmp(s, "-v") == 0)
//		{
//			flipVertical = true;
//		}
//		else if (strcmp(s, "-V") == 0)
//		{
//			videofile = true;
//		}
//		else if (strcmp(s, "-o") == 0)
//		{
//			outputFilename = argv[++i];
//		}
//		else if (strcmp(s, "-su") == 0)
//		{
//			showUndistorted = true;
//		}
//		else if (s[0] != '-')
//		{
//			if (isdigit(s[0]))
//				sscanf(s, "%d", &cameraId);
//			else
//				inputFilename = s;
//		}
//		else
//			return fprintf(stderr, "Unknown option %s", s), -1;
//	}
//
//	if (inputFilename)
//	{
//		if (!videofile && readStringList(inputFilename, imageList))
//			mode = CAPTURING;
//		else
//			capture.open(inputFilename);
//	}
//	else
//		capture.open(cameraId);
//
//	if (!capture.isOpened() && imageList.empty())
//		return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraId), -2;
//
//	if (!imageList.empty())
//		nframes = (int)imageList.size();
//
//	if (capture.isOpened())
//		printf("%s", liveCaptureHelp);
//
//	namedWindow("Image View", 1);
//
//	for (i = 0;; i++)
//	{
//		Mat view, viewGray;
//		bool blink = false;
//
//		if (capture.isOpened())
//		{
//			Mat view0;
//			capture >> view0;
//			view0.copyTo(view);
//		}
//		else if (i < (int)imageList.size())
//			view = imread(imageList[i], 1);
//
//		if (view.empty())
//		{
//			if (imagePoints.size() > 0)
//				runAndSave(outputFilename, imagePoints, imageSize,
//				boardSize, pattern, squareSize, aspectRatio,
//				flags, cameraMatrix, distCoeffs,
//				writeExtrinsics, writePoints);
//			break;
//		}
//
//		imageSize = view.size();
//
//		if (flipVertical)
//			flip(view, view, 0);
//
//		vector<Point2f> pointbuf;
//		cvtColor(view, viewGray, COLOR_BGR2GRAY);
//
//		bool found;
//		switch (pattern)
//		{
//		case CHESSBOARD:
//			found = findChessboardCorners(view, boardSize, pointbuf,
//				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
//			break;
//		case CIRCLES_GRID:
//			found = findCirclesGrid(view, boardSize, pointbuf);
//			break;
//		case ASYMMETRIC_CIRCLES_GRID:
//			found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
//			break;
//		default:
//			return fprintf(stderr, "Unknown pattern type\n"), -1;
//		}
//
//		// improve the found corners' coordinate accuracy
//		if (pattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(11, 11),
//			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
//
//		if (mode == CAPTURING && found &&
//			(!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC))
//		{
//			imagePoints.push_back(pointbuf);
//			prevTimestamp = clock();
//			blink = capture.isOpened();
//		}
//
//		if (found)
//			drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
//
//		string msg = mode == CAPTURING ? "100/100" :
//			mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
//		int baseLine = 0;
//		Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
//		Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);
//
//		if (mode == CAPTURING)
//		{
//			if (undistortImage)
//				msg = format("%d/%d Undist", (int)imagePoints.size(), nframes);
//			else
//				msg = format("%d/%d", (int)imagePoints.size(), nframes);
//		}
//
//		putText(view, msg, textOrigin, 1, 1,
//			mode != CALIBRATED ? Scalar(0, 0, 255) : Scalar(0, 255, 0));
//
//		if (blink)
//			bitwise_not(view, view);
//
//		if (mode == CALIBRATED && undistortImage)
//		{
//			Mat temp = view.clone();
//			undistort(temp, view, cameraMatrix, distCoeffs);
//		}
//
//		imshow("Image View", view);
//		int key = 0xff & waitKey(capture.isOpened() ? 50 : 500);
//
//		if ((key & 255) == 27)
//			break;
//
//		if (key == 'u' && mode == CALIBRATED)
//			undistortImage = !undistortImage;
//
//		if (capture.isOpened() && key == 'g')
//		{
//			mode = CAPTURING;
//			imagePoints.clear();
//		}
//
//		if (mode == CAPTURING && imagePoints.size() >= (unsigned)nframes)
//		{
//			if (runAndSave(outputFilename, imagePoints, imageSize,
//				boardSize, pattern, squareSize, aspectRatio,
//				flags, cameraMatrix, distCoeffs,
//				writeExtrinsics, writePoints))
//				mode = CALIBRATED;
//			else
//				mode = DETECTION;
//			if (!capture.isOpened())
//				break;
//		}
//	}
//
//	if (!capture.isOpened() && showUndistorted)
//	{
//		Mat view, rview, map1, map2;
//		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
//			getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
//			imageSize, CV_16SC2, map1, map2);
//
//		for (i = 0; i < (int)imageList.size(); i++)
//		{
//			view = imread(imageList[i], 1);
//			if (view.empty())
//				continue;
//			//undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
//			remap(view, rview, map1, map2, INTER_LINEAR);
//			imshow("Image View", rview);
//			int c = 0xff & waitKey();
//			if ((c & 255) == 27 || c == 'q' || c == 'Q')
//				break;
//		}
//	}
//
//	return 0;
//}
//int main(int argc, char **argv)
//{
//
//	//Mat camera_matrix;
//	//Mat distortion_coefficients;
//	//Mat extrinsic_parameters;
//	//Mat extrinsic;
//
//
//
//	//FileStorage fs("camera.yml", FileStorage::READ);
//
//
//	//if (!fs.isOpened())
//	//{
//	//	
//	//	return 1;
//	//}
//	//fs["camera_matrix"] >> camera_matrix;
//	//fs["distortion_coefficients"] >> distortion_coefficients;
//	//fs["extrinsic_parameters"] >> extrinsic_parameters;
//	//extrinsic_parameters.row(0).copyTo(extrinsic);
//	//getchar();
//
//}
//int main(int argc, char **argv)
//{
//	Mat view, viewGray;
//	float squareSize = 1.f, aspectRatio = 1.f;
//	Mat cameraMatrix, distCoeffs;
//	vector<vector<Point2f> > imagePoints;
//	Size imageSize, boardSize(6,9);
//	namedWindow("Image View", 1);
//	view = imread("left01.jpg", 1);
//
//	imageSize = view.size();
//	vector<Point2f> pointbuf;
//	cvtColor(view, viewGray, COLOR_BGR2GRAY);
//	bool found;
//	found = findChessboardCorners(view, boardSize, pointbuf,CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
//	cornerSubPix(viewGray, pointbuf, Size(11, 11),Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
//
//	imagePoints.push_back(pointbuf);
//
//
//	if (found)
//		drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
//
//	imshow("Image View", view);
//	waitKey();
//
//
//	cameraMatrix = Mat::eye(3, 3, CV_64F);
//	distCoeffs = Mat::zeros(8, 1, CV_64F);
//
//
//	vector<Mat> rvecs, tvecs;
//	vector<float> reprojErrs;
//	double totalAvgErr = 0;
//	vector<vector<Point3f> > objectPoints(1);
//
//	for (int i = 0; i < boardSize.height; i++)
//	for (int j = 0; j < boardSize.width; j++)
//		objectPoints[0].push_back(Point3f(float(j*squareSize),float(i*squareSize), 0));
//
//
//	objectPoints.resize(imagePoints.size(), objectPoints[0]);
//	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
//		distCoeffs, rvecs, tvecs, CALIB_FIX_K4 | CALIB_FIX_K5);
//
//	Mat tmp;
//	for (int i = 0; i < rvecs.size(); i++)
//		tmp = rvecs[i];
//	for (int i = 0; i < tvecs.size(); i++)
//		tmp = tvecs[i];
//
//	vector<float> perViewErrors;
//
//	vector<Point2f> imagePoints2;
//	int i, totalPoints = 0;
//	double totalErr = 0, err;
//	perViewErrors.resize(objectPoints.size());
//
//	for (i = 0; i < (int)objectPoints.size(); i++)
//	{
//		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
//			cameraMatrix, distCoeffs, imagePoints2);
//		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
//		int n = (int)objectPoints[i].size();
//		perViewErrors[i] = (float)std::sqrt(err*err / n);
//		totalErr += err*err;
//		totalPoints += n;
//	}
//
//
//
//	getchar();
//
//
//
//
//
//
//
//
//
//
//
//
//	//Mat temp = view.clone();
//	//undistort(temp, view, cameraMatrix, distCoeffs);
//
//
//}

//void GramSchmidt(Mat &src, Mat &dst)
//{
//	int i, j;
//	double param, param1, param2;
//
//	Mat VecSrc = Mat::zeros(1, src.cols, CV_64FC1);
//	Mat VecDst = Mat::zeros(1, dst.cols, CV_64FC1);
//	Mat VecTmp = Mat::zeros(1, dst.cols, CV_64FC1);
//
//	for (i = 0; i<dst.rows; i++)
//	{
//		src.row(i).copyTo(VecDst);
//		src.row(i).copyTo(VecSrc);
//
//		for (j = 0; j<i; j++)
//		{
//			
//			dst.row(j).copyTo(VecTmp);
//			param1 = DotProduct(VecSrc, VecTmp);
//			param2 = cvDotProduct(VecTmp, VecTmp);
//			param = param1 / param2;
//			cvAddWeighted(VecTmp, -param, VecDst, 1.0, 0, VecDst);
//		}
//		memcpy(dst->data.db + i*dst->cols, VecDst->data.db, dst->cols*sizeof(double));
//	}
//	cvReleaseMat(&VecSrc);
//	cvReleaseMat(&VecDst);
//	cvReleaseMat(&VecTmp);
//}

bool  norm_vec(Mat &src)
{
	double len = sqrt(src.at<double>(0, 0)*src.at<double>(0, 0) + src.at<double>(1, 0)*src.at<double>(1, 0) + src.at<double>(2, 0)*src.at<double>(2, 0));
	if (len == 0)
	{
		return false;
	}
	else
	{
		src /= len;
		return true;
	}
}
bool gram_schmidt(Mat &src)
{
	Mat a(3, 1, CV_64FC1);
	Mat b(3, 1, CV_64FC1);
	Mat c(3, 1, CV_64FC1);


	src.col(0).copyTo(a);
	src.col(1).copyTo(b);
	src.col(2).copyTo(c);



	b = b - ((a.dot(b)) / (a.dot(a)))*a;



	c = c - a.dot(c) / a.dot(a)*a - b.dot(c)/b.dot(b)*b;




	norm_vec(a);
	norm_vec(b);
	norm_vec(c);

	a.copyTo(src.col(0));
	b.copyTo(src.col(1));
	c.copyTo(src.col(2));

	return true;

}
bool cross_33(Mat &vec_a, Mat &vec_b, Mat &out)
{
	if (vec_a.rows ==3 &&  vec_b.rows == 3 && out.rows == 3)
	{
		out.at<double>(0, 0) = vec_a.at<double>(1, 0) * vec_b.at<double>(2, 0) - vec_a.at<double>(2, 0) * vec_b.at<double>(1, 0);
		out.at<double>(1, 0) = vec_a.at<double>(2, 0) * vec_b.at<double>(0, 0) - vec_a.at<double>(0, 0) * vec_b.at<double>(2, 0);
		out.at<double>(2, 0) = vec_a.at<double>(0, 0) * vec_b.at<double>(1, 0) - vec_a.at<double>(1, 0) * vec_b.at<double>(0, 0);
	}
	else
	{
		return false;
	}
	return true;
}
bool get_axis(Mat &matrix, Mat &axis)
{
	if (matrix.cols != matrix.rows)
	{
		return false;
	}
	else
	{
		Mat src = Mat::zeros(3, 3, CV_64F);
		matrix.copyTo(src);

		Mat U = Mat::zeros(3, 3, CV_64F);
		Mat V = Mat::zeros(3, 3, CV_64F);
		Mat W = Mat::zeros(3, 1, CV_64F);

		SVD thissvd;
		thissvd.compute(src, W, U, V, SVD::FULL_UV | SVD::MODIFY_A);

		src = U*V;

		double rx = src.at<double>(2, 1) - src.at<double>(1, 2);
		double ry = src.at<double>(0, 2) - src.at<double>(2, 0);
		double rz = src.at<double>(1, 0) - src.at<double>(0, 1);

		double s = std::sqrt((rx*rx + ry*ry + rz*rz)*0.25);
		double c = (src.at<double>(0, 0) + src.at<double>(1, 1) + src.at<double>(2, 2) - 1)*0.5;
		c = c > 1. ? 1. : c < -1. ? -1. : c;
		double theta = acos(c);

		if (s < 1e-5)
		{
			double t;

			if (c > 0)
				rx = ry = rz = 0;
			else
			{
				t = (src.at<double>(0, 0) + 1)*0.5;
				rx = std::sqrt(MAX(t, 0.));
				t = (src.at<double>(1, 1) + 1)*0.5;
				ry = std::sqrt(MAX(t, 0.))*(src.at<double>(0, 1) < 0 ? -1. : 1.);
				t = (src.at<double>(2, 2) + 1)*0.5;
				rz = std::sqrt(MAX(t, 0.))*(src.at<double>(0, 2) < 0 ? -1. : 1.);
				if (fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (src.at<double>(1, 2) > 0) != (ry*rz > 0))
					rz = -rz;
				theta = std::sqrt(rx*rx + ry*ry + rz*rz);
				rx /= theta;
				ry /= theta;
				rz /= theta;
			}

			
		}
		else
		{
			double vth = 1 / (2 * s);
			rx *= vth; ry *= vth; rz *= vth;
		}
		axis.at<double>(0, 0) = rx;
		axis.at<double>(1, 0) = ry;
		axis.at<double>(2, 0) = rz;
	}
	
	return true;



}
bool merge_col(Mat &col_1, Mat &col_2, Mat &col_3, Mat &out)
{
	if (col_1.rows == col_2.rows && col_2.rows == col_3.rows &&  col_3.rows == out.rows)
	{
		col_1.copyTo(out.col(0));
		col_2.copyTo(out.col(1));
		col_3.copyTo(out.col(2));
	}
	else
	{
		return false;
	}
	
	return true;
}

bool computeX(Mat &C1, Mat &C2, Mat &D1, Mat &D2, Mat &tc1, Mat &tc2, Mat &td1, Mat &td2, Mat &X, Mat &t)
{
	Mat Kc1 = Mat::zeros(3, 1, CV_64FC1); 
	Mat Kc2 = Mat::zeros(3, 1, CV_64FC1);
	Mat Kc1_Kc2 = Mat::zeros(3, 1, CV_64FC1);
	Mat Kd1 = Mat::zeros(3, 1, CV_64FC1);
	Mat Kd2 = Mat::zeros(3, 1, CV_64FC1);
	Mat Kd1_Kd2 = Mat::zeros(3, 1, CV_64FC1);
	Mat Kc = Mat::zeros(3, 3, CV_64FC1);
	Mat Kd = Mat::zeros(3, 3, CV_64FC1);
	Mat Kd_invert;

	Mat unity = Mat::eye(3, 3, CV_64FC1);
	Mat for_T_C1 = Mat::zeros(3, 3, CV_64FC1);
	Mat for_T_C2 = Mat::zeros(3, 3, CV_64FC1);
	Mat for_T_C12 = Mat::zeros(6, 3, CV_64FC1);

	Mat for_T_X1 = Mat::zeros(3, 1, CV_64FC1);
	Mat for_T_X2 = Mat::zeros(3, 1, CV_64FC1);
	Mat for_T_X12 = Mat::zeros(6, 1, CV_64FC1);

	get_axis(C1, Kc1);
	get_axis(C2, Kc2);
	get_axis(D1, Kd1);
	get_axis(D2, Kd2);


	cross_33(Kc1, Kc2, Kc1_Kc2);
	cross_33(Kd1, Kd2, Kd1_Kd2);
	
	merge_col(Kc1, Kc2, Kc1_Kc2, Kc);
	merge_col(Kd1, Kd2, Kd1_Kd2, Kd);

	invert(Kd, Kd_invert);
	X = Kc * Kd_invert;

	for_T_C1 = C1 - unity;
	for_T_C2 = C2 - unity;
	for (int i = 0; i < 3; i++)
	{
		for_T_C1.row(i).copyTo(for_T_C12.row(i));
		for_T_C2.row(i).copyTo(for_T_C12.row(3+i));
	}

	for_T_X1 = X*td1 - tc1;
	for_T_X2 = X*td2 - tc2;
	for (int i = 0; i < 3; i++)
	{
		for_T_X1.row(i).copyTo(for_T_X12.row(i));
		for_T_X2.row(i).copyTo(for_T_X12.row(3 + i));
	}

	solve(for_T_C12, for_T_X12, t, 1);

	return true;


}
int main(int argc, char **argv)
{
	
#ifdef SS
	double xishu[3][2] = {
		3, 1,
		2, 2,
		1,1
	};

	double jieguo[3][1] = {
		5,
		6,
		3
	};
	
	Mat xy = Mat::zeros(2, 1, CV_64FC1);
	Mat xs(3, 2, CV_64FC1, xishu);
	Mat jg(3, 1, CV_64FC1, jieguo);
	solve(xs, jg, xy,1);


	double A1[3][3] = {
		-0.989992, -0.141120, 0,
		-0.141120, -0.989992, 0,
		0, 0, 1
	};

	double B1[3][3] = {
		0.0191, 0.9982, -0.056,
		0.9837, -0.0228, -0.1776,
		-0.1789, -0.0517, -0.9825
	};
	double A2[3][3] = {
		0.070737, 0, 0.997495,
		0, 1, 0,
		-0.997495, 0, 0.070737
	};
	double B2[3][3] = {
		0.0201, 0.9991, 0.0366,
		0.9851, -0.0098, -0.2619,
		-0.2613, -0.0406, -0.9644
	};

	Mat extrinsic_parameters;

	FileStorage fs("camera.yml", FileStorage::READ);
	fs["extrinsic_parameters"] >> extrinsic_parameters;
	fs.release();
	Mat rc1(3, 1, CV_64FC1);
	Mat rc2(3, 1, CV_64FC1);
	Mat rd1(3, 1, CV_64FC1);
	Mat rd2(3, 1, CV_64FC1);

	Mat C1(3, 3, CV_64FC1);
	Mat C2(3, 3, CV_64FC1);
	Mat D1(3, 3, CV_64FC1);
	Mat D2(3, 3, CV_64FC1);

	Mat X(3, 3, CV_64FC1);


	Mat tc1(3, 1, CV_64FC1);
	Mat tc2(3, 1, CV_64FC1);
	Mat td1(3, 1, CV_64FC1);
	Mat td2(3, 1, CV_64FC1);

	Mat eValuesMat;
	Mat eVectorsMat;

	Mat t(3, 1, CV_64FC1);

	rc1.at<double>(0, 0) = extrinsic_parameters.at<double>(0, 0);
	rc1.at<double>(1, 0) = extrinsic_parameters.at<double>(0, 1);
	rc1.at<double>(2, 0) = extrinsic_parameters.at<double>(0, 2);
	tc1.at<double>(0, 0) = extrinsic_parameters.at<double>(0, 3);
	tc1.at<double>(1, 0) = extrinsic_parameters.at<double>(0, 4);
	tc1.at<double>(2, 0) = extrinsic_parameters.at<double>(0, 5);

	rc2.at<double>(0, 0) = extrinsic_parameters.at<double>(1, 0);
	rc2.at<double>(1, 0) = extrinsic_parameters.at<double>(1, 1);
	rc2.at<double>(2, 0) = extrinsic_parameters.at<double>(1, 2);
	tc2.at<double>(0, 0) = extrinsic_parameters.at<double>(1, 3);
	tc2.at<double>(1, 0) = extrinsic_parameters.at<double>(1, 4);
	tc2.at<double>(2, 0) = extrinsic_parameters.at<double>(1, 5);


	rd1.at<double>(0, 0) = extrinsic_parameters.at<double>(2, 0);
	rd1.at<double>(1, 0) = extrinsic_parameters.at<double>(2, 1);
	rd1.at<double>(2, 0) = extrinsic_parameters.at<double>(2, 2);
	td1.at<double>(0, 0) = extrinsic_parameters.at<double>(2, 3);
	td1.at<double>(1, 0) = extrinsic_parameters.at<double>(2, 4);
	td1.at<double>(2, 0) = extrinsic_parameters.at<double>(2, 5);

	rd2.at<double>(0, 0) = extrinsic_parameters.at<double>(3, 0);
	rd2.at<double>(1, 0) = extrinsic_parameters.at<double>(3, 1);
	rd2.at<double>(2, 0) = extrinsic_parameters.at<double>(3, 2);
	td2.at<double>(0, 0) = extrinsic_parameters.at<double>(3, 3);
	td2.at<double>(1, 0) = extrinsic_parameters.at<double>(3, 4);
	td2.at<double>(2, 0) = extrinsic_parameters.at<double>(3, 5);


	Rodrigues(rc1, C1);


	Rodrigues(rc2, C2);
	

	Rodrigues(rd1, D1);


	Rodrigues(rd2, D2);
	

	Mat m_A1(3, 3, CV_64FC1, A1);
	Mat m_B1(3, 3, CV_64FC1, A2);
	Mat m_A2(3, 3, CV_64FC1, B1);
	Mat m_B2(3, 3, CV_64FC1, B2);





	computeX(C1, C2, D1, D2, tc1, tc2, td1, td2, X, t);
	
	//computeX(m_A1, m_B1, m_A2, m_B2, tc1, tc2, td1, td2, X, t);

	norm_vec(X.col(0));
	norm_vec(X.col(1));
	norm_vec(X.col(2));

	
	double put = determinant(X);

	FileStorage fs1("camera1.yml", FileStorage::WRITE);
	fs1<<"X" << X;
	fs1.release();


	/*Mat myMat_rv;
	transpose(myMat, myMat_rv);
	Mat out = myMat * myMat_rv;
	Mat eValuesMat;
	Mat eVectorsMat;
	Mat pr_vec = Mat::zeros(1, 3, CV_64FC1);
	Rodrigues(myMat, pr_vec);
	Rodrigues(pr_vec, myMat);
	eigen(myMat, eValuesMat, eVectorsMat);*/

	//C1.convertTo(C2, CV_8UC1);
#endif
	int size[2] = { 2, 2};
	Mat aaa(2, size, CV_64FC3);

	double abc[3][3] = {
		1, 2, 3,
		-1, 0, -3,
		0, -2, 3
	};

	Mat m_abc(3, 3, CV_64FC1, abc);
	gram_schmidt(m_abc);


	CV_MAT_ELEM

	getchar();
}