//#include <iostream>
//#include <vector>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
////#include <cv.h>
//#include<opencv2/imgproc/imgproc.hpp>
////#include <highgui.h>
//#include <algorithm>
//#include <cmath>
//#include <opencv2/highgui/highgui.hpp>
//using namespace std;
//using namespace cv;
//Mat image, res, image2, dst, dst2;
//typedef vector<Vec3b> row;
//typedef vector<row> mat;
//mat in, tmp, output;
//int M[800][800];//梯度幅值
//unsigned char N[800][800];//非极大值抑或的结果
//double g1, g2, g3, g4; //用于进行插值，得到亚像素点坐标值
//double dtmp1, dtmp2;//保存两个亚像素点插值得到的灰度数据  
//double dweight;//插值的权重
//double theta[800][800];
//double P[800][800];
//double Q[800][800];
//int w, h;
//
//void grey_guess()
//{
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++)
//	{
//		Vec3b tt = in[i][j];
//		//Y=0.3R+0.59G+0.11B
//		int tmp = (tt[0] * 11 + tt[1] * 59 + tt[2] * 30) / 100;
//		tt[0] = tmp;
//		tt[1] = tmp;
//		tt[2] = tmp;
//		output[i][j] = tt;
//	}
//
//	res.create(h, w, image.type());
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++)
//		res.at<Vec3b>(i, j) = output[i][j];
//	//namedWindow("windowmyself");
//	//imshow("windowmyself", res);
//
//	//图像高斯滤波一次
//	GaussianBlur(res, image2, Size(3, 3), 1, 1);
//	//imshow("windowopencv" ,image2 );
//
//}
//
//int xNum[8] = { 1, 1, 0, -1, -1, -1, 0, 1 };
//int yNum[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };
//void trace(int y, int x, int nThrLow)
//{
//	for (int i = 0; i<8; i++)
//	{
//		int yy = y + yNum[i];
//		int xx = x + xNum[i];
//		if (N[yy][xx] == 128 && M[yy][xx] >= nThrLow)
//		{
//			N[yy][xx] = 255;
//			trace(yy, xx, nThrLow);
//		}
//	}
//}
//
//typedef Vec3b mydef;
//void addgradient()
//{
//	//    for(int i=0;i<4;i++)
//	//    {
//	//        for(int j=0;j<4;j++)
//	//            cout<<image2.at<Vec3b>(i,j)[0]<<","<<image2.at<Vec3b>(i,j)[1]<<","<<image2.at<Vec3b>(i,j)[2]<<"   ";
//	//        cout<<endl;
//	//    }
//	//    waitKey();
//	//将高斯滤波的点放入图像2中
//	//其中P Q 分别代表X方向Y方向的偏导数
//	//M代表梯度的幅度值
//	//theta为梯度方向
//
//	//计算偏导数
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++){
//		P[i][j] = (image2.at<mydef>(i, MIN(j + 1, w - 1))[0] -
//			image2.at<mydef>(i, j)[0] +
//			image2.at<mydef>(MIN(i + 1, h - 1), MIN(j + 1, w - 1))[0] -
//			image2.at<mydef>(MIN(i + 1, h - 1), j)[0]
//			);
//		P[i][j] *= 0.5;
//		Q[i][j] = (image2.at<mydef>(i, j)[0] -
//			image2.at<mydef>(MIN(i + 1, h - 1), j)[0] +
//			image2.at<mydef>(i, MIN(j + 1, w - 1))[0] -
//			image2.at<mydef>(MIN(i + 1, h - 1), MIN(j + 1, w - 1))[0]
//			);
//		Q[i][j] *= 0.5;
//	}
//	//计算梯度值和方向
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++){
//		M[i][j] = (int)(sqrt(P[i][j] * P[i][j] + Q[i][j] * Q[i][j]) + 0.5);
//		theta[i][j] = atan2(Q[i][j], P[i][j]);
//		//为什么要*57.3
//		if (theta[i][j]<0)
//			theta[i][j] += 360; //如果小于0 则＋360度换成正数
//	}
//
//	//下面处理非极大值抑或 结果放到N中
//	//g1 g2 g3 g4 代表插值时候亚像素点的坐标
//	//dtmp1 dtmp2 代表插入值的数据
//	//dweigh为权重
//	memset(N, 0, sizeof(N));
//	for (int i = 1; i<w; i++)
//	for (int j = 1; j<h; j++)
//	{
//		if (M[j][i] == 0)
//			N[j][i] = 0;//如果当前梯度幅值为0，则不是局部最大，对该点的赋值为0
//		else
//		{
//			////////首先判断属于那种情况，然后根据情况插值///////
//			////////////////////第一种情况///////////////////////
//			/////////       g1  g2                  /////////////
//			/////////           C                   /////////////
//			/////////           g3  g4              /////////////
//			/////////////////////////////////////////////////////
//			if (((theta[j][i] >= 90) && (theta[j][i]<135)) ||
//				((theta[j][i] >= 270) && (theta[j][i]<315))){
//				g1 = M[j - 1][i - 1];
//				g2 = M[j - 1][i];
//				g3 = M[j + 1][i];
//				g4 = M[j + 1][i + 1];
//				dweight = fabs(P[j][i]) / fabs(Q[j][i]);//反正切
//				dtmp1 = g1*dweight + g2*(1 - dweight);
//				dtmp2 = g4*dweight + g3*(1 - dweight);
//			}
//			////////////////////第二种情况///////////////////////
//			/////////       g1                      /////////////
//			/////////       g2  C   g3              /////////////
//			/////////               g4              /////////////
//			/////////////////////////////////////////////////////
//			else if (((theta[j][i] >= 135) && (theta[j][i]<180)) ||
//				((theta[j][i] >= 315) && (theta[j][i]<360)))
//			{
//				g1 = M[j - 1][i - 1];
//				g2 = M[j][i - 1];
//				g3 = M[j][i + 1];
//				g4 = M[j + 1][i + 1];
//				dweight = fabs(Q[j][i]) / fabs(P[j][i]);
//				dtmp1 = g2*dweight + g1*(1 - dweight);
//				dtmp2 = g4*dweight + g3*(1 - dweight);
//			}
//			////////////////////第三种情况///////////////////////
//			/////////           g1  g2              /////////////
//			/////////           C                   /////////////
//			/////////       g4  g3                  /////////////
//			/////////////////////////////////////////////////////
//			else if (((theta[j][i] >= 45) && (theta[j][i]<90)) ||
//				((theta[j][i] >= 225) && (theta[j][i]<270)))
//			{
//				g1 = M[j - 1][i];
//				g2 = M[j - 1][i + 1];
//				g3 = M[j + 1][i];
//				g4 = M[j + 1][i - 1];
//				dweight = fabs(P[j][i]) / fabs(Q[j][i]);
//				dtmp1 = g2*dweight + g1*(1 - dweight);
//				dtmp2 = g3*dweight + g4*(1 - dweight);
//			}
//			////////////////////第四种情况///////////////////////
//			/////////               g1              /////////////
//			/////////       g4  C   g2              /////////////
//			/////////       g3                      /////////////
//			/////////////////////////////////////////////////////
//			else if (((theta[j][i] >= 0) && (theta[j][i]<45)) ||
//				((theta[j][i] >= 180) && (theta[j][i]<225)))
//			{
//				g1 = M[j - 1][i + 1];
//				g2 = M[j][i + 1];
//				g3 = M[j + 1][i - 1];
//				g4 = M[j][i - 1];
//				dweight = fabs(Q[j][i]) / fabs(P[j][i]);
//				dtmp1 = g1*dweight + g2*(1 - dweight);
//				dtmp2 = g3*dweight + g4*(1 - dweight);
//			}
//		}
//		//////////进行局部最大值判断，并写入检测结果////////////////
//		if ((M[j][i] >= dtmp1) && (M[j][i] >= dtmp2))
//			N[j][i] = 128;
//		else
//			N[j][i] = 0;
//	}
//	int nhist[1024]; memset(nhist, 0, sizeof(nhist));//统计各个梯度值
//	int nedgenum;//可能的边界数
//	int nmaxmag = 0;//最大梯度数
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++)
//	{
//		if (N[i][j] == 128)
//			nhist[M[i][j]]++;
//	}
//	//获取最大梯度幅值以及潜在边缘点个数
//	nedgenum = nhist[0];
//	nmaxmag = 0;
//	for (int i = 0; i<1024; i++)
//	{
//		if (nhist[i] != 0)//梯度为0的点是不可能为边界点的
//		{
//			nmaxmag = i;
//		}
//		nedgenum += nhist[i];//统计可能边界个数，经过non-maximum suppression后有多少像素
//	}
//	/**下面计算阈值
//	这段代码的意思是，按照灰度值从低到高的顺序，选取前79%个灰度值中的最大的灰度值为高阈值，低阈值大约为高阈值的一半。这是根据经验数据的来的，至于更好地参数选取方法，作者后面会另文研究。*/
//	int nhighcount;//通过79%灰度值算出来的的高阈值
//	double  dRatHigh = 0.79;//0.79
//	double  dThrHigh;//通过算出高阈值下实际的最高的灰度值
//	double  dThrLow;
//	double  dRatLow = 0.5;//0.5
//	nhighcount = (int)(dRatHigh * nedgenum + 0.5);
//	int j = 1;
//	nedgenum = nhist[1];
//	while ((j <= (nmaxmag - 1)) && (nedgenum < nhighcount))
//	{
//		j++;
//		nedgenum += nhist[j];
//	}
//	dThrHigh = j;                                   //高阈值
//	dThrLow = (int)((dThrHigh)* dRatLow + 0.5);    //低阈值
//
//	// 以上代码在非极大值抑制产生的二值灰度矩阵的潜在点中按照高阈值寻找边缘，并以所找到的点为中心寻找邻域内满足低阈值的点，从而形成一个闭合的轮廓。
//	cout << "高阈值:" << dThrHigh << " 低阈值:" << dThrLow << endl;
//	cout << "tacebefore" << endl;
//	for (int i = 0; i<h; i++)
//	for (j = 0; j<w; j++)
//	{
//		//如果是非极大值抑或后的边缘图像灰度值为0，这样一个结果可能会是边缘
//		//并且如果还是大于高阈值的情况下 我们需要进行边缘搜索
//		//在通过低阈值的搜索，找出闭合图像
//		if ((N[i][j] == 128) && (M[i][j] >= dThrHigh))
//		{
//			N[i][j] = 255;
//			trace(i, j, dThrLow);
//		}
//	}
//	cout << "traceend" << endl;
//	for (int i = 0; i<h; i++)
//	for (j = 0; j<w; j++)
//	{
//		if (N[i][j] != 255)
//		{
//			N[i][j] = 0;
//		}
//	}
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++){
//		int tttt = N[i][j];
//		res.at<Vec3b>(i, j)[0] = tttt;
//		res.at<Vec3b>(i, j)[1] = tttt;
//		res.at<Vec3b>(i, j)[2] = tttt;
//
//	}
//	//namedWindow("windowmyself");
//	imshow("windowmyselfcany", res);
//	//    waitKey();
//}
//int main()
//{
//	image = imread("1.jpg");
//	h = image.rows, w = image.cols;
//
//	cout << h << " " << w << endl;
//	in.resize(h);
//	output.resize(h);
//	for (int i = 0; i<h; i++){
//		in[i].resize(w);
//		output[i].resize(w);
//		for (int j = 0; j<w; j++){
//			in[i][j] = image.at<Vec3b>(i, j);
//		}
//	}
//	//处理灰度＋高斯
//	grey_guess();
//	addgradient();
//	Mat image3 = image2.clone();
//	Canny(image3, image3, 100, 33, 3);
//	dst.create(res.size(), res.type());
//	dst = Scalar::all(0);
//	dst2.create(image.size(), image.type());
//	dst2 = Scalar::all(0);
//	image.copyTo(dst, res);
//	image.copyTo(dst2, image3);
//	imshow("canny_opencv", image3);
//	imshow("show_myself", dst);
//	imshow("show_opencv", dst2);
//	imwrite("/Users/luyuncheng/Pictures/com.tencent.ScreenCapture/test10_myself.png", res);
//	imwrite("/Users/luyuncheng/Pictures/com.tencent.ScreenCapture/test10_opencv.png", image3);
//	waitKey();
//	return 0;
//}