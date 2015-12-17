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
//int M[800][800];//�ݶȷ�ֵ
//unsigned char N[800][800];//�Ǽ���ֵ�ֻ�Ľ��
//double g1, g2, g3, g4; //���ڽ��в�ֵ���õ������ص�����ֵ
//double dtmp1, dtmp2;//�������������ص��ֵ�õ��ĻҶ�����  
//double dweight;//��ֵ��Ȩ��
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
//	//ͼ���˹�˲�һ��
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
//	//����˹�˲��ĵ����ͼ��2��
//	//����P Q �ֱ����X����Y�����ƫ����
//	//M�����ݶȵķ���ֵ
//	//thetaΪ�ݶȷ���
//
//	//����ƫ����
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
//	//�����ݶ�ֵ�ͷ���
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++){
//		M[i][j] = (int)(sqrt(P[i][j] * P[i][j] + Q[i][j] * Q[i][j]) + 0.5);
//		theta[i][j] = atan2(Q[i][j], P[i][j]);
//		//ΪʲôҪ*57.3
//		if (theta[i][j]<0)
//			theta[i][j] += 360; //���С��0 ��360�Ȼ�������
//	}
//
//	//���洦��Ǽ���ֵ�ֻ� ����ŵ�N��
//	//g1 g2 g3 g4 �����ֵʱ�������ص������
//	//dtmp1 dtmp2 �������ֵ������
//	//dweighΪȨ��
//	memset(N, 0, sizeof(N));
//	for (int i = 1; i<w; i++)
//	for (int j = 1; j<h; j++)
//	{
//		if (M[j][i] == 0)
//			N[j][i] = 0;//�����ǰ�ݶȷ�ֵΪ0�����Ǿֲ���󣬶Ըõ�ĸ�ֵΪ0
//		else
//		{
//			////////�����ж��������������Ȼ����������ֵ///////
//			////////////////////��һ�����///////////////////////
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
//				dweight = fabs(P[j][i]) / fabs(Q[j][i]);//������
//				dtmp1 = g1*dweight + g2*(1 - dweight);
//				dtmp2 = g4*dweight + g3*(1 - dweight);
//			}
//			////////////////////�ڶ������///////////////////////
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
//			////////////////////���������///////////////////////
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
//			////////////////////���������///////////////////////
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
//		//////////���оֲ����ֵ�жϣ���д������////////////////
//		if ((M[j][i] >= dtmp1) && (M[j][i] >= dtmp2))
//			N[j][i] = 128;
//		else
//			N[j][i] = 0;
//	}
//	int nhist[1024]; memset(nhist, 0, sizeof(nhist));//ͳ�Ƹ����ݶ�ֵ
//	int nedgenum;//���ܵı߽���
//	int nmaxmag = 0;//����ݶ���
//	for (int i = 0; i<h; i++)
//	for (int j = 0; j<w; j++)
//	{
//		if (N[i][j] == 128)
//			nhist[M[i][j]]++;
//	}
//	//��ȡ����ݶȷ�ֵ�Լ�Ǳ�ڱ�Ե�����
//	nedgenum = nhist[0];
//	nmaxmag = 0;
//	for (int i = 0; i<1024; i++)
//	{
//		if (nhist[i] != 0)//�ݶ�Ϊ0�ĵ��ǲ�����Ϊ�߽���
//		{
//			nmaxmag = i;
//		}
//		nedgenum += nhist[i];//ͳ�ƿ��ܱ߽����������non-maximum suppression���ж�������
//	}
//	/**���������ֵ
//	��δ������˼�ǣ����ջҶ�ֵ�ӵ͵��ߵ�˳��ѡȡǰ79%���Ҷ�ֵ�е����ĻҶ�ֵΪ����ֵ������ֵ��ԼΪ����ֵ��һ�롣���Ǹ��ݾ������ݵ����ģ����ڸ��õز���ѡȡ���������ߺ���������о���*/
//	int nhighcount;//ͨ��79%�Ҷ�ֵ������ĵĸ���ֵ
//	double  dRatHigh = 0.79;//0.79
//	double  dThrHigh;//ͨ���������ֵ��ʵ�ʵ���ߵĻҶ�ֵ
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
//	dThrHigh = j;                                   //����ֵ
//	dThrLow = (int)((dThrHigh)* dRatLow + 0.5);    //����ֵ
//
//	// ���ϴ����ڷǼ���ֵ���Ʋ����Ķ�ֵ�ҶȾ����Ǳ�ڵ��а��ո���ֵѰ�ұ�Ե���������ҵ��ĵ�Ϊ����Ѱ���������������ֵ�ĵ㣬�Ӷ��γ�һ���պϵ�������
//	cout << "����ֵ:" << dThrHigh << " ����ֵ:" << dThrLow << endl;
//	cout << "tacebefore" << endl;
//	for (int i = 0; i<h; i++)
//	for (j = 0; j<w; j++)
//	{
//		//����ǷǼ���ֵ�ֻ��ı�Եͼ��Ҷ�ֵΪ0������һ��������ܻ��Ǳ�Ե
//		//����������Ǵ��ڸ���ֵ������� ������Ҫ���б�Ե����
//		//��ͨ������ֵ���������ҳ��պ�ͼ��
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
//	//����Ҷȣ���˹
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