#pragma once

#include<iostream>
#include<opencv.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <climits>
#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include<fstream>
#include <numeric>

using namespace std;
using namespace cv;

#define DEBUG1                         //������
#define DEBUG20                       //��б����
#define DEBUG33                       //��������
#define COLOR 0//0��ɫ��1��ɫ,2��ɫ
#define ISMAP  //1��ͼƬ��2����Ƶ


// �����νṹ�壬��������ֱ�ߵ���ǣ������������
struct Triangle {
	double angle1 = 0;
	double angle2 = 0;
	double angle3 = 0;
	double edge_len1 = 0;
	double edge_len2 = 0;
	double edge_len3 = 0;
	Point2f pt1;
	Point2f pt2;
	Point2f pt3;
	vector<Point2f> triangle_points;

	// ��������������ĺ���
	double getArea() {
		// ��triangle_points�е�������������������
		double a = edge_len1;
		double b = edge_len2;
		double c = edge_len3;
		double s = (a + b + c) / 2;
		double area = sqrt(s * (s - a) * (s - b) * (s - c));
		return area;
	}

	void init() {
		// ��ȡ������
		pt1 = triangle_points[0];
		pt2 = triangle_points[1];
		pt3 = triangle_points[2];

		// ���������ߵĳ���
		edge_len1 = sqrt(pow(pt2.x - pt3.x, 2) + pow(pt2.y - pt3.y, 2));
		edge_len2 = sqrt(pow(pt1.x - pt3.x, 2) + pow(pt1.y - pt3.y, 2));
		edge_len3 = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));

		// ���������Ƕȵ�ֵ
		angle1 = acos((edge_len2 * edge_len2 + edge_len3 * edge_len3 - edge_len1 * edge_len1) / (2 * edge_len2 * edge_len3)) * 180 / CV_PI;
		angle2 = acos((edge_len1 * edge_len1 + edge_len3 * edge_len3 - edge_len2 * edge_len2) / (2 * edge_len1 * edge_len3)) * 180 / CV_PI;
		angle3 = acos((edge_len1 * edge_len1 + edge_len2 * edge_len2 - edge_len3 * edge_len3) / (2 * edge_len1 * edge_len2)) * 180 / CV_PI;
	}
};


double transf(double a);

//double�������ֵ
const double INF = numeric_limits<double>::infinity();

Mat precessing(Mat image);

void find_min_diff_indices(double arr[], int n, int& ind1, int& ind2, int& ind3);

void find_min_diff_angles(double arr[], int n, int& ind1, int& ind2, int& ind3);

double findMinangles(vector<double>& angles);

vector<Point2f> findApexs(vector<RotatedRect> minRect, vector<vector<Point>> contours, vector<Triangle> triangles, vector<int> index);

vector<Triangle> detectTriangles(const vector<vector<Point>>& contours, Mat& cap);

int find_four_apex(vector<vector<Point>> contours, vector<Triangle> triangle, vector<int> findMinindex, Mat cap);

vector<int> findMinareas(vector<double> areas);

vector<int> handleLight(vector<vector<Point>> contours, vector<RotatedRect> minRect, vector<Triangle> triangle, Mat cap);

vector<Point2f> handleMat(Mat src, Mat image);

void rotationMatrixToEulerAngles(Mat& R, double& roll, double& pitch, double& yaw);

void solveXYZ(std::vector<cv::Point2f> vertices, cv::Mat image);

cv::RotatedRect findMinBoundingRect(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::RotatedRect rect3);

cv::RotatedRect findEnclosingRotatedRect(const std::vector<cv::Rect>& inputRectangles);

cv::Mat createMask(const cv::Mat& inputImage, const cv::RotatedRect& rotatedRect);

cv::Mat blackOutRectangles(const cv::Mat& inputImage, const std::vector<cv::RotatedRect>& rotatedRectangles);

void all(Mat image);

double drawMostSimilarContours(const std::vector<std::vector<cv::Point>>& contours, vector<int> index);
