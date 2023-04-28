#pragma once

#include<iostream>
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

#include "../serial/general.h"

using namespace std;
using namespace cv;

#define DEBUG1                         //画顶点
#define DEBUG20                       //画斜矩形
#define DEBUG33                       //画三角形
#define COLOR 0//0红色，1蓝色,2黄色
#define ISMAP  //1是图片，2是视频


// 三角形结构体，包括三条直线的倾角，三个点的坐标
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

	// 计算三角形面积的函数
	double getArea() {
		// 用triangle_points中的三个点计算三角形面积
		double a = edge_len1;
		double b = edge_len2;
		double c = edge_len3;
		double s = (a + b + c) / 2;
		double area = sqrt(s * (s - a) * (s - b) * (s - c));
		return area;
	}

	void init() {
		// 提取三个点
		pt1 = triangle_points[0];
		pt2 = triangle_points[1];
		pt3 = triangle_points[2];

		// 计算三个边的长度
		edge_len1 = sqrt(pow(pt2.x - pt3.x, 2) + pow(pt2.y - pt3.y, 2));
		edge_len2 = sqrt(pow(pt1.x - pt3.x, 2) + pow(pt1.y - pt3.y, 2));
		edge_len3 = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));

		// 计算三个角度的值
		angle1 = acos((edge_len2 * edge_len2 + edge_len3 * edge_len3 - edge_len1 * edge_len1) / (2 * edge_len2 * edge_len3)) * 180 / CV_PI;
		angle2 = acos((edge_len1 * edge_len1 + edge_len3 * edge_len3 - edge_len2 * edge_len2) / (2 * edge_len1 * edge_len3)) * 180 / CV_PI;
		angle3 = acos((edge_len1 * edge_len1 + edge_len2 * edge_len2 - edge_len3 * edge_len3) / (2 * edge_len1 * edge_len2)) * 180 / CV_PI;
	}
};


double transf(double a);

//double类型最大值
const double INF = numeric_limits<double>::infinity();

Mat precessing(Mat image);

void find_min_diff_indices(double arr[], int n, int& ind1, int& ind2, int& ind3);

void find_min_diff_angles(double arr[], int n, int& ind1, int& ind2, int& ind3);

double findMinangles(vector<double>& angles);

vector<Point2f> findApexs(vector<RotatedRect> minRect, vector<vector<Point>> contours, vector<Triangle> triangles, vector<int> index);

vector<Triangle> detectTriangles(const vector<vector<Point>>& contours, Mat& cap);

int find_four_apex(vector<vector<Point>> contours, vector<Triangle> triangle, vector<int> findMinindex, Mat cap);

vector<int> findMinareas(vector<double> areas);

vector<int> handleLight(vector<vector<Point>> contours, vector<RotatedRect> minRect, vector<Triangle> triangle, Mat cap,int capcols,int caprows,VisionData& visiondata);

vector<Point2f> handleMat(Mat src, Mat image, int capcols, int caprows);

void rotationMatrixToEulerAngles(Mat& R, double& roll, double& pitch, double& yaw);

void solveXYZ(std::vector<cv::Point2f> vertices, cv::Mat image,VisionData& visiondata);

cv::RotatedRect findMinBoundingRect(cv::RotatedRect rect1, cv::RotatedRect rect2, cv::RotatedRect rect3);

cv::RotatedRect findEnclosingRotatedRect(const std::vector<cv::Rect>& inputRectangles);

cv::Mat createMask(const cv::Mat& inputImage, const cv::RotatedRect& rotatedRect);

cv::Mat blackOutRectangles(const cv::Mat& inputImage, const std::vector<cv::RotatedRect>& rotatedRectangles);

void all(Mat image, int capcols, int caprows,VisionData& visiondata);
