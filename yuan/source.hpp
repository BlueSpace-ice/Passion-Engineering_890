#pragma once
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
#include <vector>
#include <algorithm>

#define DEBUG1                         //������
#define DEBUG2                       //��б����
#define DEBUG3                       //��������
#define COLOR 0//0��ɫ��1��ɫ,2��ɫ
#define CAMERA
using namespace std;
using namespace cv;

struct Triangle {
	double angle1 = 0;
	double angle2 = 0;
	double angle3 = 0;
	double edge_len1 = 0;
	double edge_len2 = 0;
	double edge_len3 = 0;
	vector<Point2f> triangle_points;
};

double transf(double a);

bool check_overlap(Mat& img, Triangle tri1, Triangle tri2);

//double�������ֵ
// const double INF = numeric_limits<double>::infinity();

Mat precessing(Mat image);

void find_min_diff_indices(double arr[], int n, int& ind1, int& ind2, int& ind3);

void find_min_diff_angles(double arr[], int n, int& ind1, int& ind2, int& ind3);

double findMinangles(vector<double>& angles);

vector<Point2f> findApexs(vector<RotatedRect> minRect, vector<vector<Point>> contours, vector<Triangle> triangles, vector<int> index);

vector<Triangle>Triangles(const vector<vector<Point>>& contours, Mat& cap);

int find_four_apex(vector<vector<Point>> contours, vector<Triangle> triangle, vector<int> findMinindex, Mat cap);

vector<int> findMinareas(vector<double> areas);

vector<int> handleLight(vector<vector<Point>> contours, vector<RotatedRect> minRect, vector<Triangle> triangle, Mat cap);

vector<Point2f> handleMat(Mat src, Mat image);

void rotationMatrixToEulerAngles(Mat& R, double& roll, double& pitch, double& yaw);

void solveXYZ(vector<Point2f> vertices,Mat image);

void all(Mat image);