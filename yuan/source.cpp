#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>
#include <cmath>
#include <limits>
#include<opencv2/opencv.hpp>
#define DEBUG

using namespace std;
using namespace cv;

//最终提供的数据
struct answer
{
	double dist_x, dist_y, dist_z, ag_x, ag_y, ag_z;
};
//double类型最大值
const double INF = numeric_limits<double>::infinity();

//图像预处理
Mat precessing(Mat image)
{
	// 转换为灰度图像
	Mat gray;
	inRange(image, Scalar(47, 118, 210), Scalar(255, 255, 255), gray);
	return gray;
}

//找四个顶点
vector<Point2f> findApexs(vector<RotatedRect> minRect, vector<vector<Point>> contours, vector<int> index)
{
	//中心坐标计算
	float x = 0, y = 0;
	for (int i = 0; i < 4; i++)
	{
		x += minRect[index[i]].center.x;
		y += minRect[index[i]].center.y;

	}
	x /= 4;
	y /= 4;//Point(x,y)是中心点
	cout << Point(x, y) << endl;

	//确定四个点
	vector<Point2f> bigRect_points;
	for (int i = 0; i < index.size(); i++)
	{
		int max = 0, max_i = 0;
		float true_x = 0, true_y = 0;
		for (int j = 0; j < contours[index[i]].size(); j++)
		{
			float anotherx = contours[index[i]][j].x;
			float anothery = contours[index[i]][j].y;
			float differ = pow(x - anotherx, 2) + pow(y - anothery, 2);
			if (differ > max)
			{
				max = differ;
				true_x = anotherx;
				true_y = anothery;
			}
		}
		bigRect_points.push_back(Point2f(true_x, true_y));
	}
	return bigRect_points;
}

//计算差异最小的三个数
void findMinDiff(double nums[], int n, int& idx1, int& idx2, int& idx3) {
	if (n < 3) {
		cout << endl << endl << endl;
		cerr << "Error: input array too short." << endl;
		return;
	}
	sort(nums, nums + n);
	double minDiff = INF;
	for (int i = 0; i < n - 1; ++i) {
		double curMinDiff = fabs(nums[i] - nums[i + 1]);
		if (curMinDiff < minDiff) {
			minDiff = curMinDiff;
			idx1 = i;
			idx2 = i + 1;
		}
	}
	double thirdNum = (nums[idx1] + nums[idx2]) / 2.0;
	minDiff = INF;
	idx3 = -1;
	for (int i = 0; i < n; ++i) {
		double curDiff = fabs(nums[i] - thirdNum);
		if (curDiff < minDiff && i != idx1 && i != idx2) {
			minDiff = curDiff;
			idx3 = i;
		}
	}

}

//一个转换作用，输入面积序列，传出最小的三个面积下标
vector<int> findMinareas(vector<double> areas)//areas已经排序了
{
	double nums[7] = { 0 };
	int min1 = min((int)areas.size(), 7);
	for (int i = 0; i < min1; i++)
		nums[i] = areas[i];
	int n = min1;
	int idx1, idx2, idx3;
	findMinDiff(nums, n, idx1, idx2, idx3);
	vector<int> ans;
	ans.push_back(idx1);
	ans.push_back(idx2);
	ans.push_back(idx3);
	return ans;
}

//筛选出四个直角的灯条
vector<int> handleLight(vector<RotatedRect> minRect)
{
	//干扰项排除(需要改进)    现在的方案是1.去除太小的矩形2.找矩形面积最相似的三个面积
	vector<double> areas;
	for (int i = 0; i < minRect.size(); i++)
		if (minRect[i].size.area() > 50)
			areas.push_back(minRect[i].size.area());
	//findMin是下标
	vector<int> findMinindex = findMinareas(areas);
	//第四个角(没什么好方法,先乱写)  现在的方案是之间找最小的面积的下一个面积
	findMinindex.push_back((*max_element(findMinindex.begin(), findMinindex.end())) + 1);
	return findMinindex;

}

//主函数，负责调用二值图后的全流程
vector<Point2f> handleMat(Mat src, Mat image)
{
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	// 找到所有轮廓
	findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	// 为每个轮廓找到一个斜矩形
	// 按照面积从大到小排序

	vector<RotatedRect> minRect(contours.size());
	for (int i = 0; i < contours.size(); i++)
		minRect[i] = minAreaRect(contours[i]);
	//将按照斜矩形的面积轮廓和斜矩形都排序，并保持轮廓和斜矩形变长数组的坐标一一对应
	vector<RotatedRect> handle_minRect(contours.size());
	vector<vector<Point>> handle_contours(contours.size());
	//保证斜矩形与轮廓的下标一样
	//自己写的选择排序(不想优化)
	for (int i = 0; i < contours.size(); i++) {
		int max = 0, max_i = 0;
		for (int j = 0; j < contours.size(); j++)
		{
			if (minRect[j].size.area() > max)
			{
				max = minRect[j].size.area();
				max_i = j;
			}
		}
		handle_minRect[i] = minRect[max_i];
		handle_contours[i] = contours[max_i];
		minRect[max_i] = RotatedRect(cv::Point2f(100, 100), cv::Size2f(0, 0), 0);
	}

	//判断至少有四个角
	vector<int> true_minRect_index;
	if (minRect.size() >= 4)
		true_minRect_index = handleLight(handle_minRect);
	else
		cout << "没有识别到四个角" << endl;

	//传参,找四个顶点
	vector<Point2f> true_vertexs = findApexs(handle_minRect, handle_contours, true_minRect_index);


#ifdef DEBUG
	// 定义颜色数组
	Scalar colors[] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(0, 255, 255) };
	// 创建输出图像
	Mat map = image.clone();
	for (int i = 0; i < true_vertexs.size(); i++)
		circle(map, true_vertexs[i], 5, colors[i], -1);
	cv::imshow("opencv", map);
	cv::waitKey(0);

	// 绘制旋转矩形
	for (size_t i = 0; i < true_minRect_index.size(); i++) {
		// 根据面积确定颜色
		Scalar color;
		color = colors[i];

		Point2f rect_points[4];
		handle_minRect[true_minRect_index[i]].points(rect_points);
		for (int j = 0; j < 4; j++) {
			line(map, rect_points[j], rect_points[(j + 1) % 4], color, 2, LINE_AA);
		}
	}

	// 显示输出图像
	namedWindow("Output Image", WINDOW_NORMAL);
	cv::imshow("Output Image", map);
	imwrite("C:\\Users\\Bluespace1\\OneDrive\\桌面\\草稿.jpg", map);
	cv::waitKey(0);
#endif // DEBUG
	//返回值处理
	return true_vertexs;
}

//pnp算法
void solveXYZ(vector<Point2f> vertices)
{
	double half_x;
	double half_y;
	double width_target;
	double height_target;

	double cam1[3][3] = {                                  //内参矩阵
		1689.2, 0, 624.7565,
		0, 1688.1, 496.4914,
		0, 0, 1 };

	double distCoeff1[5] = { 0.0163, -0.3351, 0, 0, 0 };   //畸变参数

	Mat cam_matrix = Mat(3, 3, CV_64FC1, cam1);
	Mat distortion_coeff = Mat(5, 1, CV_64FC1, distCoeff1);

	width_target = 24;             //长宽
	height_target = 24;

	std::vector<cv::Point2f> Points2D;    //图片坐标
	//Point2f vertices[4];
	Points2D.push_back(vertices[1]);
	Points2D.push_back(vertices[2]);
	Points2D.push_back(vertices[3]);
	Points2D.push_back(vertices[0]);

	std::vector<cv::Point3f> Point3d;     //世界坐标

	half_x = (width_target) / 2.0;
	half_y = (height_target) / 2.0;

	Point3d.push_back(Point3f(-half_x, -half_y, 0));
	Point3d.push_back(Point3f(half_x, -half_y, 0));
	Point3d.push_back(Point3f(half_x, half_y, 0));
	Point3d.push_back(Point3f(-half_x, half_y, 0));

	Mat rot1 = Mat::eye(3, 3, CV_64FC1);//旋转矩阵
	Mat trans1 = Mat::zeros(3, 1, CV_64FC1);//平移矩阵

	solvePnP(Point3d, Points2D, cam_matrix, distortion_coeff, rot1, trans1, false);
	Mat_<double> rot_mat;
	Rodrigues(rot1, rot_mat);

	double sy = std::sqrt(rot_mat.at<double>(0, 0) * rot_mat.at<double>(0, 0) + rot_mat.at<double>(1, 0) * rot_mat.at<double>(1, 0));

	bool singularot_mat = sy < 1e-6;

	double x, y, z;
	if (!singularot_mat)
	{
		x = std::atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
		y = std::atan2(-rot_mat.at<double>(2, 0), sy);
		z = std::atan2(rot_mat.at<double>(1, 0), rot_mat.at<double>(0, 0));
	}
	else
	{
		x = std::atan2(-rot_mat.at<double>(1, 2), rot_mat.at<double>(1, 1));
		y = std::atan2(-rot_mat.at<double>(2, 0), sy);
		z = 0;
	}

	// Convert angles to degrees
	x = x * 180.0 / CV_PI;
	y = y * 180.0 / CV_PI;
	z = z * 180.0 / CV_PI;
	cout << trans1 << endl;
	cout << "z轴转：" << x << "   x轴转：" << y << "   y轴转：" << z << endl;
}

