#include"yuan.hpp"
#include"../camera/Mycamera.hpp"
#include"../serial/serial_port.hpp"

std::ofstream myfile("D:\\桌面\\output1.txt");
std::ofstream myfile2("D:\\桌面\\output2.txt");
std::ofstream myfile3("D:\\桌面\\output3.txt");

int d1, d2, d3, d4;
int d = 0;
int capcols, caprows;
int mean_lenth = 0;
std::vector<cv::Point2f> vertices3(4);

//把角度转换成统一的          (无需改动，跳过)
double transf(double a)
{
	if (a < 0)
		return 180.0 + a;
	else if (a >= 180)
		return 0;
	else
		return a;
}


//图像预处理                  (无需改动，跳过)
Mat precessing(Mat image)
{
	// 转换为灰度图像
	Mat gray;
#if COLOR==1
	inRange(image, Scalar(230, 230, 230), Scalar(255, 255, 255), gray);
#elif COLOR==0        
	inRange(image, Scalar(5, 15, 150), Scalar(255, 255, 255), gray);             //测试的时候记得看素材改数据(这个是图片的）
#elif
	inRange(image,Scalar())
#endif
	cv::namedWindow("gray", WINDOW_NORMAL);                                      //屏蔽输出提高点效率
	cv::imshow("gray", gray);
	cv::waitKey(1);
	return gray;
}

//计算差异最小的三个数        (无需改动，跳过)
void find_min_diff_indices(double arr[], int n, int& ind1, int& ind2, int& ind3)
{
	if (n <= 3) {
		cout << "find_min_diff_indices error" << endl;
		return;
	}
	double min_var = numeric_limits<double>::infinity();
	int idx1 = -1, idx2 = -1, idx3 = -1;     // 方差最小的三个数的下标
	for (int i = 1; i < n - 1; i++)
	{
		double mean = (arr[i - 1] + arr[i] + arr[i + 1]) / 3.0;
		double variance = ((arr[i - 1] - mean) * (arr[i - 1] - mean) + (arr[i] - mean) * (arr[i] - mean) + (arr[i + 1] - mean) * (arr[i + 1] - mean)) / (mean * mean * mean);
		if (variance < min_var)
		{
			min_var = variance;
			idx1 = i - 1;
			idx2 = i;
			idx3 = i + 1;
		}
	}
	ind1 = idx1;
	ind2 = idx2;
	ind3 = idx3;
}

//找到三个三角形的相似度返回
double drawMostSimilarContours(const std::vector<std::vector<cv::Point>>& contours)
{
	if (contours.size() != 3)
	{
		std::cout << "error" << std::endl;
		return 0;
	}
	double ans = 0;
	ans += matchShapes(contours[0], contours[1], CONTOURS_MATCH_I3, 0);
	ans += matchShapes(contours[1], contours[2], CONTOURS_MATCH_I3, 0);
	ans += matchShapes(contours[2], contours[0], CONTOURS_MATCH_I3, 0);
	return ans;
}

//计算差异最小的三个角度
void find_min_diff_angles(double arr[], int n, int& ind1, int& ind2, int& ind3)
{
	double min_var = numeric_limits<double>::infinity();
	int idx1 = -1, idx2 = -1, idx3 = -1; // 方差最小的三个数的下标
	for (int i = 1; i < n - 1; i++)
	{
		double mean = (arr[i - 1] + arr[i] + arr[i + 1]) / 3.0;
		double variance = abs(arr[i - 1] - mean) + abs(arr[i] - mean) + abs(arr[i + 1] - mean);
		if (variance < min_var)
		{
			min_var = variance;
			idx1 = i - 1;
			idx2 = i;
			idx3 = i + 1;
		}
	}
	ind1 = idx1;
	ind2 = idx2;
	ind3 = idx3;
}

//返回最小差异角的平均值      (无需改动，跳过)
double findMinangles(vector<double>& angles)
{
	if (angles.size() < 3)
	{
		return 0;
	}
	double nums[10] = { 0 };
	int min1 = min((int)angles.size(), 10);
	for (int i = 0; i < min1; i++)
		nums[i] = angles[i];
	int n = min1;
	int idx1, idx2, idx3;
	find_min_diff_angles(nums, n, idx1, idx2, idx3);
	if (idx1 >= angles.size() || idx2 >= angles.size() || idx3 >= angles.size())
	{
		return 0;
	}
	angles.push_back(INF);
	angles.erase(angles.begin() + idx1);
	angles.erase(angles.begin() + idx2 - 1);
	angles.erase(angles.begin() + idx3 - 2);
	double ans = (nums[idx1] + nums[idx2] + nums[idx3]) / 3.0;
	return ans;
}

//找四个顶点，返回四个顶点的vector
vector<Point2f> findApexs(vector<RotatedRect> minRect, vector<vector<cv::Point>> contours, vector<Triangle> triangles, vector<int> index)
{
	//中心坐标计算
	float x = 0, y = 0;
	//std::cout << "findApexs中的index:";
	for (int i = 0; i < index.size(); i++)
	{
		//std::cout << index[i];
	}
	//std::cout << endl;
	for (int i = 0; i < 4; i++)
	{
		x += minRect[index[i]].center.x;
		y += minRect[index[i]].center.y;
	}
	x /= 4;
	y /= 4;//Point(x,y)是中心点
	//std::cout << "矩形中心点：" << cv::Point(x, y) << endl;

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

// 检测图像中的三角形并计算每个三角形的三条边的角度和长度
vector<Triangle> detectTriangles(const vector<vector<cv::Point>>& contours, Mat& cap)
{
	if (contours.size() <= 3) {
		cout << "detectTriangles error" << endl;
		return *new vector<Triangle>;
	}
	vector<Triangle> triangles;

	// 对每个轮廓进行处理
	for (int i = 0; i < contours.size(); i++)
	{
		// 使用minEnclosingTriangle函数获取最小外接三角形
		vector<Point2f> triangle_points;
		minEnclosingTriangle(contours[i], triangle_points);

		Triangle t;
		t.triangle_points = triangle_points;
		t.init();
		triangles.push_back(t);
	}
	return triangles;
}

//两点间距离
float getDistance(Point2f a, Point2f b)
{
	float distance;
	distance = powf((a.x - b.x), 2) + powf((a.y - b.y), 2);
	distance = sqrtf(distance);
	return distance;
}

//找第四个发光的角，返回下标
int find_four_apex(vector<vector<cv::Point>> contours, vector<Triangle> triangle, vector<int> findMinindex, Mat cap,vector<Point2f> p3_to_circles)
{
	if (findMinindex.size() != 3)
	{
		//std::cout << "error:find_four_apex" << endl;
		return 0;
	}
	else
	{

		//std::cout << "以下是三个三角形的所有信息：" << endl;

		// 遍历所有需要绘制的三角形
		for (int i = 0; i < findMinindex.size(); ++i) {
			//std::cout << "三条边:" << endl;

			// 绘制当前三角形的颜色
			Scalar color(0, 255, 0);
			//std::cout << triangle[findMinindex[i]].edge_len1 << "   " << triangle[findMinindex[i]].edge_len2 << "   " << triangle[findMinindex[i]].edge_len3 << endl;
			//std::cout << "三个角：" << endl;
			//std::cout << triangle[findMinindex[i]].angle1 << "   " << triangle[findMinindex[i]].angle2 << "   " << triangle[findMinindex[i]].angle3 << endl;
			//std::cout << "三个顶点坐标" << endl;
			//for (int j = 0; j < triangle[findMinindex[i]].triangle_points.size(); j++)
				//cout << triangle[findMinindex[i]].triangle_points[j] << "     ";
			//cout << endl;
			// 在 cap 图像中绘制当前三角形
			for (int j = 0; j < 3; j++) {
				line(cap, triangle[findMinindex[i]].triangle_points[j], triangle[findMinindex[i]].triangle_points[(j + 1) % 3], color, 2);
			}
		}

		//把三角形的三个角汇总
		vector<double> handle_angles;
		for (int i = 0; i < findMinindex.size(); i++)
		{
			handle_angles.push_back(transf(triangle[findMinindex[i]].angle1));
			handle_angles.push_back(transf(triangle[findMinindex[i]].angle2));
			handle_angles.push_back(transf(triangle[findMinindex[i]].angle3));
		}
		//std::cout << endl;

		//std::cout << "handle_angles没排序的输出：" << endl;
		//for (int i = 0; i < handle_angles.size(); i++)
			//std::cout << handle_angles[i] << "   ";
		//std::cout << endl;

		std::sort(handle_angles.rbegin(), handle_angles.rend());
		//std::cout << "三个三角形的三条边的夹角排序结果：" << endl;
		//for (int i = 0; i < handle_angles.size(); i++)
			//std::cout << handle_angles[i] << "   ";

		double angle1, angle2, angle3;
		angle1 = findMinangles(handle_angles);
		angle2 = findMinangles(handle_angles);
		//std::cout << "第一个平均值" << angle1 << "  第二个平均值  " << angle2 << endl;
		//std::cout << endl << endl;

		double min1 = INF;
		int min1index=0;
		for (int i = 0; i < triangle.size(); i++)
		{
			bool isture = 0;
			for (int j = 0; j < p3_to_circles.size(); j++)
			{
				for (int k = 0; k < triangle[i].triangle_points.size(); k++)
				{
					if (getDistance(p3_to_circles[j], triangle[i].triangle_points[k] )<= mean_lenth)
						isture = 1;
				}
			}
			if (isture) {
				double ans11 = min(triangle[i].angle1 - angle1, triangle[i].angle2 - angle1);
					double ans1 = min(ans11, triangle[i].angle3 - angle1);
					double ans22 = min(triangle[i].angle1 - angle2, triangle[i].angle2 - angle2);
					double ans2 = min(ans22, triangle[i].angle3 - angle2);
					double diff = abs(ans1) + abs(ans2);

					//以下在算面积
					double s = (triangle[i].edge_len1 + triangle[i].edge_len2 + triangle[i].edge_len3) / 2.0;
				double area = s * (s - triangle[i].edge_len1) * (s - triangle[i].edge_len2) * (s - triangle[i].edge_len3);

				//综合判据
				diff = diff / area;
				//std::cout << "   ans1:" << ans1 << "  ans2:" << ans2;
				//std::cout << "   diff:" << diff << endl;
				if (min1 > diff && (triangle[i].edge_len1 > 8 && triangle[i].edge_len2 > 8 && triangle[i].edge_len3 > 8) && find(findMinindex.begin(), findMinindex.end(), i) == findMinindex.end())  //找不到的意思
				{
					min1 = diff;
					min1index = i;
				}
			}
		}
		//std::cout << endl;
		//std::cout << "find_four_index:" << min1index << endl;
		return min1index;
	}
}

//一个转换作用，输入面积序列，传出差异最小的三个面积下标
vector<int> findMinareas(vector<double> areas)    //areas已经排序了(降序)
{
	double nums[7] = { 0 };
	int min1 = min((int)areas.size(), 7);
	for (int i = 0; i < min1; i++)
		nums[i] = areas[i];
	int n = min1;
	int idx1, idx2, idx3;
	find_min_diff_indices(nums, n, idx1, idx2, idx3);

	vector<int> ans;
	ans.push_back(idx1);
	ans.push_back(idx2);
	ans.push_back(idx3);
	return ans;
}
//把roi的点画出来
vector<Point2f> detectmask(vector<RotatedRect> minRect, vector<int> Minindex,Mat cap, int capcols, int caprows)
{
	vector<Point2f> ans,res;
	double maxDiagonal = 0;
	for (int i=0;i<Minindex.size();i++) 
	{
		RotatedRect rect = minRect[Minindex[i]];
		double diagonal = std::sqrt(rect.size.width * rect.size.width + rect.size.height * rect.size.height);
		if (diagonal > maxDiagonal) {
			maxDiagonal = diagonal;
		}
	}
	mean_lenth= maxDiagonal;
	
	ans.push_back(minRect[Minindex[0]].center + minRect[Minindex[1]].center - minRect[Minindex[2]].center);
	ans.push_back(minRect[Minindex[1]].center + minRect[Minindex[2]].center - minRect[Minindex[0]].center);
	ans.push_back(minRect[Minindex[2]].center + minRect[Minindex[0]].center - minRect[Minindex[1]].center);
	for (int i = 0; i < ans.size(); i++)
		if (ans[i].x > 0 && ans[i].x < capcols && ans[i].y>0 && ans[i].y < caprows)
			res.push_back(ans[i]);
	//调试输出
	Mat capshow = cap.clone();
	if (!res.empty())
		for (int i = 0; i < res.size(); i++)
			circle(capshow, res[i], mean_lenth, Scalar(255, 255, 255), -1);
	imshow("capshow", capshow);
	waitKey(1);
	return res;	
}
//筛选出四个直角的灯条，返回下标
vector<int> handleLight(vector<vector<cv::Point>> contours, vector<RotatedRect> minRect, vector<Triangle> triangle, Mat cap,int capcols,int caprows)
{
	//干扰项排除    现在的方案是1.去除太小的矩形2.找三角形面积最相似的三个面积
	vector<double> areas;
	for (int i = 0; i < triangle.size(); i++) {
		if (triangle[i].getArea() > 20)
			areas.push_back(triangle[i].getArea());
	}
	//findMinindex是下标
	//std::cout << "干扰项排除完成" << endl;
	vector<int> Minindex = findMinareas(areas);
	//插入调试区，调试角点识别是否准确

	for (int i = 0; i < Minindex.size(); i++)
	{
		for (int j = 0; j < triangle[Minindex[i]].triangle_points.size(); j++)
			myfile3 << triangle[Minindex[i]].triangle_points[j] << ",";
		myfile3 << endl;
	}
	if (drawMostSimilarContours(tmp_contours) > 10)
		return *new vector<int>;
	//结束插入调试
	if (Minindex.size() == 3) {
		//找第四个顶点+转换参数+加入掩码机制

		vector<Point2f> p3_to_circles = detectmask(minRect, Minindex,cap,capcols,caprows);//返回三个点待选点的坐标(不一定是三个）
		vector<vector<cv::Point>> tmp_contours;
		Minindex.push_back(find_four_apex(tmp_contours, triangle, Minindex, cap,p3_to_circles));//这里把第四个点找完了
		//std::cout << "所有找到的斜矩形面积：" << endl;
		//for (int i = 0; i < minRect.size(); i++)
			//std::cout << minRect[i].size.area() << "   ";
		//std::cout << endl << "handleLight里所有三角形的信息：" << endl;
		//for (int i = 0; i < triangle.size(); i++)
			//std::cout << "第一个角" << triangle[i].angle1 << " 第二个角  " << triangle[i].angle2 << "  第三个角  " << triangle[i].angle3 << endl << "  三个点   " << endl << triangle[i].triangle_points << endl;
		//std::cout << endl;
		//std::cout << "找第四个角完成" << endl;
		for (int i = 0; i < Minindex.size(); i++)
			if (Minindex[i] < 0)
				return *new vector<int>;
		return Minindex;
	}
	else
		return *new vector<int>;

}

//主函数，负责调用二值图后的全流程
vector<Point2f> handleMat(Mat src, Mat image, int capcols, int caprows,VisionData& visiondata)
{
	vector<vector<cv::Point>> contours;
	vector<Vec4i> hierarchy;
	visiondata.isDetected = true;
	// 找到所有轮廓
	findContours(src, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
#ifdef GRAY
	std::cout << "轮廓数量：" << contours.size() << endl;
	if (contours.size() < 10000000000) {
		vector<Point2f> a;
		return a;
	}
#endif // GRAY
	// 为每个轮廓找到一个斜矩形和最小外接三角形
	vector<RotatedRect> minRect(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		minRect[i] = minAreaRect(contours[i]);
	}
	vector<Triangle> triangles = detectTriangles(contours, image);//调试，这个有用

	//将按照斜矩形的面积轮廓和斜矩形都排序，并保持轮廓和斜矩形变长数组的坐标一一对应
	vector<RotatedRect> handle_minRect(minRect.size());
	vector<vector<cv::Point>> handle_contours(contours.size());
	vector<Triangle> handle_triangles(triangles.size());
	//vector<Triangle> handle_triangles(triangles.size());
	//自己写的选择排序(不想优化),按照minRect的面积大小排序
	for (int i = 0; i < contours.size(); i++)
	{
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
		handle_triangles[i] = triangles[max_i];
		minRect[max_i] =* new RotatedRect;
	}
	//std::cout << "轮廓，矩形排序完成" << endl;


	//判断至少有四个角
	vector<int> true_index;
	if (minRect.size() >= 4)
	{
		true_index = handleLight(handle_contours, handle_minRect, handle_triangles, image,capcols,caprows);
		visiondata.isDetected = true;
	}
	else
	{
		std::cout << "没有识别到四个角" << endl;
		visiondata.isDetected = false;
		return *new vector<Point2f>;
	}
	if (true_index.empty())
	{
		std::cout << "下标错误" << endl;
		return *new vector<Point2f>;
	}
	//传参,找四个顶点
	vector<Point2f> true_vertexs = findApexs(handle_minRect, handle_contours, handle_triangles, true_index);
	//std::cout << "findApexs finish" << endl;
	Scalar colors[] = { Scalar(0, 0, 255), Scalar(0, 255, 0), Scalar(255, 0, 0), Scalar(0, 128, 255) };
	Mat cap1 = image.clone();
	Mat cap2 = image.clone();
	Mat cap3 = image.clone();
#ifdef DEBUG1//顶点
	// 定义颜色数组,RGB顺序分别是蓝，绿，红，亮蓝

	// 创建输出图像

	for (int i = 0; i < true_vertexs.size(); i++)
		circle(cap1, true_vertexs[i], 5, colors[i], -1);
	cv::namedWindow("cap1", WINDOW_NORMAL);
	cv::imshow("cap1", cap1);
#endif

#ifdef DEBUG2//斜矩形
	// 绘制旋转矩形
	for (int i = 0; i < true_index.size(); i++)
	{
		// 根据面积确定颜色
		Scalar color;
		color = colors[i];

		Point2f rect_points[4];
		handle_minRect[true_index[i]].points(rect_points);
		for (int j = 0; j < 4; j++)
		{
			line(cap2, rect_points[j], rect_points[(j + 1) % 4], color, 7, LINE_AA);
		}
	}
	cv::namedWindow("cap2", WINDOW_NORMAL);
	cv::imshow("cap2", cap2);
#endif // DEBUG2

#ifdef DEBUG3
	for (int i = 0; i < true_index.size(); i++)
	{
		Scalar color;
		color = colors[i];
		for (int j = 0; j < 3; j++)
		{
			line(cap3, handle_triangles[i].triangle_points[j], handle_triangles[i].triangle_points[(j + 1) % 3], color, 7);
		}
	}
	cv::namedWindow("cap3", WINDOW_NORMAL);
	cv::imshow("cap3", cap3);
#endif
	cv::waitKey(1);

	//返回值处理
	return true_vertexs;
}

//旋转向量转换成欧拉角
void rotationMatrixToEulerAngles(Mat& R, double& roll, double& pitch, double& yaw)
{
	double r11 = R.at<double>(0, 0);
	double r12 = R.at<double>(0, 1);
	double r13 = R.at<double>(0, 2);
	double r21 = R.at<double>(1, 0);
	double r22 = R.at<double>(1, 1);
	double r23 = R.at<double>(1, 2);
	double r31 = R.at<double>(2, 0);
	double r32 = R.at<double>(2, 1);
	double r33 = R.at<double>(2, 2);

	pitch = asin(-r31);
	if (cos(pitch) != 0) {
		roll = atan2(r32 / cos(pitch), r33 / cos(pitch));
		yaw = atan2(r21 / cos(pitch), r11 / cos(pitch));
	}
	else {
		roll = 0;
		yaw = atan2(-r12, r22);
	}
}

//排序算法1 会跳变，建议用2
std::vector<cv::Point2f> uuusortxy(std::vector<cv::Point2f> vertices) 
{
	std::vector<cv::Point2f> vertices2(vertices.size());
	for (int i = 0; i < 4; i++) 
	{
		int cx, cy, tx, ty;
		cx = cy = tx = ty = 0;
		for (int j = 0; j < 4; j++) 
		{
			if (j == i) 
			{
				continue;
			}
			if (vertices[i].x < vertices[j].x) 
			{
				cx++;
			}
			else 
			{
				tx++;
			}
			if (vertices[i].y < vertices[j].y) 
			{
				cy++;
			}
			else 
			{
				ty++;
			}
		}
		if (cx >= 2 && cy >= 2) 
		{
			vertices2[0] = vertices[i];
		}
		if (cx >= 2 && ty >= 2) 
		{
			vertices2[1] = vertices[i];
		}
		if (tx >= 2 && ty >= 2) 
		{
			vertices2[2] = vertices[i];
		}
		if (tx >= 2 && cy >= 2) 
		{
			vertices2[3] = vertices[i];
		}
	}
	return vertices2;
}

//排序算法2 共四个函数
bool cmpy(Point2f a, Point2f b) {                
	if (a.y == b.y) return a.x < b.x;
	return a.y < b.y;
}

bool cmpx(Point2f a, Point2f b) {                
	if (a.x == b.x) return a.y < b.y;
	return a.x < b.x;
}

int cross(Point2f a, Point2f b, Point2f c) {                    //通过向量叉乘确定点c在直线ab的哪一侧。值为负则是左侧。
	return (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
}

vector<Point2f> convex_hull(vector<Point2f>& p) {
	int n = p.size();
	sort(p.begin(), p.end(), cmpy);   //升序排列
	Point2f res;
	if (p[0].x > p[1].x) {
		res = p[0];
		p[0] = p[1];
		p[1] = res;
	}
	int k = 0;					     //当前凸包中点的数量	
	vector<Point2f> q(n * 2);
	for (int i = 0; i < n; i++) {
		while (k > 1 && cross(q[k - 2], q[k - 1], p[i]) <= 0) k--;  //通过计算确定p[i]点相对于直线q[k-2]q[k-1]的位置，若在左侧则q[k-1]需要删去
		q[k++] = p[i];
	}
	for (int i = n - 2, t = k; i >= 0; i--) {
		while (k > t && cross(q[k - 2], q[k - 1], p[i]) <= 0) k--;  //同理
		q[k++] = p[i];
	}
	q.resize(k - 1);                                                //删去无用点
	return q;
}

void sort_points(vector<Point2f>& p) {
	vector<Point2f> q = convex_hull(p);
	int n = q.size();
	int pos = 0;
	for (int i = 1; i < n; i++) {
		if (abs(q[i].x - q[pos].x) < 50) pos = i;
	}
	vector<Point2f> ans(n);
	int cnt = 0;
	for (int i = pos; i < n; i++) ans[cnt++] = q[i];
	for (int i = 0; i < pos; i++) ans[cnt++] = q[i];
	p = ans;
	for (int i = 0; i < p.size(); i++) {
		cout << "排序后：" << endl;
		cout << "(" << p[i].x << ", " << p[i].y << ")" << endl;
	}
	return;
}

//pnp算法
void solveXYZ(std::vector<cv::Point2f> vertices, cv::Mat image,VisionData& visiondata)
{
	//cv::Mat image2 = cv::imread("C:\\Users\\ASUS\\Desktop\\1.png");
	double half_x;
	double half_y;
	double width_target;
	double height_target;

	double cam1[3][3] = {                                      //内参矩阵  
		1325.0, 0, 625.0,
		0, 1324.4, 516.7,
		0, 0, 1 };

	double distCoeff1[5] = {-0.0851,0.1689, 0, 0, 0 };     //畸变参数  

	cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, cam1);
	cv::Mat distortion_coeff = cv::Mat(5, 1, CV_64FC1, distCoeff1);

	width_target = 240;             //长宽
	height_target = 240;

	std::vector<cv::Point2f> Points2D;    //图像坐标系坐标点
	Points2D.push_back(vertices[0]);
	Points2D.push_back(vertices[1]);
	Points2D.push_back(vertices[2]);
	Points2D.push_back(vertices[3]);

	std::vector<cv::Point3f> Point3d;     //世界坐标系坐标点

	half_x = (width_target) / 2.0;
	half_y = (height_target) / 2.0;

	Point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
	Point3d.push_back(cv::Point3f(-half_x, half_y, 0));
	Point3d.push_back(cv::Point3f(half_x, half_y, 0));
	Point3d.push_back(cv::Point3f(half_x, -half_y, 0));

	cv::Mat rot1 = cv::Mat::eye(3, 3, CV_64FC1);           //旋转矩阵
	cv::Mat trans1 = cv::Mat::zeros(3, 1, CV_64FC1);       //平移矩阵

	visiondata.x = (int)trans1.at<float>(0,0);
	visiondata.y = (int)trans1.at<float>(1,0);
	visiondata.z = (int)trans1.at<float>(2,0);
 
	cv::solvePnP(Point3d, Points2D, cam_matrix, distortion_coeff, rot1, trans1, false);
	cv::Mat_<double> rot_mat;
	cv::Mat_<double> trans_mat;
	cv::Rodrigues(rot1, rot_mat);
	cv::Rodrigues(trans1, trans_mat);

	cv::Mat xyz = image.clone();
	std::vector<cv::Point3f> Point3ds;
	std::vector<cv::Point2f> outputPoints;

	Point3ds.push_back(cv::Point3d(0, 0, 0));
	Point3ds.push_back(cv::Point3d(120, 0, 0));
	Point3ds.push_back(cv::Point3d(0, -120, 0));
	Point3ds.push_back(cv::Point3d(0, 0, 120));

	cv::projectPoints(Point3ds, rot1, trans1, cam_matrix, distortion_coeff, outputPoints);
	//std::cout << outputPoints << endl;

	line(xyz, outputPoints[3], outputPoints[0], cv::Scalar(0, 0, 255), 3, 8);
	line(xyz, outputPoints[2], outputPoints[0], cv::Scalar(0, 255, 0), 3, 8);
	line(xyz, outputPoints[1], outputPoints[0], cv::Scalar(255, 0, 0), 3, 8);

	cv::putText(xyz, "Y", outputPoints[3], cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 0, 255), 3, 8);
	cv::putText(xyz, "Z", outputPoints[2], cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 0), 3, 8);
	cv::putText(xyz, "X", outputPoints[1], cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 0, 0), 3, 8);

	std::string rot, trans;
	rot << rot1;
	trans << trans1;

	cv::putText(xyz, "rot" + rot, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1);
	cv::putText(xyz, "trans" + trans, cv::Point(30, 80), cv::FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1);

	double dist = sqrt(trans1.at<double>(0, 0) * trans1.at<double>(0, 0) + trans1.at<double>(1, 0) * trans1.at<double>(1, 0) + trans1.at<double>(2, 0) * trans1.at<double>(2, 0));
	//std::cout << "dist =  " << dist << endl;

	double sy = std::sqrt(rot_mat.at<double>(0, 0) * rot_mat.at<double>(0, 0) + rot_mat.at<double>(1, 0) * rot_mat.at<double>(1, 0));

	bool singularot_mat = sy < 1e-6;

	double x, y, z;
	if (!singularot_mat)
	{
		x = std::atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(1, 1));
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
	//cout << trans1 << endl;
	std::cout << "z轴转：" << x << "   x轴转：" << y << "   y轴转：" << z << endl;
	if (-180 < x < 180 && -180 < y < 180 && -180 < z < 180)
		myfile2 << "z轴转：" << x << "   x轴转：" << y << "   y轴转：" << z << endl;
	visiondata.pitch_angle = (int)(y * 100);
    visiondata.yaw_angle = (int)(z * 100);
	visiondata.roll_angle = (int)(x * 100);
    visiondata.dis = (int)(dist);
	/*bool isput = 1;
	for (int i = 0; i < 3; i++) {
		if (trans1.at<double>(i, 0) < 3000 && trans1.at<double>(i, 0) > -3000)
			myfile << trans1.at<double>(i, 0) << ",";
		isput = 0;
	}
	for (int i = 0; i < 3; i++) {
		if (isput = 1 && -180 < x < 180 && -180 < y < 180 && -180 < z < 180)
			myfile << rot1.at<double>(i, 0) << "," << std::endl;
	}*/
	std::string s1, s2, s3, s4;
	s1 = "y: " + std::to_string(z) + " ";
	s2 = "z: " + std::to_string(y) + " ";
	s3 = "x: " + std::to_string(x) + " ";
	s4 = "dist: " + std::to_string(dist) + " ";
	cv::putText(xyz, (s1 + s2 + s3 + s4), cv::Point(30, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1);
	cv::imshow("xyz", xyz);
	cv::waitKey(1);
}

//提供给main的全调用函数
void all(Mat image, int capcols, int caprows,VisionData& visiondata)
{
	// 导入图像
	if (image.empty())
		return;
	Mat processmat = precessing(image);//预处理完
	std::cout << "precessing finish" << endl;
	vector<Point2f> vertexs;//这是四个顶点
	vertexs = handleMat(processmat, image, capcols, caprows, visiondata);//返回四个顶点
	if (vertexs.size() != 4){
		visiondata.isDetected = false;
		return;
	}
	vertexs = uuusortxy(vertexs);
	
	//vertexs = uuusortxy(vertexs);
	solveXYZ(vertexs, image, visiondata);
	return;
}
// #ifdef ISMAP
// int main()
// {
// 	VideoCapture capture("/home/nuc/下载/890.mp4");
// 	Mat image;
// 	while (1) {
// 		capture.read(image);
// 		if (!image.empty())
// 		{
// 			capcols = image.cols;
// 			caprows = image.rows;
// 			all(image);
// 			cv::namedWindow("image", 1);
// 			imshow("image", image);
// 			std::cout << endl << endl << endl;
// 			int c = cv::waitKey(1);
// 			if (c == 27)
// 				break;
// 		}
// 		else
// 			return 0;
// 	}
// }

// #else
// int main()
// {
// 	// 导入图像
// 	Mat image = imread("/home/nuc/下载/890.mp4");
// 	if (!image.empty())
// 	{
// 		capcols = image.cols;
// 		caprows = image.rows;
// 		all(image);
// 		cv::namedWindow("image", WINDOW_NORMAL);
// 		cv::imshow("image", image);
// 		std::cout << endl << endl << endl;
// 		int c = cv::waitKey(0);
// 		return 0;
// 	}
// }
// #endif 

// int main(){
// 	HaiKang HaiKangCamera;
// 	// 设置曝光时间 ≥ 0.0 ，单位 us     模式：R/W
// 	HaiKangCamera.SetFloatValue("ExposureTime", 8000); // 具体的范围：65.0000 - 9999648.0000
// 	// 设置增益 ≥ 0.0 ，单位 dB     模式：R/W
// 	HaiKangCamera.SetFloatValue("Gain", 8.0062); // 具体的范围：0.00- 15.0062

// 	// Gamma值 1 打开
// 	HaiKangCamera.SetBoolValue("GammaEnable", 1); // Gamma使能 1：打开，0关闭
// 	// ＞ 0.0   模式：R/W
// 	HaiKangCamera.SetFloatValue("Gamma", 1); // 具体的范围：0.0000 - 4.0000
// 	int c=0;
// 	while (c!=27)
// 	{
// 		cv::Mat image = HaiKangCamera.GetOneFrameForOpencv();
// 		if (!image.empty())
// 		{
// 			capcols = image.cols;
// 			caprows = image.rows;
// 			all(image);
// 			cv::namedWindow("image", 1);
// 			imshow("image", image);
// 			std::cout << endl << endl << endl;
// 			int c = cv::waitKey(1);
// 			if (c == 27)
// 				break;
// 		}
// 		else
// 			return 0;
// 	}
// 	return 0;
// }