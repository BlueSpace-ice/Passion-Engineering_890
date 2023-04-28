#include "camera/Mycamera.hpp"
#include "serial/general.h"
#include "serial/serial_port.hpp"
#include "yuan/yuan.hpp"
#include <opencv2/opencv.hpp>

//#define USING_HK;
//#define USING_MV;
#define USING_VIDEO;

int64_t timestamp;
cv::Mat image;

bool serialWatcher(Serial_Port &serial);
bool serialWatcher(Serial_Port &serial)
{
	// 串口通讯相关程序
	printf("\n*******Begin setting Com*********\n ");
	char const *device0 = UART_NAME;
	// 串口无权限时自动开启所有权限
	if (access(UART_NAME, W_OK) != 0) // 成功执行时，返回0。失败返回-1
	{
		char commed1[100], dest[30];
		strcpy(commed1, "echo ");
		strcpy(dest, " | sudo -S chmod 777 ");
		strcat(commed1, PASSWD);
		strcat(commed1, dest);
		strcat(commed1, UART_NAME);
		std::cout << commed1 << std::endl;
		system(commed1);
	}

	// 开机时系统会占用串口向外发送开机信息，所以此处不断轮询，直到系统释放所有权为止
	//  while (1)
	//  {
	sleep(1);
	// 检测文件夹是否存在或串口需要初始化
	if (access(UART_NAME, F_OK) == -1 || serial.is_serial_opened == false)
	{
		serial.is_serial_opened = false;
		//* 初始化串口
		serial.init_serial(device0);
	}
	//}
}

int main()
{
#ifdef USING_HK	
	HaiKang HaiKangCamera;
	// 设置曝光时间 ≥ 0.0 ，单位 us     模式：R/W
	HaiKangCamera.SetFloatValue("ExposureTime", 4000); // 具体的范围：65.0000 - 9999648.0000
	// 设置增益 ≥ 0.0 ，单位 dB     模式：R/W
	HaiKangCamera.SetFloatValue("Gain", 8.0062); // 具体的范围：0.00- 15.0062

	// Gamma值 1 打开
	HaiKangCamera.SetBoolValue("GammaEnable", 1); // Gamma使能 1：打开，0关闭
	// ＞ 0.0   模式：R/W
	HaiKangCamera.SetFloatValue("Gamma", 1); // 具体的范围：0.0000 - 4.0000
#endif

#ifdef USING_VIDEO
	VideoCapture capture("/home/nuc/下载/890.mp4");
#endif

// #ifdef USING_MV
// 	MindVision MindVisionCamera;

//     // 设置参数 注意设置参数一定要在相机初始化之前设置

//     // 设置曝光时间，默认设置曝光时间为3ms，单位：微妙，设置为0，则使用相机基础配置文件
//     MindVisionCamera.setExposureTime = 3000;
//     // 设置gamma值，为0，则不设置，默认为55
//     MindVisionCamera.setGamma = 55;
//     // 设置模拟增益，范围[1,6]，为0，则不设置，默认为1
//     MindVisionCamera.setAnalogGain = 1;
//     // 设置锐利值，范围[0,100]，为0，则不设置，默认为0
//     MindVisionCamera.setSharpness = 0;
//     // 设置饱和度，为100，则表示使用默认图像，默认为100
//     MindVisionCamera.setSaturation = 100;
//     // 设置采集图片的宽,默认为1280
//     MindVisionCamera.setWidth = 640;
//     // 设置采集图片的高,默认为1024
//     MindVisionCamera.setHeight = 640;

//     // 相机初始化
//     MindVisionCamera.init();
//     // 开始采集图像
//     MindVisionCamera.play();
// 	if (!MindVisionCamera.isOpened())
//         {
//             cout << "MindVisionCamera is not opened!!" << endl;
//             continue;
//         }
//         MindVisionCamera.initFrameMat(image);
//         MindVisionCamera.wait_and_get(image, DataFrame.timestamp, []() {}); // 获取图像
// #endif
	cout << "串口测试中" << endl;
	Serial_Port serial_port;
	serialWatcher(serial_port);
	int q = 0;
	int c = 0;
	while (c != 27)
	{    
	#ifdef USING_VIDEO
		capture.read(image);
	#endif
	
	#ifdef USING_HK
		cv::Mat image = HaiKangCamera.GetOneFrameForOpencv();
	#endif
	
	// #ifdef USING_MV
	// 	if (!MindVisionCamera.isOpened())
    //     {
    //         cout << "MindVisionCamera is not opened!!" << endl;
    //         continue;
    //     }
    //     MindVisionCamera.initFrameMat(image);
    //     MindVisionCamera.wait_and_get(image, DataFrame.timestamp, []() {}); // 获取图像
	// #endif
		if (!image.empty())
		{
			int capcols = image.cols;
			int caprows = image.rows;
			q++;
			VisionData visiondata;
			all(image, capcols, caprows, visiondata);
			cv::namedWindow("image", 1);
			imshow("image", image);
		    int test = serial_port.uart_send(visiondata, 17);
			cout << "数据已发送" << endl;
			std::cout << endl<< endl<< endl;
			int c = cv::waitKey(1);
			if (c == 27)
				break;
		}
		else
			return 0;
	}
	return 0;
}