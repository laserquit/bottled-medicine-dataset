#include <opencv2/opencv.hpp>
#include "cpm.hpp"
#include "infer.hpp"
#include "yolo.hpp"
#include"DxImageProc.h"
#include"GxIAPI.h"
#include "thread"
#include<math.h>
#include<Serial_writer.h>
#include<fstream>
#include<mutex>

using namespace std;
using namespace cv;

#define IS_SNAP_SINGLE 1//选择单帧还是连续采集模式

std::string model_path = "test_fp16.trt";//设置模型路径
BYTE* pImageBuffer;

Mat c=Mat::zeros(60,130,CV_32FC1);//面积计算权重矩阵
float all;//总面积
int key1=0,key2=0,key3=0;//键盘输入
int badbottle = 0;//坏瓶推出信号
double percent1=0,percent2=0,percent3=0;//三个摄像头的损坏区域占比
float 
squarethreshold=0.95;//损坏面积阈值，默认为90%
int totalnum = 0;//总瓶数
int badnum;//坏瓶数

////摄像头连续调用函数
static void __stdcall OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{


	std::cout << " succeed！ " << std::endl;

	if (pFrame->status == 0)  // 0 是正常帧
	{

				//(GX_FRAME_CALLBACK_PARAM *)

		uchar* din = ((BYTE*)pFrame->pImgBuf);  //
		Mat frame(1536, 2048, CV_8UC1, din);
		//frame.copyTo(image);
		cout << "successful" << endl;	
		cv::namedWindow("win", 1);
		cv::imshow("win", frame);
		cv::waitKey(5);
	}




	return;
}

////调用摄像头函数
int camera()
{
	GX_DEV_HANDLE m_hDevice;  //USB 数字摄像机句柄
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[2048 * 1536];   // 灰度  
	GXInitLib(); //初始化库
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//获得设备个数  //如果在用户指定超时时间内成功枚举到设备，则立即返回
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//获取设备信息

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(1, &m_hDevice);		//打开数字相机，注：已经包含默认参数设

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 2048);		//设置图像宽度2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 1536);		//设置图像高度1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//设置采集模式为连续采集
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//设置采集速度，范围(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//设置增益,增益范围(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 30000);	//曝光时间30ms
#if IS_SNAP_SINGLE //采集单帧模式
	GX_FRAME_DATA frameData;
	frameData.pImgBuf = pImageBuffer;
	frameData.nStatus = -1;
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	do
	{
		status = GXGetImage(m_hDevice, &frameData, 5);
	} while (frameData.nStatus != 0);

	
	
	//可对pImageBuffer 进行图像处理或者显示操作
		//………………………
	
		
		
		//结束图像处理操作
		status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
		
#else//连续采集模式								
							//pUserParam指向用户将在回调处理函数中使用的私有数据指针
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//注册回调函数

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);
	
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//注销回调函数
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

//专门用于发送线程的函数,发送一个字符a用于单片机判断是否接收到数据
void senda(){
	serial_writer Serial_writer;
	Serial_writer.set_serial("COM3", 9600);//设置发送端口名称为：COM3；设置发送波特率为：9600
	std::this_thread::sleep_for(std::chrono::microseconds(4850000));//等待4.85s发送串口
	cout << "准备发送" << endl;
	Serial_writer.send_data('a');//发送1字节数据：'a'	
}
//以下为初始化串口的函数
void serial_writer::set_serial(CString port_name, int baud_rate) {
	this->COM_port = port_name.AllocSysString();
	this->baud_rate = baud_rate;
}

void serial_writer::send_data(char input_data_1_byte) {
	bool serial_open = FALSE;
	serial_open = this->serial_open(this->COM_port, this->baud_rate);
	if (serial_open) {
		this->serial_write(&input_data_1_byte);
		this->Serial_close();
	}
}

int serial_writer::serial_open(LPCWSTR COMx, int BaudRate) {
	this->hCom = CreateFile(COMx, //COMx口    
		GENERIC_READ | GENERIC_WRITE, //允许读和写    
		0, //独占方式    
		NULL,
		OPEN_EXISTING, //打开而不是创建     
		0, //重叠方式FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED  (同步方式设置为0)
		NULL);
	if (this->hCom == INVALID_HANDLE_VALUE)
	{
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED);
		std::cout << "-------------------------  无法打开串口！  -------------------------" << std::endl;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY |
			FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
		std::cout << std::endl;
		return FALSE;
	}
	SetupComm(this->hCom, 1024, 1024); //输入缓冲区和输出缓冲区的大小都是1024 

								 //设定读写超时 
								 /*COMMTIMEOUTS TimeOuts;
								 TimeOuts.ReadIntervalTimeout=1000;
								 TimeOuts.ReadTotalTimeoutMultiplier=500;
								 TimeOuts.ReadTotalTimeoutConstant=5000; //设定写超时
								 TimeOuts.WriteTotalTimeoutMultiplier=500;
								 TimeOuts.WriteTotalTimeoutConstant = 2000;
								 SetCommTimeouts(hCom, &TimeOuts); //设置超时
								 */
	DCB dcb;
	GetCommState(this->hCom, &dcb);
	dcb.BaudRate = BaudRate;		//设置波特率为BaudRate
	dcb.ByteSize = 8;					//每个字节有8位 
	dcb.Parity = NOPARITY;			//无奇偶校验位 
	dcb.StopBits = ONESTOPBIT;		//一个停止位
	SetCommState(this->hCom, &dcb);		//设置参数到hCom
	PurgeComm(this->hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//清空缓存区		//PURGE_TXABORT 中断所有写操作并立即返回，即使写操作还没有完成。
												   //PURGE_RXABORT 中断所有读操作并立即返回，即使读操作还没有完成。
												   //PURGE_TXCLEAR 清除输出缓冲区 
												   //PURGE_RXCLEAR 清除输入缓冲区  
	return TRUE;
}

int serial_writer::serial_write(char lpOutBuffer[]) {//同步写串口
	DWORD dwBytesWrite = sizeof(lpOutBuffer);
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(this->hCom, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(this->hCom, lpOutBuffer, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		printf("写串口失败!\n");
		return FALSE;
	}
	PurgeComm(this->hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return TRUE;
}

void serial_writer::Serial_close(void) {//关闭串口
	CloseHandle(this->hCom);
}

//以下是测试代码
static const char* cocolabels[] = { "good","bad" };//设置数据集标签，必须按照训练的顺序
yolo::Image cvimg(const cv::Mat& image) { return yolo::Image(image.data, image.cols, image.rows); }//定义cvimg

mutex mutex_1;//加入互斥锁，防止数据调用冲突
mutex mutex_2;
mutex mutex_3;

int camerathread1()
{
	mutex_1.lock();
	Mat image1;
	double temp=0;
	int k;//用来移动过程中计数
	float confidence_threshold = 0.7f;
	float nms_threshold = 0.8f;
	auto yolo1 = yolo::load(model_path, yolo::Type::V8Seg, confidence_threshold, nms_threshold);
	//以上是初始化yolo模型

	GX_DEV_HANDLE m_hDevice;  //USB 数字摄像机句柄
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682 ];   // 灰度  
	GXInitLib(); //初始化库
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//获得设备个数  //如果在用户指定超时时间内成功枚举到设备，则立即返回
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//获取设备信息
	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(1, &m_hDevice);		//打开数字相机，注：已经包含默认参数设
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//设置图像宽度2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//设置图像高度1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//设置采集模式为连续采集
	

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//设置采集速度，范围(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//设置增益,增益范围(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//曝光时间30ms
#if IS_SNAP_SINGLE //采集单帧模式
	cout << "single1 "<< endl;
	while (1) {
		if (key1 == 1) { //如果连续拍而不是用激光控制的话就注释掉这个if
			//key = 0;
			Mat seg;//创建空的segment图像
			GX_FRAME_DATA frameData;
			frameData.pImgBuf = pImageBuffer;
			frameData.nStatus = -1;
			//int32_t x = frameData.nPixelFormat;
			//cout << x << endl;
			status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
			do
			{
				status = GXGetImage(m_hDevice, &frameData, 5);
			} while (frameData.nStatus != 0);
			//可对pImageBuffer 进行图像处理或者显示操作
			Mat frame(682, 816, CV_8UC1, pImageBuffer);
			//imshow("test", frame);
			cvtColor(frame, frame, COLOR_GRAY2RGB);
			//cout << frame.type() << endl;
			namedWindow("test", WINDOW_NORMAL);
			imshow("test", frame);
			imwrite("C:/text/2.jpg", frame);
			waitKey(2);
			//image1 = cv::imread("2023-07-12_15_42_52_082.jpg");
			//cout << image1.type() << endl;
		   //开始yolo预测完好部分并生成segment图像
		   //yolo1->forward(cvimg(frame));
			auto objs = yolo1->forward(cvimg(frame));
			for (auto& obj : objs)
			{
				if (obj.class_label == 0 && obj.seg)
				{
					Mat seg1(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
					seg1.copyTo(seg);
					imshow("11", seg1);
					imwrite("C:/text/1.jpg", seg1);
					//cout << seg.rows << seg.cols << endl;
					waitKey(2);
				}

			}

			//面积计算算法
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //二值化突出药片完好区域
				seg.release();    // 释放内存空间
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //修改图片尺寸使符合输入
				Mat crop = resized(Range(0, 60), Range(9, 139));     // 剪除无关区域
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //输出的宽和高
				int up_height = output.rows;
				
				int i; //记录像素位置
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //构建全1矩阵

				//cout << "总面积是" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //计算无缺陷区域面积
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//取几次检测到的面积最大值
				if (percent1 < temp)
					percent1 = temp;
				cout << "摄像头1正常面积占比：" << percent1 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "正常" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;
					
					cout << "不合格药品";
					
				}*/
				
				//destroyAllWindows();
			}
			else
			{
				percent1 = 0.9;
				//cout << "没有检测到药瓶1" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
		//结束图像处理操作
	    mutex_1.unlock();
		status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
		
#else//连续采集模式								
							//pUserParam指向用户将在回调处理函数中使用的私有数据指针
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//注册回调函数

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);
	
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//注销回调函数
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

int camerathread2()
{
	mutex_2.lock();
	Mat image2;
	float confidence_threshold = 0.7f;
	float nms_threshold = 0.8f;
	auto yolo2 = yolo::load(model_path, yolo::Type::V8Seg, confidence_threshold, nms_threshold);
	//以上是初始化yolo模型
	double temp;

	GX_DEV_HANDLE m_hDevice;  //USB 数字摄像机句柄
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682];   // 灰度  
	GXInitLib(); //初始化库
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//获得设备个数  //如果在用户指定超时时间内成功枚举到设备，则立即返回
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//获取设备信息

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(2, &m_hDevice);		//打开数字相机，注：已经包含默认参数设
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//设置图像宽度2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//设置图像高度1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//设置采集模式为连续采集
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//设置采集速度，范围(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//设置增益,增益范围(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//曝光时间30ms
#if IS_SNAP_SINGLE //采集单帧模式
	cout << "single2 " << endl;
	while (1) {
		if (key2 == 1) { //如果连续拍而不是用激光控制的话就注释掉这个if
			//key = 0;
			Mat seg;//创建空的segment图像
			GX_FRAME_DATA frameData;
			frameData.pImgBuf = pImageBuffer;
			frameData.nStatus = -1;
			//int32_t x = frameData.nPixelFormat;
			//cout << x << endl;
			status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
			do
			{
				status = GXGetImage(m_hDevice, &frameData, 5);
			} while (frameData.nStatus != 0);
			//可对pImageBuffer 进行图像处理或者显示操作
			Mat frame2(682, 816, CV_8UC1, pImageBuffer);
			//imshow("test", frame);
			cvtColor(frame2, frame2, COLOR_GRAY2RGB);
			//cout << frame.type() << endl;
			namedWindow("test2", WINDOW_NORMAL);
			imshow("test2", frame2);
			waitKey(2);
			//image1 = cv::imread("2023-07-12_15_42_52_082.jpg");
			//cout << image1.type() << endl;
		   //开始yolo预测完好部分并生成segment图像
		   //yolo1->forward(cvimg(frame));
			auto objs = yolo2->forward(cvimg(frame2));
			for (auto& obj : objs)
			{
				if (obj.class_label == 0 && obj.seg)
				{
					Mat seg1(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
					seg1.copyTo(seg);
					imshow("22", seg1);
					//cout << seg.rows << seg.cols << endl;
					waitKey(2);
				}

			}

			//面积计算算法
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //二值化突出药片完好区域
				seg.release();    // 释放内存空间
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //修改图片尺寸使符合输入
				Mat crop = resized(Range(0, 60), Range(9, 139));     // 剪除无关区域
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output2", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //输出的宽和高
				int up_height = output.rows;

				int i; //记录像素位置
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //构建全1矩阵

				//cout << "总面积是" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //计算无缺陷区域面积
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//取几次检测到的面积最大值
				if (percent2 < temp)
					percent2 = temp;
				cout << "摄像头2正常面积占比：" << percent2 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "正常" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;

					cout << "不合格药品";

				}*/

				//destroyAllWindows();
			}
			else
			{
				percent2 = 0.9;
				//cout << "没有检测到药瓶2" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
	//结束图像处理操作
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	mutex_2.unlock();

#else//连续采集模式								
							//pUserParam指向用户将在回调处理函数中使用的私有数据指针
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//注册回调函数

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//注销回调函数
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

int camerathread3()
{
	mutex_3.lock();
	Mat image3;
	float confidence_threshold = 0.7f;
	float nms_threshold = 0.8f;
	auto yolo3 = yolo::load(model_path, yolo::Type::V8Seg, confidence_threshold, nms_threshold);
	//以上是初始化yolo模型
	double temp;

	GX_DEV_HANDLE m_hDevice;  //USB 数字摄像机句柄
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682];   // 灰度  
	GXInitLib(); //初始化库
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//获得设备个数  //如果在用户指定超时时间内成功枚举到设备，则立即返回
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//获取设备信息

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(3, &m_hDevice);		//打开数字相机，注：已经包含默认参数设
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//设置图像宽度2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//设置图像高度1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//设置采集模式为连续采集
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//设置采集速度，范围(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//设置增益,增益范围(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//曝光时间30ms
#if IS_SNAP_SINGLE //采集单帧模式
	cout << "single3 " << endl;
	while (1) {
		if (key3 == 1) { //如果连续拍而不是用激光控制的话就注释掉这个if
			//key = 0;
			Mat seg;//创建空的segment图像
			GX_FRAME_DATA frameData;
			frameData.pImgBuf = pImageBuffer;
			frameData.nStatus = -1;
			//int32_t x = frameData.nPixelFormat;
			//cout << x << endl;
			status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
			do
			{
				status = GXGetImage(m_hDevice, &frameData, 5);
			} while (frameData.nStatus != 0);
			//可对pImageBuffer 进行图像处理或者显示操作
			Mat frame(682, 816, CV_8UC1, pImageBuffer);
			//imshow("test", frame);
			cvtColor(frame, frame, COLOR_GRAY2RGB);
			//cout << frame.type() << endl;
			namedWindow("test3", WINDOW_NORMAL);
			imshow("test3", frame);
			waitKey(2);
			//image1 = cv::imread("2023-07-12_15_42_52_082.jpg");
			//cout << image1.type() << endl;
		   //开始yolo预测完好部分并生成segment图像
		   //yolo1->forward(cvimg(frame));
			auto objs = yolo3->forward(cvimg(frame));
			for (auto& obj : objs)
			{
				if (obj.class_label == 0 && obj.seg)
				{
					Mat seg1(obj.seg->height, obj.seg->width, CV_8U, obj.seg->data);
					seg1.copyTo(seg);
					imshow("33", seg1);
					//cout << seg.rows << seg.cols << endl;
					waitKey(2);
				}

			}

			//面积计算算法
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //二值化突出药片完好区域
				seg.release();    // 释放内存空间
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //修改图片尺寸使符合输入
				Mat crop = resized(Range(0, 60), Range(9, 139));     // 剪除无关区域
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output3", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //输出的宽和高
				int up_height = output.rows;

				int i; //记录像素位置
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //构建全1矩阵

				//cout << "总面积是" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //计算无缺陷区域面积
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//取几次检测到的面积最大值
				if (percent3 < temp)
					percent3 = temp;
				cout << "摄像头3正常面积占比：" << percent3 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "正常" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;

					cout << "不合格药品";

				}*/

				//destroyAllWindows();
			}
			else
			{
				percent3 = 0.9;
				//cout << "没有检测到药瓶3" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
	//结束图像处理操作
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	mutex_3.unlock();

#else//连续采集模式								
							//pUserParam指向用户将在回调处理函数中使用的私有数据指针
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//注册回调函数

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//注销回调函数
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

int judgethread() //判断是否为坏瓶并且写入qt坏瓶数量的线程，使用keythread打开
{
	double percent = 0; //总损坏面积占比
	cout << "judege 开始";
	percent = (percent1 + percent2 + percent3) / 3;
	cout << endl << "总损坏面积占比 " << percent << endl;

	//判断条件 有低于阈值且所有摄像头都检测到
	if ((percent1 <= squarethreshold || percent2 <= squarethreshold || percent3 <= squarethreshold))
	{
		thread s1(senda);//满足判断条件后等待x秒发送
		s1.detach();

		badbottle++;//坏瓶数加一
		percent = 0;//将信号清除

		//坏瓶数写入
		fstream badtxt;
		badtxt.open("C:/text/nopass.txt", ios::out);
		if (!badtxt.is_open())
		{
			cout << "badtxt文件打开失败。" << endl;
			return 0;
		}
		string trans2 = to_string(badbottle);	//将总数由int转化为string类型
		badtxt << trans2;
		badtxt.close();
	}
}

int keythread()//键盘输入线程 从qt读取阈值和写入总数都在这个线程中
{
	cout << "keythread begin ";
	int right = 0;
	int mid = 0;
	while (1) {
		right = GetKeyState(0x42) & 0x8000;
		if (right && (mid == 0)) {
			cout << "红外触发\n";

			//红外每检测到一次就使用文件方法读取面积阈值
			fstream squaretxt;
			squaretxt.open("C:/text/square.txt", ios::in);
			if (!squaretxt.is_open())
			{
				cout << "文件打开失败。" << endl;
				return 0;
			}
			string buf;					//定义字符串
			getline(squaretxt, buf);	//利用getline函数，将一行内容放入字符串中
			squarethreshold = stof(buf)/100;
			cout << "squarethreshold" << squarethreshold << endl;
			squaretxt.close();
            
			totalnum++;//红外每检测到一次总数加一
			fstream totaltxt;
			totaltxt.open("C:/text/total.txt", ios::out);
			if (!totaltxt.is_open())
			{
				cout << "文件打开失败。" << endl;
				return 0;
			}
			string trans1 = to_string(totalnum);	//将总数由int转化为string类型
			totaltxt << trans1;
			totaltxt.close();

			percent1 = percent2 = percent3 = 0;//每次开始检测之前把面积占比清零，之后取几次检测到的最大值。
			//三个线
			key1 = 1;
			key2 = 1;
			key3 = 1;
			waitKey(110);//相机拍摄0.10s
			key1 = 0;
			key2 = 0;//
			key3 = 0;
			waitKey(800);//等待模型处理完毕
			thread judge1(judgethread);
			judge1.detach();
		}
		mid = right;
	}
}

int main() {
//设置单个图片
//std::string img_path = "111.jpg";
//Mat image = cv::imread(img_path);
	
	//下面代码用来计算面积转换的权重矩阵
	double mid;
	for (int i = 0; i < c.rows; i++)     //构建权重矩阵
	{
		for (int j = 0; j < c.cols; j++)
		{
			if (j < 65)
			{
				mid = asin((64.5 - j) / 75);

				c.at<float>(i, j) = 1 / cos(mid);//cout << c.at<float>(i, j) <<endl;
				all += c.at<float>(i, j);
}
			else
			{
				mid = asin((j - 64.5) / 75);

				c.at<float>(i, j) = 1 / cos(mid); // cout << c.at<float>(i, j) << endl;
				all += c.at<float>(i, j);
			}
				}
			}



	cout << "start process";
	thread t1(camerathread1);
	thread t2 (camerathread2);
	thread t3 (camerathread3);
	thread tk(keythread);
	t1.join();
	tk.join();
	t2.join();
	t3.join();

	cout << "end process";
waitKey(100);
return 0;
}
