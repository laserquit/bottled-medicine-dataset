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

#define IS_SNAP_SINGLE 1//ѡ��֡���������ɼ�ģʽ

std::string model_path = "test_fp16.trt";//����ģ��·��
BYTE* pImageBuffer;

Mat c=Mat::zeros(60,130,CV_32FC1);//�������Ȩ�ؾ���
float all;//�����
int key1=0,key2=0,key3=0;//��������
int badbottle = 0;//��ƿ�Ƴ��ź�
double percent1=0,percent2=0,percent3=0;//��������ͷ��������ռ��
float 
squarethreshold=0.95;//�������ֵ��Ĭ��Ϊ90%
int totalnum = 0;//��ƿ��
int badnum;//��ƿ��

////����ͷ�������ú���
static void __stdcall OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{


	std::cout << " succeed�� " << std::endl;

	if (pFrame->status == 0)  // 0 ������֡
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

////��������ͷ����
int camera()
{
	GX_DEV_HANDLE m_hDevice;  //USB ������������
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[2048 * 1536];   // �Ҷ�  
	GXInitLib(); //��ʼ����
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//����豸����  //������û�ָ����ʱʱ���ڳɹ�ö�ٵ��豸������������
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//��ȡ�豸��Ϣ

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(1, &m_hDevice);		//�����������ע���Ѿ�����Ĭ�ϲ�����

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 2048);		//����ͼ����2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 1536);		//����ͼ��߶�1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//���òɼ�ģʽΪ�����ɼ�
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//���òɼ��ٶȣ���Χ(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//��������,���淶Χ(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 30000);	//�ع�ʱ��30ms
#if IS_SNAP_SINGLE //�ɼ���֡ģʽ
	GX_FRAME_DATA frameData;
	frameData.pImgBuf = pImageBuffer;
	frameData.nStatus = -1;
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	do
	{
		status = GXGetImage(m_hDevice, &frameData, 5);
	} while (frameData.nStatus != 0);

	
	
	//�ɶ�pImageBuffer ����ͼ���������ʾ����
		//������������������
	
		
		
		//����ͼ�������
		status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
		
#else//�����ɼ�ģʽ								
							//pUserParamָ���û����ڻص���������ʹ�õ�˽������ָ��
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//ע��ص�����

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);
	
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//ע���ص�����
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

//ר�����ڷ����̵߳ĺ���,����һ���ַ�a���ڵ�Ƭ���ж��Ƿ���յ�����
void senda(){
	serial_writer Serial_writer;
	Serial_writer.set_serial("COM3", 9600);//���÷��Ͷ˿�����Ϊ��COM3�����÷��Ͳ�����Ϊ��9600
	std::this_thread::sleep_for(std::chrono::microseconds(4850000));//�ȴ�4.85s���ʹ���
	cout << "׼������" << endl;
	Serial_writer.send_data('a');//����1�ֽ����ݣ�'a'	
}
//����Ϊ��ʼ�����ڵĺ���
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
	this->hCom = CreateFile(COMx, //COMx��    
		GENERIC_READ | GENERIC_WRITE, //�������д    
		0, //��ռ��ʽ    
		NULL,
		OPEN_EXISTING, //�򿪶����Ǵ���     
		0, //�ص���ʽFILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED  (ͬ����ʽ����Ϊ0)
		NULL);
	if (this->hCom == INVALID_HANDLE_VALUE)
	{
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY | FOREGROUND_RED);
		std::cout << "-------------------------  �޷��򿪴��ڣ�  -------------------------" << std::endl;
		SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY |
			FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
		std::cout << std::endl;
		return FALSE;
	}
	SetupComm(this->hCom, 1024, 1024); //���뻺����������������Ĵ�С����1024 

								 //�趨��д��ʱ 
								 /*COMMTIMEOUTS TimeOuts;
								 TimeOuts.ReadIntervalTimeout=1000;
								 TimeOuts.ReadTotalTimeoutMultiplier=500;
								 TimeOuts.ReadTotalTimeoutConstant=5000; //�趨д��ʱ
								 TimeOuts.WriteTotalTimeoutMultiplier=500;
								 TimeOuts.WriteTotalTimeoutConstant = 2000;
								 SetCommTimeouts(hCom, &TimeOuts); //���ó�ʱ
								 */
	DCB dcb;
	GetCommState(this->hCom, &dcb);
	dcb.BaudRate = BaudRate;		//���ò�����ΪBaudRate
	dcb.ByteSize = 8;					//ÿ���ֽ���8λ 
	dcb.Parity = NOPARITY;			//����żУ��λ 
	dcb.StopBits = ONESTOPBIT;		//һ��ֹͣλ
	SetCommState(this->hCom, &dcb);		//���ò�����hCom
	PurgeComm(this->hCom, PURGE_TXCLEAR | PURGE_RXCLEAR);//��ջ�����		//PURGE_TXABORT �ж�����д�������������أ���ʹд������û����ɡ�
												   //PURGE_RXABORT �ж����ж��������������أ���ʹ��������û����ɡ�
												   //PURGE_TXCLEAR ������������ 
												   //PURGE_RXCLEAR ������뻺����  
	return TRUE;
}

int serial_writer::serial_write(char lpOutBuffer[]) {//ͬ��д����
	DWORD dwBytesWrite = sizeof(lpOutBuffer);
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;
	ClearCommError(this->hCom, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(this->hCom, lpOutBuffer, dwBytesWrite, &dwBytesWrite, NULL);
	if (!bWriteStat)
	{
		printf("д����ʧ��!\n");
		return FALSE;
	}
	PurgeComm(this->hCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);
	return TRUE;
}

void serial_writer::Serial_close(void) {//�رմ���
	CloseHandle(this->hCom);
}

//�����ǲ��Դ���
static const char* cocolabels[] = { "good","bad" };//�������ݼ���ǩ�����밴��ѵ����˳��
yolo::Image cvimg(const cv::Mat& image) { return yolo::Image(image.data, image.cols, image.rows); }//����cvimg

mutex mutex_1;//���뻥��������ֹ���ݵ��ó�ͻ
mutex mutex_2;
mutex mutex_3;

int camerathread1()
{
	mutex_1.lock();
	Mat image1;
	double temp=0;
	int k;//�����ƶ������м���
	float confidence_threshold = 0.7f;
	float nms_threshold = 0.8f;
	auto yolo1 = yolo::load(model_path, yolo::Type::V8Seg, confidence_threshold, nms_threshold);
	//�����ǳ�ʼ��yoloģ��

	GX_DEV_HANDLE m_hDevice;  //USB ������������
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682 ];   // �Ҷ�  
	GXInitLib(); //��ʼ����
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//����豸����  //������û�ָ����ʱʱ���ڳɹ�ö�ٵ��豸������������
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//��ȡ�豸��Ϣ
	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(1, &m_hDevice);		//�����������ע���Ѿ�����Ĭ�ϲ�����
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//����ͼ����2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//����ͼ��߶�1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//���òɼ�ģʽΪ�����ɼ�
	

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//���òɼ��ٶȣ���Χ(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//��������,���淶Χ(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//�ع�ʱ��30ms
#if IS_SNAP_SINGLE //�ɼ���֡ģʽ
	cout << "single1 "<< endl;
	while (1) {
		if (key1 == 1) { //��������Ķ������ü�����ƵĻ���ע�͵����if
			//key = 0;
			Mat seg;//�����յ�segmentͼ��
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
			//�ɶ�pImageBuffer ����ͼ���������ʾ����
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
		   //��ʼyoloԤ����ò��ֲ�����segmentͼ��
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

			//��������㷨
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //��ֵ��ͻ��ҩƬ�������
				seg.release();    // �ͷ��ڴ�ռ�
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //�޸�ͼƬ�ߴ�ʹ��������
				Mat crop = resized(Range(0, 60), Range(9, 139));     // �����޹�����
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //����Ŀ�͸�
				int up_height = output.rows;
				
				int i; //��¼����λ��
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //����ȫ1����

				//cout << "�������" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //������ȱ���������
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//ȡ���μ�⵽��������ֵ
				if (percent1 < temp)
					percent1 = temp;
				cout << "����ͷ1�������ռ�ȣ�" << percent1 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "����" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;
					
					cout << "���ϸ�ҩƷ";
					
				}*/
				
				//destroyAllWindows();
			}
			else
			{
				percent1 = 0.9;
				//cout << "û�м�⵽ҩƿ1" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
		//����ͼ�������
	    mutex_1.unlock();
		status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
		
#else//�����ɼ�ģʽ								
							//pUserParamָ���û����ڻص���������ʹ�õ�˽������ָ��
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//ע��ص�����

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);
	
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//ע���ص�����
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
	//�����ǳ�ʼ��yoloģ��
	double temp;

	GX_DEV_HANDLE m_hDevice;  //USB ������������
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682];   // �Ҷ�  
	GXInitLib(); //��ʼ����
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//����豸����  //������û�ָ����ʱʱ���ڳɹ�ö�ٵ��豸������������
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//��ȡ�豸��Ϣ

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(2, &m_hDevice);		//�����������ע���Ѿ�����Ĭ�ϲ�����
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//����ͼ����2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//����ͼ��߶�1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//���òɼ�ģʽΪ�����ɼ�
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//���òɼ��ٶȣ���Χ(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//��������,���淶Χ(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//�ع�ʱ��30ms
#if IS_SNAP_SINGLE //�ɼ���֡ģʽ
	cout << "single2 " << endl;
	while (1) {
		if (key2 == 1) { //��������Ķ������ü�����ƵĻ���ע�͵����if
			//key = 0;
			Mat seg;//�����յ�segmentͼ��
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
			//�ɶ�pImageBuffer ����ͼ���������ʾ����
			Mat frame2(682, 816, CV_8UC1, pImageBuffer);
			//imshow("test", frame);
			cvtColor(frame2, frame2, COLOR_GRAY2RGB);
			//cout << frame.type() << endl;
			namedWindow("test2", WINDOW_NORMAL);
			imshow("test2", frame2);
			waitKey(2);
			//image1 = cv::imread("2023-07-12_15_42_52_082.jpg");
			//cout << image1.type() << endl;
		   //��ʼyoloԤ����ò��ֲ�����segmentͼ��
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

			//��������㷨
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //��ֵ��ͻ��ҩƬ�������
				seg.release();    // �ͷ��ڴ�ռ�
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //�޸�ͼƬ�ߴ�ʹ��������
				Mat crop = resized(Range(0, 60), Range(9, 139));     // �����޹�����
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output2", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //����Ŀ�͸�
				int up_height = output.rows;

				int i; //��¼����λ��
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //����ȫ1����

				//cout << "�������" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //������ȱ���������
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//ȡ���μ�⵽��������ֵ
				if (percent2 < temp)
					percent2 = temp;
				cout << "����ͷ2�������ռ�ȣ�" << percent2 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "����" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;

					cout << "���ϸ�ҩƷ";

				}*/

				//destroyAllWindows();
			}
			else
			{
				percent2 = 0.9;
				//cout << "û�м�⵽ҩƿ2" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
	//����ͼ�������
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	mutex_2.unlock();

#else//�����ɼ�ģʽ								
							//pUserParamָ���û����ڻص���������ʹ�õ�˽������ָ��
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//ע��ص�����

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//ע���ص�����
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
	//�����ǳ�ʼ��yoloģ��
	double temp;

	GX_DEV_HANDLE m_hDevice;  //USB ������������
	GX_STATUS status = GX_STATUS_SUCCESS;
	pImageBuffer = new BYTE[816 * 682];   // �Ҷ�  
	GXInitLib(); //��ʼ����
	uint32_t m_nNumberDevice;
	GXUpdateDeviceList(&m_nNumberDevice, 1000);		//����豸����  //������û�ָ����ʱʱ���ڳɹ�ö�ٵ��豸������������
	GX_DEVICE_BASE_INFO* baseinfo = new GX_DEVICE_BASE_INFO[m_nNumberDevice];
	size_t nSize = m_nNumberDevice * sizeof(GX_DEVICE_BASE_INFO);		//��ȡ�豸��Ϣ

	status = GXGetAllDeviceBaseInfo(baseinfo, &nSize);
	status = GXOpenDeviceByIndex(3, &m_hDevice);		//�����������ע���Ѿ�����Ĭ�ϲ�����
	//status = GXOpenDeviceByIndex(2, &m_hDevice);

	status = GXSetInt(m_hDevice, GX_INT_WIDTH, 816);		//����ͼ����2048
	status = GXSetInt(m_hDevice, GX_INT_HEIGHT, 682);		//����ͼ��߶�1536
	status = GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);	//���òɼ�ģʽΪ�����ɼ�
	//status = GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE,GX_TRIGGER_MODE_ON);

	status = GXSetInt(m_hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 12);	//���òɼ��ٶȣ���Χ(0 - 12)

	status = GXSetInt(m_hDevice, GX_INT_GAIN, 8);	//��������,���淶Χ(0-63)
	status = GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, 10000);	//�ع�ʱ��30ms
#if IS_SNAP_SINGLE //�ɼ���֡ģʽ
	cout << "single3 " << endl;
	while (1) {
		if (key3 == 1) { //��������Ķ������ü�����ƵĻ���ע�͵����if
			//key = 0;
			Mat seg;//�����յ�segmentͼ��
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
			//�ɶ�pImageBuffer ����ͼ���������ʾ����
			Mat frame(682, 816, CV_8UC1, pImageBuffer);
			//imshow("test", frame);
			cvtColor(frame, frame, COLOR_GRAY2RGB);
			//cout << frame.type() << endl;
			namedWindow("test3", WINDOW_NORMAL);
			imshow("test3", frame);
			waitKey(2);
			//image1 = cv::imread("2023-07-12_15_42_52_082.jpg");
			//cout << image1.type() << endl;
		   //��ʼyoloԤ����ò��ֲ�����segmentͼ��
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

			//��������㷨
			if (!seg.empty())
			{
				cout << "start" << endl;
				Mat imgred_out, resized, output;
				threshold(seg, imgred_out, 150, 255, THRESH_BINARY);    //��ֵ��ͻ��ҩƬ�������
				seg.release();    // �ͷ��ڴ�ռ�
				cv::resize(imgred_out, resized, cv::Size(150, 60), 0, 0, cv::INTER_LINEAR); //�޸�ͼƬ�ߴ�ʹ��������
				Mat crop = resized(Range(0, 60), Range(9, 139));     // �����޹�����
				threshold(crop, output, 1, 0, THRESH_TRUNC);
				imshow("output3", output);
				output.convertTo(output, CV_32FC1);
				int up_width = output.cols;         //����Ŀ�͸�
				int up_height = output.rows;

				int i; //��¼����λ��
				double mid;
				//Mat c = cv::Mat::zeros(up_height, up_width, CV_32FC1);  //����ȫ1����

				//cout << "�������" << all << "  " << endl;
				////imshow("cout", c);
				cv::Mat m = cv::Mat_<float>(up_height, up_width);
				double b, actual = 0;
				cv::Mat out = cv::Mat_<float>(up_height, up_width);
				for (i = 0; i < up_height; i++)         //������ȱ���������
				{
					for (int j = 0; j < up_width; j++)
					{


						b = cv::saturate_cast<float>(output.at<float>(i, j) * c.at<float>(i, j));
						out.at<float>(i, j) = b;
						actual += b;

					}
				}

				temp = actual / all;//ȡ���μ�⵽��������ֵ
				if (percent3 < temp)
					percent3 = temp;
				cout << "����ͷ3�������ռ�ȣ�" << percent3 << "%" << endl;
				//waitKey(5);
			/*	if (percent1 > 0.9) {
					cout << "����" << endl;
					k = 0;
				}
				if(percent1<0.9)
				{
					badbottle = 1;

					cout << "���ϸ�ҩƷ";

				}*/

				//destroyAllWindows();
			}
			else
			{
				percent3 = 0.9;
				//cout << "û�м�⵽ҩƿ3" << endl;
				temp = 0;
				//k = 0;
			}
			//printf(" %d objects\n", (int)objs.size());
			//cout << "1" << endl;
		}
	}
	//����ͼ�������
	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	mutex_3.unlock();

#else//�����ɼ�ģʽ								
							//pUserParamָ���û����ڻص���������ʹ�õ�˽������ָ��
	status = GXRegisterCaptureCallback(m_hDevice, NULL, OnFrameCallbackFun);	//ע��ص�����

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START);
	cout << 1;

	while (1);

	status = GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP);
	status = GXUnregisterCaptureCallback(m_hDevice);	//ע���ص�����
#endif
	status = GXCloseDevice(m_hDevice);
	delete[]pImageBuffer;
	GXCloseLib();
	return 0;
}

int judgethread() //�ж��Ƿ�Ϊ��ƿ����д��qt��ƿ�������̣߳�ʹ��keythread��
{
	double percent = 0; //�������ռ��
	cout << "judege ��ʼ";
	percent = (percent1 + percent2 + percent3) / 3;
	cout << endl << "�������ռ�� " << percent << endl;

	//�ж����� �е�����ֵ����������ͷ����⵽
	if ((percent1 <= squarethreshold || percent2 <= squarethreshold || percent3 <= squarethreshold))
	{
		thread s1(senda);//�����ж�������ȴ�x�뷢��
		s1.detach();

		badbottle++;//��ƿ����һ
		percent = 0;//���ź����

		//��ƿ��д��
		fstream badtxt;
		badtxt.open("C:/text/nopass.txt", ios::out);
		if (!badtxt.is_open())
		{
			cout << "badtxt�ļ���ʧ�ܡ�" << endl;
			return 0;
		}
		string trans2 = to_string(badbottle);	//��������intת��Ϊstring����
		badtxt << trans2;
		badtxt.close();
	}
}

int keythread()//���������߳� ��qt��ȡ��ֵ��д��������������߳���
{
	cout << "keythread begin ";
	int right = 0;
	int mid = 0;
	while (1) {
		right = GetKeyState(0x42) & 0x8000;
		if (right && (mid == 0)) {
			cout << "���ⴥ��\n";

			//����ÿ��⵽һ�ξ�ʹ���ļ�������ȡ�����ֵ
			fstream squaretxt;
			squaretxt.open("C:/text/square.txt", ios::in);
			if (!squaretxt.is_open())
			{
				cout << "�ļ���ʧ�ܡ�" << endl;
				return 0;
			}
			string buf;					//�����ַ���
			getline(squaretxt, buf);	//����getline��������һ�����ݷ����ַ�����
			squarethreshold = stof(buf)/100;
			cout << "squarethreshold" << squarethreshold << endl;
			squaretxt.close();
            
			totalnum++;//����ÿ��⵽һ��������һ
			fstream totaltxt;
			totaltxt.open("C:/text/total.txt", ios::out);
			if (!totaltxt.is_open())
			{
				cout << "�ļ���ʧ�ܡ�" << endl;
				return 0;
			}
			string trans1 = to_string(totalnum);	//��������intת��Ϊstring����
			totaltxt << trans1;
			totaltxt.close();

			percent1 = percent2 = percent3 = 0;//ÿ�ο�ʼ���֮ǰ�����ռ�����㣬֮��ȡ���μ�⵽�����ֵ��
			//������
			key1 = 1;
			key2 = 1;
			key3 = 1;
			waitKey(110);//�������0.10s
			key1 = 0;
			key2 = 0;//
			key3 = 0;
			waitKey(800);//�ȴ�ģ�ʹ������
			thread judge1(judgethread);
			judge1.detach();
		}
		mid = right;
	}
}

int main() {
//���õ���ͼƬ
//std::string img_path = "111.jpg";
//Mat image = cv::imread(img_path);
	
	//������������������ת����Ȩ�ؾ���
	double mid;
	for (int i = 0; i < c.rows; i++)     //����Ȩ�ؾ���
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
