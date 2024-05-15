#pragma once
#pragma once
#include"windows.h"
#include <TCHAR.H>
#include <string.h>
#include <iostream>
#include "atlstr.h"
#include <vector>
class serial_writer
{
public:
	/** @brief ���ô��ڲ�����
	@param port_name �������ƣ����<�豸������>���ҵ������ӵ�Ƭ���Ĵ��ڣ�����port_name = "COM5";
	@param baud_rate �����ʡ�
	*/
	void set_serial(CString port_name, int baud_rate = 9600);
	/** @brief ���͵��ֽ����ݣ����øú�����������
	@param input_data_1_byte �ַ���(8λ)���ݡ�
	*/
	void send_data(char input_data_1_byte);
	/** @brief ����һ�����ݰ������øú����������ͣ���input_data[0]��ʼ���͡�
	@param input_data �����͵��ַ�������
	*/
private:
	LPCWSTR COM_port = _T("COM3");
	HANDLE hCom;
	int baud_rate = 9600;
	int serial_open(LPCWSTR COMx, int BaudRate);
	int serial_write(char lpOutBuffer[]);
	void Serial_close(void);
};