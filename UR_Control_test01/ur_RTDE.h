#pragma once
#include <Python.h>
#include <string>


#define INPUT_STR_LENGTH 64
#define UR_IP "192.168.1.6"


using namespace std;

// 简化代码的宏定义
class ur_RTDE
{
public:
	ur_RTDE();
	~ur_RTDE();

	// ================= 变量 ========================
	
	string IP_str;
	string xml_config_file_name;
	
	// ==============================================



	// ================= 函数 ========================

	// 初始化 RTDE 连接，包括：
	// 1. 建立与目标机器人30004端口的连接
	// 2. 配置目标机器人的发送与接收内容
	// 注意初始化并不负责启动发送与停止发送
	void RTDE_Initialize(void);

	// 启动RTDE的发送与接收
	int RTDE_Send_Start(void);

	// 停止RTDE的发送与接收
	void RTDE_Send_Stop(void);

	// 发送特定的变量到机器人
	// 这部分的函数需要根据具体变量进行决定，不过可以设置多个
	void RTDE_Send_BIT32(UINT32 BIT32_Value);
	void RTDE_Send_POINT(double *pt_seq, int pt_num);

	// 从机器人接收数据
	void RTDE_Recv_BIT32(void);
	UINT32 state_res; // 接收数据的结果
	// ================================================

private:
	// 负责控制与UR连接的通断
	PyObject * pInstance_con;

	// 和配置文件有关，这些变量将会参与到后续的操作当中
	PyObject *setp;
	PyObject *ctrlflag;

	PyObject *state;
	PyObject *state_output;
	

	// 用于设置的字符串
	char input_str[INPUT_STR_LENGTH];
};





