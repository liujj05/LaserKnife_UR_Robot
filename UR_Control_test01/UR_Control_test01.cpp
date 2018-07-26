// UR_Control_test01.cpp: 定义控制台应用程序的入口点。
// 本程序对应机器人的laser_DEMO4.urp
#pragma once
#include "stdafx.h"
#include "ur_RTDE.h"
#include <ctime>

using namespace std;

// 用于调试
void Delay(int time)//time*1000为秒数 
{
	clock_t   now = clock();
	while (clock() - now   <   time);
}


int main()
{
	// 启动Python的环境
	Py_Initialize();
	// 启动与机器人的连接并逐个测试功能

	// 初始化一个ur_RTDE类
	ur_RTDE my_RTDE;
	
	my_RTDE.RTDE_Initialize();

	// 建立连接
	my_RTDE.RTDE_Send_Start();

	
	UINT32 input_ctrl_flag = 0;
	UINT32 gui_ctrl_int = 0;

	// Step1 机器人从待命状态启动
	input_ctrl_flag = 1;
	cout << "Start Robot? (1--YES, other--NO)" << endl;
	cin >> input_ctrl_flag;

	if (input_ctrl_flag != 1)
	{
		input_ctrl_flag = 0;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
		Delay(1 * 1000);
		
		my_RTDE.RTDE_Send_Stop();
		
		// 退出Python的环境
		Py_Finalize();

		return 0;
	}

	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag); // 发送启动指令：1

	// 等待机器人通知：移动到准备抓取的位置
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		Delay(2 * 1000);
		cout << "UR Control Waiting... Flag 0 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res == 1)
		{
			cout << "UR Control Waiting... break" << endl;
			break;
		}
	}
	cout << "Robot has arrived at CATCH position" << endl;

	cout << "Take the knife to CAMERA position? (1--YES, other--NO)" << endl;
	cin >> gui_ctrl_int;

	if (gui_ctrl_int != 1)
	{
		input_ctrl_flag = 0;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
		Delay(1 * 1000);
		my_RTDE.RTDE_Send_Stop();

		// 退出Python的环境
		Py_Finalize();

		return 0;
	}

	input_ctrl_flag = 3;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag); // 发送移动至图像采集位置命令 + 进入循环命令 ：2 + 1

	// 等待机器人通知：到达图像采集位置
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		Delay(2 * 1000);
		cout << "UR Control Waiting... Flag 1 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res == 2)
		{
			cout << "UR Control Waiting... break" << endl;
			break;
		}
	}

	cout << "Robot has arrived at CAMERA position" << endl;
	
	
	// =================== 相机采集流程 ======================= 采集基准图像

	// ======================================================
	cout << "Grabbing image..." << endl;
	Delay(3 * 1000);
	cout << "Image grabbed, free moving" << endl;


	input_ctrl_flag = 4;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);	// 发送自由移动命令：4

	// 等待机器人自由运动完毕
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		Delay(2 * 1000);
		cout << "UR Control Waiting... Flag 2 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res == 4)
		{
			cout << "UR Control Waiting... break" << endl;
			break;
		}
	}

	// ====================== 进入大循环 ==========================

	while (true)
	{
		cout << "Take the knife to CAMERA position? (1--YES, other--NO)" << endl;
		cin >> gui_ctrl_int;

		if (gui_ctrl_int != 1)
		{
			// 此时机器人自由运动完成，正在执行：wait: read_input_boolean_register(1) == true
			input_ctrl_flag = 2;
			my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
			Delay(1 * 1000);
			break;
		}

		input_ctrl_flag = 3;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag); // 发送移动至图像采集位置命令 + 进入循环命令 ：2 + 1

		// 等待机器人通知：到达图像采集位置
		while (true)
		{
			my_RTDE.RTDE_Recv_BIT32();
			Delay(2 * 1000);
			cout << "UR Control Waiting... Flag 1 --- " << my_RTDE.state_res << endl;
			if (my_RTDE.state_res == 2)
			{
				cout << "UR Control Waiting... break" << endl;
				break;
			}
		}
		cout << "Robot has arrived at CAMERA position" << endl;
		
		cout << "Grabbing image..." << endl;
		// =================== 相机采集流程 =======================

		// ======================================================
		Delay(3 * 1000);
		// =================== ICP匹配流程 =======================

		// ======================================================
		
		cout << "Image grabbed and processed, free moving" << endl;


		input_ctrl_flag = 4;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag); // 发送自由移动命令：4

		// 等待机器人自由运动完毕
		while (true)
		{
			my_RTDE.RTDE_Recv_BIT32();
			Delay(2 * 1000);
			cout << "UR Control Waiting... Flag 2 --- " << my_RTDE.state_res << endl;
			if (my_RTDE.state_res == 4)
			{
				cout << "UR Control Waiting... break" << endl;
				break;
			}
		}

	}

	// 确定机器人从循环中退出
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		Delay(2 * 1000);
		cout << "UR Control Waiting... Flag 1 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res == 0)
		{
			cout << "UR Control Waiting... break" << endl;
			break;
		}
	}

	input_ctrl_flag = 0;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
	Delay(1 * 1000);

	my_RTDE.RTDE_Send_Stop();
	// 退出Python的环境
	Py_Finalize();

	return 0;
}

