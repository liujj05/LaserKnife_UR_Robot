// UR_Control_test01.cpp: 定义控制台应用程序的入口点。
//
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

	// 初始化一个类
	ur_RTDE my_RTDE;
	
	my_RTDE.RTDE_Initialize();

	// 建立连接
	my_RTDE.RTDE_Send_Start();

	
	UINT32 input_ctrl_flag = 0;
	while (true)
	{
		// 测试发送是否正常 - 已经正常 - 注释掉
		/*
		cout << "========= Send Test ==========" << endl;
		cout << "Input control flag: " << endl;
		cin >> input_ctrl_flag;
		
		// 只允许输入 0 或者 1
		if (input_ctrl_flag > 1)
			break;

		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
		
		Delay(5 * 1000);
		*/
		// 测试接收是否正常 - 这种方法测试发现不正常
		//cout << "========= Receive Test ==========" << endl;
		//cout << "Input 1 to start a receive, other to exit:" << endl;
		//cin >> input_ctrl_flag;
		//if (input_ctrl_flag != 1)
		//	break;

		// 但是这种发现正常了
		// my_RTDE.RTDE_Recv_BIT32();
		// Delay(1 * 1000);
		// cout << "UR Control Flag --- " << my_RTDE.state_res << endl;
		// if (my_RTDE.state_res == 0)
		// 	break;

		// -- 以下将接收作为等待 进行测试 -- ：测试通过！（配合的程序是laser_DEMO3.urp）
		while (true)
		{
			 my_RTDE.RTDE_Recv_BIT32();
			 Delay(1 * 1000);
			 cout << "UR Control Flag --- " << my_RTDE.state_res << endl;
			 if (my_RTDE.state_res == 1)
			 	break;
		}

		cout << "========= Send Test ==========" << endl;
		cout << "Input control flag: " << endl;
		cin >> input_ctrl_flag;
		// 只允许输入 0 或者 1
		if (input_ctrl_flag > 1)
			break;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
		Delay(1 * 1000);

		while (true)
		{
			my_RTDE.RTDE_Recv_BIT32();
			Delay(1 * 1000);
			cout << "UR Control Flag --- " << my_RTDE.state_res << endl;
			if (my_RTDE.state_res == 0)
				break;
		}

		cout << "========= Send Test ==========" << endl;
		cout << "Input control flag: " << endl;
		cin >> input_ctrl_flag;
		// 只允许输入 0 或者 1
		if (input_ctrl_flag > 1)
			break;
		my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
		Delay(1 * 1000);
	}

	

	// 断开连接
	my_RTDE.RTDE_Send_Stop();

	// 退出Python的环境
	Py_Finalize();
    return 0;
}

