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
		/*
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
		*/

		// 实际机器人流程
		// Step.01 - 机器人启动，通知PC端准备完毕，PC端询问操作人员是否正式开始，确认开始后PC发送开始flag
		//		需要flag：
		//			-机器人端设置(output_bit_registers0_to_31)：
		//					FLAG_START_READY
		//			-PC端设置(input_bit_registers0_to_31)
		//					FLAG_UR_START

		// Step.02 - 机器人运动至料仓，抓取刀片（是否有办法判断抓取成功与否），运动至图像采集姿态，通知PC并进入等待，PC进行处理
		//		需要flag
		//			-机器人端设置(output_bit_registers0_to_31):
		//					FLAG_PC_IMAGE
		//			-PC端设置(input_bit_registers0_to_31)
		//					NULL

		// Step.03 - PC进行图像处理，定位刀片，计算目标路径点（最多8个路径点，但是也许多线程技术可以让路点循环执行），计算完成后，
		//           PC上传数据至UR，并通知机器人继续下一步动作，之后进入等待。注意此时需要保存当前拍摄的图片 A
		//		
		// Step.04 - 机器人启动加工运动
		// Step.05 - 机器人加工运动完成，通知PC并运动至原图像测量位置，进入等待
		// Step.06 - PC得到完成消息，进入检测流程，拍摄图片B，与图片A进行比较，从而得到刀片熔覆层厚度
		// Step.07 - PC通知机器人检测完成，机器人将成品放入料库，将废品放入回收区，进入下个循环


	}

	

	// 断开连接
	my_RTDE.RTDE_Send_Stop();

	// 退出Python的环境
	Py_Finalize();
    return 0;
}

