// UR_Control_test01.cpp: 定义控制台应用程序的入口点。
// 本程序对应机器人的laser_DEMO4.urp
#pragma once
#include "stdafx.h"
#include "ur_RTDE.h"
#include "CBasler_Single_Cam_Easy.h"
#include "ICP2D.h"
#include <ctime>
#include "CRobotTransE2H.h"

#include <windows.h>					// 精确计时

using namespace std;
using namespace cv;

// == 精确计时宏定义 ==
#define TIMER_START QueryPerformanceCounter(&start_t)
#define TIMER_STOP QueryPerformanceCounter(&stop_t);\
exe_time = 1e3*(stop_t.QuadPart - start_t.QuadPart) / freq.QuadPart
// ==================

// 用于调试
void Delay(int time)//time*1000为秒数 
{
	clock_t   now = clock();
	while (clock() - now   <   time);
}


int main()
{
	//=============== Debug ================= // 该版本对应UR5上面的 laser_pose_seq_download01.urp
	// 启动Python的环境
	Py_Initialize();
	// 初始化一个ur_RTDE类
	ur_RTDE my_RTDE2;
	my_RTDE2.RTDE_Initialize();			// RTDE 通信初始化
	my_RTDE2.RTDE_Send_Start();			// 建立 RTDE 连接
	// 初始化标志位
	my_RTDE2.RTDE_Send_BIT32(0);
	Delay(100);
	//while (1)
	//{
	//	my_RTDE2.RTDE_Recv_BIT32();				// 接收机器人发送的标志位
	//	Delay(100);
	//}
	// 读入示教点
	CRobotTransE2H my_robot_trans2;
	my_robot_trans2.Load_Origin_Teach_Pts("teach_pose.txt");
	// 生成新示教点并发送
	double new_teach_pose[6];	// 6自由度
	int pt_num = 4;				// 4示教点
	double y_offset = 0.02;		// UR端的单位貌似是m
	// 发送示教点
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			new_teach_pose[j] = my_robot_trans2.Origin_Teach_Pts.at<double>(i, j);
		}
		new_teach_pose[1] = new_teach_pose[1] + y_offset;
		my_RTDE2.RTDE_Send_POINT(new_teach_pose, 6); // 发送新示教点
		Delay(100);									// 额外给点时间确保发送完成
		my_RTDE2.RTDE_Send_BIT32(0x01<<i);			// 通知机器人收第i个点
		cout << "Send point " << i << " over, waiting..." << endl;
		while (1)
		{
			my_RTDE2.RTDE_Recv_BIT32();				// 接收机器人发送的标志位
			Delay(100);
			if (my_RTDE2.state_res & (0x01 << i))	// 提取对应位数，是1证明赋值完毕
				break;
		}
		cout << "---- point " << i << " received" << endl;
	}
	my_RTDE2.RTDE_Send_Stop();
	Py_Finalize();
	getchar();
	return 0;
	//=======================================
	//
	//  ----  正式程序  ----
	//
	//=======================================
	// == 精确计时 ==
	LARGE_INTEGER freq;
	LARGE_INTEGER start_t, stop_t;
	double exe_time;
	QueryPerformanceFrequency(&freq);
	//==================================================================================
	// 定义两个矩阵
	Mat Image_Ref_Knife;
	Mat Image_New_Knife;

	// 启动Python的环境
	Py_Initialize();
	// 启动与机器人的连接并逐个测试功能

	// 初始化一个ur_RTDE类
	ur_RTDE my_RTDE;
	// 初始化一个CBasler_Single_Cam_Easy类
	CBasler_Single_Cam_Easy my_Cam;
	// 初始化一个ICP2D类
	ICP2D my_2D_ICP;
	my_2D_ICP.iter_max = 1000;
	my_2D_ICP.iter_thresh = 0.001;
	my_2D_ICP.ref_down_sample_rate = 5;
	my_2D_ICP.new_down_sample_rate = 5;
	my_2D_ICP.binary_thresh = 50;

	// 初始化一个E2H类
	CRobotTransE2H my_robot_trans;

	// 初始化各个功能模块
	my_RTDE.RTDE_Initialize();			// RTDE 通信初始化
	my_Cam.Init_Cam();					// Basler 相机初始化
	// == 采集 new 图像（新匹配）来初始化 Image_New_Knife ==
	my_Cam.Cap_single_image();
	Image_New_Knife = my_Cam.Current_Mat.clone();
	// ======================================================

	// ==== 读入初始化数据 ====
	// #1 读入基准图像
	try
	{
		Image_Ref_Knife = imread("base_knife.bmp", IMREAD_UNCHANGED);
	}
	catch(...)
	{
		cout << "# Error: Cannot open file: base_knife.bmp!" << endl;
		Py_Finalize();
		getchar();
		return 0;
	}
	my_2D_ICP.Image_ref = Image_Ref_Knife;
	// #2 读入示教点
	my_robot_trans.Load_Origin_Teach_Pts("teach_pose.txt");
	

	// 建立 RTDE 连接
	my_RTDE.RTDE_Send_Start();



	UINT32 input_ctrl_flag = 0;
	UINT32 gui_ctrl_int = 0;

	// Step1 机器人从待命状态启动
	input_ctrl_flag = 1;
	cout << "# Start Robot? (1--YES, other--NO)" << endl;
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

	// Step2 等待机器人通知：移动到测量位置
	// 第一版的 Delay 延时程序会出现一个问题：无法及时与机器人进行交互（机器人置位后相当长时间内PC探测不到标志位改变）
	// 本次首先测试快速连续调用接收函数看是否能提高响应速度

	TIMER_START; // 计时-观测响应时间
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		// Delay(2 * 1000); 
		//cout << "UR Control Waiting... Flag 0 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res == 1)
		{
			cout << "++++ Flag1 change detected! " << endl;
			break;
		}
	}
	TIMER_STOP;
	cout << "++++ Robot has arrived at Photo position in " << exe_time << " ms" << endl;
	
	// Step3 拍摄+测量
	
	cout << "# Grabbing image..." << endl;
	
	// == 采集 new 图像（新匹配）来初始化 Image_New_Knife ==
	my_Cam.Cap_single_image();
	// 首次采集生成存储空间后，可以用copyTo，不额外重新分配空间
	my_Cam.Current_Mat.copyTo(Image_New_Knife);
	// ======================================================
	
	cout << "# Image grabbed, matching..." << endl;

	// ICP 匹配	
	// =================== ICP匹配流程 =======================
	my_2D_ICP.Image_new = Image_New_Knife;
	// my_2D_ICP.Draw_Contrary();  // 这一句可以取消注释看一下当前刀具和基准刀具的差别
	my_2D_ICP.ICP_2D();
	// ======================================================
		

	// Step5 根据 ICP 结果改造示教点
	// Step5.1 载入示教点 （已经在初始化做过）
	// Step5.2 载入ICP结果
	my_robot_trans.RI = my_2D_ICP.R_res;
	my_robot_trans.tI = my_2D_ICP.t_res;
	// Step5.3 计算所有示教点并传输进入UR机器人


	input_ctrl_flag = 0;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
	Delay(1 * 1000);

	my_RTDE.RTDE_Send_Stop();
	// 退出Python的环境
	Py_Finalize();

	

	return 0;
}

