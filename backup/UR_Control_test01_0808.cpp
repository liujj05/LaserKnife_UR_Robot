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


// 标志位宏定义
#define FLAG_AT_CAM_POS  0x80000000
#define FLAG_AT_CAM_POS2 0x40000000


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
	//Py_Initialize();
	//// 初始化一个ur_RTDE类
	//ur_RTDE my_RTDE2;
	//my_RTDE2.RTDE_Initialize();			// RTDE 通信初始化
	//my_RTDE2.RTDE_Send_Start();			// 建立 RTDE 连接
	//// 初始化标志位
	//my_RTDE2.RTDE_Send_BIT32(0);
	//Delay(100);
	////while (1)
	////{
	////	my_RTDE2.RTDE_Recv_BIT32();				// 接收机器人发送的标志位
	////	Delay(100);
	////}
	//// 读入示教点
	//CRobotTransE2H my_robot_trans2;
	//my_robot_trans2.Load_Origin_Teach_Pts("teach_pose.txt");
	//// 生成新示教点并发送
	//double new_teach_pose[6];	// 6自由度
	//int pt_num = 4;				// 4示教点
	//double y_offset = 0.02;		// UR端的单位貌似是m
	//// 发送示教点
	//for (int i = 0; i < 4; i++)
	//{
	//	for (int j = 0; j < 6; j++)
	//	{
	//		new_teach_pose[j] = my_robot_trans2.Origin_Teach_Pts.at<double>(i, j);
	//	}
	//	new_teach_pose[1] = new_teach_pose[1] + y_offset;
	//	my_RTDE2.RTDE_Send_POINT(new_teach_pose, 6); // 发送新示教点
	//	Delay(100);									// 额外给点时间确保发送完成
	//	my_RTDE2.RTDE_Send_BIT32(0x01<<i);			// 通知机器人收第i个点
	//	cout << "Send point " << i << " over, waiting..." << endl;
	//	while (1)
	//	{
	//		my_RTDE2.RTDE_Recv_BIT32();				// 接收机器人发送的标志位
	//		Delay(100);
	//		if (my_RTDE2.state_res & (0x01 << i))	// 提取对应位数，是1证明赋值完毕
	//			break;
	//	}
	//	cout << "---- point " << i << " received" << endl;
	//}
	//my_RTDE2.RTDE_Send_Stop();
	//Py_Finalize();
	//getchar();
	//return 0;



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
	// #0 初始化
	// #0.1 定义两个矩阵
	Mat Image_Ref_Knife;
	Mat Image_New_Knife;

	// #0.2 启动Python的环境
	Py_Initialize();

	// #0.3 初始化一个ur_RTDE类
	ur_RTDE my_RTDE;
	// #0.4 初始化一个CBasler_Single_Cam_Easy类
	CBasler_Single_Cam_Easy my_Cam;
	// #0.5 初始化一个ICP2D类
	ICP2D my_2D_ICP;
	my_2D_ICP.iter_max = 1000;
	my_2D_ICP.iter_thresh = 0.001;
	my_2D_ICP.ref_down_sample_rate = 5;
	my_2D_ICP.new_down_sample_rate = 5;
	my_2D_ICP.binary_thresh = 50;

	// #0.6 初始化一个E2H类
	CRobotTransE2H my_robot_trans;

	// #0.7 启动RTDE连接及相机连接
	my_RTDE.RTDE_Initialize();			// RTDE 通信初始化
	my_Cam.Init_Cam();					// Basler 相机初始化

	// #0.8 采集 new 图像（新匹配）来初始化 Image_New_Knife ==
	my_Cam.Cap_single_image();
	Image_New_Knife = my_Cam.Current_Mat.clone();	// clone会新分配一块内存给 Image_New_Knife
	// ======================================================

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

	// 运行后进行一个人工输入，方便暂停一下看一看目前的状态

	// 初始化标志位
	my_RTDE.RTDE_Send_BIT32(0);
	Delay(100);

	char input_str[16];
	cout << "Press ENTER to continue..." << endl;
	cin >> input_str;



	// == 以下代码可能需要循环执行 ==

	// #3 等待机器人到达定位视觉测量位置

	cout << "# Waiting a new knife..." << endl;

	TIMER_START; // 计时-观测响应时间
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		Delay(100); // 保险起见还是等待一下 
		//cout << "UR Control Waiting... Flag 0 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res & FLAG_AT_CAM_POS)
		{
			cout << "++++ Knife under the camera! " << endl;
			break;
		}
	}
	TIMER_STOP;
	cout << "++++ Flag detected in " << exe_time << " ms" << endl;
	
	// #4 拍摄+测量
	
	TIMER_START;	// 计时开始：拍摄-测量-上传完成
	cout << "# Grabbing image..." << endl;
	
	// #4.1 采集 new 图像（新匹配）来初始化 Image_New_Knife ==
	my_Cam.Cap_single_image();
	// 首次采集生成存储空间后，可以用copyTo，不额外重新分配空间
	my_Cam.Current_Mat.copyTo(Image_New_Knife);
	// ======================================================
	
	cout << "# Image grabbed, matching..." << endl;

	// #4.2 ICP 匹配	
	// =================== ICP匹配流程 =======================
	my_2D_ICP.Image_new = Image_New_Knife;
	// my_2D_ICP.Draw_Contrary();  // 这一句可以取消注释看一下当前刀具和基准刀具的差别
	my_2D_ICP.ICP_2D();
	// ======================================================
		

	// #5 根据 ICP 结果改造示教点
	// #5.1 载入示教点 （已经在初始化做过）
	// #5.2 载入ICP结果
	my_robot_trans.RI = my_2D_ICP.R_res;
	my_robot_trans.tI = my_2D_ICP.t_res;
	// #5.3 计算所有示教点并传输进入UR机器人
	int teach_pts_num = my_robot_trans.Origin_Teach_Pts.rows;		// 示教点数
	for (int i = 0; i < teach_pts_num; i++)
	{
		for (int j = 0; j < 6; j++) // 准备进行转换
		{
			my_robot_trans.input_origin_Teach_Vec6[j] = my_robot_trans.Origin_Teach_Pts.at<double>(i, j);
		}

		my_robot_trans.Trans_Teach_Vec();	// 转换

		// Debug
		cout << "++++ Debug Test Point #" << i << endl;
		cout << "++++ Origin left and New right" << endl;
		for (int j = 0; j < 6; j++)
		{
			cout << my_robot_trans.input_origin_Teach_Vec6[j] << "\t" << my_robot_trans.output_new_Teach_Vec6[j] << endl;
		}
		

		// 传输
		my_RTDE.RTDE_Send_POINT(my_robot_trans.output_new_Teach_Vec6, 6);	// 发送新示教点
		Delay(100);															// 额外给点时间确保发送完成
		my_RTDE.RTDE_Send_BIT32(0x01 << i);									// 通知机器人收第i个点
		cout << "# Send point " << i << " over, waiting..." << endl;
		while (1)
		{
			my_RTDE.RTDE_Recv_BIT32();				// 接收机器人发送的标志位
			Delay(100);
			if (my_RTDE.state_res & (0x01 << i))	// 提取对应位数，是1证明赋值完毕
				break;
		}
		cout << "++++ point " << i << " received" << endl;
	}
	TIMER_STOP;	// 计时结束：拍摄-测量-上传完成
	cout << "# It takes " << exe_time << " ms to locate this knife!" << endl;

	// 调试保险起见-输出纠正前后的序列
	/*cout << "=================================================" << endl;
	cout << "======== check if there is any errors ===========" << endl;
	cout << "=================================================" << endl;
	for (int i = 0; i < teach_pts_num; i++)
	{
		cout << " ++++ point " << i << "origin(left) and new(right)" << endl;
		for (int j = 0; j < 6; j++)
		{
			cout << my_robot_trans.Origin_Teach_Pts
		}
		
	}*/

	// 此处有下载的等待，下载完自动就开始走了，所以不用有回应

	// #6 等待加工运动完成后第二次到达相机位置进行检测拍摄
	cout << "# Waiting for the knife under cam again..." << endl;
	while (true)
	{
		my_RTDE.RTDE_Recv_BIT32();
		// Delay(2 * 1000); 
		//cout << "UR Control Waiting... Flag 0 --- " << my_RTDE.state_res << endl;
		if (my_RTDE.state_res & FLAG_AT_CAM_POS2)
		{
			cout << "++++ Knife under the camera again! " << endl;
			break;
		}
	}

	// #7 测量熔覆层厚度
	//
	//
	//
	//
	Delay(3 * 1000);
	//
	// 通知机器人检测完毕
	//
	//

	// #8 整个流程完成，准备开始下个流程
	// #8.1 标志位全部清零
	input_ctrl_flag = 0;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
	Delay(100);
	// #8.2 通知机器人放下刀具拿下一个
	// 上面那个清零可以算作是通知了。

	// 标志位归零程序结束
	input_ctrl_flag = 0;
	my_RTDE.RTDE_Send_BIT32(input_ctrl_flag);
	Delay(1 * 1000);

	my_RTDE.RTDE_Send_Stop();
	// 退出Python的环境
	Py_Finalize();

	

	return 0;
}

