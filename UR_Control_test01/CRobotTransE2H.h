// E2H means eye to hand
// 本类用于计算机器人各关节坐标系转换
// 针对激光所项目，整合进了新功能：
// 已知末端工件偏差，计算输入示教点偏差
// 计算原理详见：《Math_Stuff_report_on_Knife_Localization》
// 对应的matlab代码详见 20180703lasercalib中的calib_HandEye

#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using namespace cv;
using namespace std;

class CRobotTransE2H
{
public:
	CRobotTransE2H();
	~CRobotTransE2H();

	Mat RI;
	Mat tI;
	Mat Origin_Teach_Pts;	// 载入的原始示教点会存放在这个矩阵里面

	double input_origin_Teach_Vec6[6];
	double output_new_Teach_Vec6[6];

	Mat Trans_6i_to_6s;
	void Compute_Trans_Mat(void);
	void Trans_Teach_Vec(void);

	bool Load_Origin_Teach_Pts(const char *fileName);

private:
	Mat cam_A;			// 内参数矩阵

	Mat T_Cal_to_C;		// 标定板坐标系到相机坐标系（此时标定板贴在基准刀具上，基准刀具处于测量位置）	-也是标定给出来的，初始化要给一下
	Mat T_Ks_to_C;		// 基准刀具坐标系到相机坐标系（即基准刀具在测量位置时，其坐标系与相机坐标系之间的关系）
	const double thickness_of_cal_board = 2.1; // 标定板厚度2.1mm，以上两个矩阵相差的就是这个参数

	Mat T_C_to_0;		// 手眼标定结果						-初始化的时候要给出
	Mat T_6_to_0_test;	// 测量位置时机器人末端的位姿			-初始化的时候要给出

	Mat Temp_H;			// 中间结果 H，计算 T_Ki_to_Ks 的必需中间量
	
	Mat T_Ks_to_6;		// 基准刀具坐标系和末端坐标系6的关系
	Mat T_Ki_to_Ks;		// 计算 T_Ki_to_6 的必需量

	void Build_H(void);	// 根据 A、RI、tI 建立 H 矩阵
	
	void Build_T_Ki_to_Ks(void);

	void Build_T_Ks_to_6(void);

	void Build_T60_from_Vec(double* pos_vec6, Mat &dst);	// 6 示教点转换为 T60
	void Build_Vec_from_T60(Mat src);

	void rotationVec2Mat(double* pos_vec3, Mat &dst);
	void rotationMat2Vec(Mat src);

	double Teach_Vec6[6];	// 中转量 - Build_Vec_from_T60 用
	double Rot_Vec3[3];		// 中转量 - rotationMat2Vec 用

	// 更精确的求逆方法
	Mat Inverse_Of_T(Mat T_src);
	Mat invA;
	void Inverse_Of_A(void);
	Mat invH;
	void Inverse_Of_H(void);
	
	vector<string> SplitString(string str, string pattern);
};

