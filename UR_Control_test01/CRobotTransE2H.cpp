#include "stdafx.h"
#include "CRobotTransE2H.h"


CRobotTransE2H::CRobotTransE2H()
{
	cam_A = Mat::zeros(3, 3, CV_64FC1);
	// ========按顺序填写内参标定数据========== 结合标定进行修改
	cam_A.at<double>(0, 0) = 4211.1;	cam_A.at<double>(0, 2) = 617.1369;
	cam_A.at<double>(1, 1) = 4213.3;	cam_A.at<double>(1, 2) = 498.0212;
	cam_A.at<double>(2, 2) = 1;
	// ====================================

	// ========手眼标定结果======== 结合标定数据修改
	T_C_to_0 = Mat::zeros(4, 4, CV_64FC1);
	T_C_to_0.at<double>(0, 0) = 0.0815;		T_C_to_0.at<double>(0, 1) = 0.9954;		T_C_to_0.at<double>(0, 2) = 0.0510;
	T_C_to_0.at<double>(1, 0) = 0.9963;		T_C_to_0.at<double>(1, 1) =-0.0800;		T_C_to_0.at<double>(1, 2) =-0.0299;
	T_C_to_0.at<double>(2, 0) =-0.0257;		T_C_to_0.at<double>(2, 1) = 0.0532;		T_C_to_0.at<double>(2, 2) =-0.9983;
	T_C_to_0.at<double>(0, 3) = 709.5354;
	T_C_to_0.at<double>(1, 3) = -480.9874;
	T_C_to_0.at<double>(2, 3) = 340.4273;
	T_C_to_0.at<double>(3, 3) = 1;

	// =====================================

	// =================测量位置的6系矩阵===================
	T_6_to_0_test = Mat::zeros(4, 4, CV_64FC1);
	T_6_to_0_test.at<double>(0, 0) = 0.0696;		T_6_to_0_test.at<double>(0, 1) = 0.0385;		T_6_to_0_test.at<double>(0, 2) = 0.9968;
	T_6_to_0_test.at<double>(1, 0) = 0.7061;		T_6_to_0_test.at<double>(1, 1) = 0.7039;		T_6_to_0_test.at<double>(1, 2) =-0.0765;
	T_6_to_0_test.at<double>(2, 0) =-0.7047;		T_6_to_0_test.at<double>(2, 1) = 0.7092;		T_6_to_0_test.at<double>(2, 2) = 0.0218;
	T_6_to_0_test.at<double>(0, 3) = 599.4320;
	T_6_to_0_test.at<double>(1, 3) = -398.2360;
	T_6_to_0_test.at<double>(2, 3) = 159.6439;
	T_6_to_0_test.at<double>(3, 3) = 1;

	// =====================================
	// =============测量位置的外参数矩阵=============
	T_Cal_to_C = Mat::zeros(4, 4, CV_64FC1);
	T_Cal_to_C.at<double>(0, 0) = 0.0671;		T_Cal_to_C.at<double>(0, 1) = -0.9973;		T_Cal_to_C.at<double>(0, 2) = 0.0286;
	T_Cal_to_C.at<double>(1, 0) = 0.9974;		T_Cal_to_C.at<double>(1, 1) = 0.0663;		T_Cal_to_C.at<double>(1, 2) = -0.0293;
	T_Cal_to_C.at<double>(2, 0) = 0.0274;		T_Cal_to_C.at<double>(2, 1) = 0.0305;		T_Cal_to_C.at<double>(2, 2) = 0.9992;
	T_Cal_to_C.at<double>(0, 3) = 10.1428;
	T_Cal_to_C.at<double>(1, 3) = -20.5069;
	T_Cal_to_C.at<double>(2, 3) = 212.0156;
	T_Cal_to_C.at<double>(3, 3) = 1;
}


CRobotTransE2H::~CRobotTransE2H()
{
}

void CRobotTransE2H::Trans_Teach_Vec(void)
{
	// Step1 将输入的6元素向量转化为T60
	Mat T60_old;
	Build_T60_from_Vec(input_origin_Teach_Vec6, T60_old);

	/*
	//--输出debug
	cout << "T60_old before 155" << endl;
	cout << T60_old << endl;
	
	// Debug Step
	Mat T_6long_to_6short = Mat::eye(4, 4, CV_64FC1);
	T_6long_to_6short.at<double>(2, 3) = 155;
	Mat T_6short_to_6long = Mat::eye(4, 4, CV_64FC1);
	T_6short_to_6long.at<double>(2, 3) = -155;
	T60_old = T60_old * T_6short_to_6long;

	cout << "T60_old after 155" << endl;
	cout << T60_old << endl;
	*/

	// Step2 转换该 T60
	Build_H();

	/*
	cout << "Temp_H" << endl;
	cout << Temp_H << endl;
	*/

	Build_T_Ki_to_Ks();

	/*
	cout << "T_Ki_to_Ks" << endl;
	cout << T_Ki_to_Ks << endl;

	cout << "If T_Ki_to_Ks is Trans Matrix?" << endl;
	*/

	Mat test_mat = T_Ki_to_Ks(Range(0, 3), Range(0, 3));
	
	/*
	cout << test_mat << endl;
	cout << test_mat.t() << endl;
	*/
	Mat test_mat2 = test_mat * test_mat.t();
	
	/*
	cout << test_mat2 << endl;
	*/

	Build_T_Ks_to_6();

	/*
	cout << "T_Ks_to_6" << endl;
	cout << T_Ks_to_6 << endl;
	*/

	Mat T_Ki_to_6 = T_Ks_to_6 * T_Ki_to_Ks;
	Mat T60_new = T60_old * T_Ks_to_6 * T_Ki_to_6.inv();

	/*
	cout << "T60_new" << endl;
	cout << T60_new << endl;

	// Debug Step
	T60_new = T60_new * T_6long_to_6short;

	*/

	// Step3 将新 T60 转化为向量
	Build_Vec_from_T60(T60_new);

	// Step4 给输出量赋值
	for (int i = 0; i < 6; i++)
	{
		output_new_Teach_Vec6[i] = Teach_Vec6[i];
	}
}

void CRobotTransE2H::Build_H(void)
{
	Mat temp1;
	hconcat(RI, tI, temp1);

	// temp1 = [RI, tI]

	Mat temp2 = Mat::zeros(2, 1, CV_64FC1);	// 2x1 零矩阵

	// temp2 = [0;
    //          0];


	Mat temp3 = Mat::zeros(2, 4, CV_64FC1);	// 2x4 零矩阵

	// temp3 = [0 0 0 0;
    //          0 0 0 0];

	temp3.at<double>(0, 2) = 1;
	temp3.at<double>(1, 3) = 1;

	// temp3 = [0 0 1 0;
	//          0 0 0 1];

	Mat temp4;
	Mat temp5;
	hconcat(temp1, temp2, temp4);
	// temp4 = [RI, tI, temp2];

	vconcat(temp4, temp3, temp5);
	// temp5 = [RI, tI, zeros(2,1);
	//          temp3 ];

	Mat temp6 = Mat::eye(4, 4, CV_64FC1);
	// temp6 = I4

	cam_A.copyTo(temp6(Range(0, 3), Range(0, 3)));
	// temp6(0:2, 0:2) = camA
	// Range包含头不包含尾


	Temp_H = temp6.inv() * temp5 * temp6;


}

void CRobotTransE2H::Build_T_Ki_to_Ks(void)
{
	Mat T_Ks_to_Cal = Mat::eye(4, 4, CV_64FC1);
	T_Ks_to_Cal.at<double>(2, 3) = thickness_of_cal_board;
	// 考虑了标定板的厚度，注意，标定板的Z轴朝下！

	/*
	cout << "T_Cal_to_C" << endl;
	cout << T_Cal_to_C << endl;
	*/

	T_Ks_to_C = T_Cal_to_C * T_Ks_to_Cal;

	/*
	cout << "T_Ks_to_C" << endl;
	cout << T_Ks_to_C << endl;
	*/

	// 为了和MATLAB对比，这里增加一步 - 实际上这一步也是要加上的，但是针对谁做SVD需要讨论
	Mat T_Ki_to_C = Temp_H.inv() * T_Ks_to_C;
	Mat R_Ki_to_C = T_Ki_to_C(Range(0, 3), Range(0, 3));
	Mat U, S, V;
	SVD::compute(R_Ki_to_C, S, U, V);
	R_Ki_to_C = U * V;

	/*
	cout << "R_Ki_to_C" << endl;
	cout << R_Ki_to_C << endl;
	*/

	R_Ki_to_C.copyTo(T_Ki_to_C(Range(0, 3), Range(0, 3)));

	T_Ki_to_Ks = T_Ks_to_C.inv() * T_Ki_to_C; 
}

void CRobotTransE2H::Build_T_Ks_to_6(void)
{
	Mat T_Ks_to_0 = T_C_to_0 * T_Ks_to_C;
	T_Ks_to_6 = T_6_to_0_test.inv() * T_Ks_to_0;
}

void CRobotTransE2H::Build_T60_from_Vec(double * pos_vec6, Mat & dst)
{
	dst = Mat::zeros(4, 4, CV_64FC1);

	Mat temp_Mat;
	double Rot_Vec[3];
	Rot_Vec[0] = pos_vec6[3];
	Rot_Vec[1] = pos_vec6[4];
	Rot_Vec[2] = pos_vec6[5];

	rotationVec2Mat(Rot_Vec, temp_Mat);

	dst.at<double>(3, 3) = 1;
	temp_Mat.copyTo(dst(Range(0, 3), Range(0, 3)));

	dst.at<double>(0, 3) = pos_vec6[0];
	dst.at<double>(1, 3) = pos_vec6[1];
	dst.at<double>(2, 3) = pos_vec6[2];
}

void CRobotTransE2H::Build_Vec_from_T60(Mat src)
{
	rotationMat2Vec(src);
	Teach_Vec6[0] = src.at<double>(0, 3);
	Teach_Vec6[1] = src.at<double>(1, 3);
	Teach_Vec6[2] = src.at<double>(2, 3);

	Teach_Vec6[3] = Rot_Vec3[0];
	Teach_Vec6[4] = Rot_Vec3[1];
	Teach_Vec6[5] = Rot_Vec3[2];
}

void CRobotTransE2H::rotationVec2Mat(double * pos_vec3, Mat & dst)
{
	double nx, ny, nz;
	double theta;

	theta = sqrt(pos_vec3[0] * pos_vec3[0] +
		pos_vec3[1] * pos_vec3[1] +
		pos_vec3[2] * pos_vec3[2]); // 求向量模长

	nx = pos_vec3[0] / theta;
	ny = pos_vec3[1] / theta;
	nz = pos_vec3[2] / theta; // 求单位向量

	dst = Mat::zeros(3, 3, CV_64FC1);
	dst.at<double>(0, 0) = nx * nx * (1 - cos(theta)) + cos(theta);
	dst.at<double>(1, 0) = nx * ny * (1 - cos(theta)) + nz * sin(theta);
	dst.at<double>(2, 0) = nx * nz * (1 - cos(theta)) - ny * sin(theta);
	dst.at<double>(0, 1) = nx * ny * (1 - cos(theta)) - nz * sin(theta);
	dst.at<double>(1, 1) = ny * ny * (1 - cos(theta)) + cos(theta);
	dst.at<double>(2, 1) = ny * nz * (1 - cos(theta)) + nx * sin(theta);
	dst.at<double>(0, 2) = nx * nz * (1 - cos(theta)) + ny * sin(theta);
	dst.at<double>(1, 2) = ny * nz * (1 - cos(theta)) - nx * sin(theta);
	dst.at<double>(2, 2) = nz * nz * (1 - cos(theta)) + cos(theta);
}

void CRobotTransE2H::rotationMat2Vec(Mat src)
{
	// 从旋转矩阵计算旋转向量
	double theta;
	Scalar src_trace = trace(src(Range(0,3), Range(0, 3)));
	theta = acos((src_trace[0] - 1) / 2);

	double nx, ny, nz;

	nx = -(src.at<double>(1, 2) - src.at<double>(2, 1)) / 2 / sin(theta);
	ny = -(src.at<double>(2, 0) - src.at<double>(0, 2)) / 2 / sin(theta);
	nz = -(src.at<double>(0, 1) - src.at<double>(1, 0)) / 2 / sin(theta);

	Rot_Vec3[0] = nx * theta;
	Rot_Vec3[1] = ny * theta;
	Rot_Vec3[2] = nz * theta;
}
