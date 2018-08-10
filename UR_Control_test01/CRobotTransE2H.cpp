#include "stdafx.h"
#include "CRobotTransE2H.h"


CRobotTransE2H::CRobotTransE2H()
{
	cam_A = Mat::zeros(3, 3, CV_64FC1);
	// ========按顺序填写内参标定数据========== 结合标定进行修改
	cam_A.at<double>(0, 0) = 4447.5;	cam_A.at<double>(0, 2) = 599.2;
	cam_A.at<double>(1, 1) = 4450.7;	cam_A.at<double>(1, 2) = 499.8;
	cam_A.at<double>(2, 2) = 1;
	// ====================================

	// ========手眼标定结果======== 结合标定数据修改
	T_C_to_0 = Mat::zeros(4, 4, CV_64FC1);
	T_C_to_0.at<double>(0, 0) = -0.0399;		T_C_to_0.at<double>(0, 1) =  0.9994;		T_C_to_0.at<double>(0, 2) = -0.0085;
	T_C_to_0.at<double>(1, 0) =  0.9991;		T_C_to_0.at<double>(1, 1) =  0.0337;		T_C_to_0.at<double>(1, 2) = -0.0246;
	T_C_to_0.at<double>(2, 0) = -0.0243;		T_C_to_0.at<double>(2, 1) = -0.0094;		T_C_to_0.at<double>(2, 2) = -0.9997;
	T_C_to_0.at<double>(0, 3) =  645.7810;
	T_C_to_0.at<double>(1, 3) = -149.1172;
	T_C_to_0.at<double>(2, 3) =  426.2558;
	T_C_to_0.at<double>(3, 3) = 1;

	// =====================================

	// =================测量位置的6系矩阵===================
	T_6_to_0_test = Mat::zeros(4, 4, CV_64FC1);
	T_6_to_0_test.at<double>(0, 0) = -0.0139;		T_6_to_0_test.at<double>(0, 1) = -0.0286;		T_6_to_0_test.at<double>(0, 2) =  0.9995;
	T_6_to_0_test.at<double>(1, 0) = -0.9993;		T_6_to_0_test.at<double>(1, 1) = -0.0347;		T_6_to_0_test.at<double>(1, 2) = -0.0149;
	T_6_to_0_test.at<double>(2, 0) =  0.0351;		T_6_to_0_test.at<double>(2, 1) = -0.9990;		T_6_to_0_test.at<double>(2, 2) = -0.0281;
	T_6_to_0_test.at<double>(0, 3) =  427.83;
	T_6_to_0_test.at<double>(1, 3) = -148.16;
	T_6_to_0_test.at<double>(2, 3) =  226.77;
	T_6_to_0_test.at<double>(3, 3) =  1;

	// =====================================
	// =============测量位置的外参数矩阵=============
	T_Cal_to_C = Mat::zeros(4, 4, CV_64FC1);
	T_Cal_to_C.at<double>(0, 0) =  0.0002;		T_Cal_to_C.at<double>(0, 1) = -0.9999;		T_Cal_to_C.at<double>(0, 2) = -0.0105;
	T_Cal_to_C.at<double>(1, 0) =  0.9998;		T_Cal_to_C.at<double>(1, 1) =  0.0004;		T_Cal_to_C.at<double>(1, 2) = -0.0196;
	T_Cal_to_C.at<double>(2, 0) =  0.0196;		T_Cal_to_C.at<double>(2, 1) = -0.0105;		T_Cal_to_C.at<double>(2, 2) =  0.9998;
	T_Cal_to_C.at<double>(0, 3) =  6.2522;
	T_Cal_to_C.at<double>(1, 3) = -3.4523;
	T_Cal_to_C.at<double>(2, 3) =  200.9674;
	T_Cal_to_C.at<double>(3, 3) =  1;

	// 初始化所有不会改变的矩阵
	// camA应该先算
	Inverse_Of_A();
}


CRobotTransE2H::~CRobotTransE2H()
{
}

void CRobotTransE2H::Compute_Trans_Mat(void)
{
	Build_H();

	Build_T_Ki_to_Ks();

	cout << "================== T_Ki_to_Ks ======================" << endl;
	cout << T_Ki_to_Ks << endl;
	cout << "====================================================" << endl;

	Build_T_Ks_to_6();

	cout << "================== T_Ks_to_6 ======================" << endl;
	cout << T_Ks_to_6 << endl;
	cout << "====================================================" << endl;

	Mat T_Ki_to_6 = T_Ks_to_6 * T_Ki_to_Ks;

	Trans_6i_to_6s = T_Ks_to_6 * Inverse_Of_T(T_Ki_to_6);
}

void CRobotTransE2H::Trans_Teach_Vec(void)
{
	
	Mat T60_old;
	Build_T60_from_Vec(input_origin_Teach_Vec6, T60_old);
	
	cout << "====================T60_old=========================" << endl;
	cout << T60_old << endl;
	cout << "====================================================" << endl;

	Mat T60_new = T60_old * Trans_6i_to_6s;
	
	cout << "====================T60_new=========================" << endl;
	cout << T60_new << endl;
	cout << "====================================================" << endl;

	Build_Vec_from_T60(T60_new);

	for (int i = 0; i < 6; i++)
	{
		output_new_Teach_Vec6[i] = Teach_Vec6[i];
	}
}

bool CRobotTransE2H::Load_Origin_Teach_Pts(const char * fileName)
{
	bool return_val = false;	// 目前没有纠错机制
	
	ifstream ifile(fileName);
	string line_data;
	vector<string> result_str;

	int data_rows = 0;

	while (getline(ifile, line_data))
	{
		data_rows++;
	}
	ifile.close();

	Origin_Teach_Pts = Mat::zeros(data_rows, 6, CV_64FC1);

	ifile.open(fileName);
	data_rows = 0;
	while (getline(ifile, line_data))
	{
		result_str.clear();
		result_str = SplitString(line_data, ",");
		for (int j = 0; j < 6; j++)
		{
			Origin_Teach_Pts.at<double>(data_rows, j) = stod(result_str[j]);
		}
		data_rows++;
	}

	return_val = true;
	
	return return_val;
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

	Mat temp7 = Mat::eye(4, 4, CV_64FC1);
	// temp6 = I4

	invA.copyTo(temp7(Range(0, 3), Range(0, 3)));
	// temp6(0:2, 0:2) = camA
	// Range包含头不包含尾

	Temp_H = temp7 * temp5 * temp6;


}

void CRobotTransE2H::Build_T_Ki_to_Ks(void)
{
	Mat T_Ks_to_Cal = Mat::eye(4, 4, CV_64FC1);
	T_Ks_to_Cal.at<double>(2, 3) = thickness_of_cal_board;
	// 考虑了标定板的厚度，注意，标定板的Z轴朝下！


	T_Ks_to_C = T_Cal_to_C * T_Ks_to_Cal;



	// 为了和MATLAB对比，这里增加一步 - 实际上这一步也是要加上的，但是针对谁做SVD需要讨论
	// Mat T_Ki_to_C = Temp_H.inv() * T_Ks_to_C;
	Inverse_Of_H();
	Mat T_Ki_to_C = invH * T_Ks_to_C;

	Mat R_Ki_to_C = T_Ki_to_C(Range(0, 3), Range(0, 3));
	Mat U, S, V;
	SVD::compute(R_Ki_to_C, S, U, V);
	R_Ki_to_C = U * V;


	R_Ki_to_C.copyTo(T_Ki_to_C(Range(0, 3), Range(0, 3)));

	// T_Ki_to_Ks = T_Ks_to_C.inv() * T_Ki_to_C; 
	T_Ki_to_Ks = Inverse_Of_T(T_Ks_to_C) * T_Ki_to_C;
}

void CRobotTransE2H::Build_T_Ks_to_6(void)
{
	Mat T_Ks_to_0 = T_C_to_0 * T_Ks_to_C;
	
	// T_Ks_to_6 = T_6_to_0_test.inv() * T_Ks_to_0;
	T_Ks_to_6 = Inverse_Of_T(T_6_to_0_test) * T_Ks_to_0;

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

	// 实际输入进来的单位制是m
	dst.at<double>(0, 3) = pos_vec6[0] * 1000;
	dst.at<double>(1, 3) = pos_vec6[1] * 1000;
	dst.at<double>(2, 3) = pos_vec6[2] * 1000;
}

void CRobotTransE2H::Build_Vec_from_T60(Mat src)
{
	rotationMat2Vec(src);
	// 仍然要注意单位制的问题
	Teach_Vec6[0] = src.at<double>(0, 3) / 1000.0;
	Teach_Vec6[1] = src.at<double>(1, 3) / 1000.0;
	Teach_Vec6[2] = src.at<double>(2, 3) / 1000.0; 

	Teach_Vec6[3] = Rot_Vec3[0];
	Teach_Vec6[4] = Rot_Vec3[1];
	Teach_Vec6[5] = Rot_Vec3[2];
}

void CRobotTransE2H::rotationVec2Mat(double * pos_vec3, Mat & dst)
{
	//double nx, ny, nz;
	//double theta;

	//theta = sqrt(pos_vec3[0] * pos_vec3[0] +
	//	pos_vec3[1] * pos_vec3[1] +
	//	pos_vec3[2] * pos_vec3[2]); // 求向量模长

	//nx = pos_vec3[0] / theta;
	//ny = pos_vec3[1] / theta;
	//nz = pos_vec3[2] / theta; // 求单位向量

	//dst = Mat::zeros(3, 3, CV_64FC1);
	//dst.at<double>(0, 0) = nx * nx * (1 - cos(theta)) + cos(theta);
	//dst.at<double>(1, 0) = nx * ny * (1 - cos(theta)) + nz * sin(theta);
	//dst.at<double>(2, 0) = nx * nz * (1 - cos(theta)) - ny * sin(theta);
	//dst.at<double>(0, 1) = nx * ny * (1 - cos(theta)) - nz * sin(theta);
	//dst.at<double>(1, 1) = ny * ny * (1 - cos(theta)) + cos(theta);
	//dst.at<double>(2, 1) = ny * nz * (1 - cos(theta)) + nx * sin(theta);
	//dst.at<double>(0, 2) = nx * nz * (1 - cos(theta)) + ny * sin(theta);
	//dst.at<double>(1, 2) = ny * nz * (1 - cos(theta)) - nx * sin(theta);
	//dst.at<double>(2, 2) = nz * nz * (1 - cos(theta)) + cos(theta);

	Mat Rot_Vec = Mat::zeros(3, 1, CV_64FC1);
	Rot_Vec.at<double>(0, 0) = pos_vec3[0];
	Rot_Vec.at<double>(1, 0) = pos_vec3[1];
	Rot_Vec.at<double>(2, 0) = pos_vec3[2];
	Rodrigues(Rot_Vec, dst);
}

void CRobotTransE2H::rotationMat2Vec(Mat src)
{
	// 从旋转矩阵计算旋转向量
	/*double theta;
	Scalar src_trace = trace(src(Range(0,3), Range(0, 3)));
	theta = acos((src_trace[0] - 1) / 2);

	double nx, ny, nz;

	nx = -(src.at<double>(1, 2) - src.at<double>(2, 1)) / 2 / sin(theta);
	ny = -(src.at<double>(2, 0) - src.at<double>(0, 2)) / 2 / sin(theta);
	nz = -(src.at<double>(0, 1) - src.at<double>(1, 0)) / 2 / sin(theta);

	Rot_Vec3[0] = nx * theta;
	Rot_Vec3[1] = ny * theta;
	Rot_Vec3[2] = nz * theta;*/

	// Try a new method
	Mat Rot_Vec;
	Rodrigues(src(Range(0, 3), Range(0, 3)), Rot_Vec);
	//cout << "Rodrigus" << endl;
	//
	cout << Rot_Vec << endl;
	Rot_Vec3[0] = Rot_Vec.at<double>(0, 0);
	Rot_Vec3[1] = Rot_Vec.at<double>(1, 0);
	Rot_Vec3[2] = Rot_Vec.at<double>(2, 0);
}

Mat CRobotTransE2H::Inverse_Of_T(Mat T_src)
{
	Mat T_res = Mat::eye(4, 4, CV_64FC1);
	Mat R, RT, t;
	Mat R_res, t_res;
	R = T_src(Range(0, 3), Range(0, 3));
	t = T_src(Range(0, 3), Range(3, 4));
	RT = R.t();
	R_res = RT;
	t_res = -RT * t;
	R_res.copyTo(T_res(Range(0, 3), Range(0, 3)));
	t_res.copyTo(T_res(Range(0, 3), Range(3, 4)));
	return T_res;
}

void CRobotTransE2H::Inverse_Of_A(void)
{
	invA = Mat::zeros(3, 3, CV_64FC1);
	invA.at<double>(0, 0) = 1 / cam_A.at<double>(0, 0);
	invA.at<double>(1, 1) = 1 / cam_A.at<double>(1, 1);
	invA.at<double>(2, 2) = 1;
	invA.at<double>(0, 2) = - cam_A.at<double>(0, 2) / cam_A.at<double>(0, 0);
	invA.at<double>(1, 2) = - cam_A.at<double>(1, 2) / cam_A.at<double>(1, 1);
}

void CRobotTransE2H::Inverse_Of_H(void)
{
	Mat temp1;
	hconcat(RI.t(), -RI.t()*tI, temp1);

	// temp1 = [RI', -RI'*tI]

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

	Mat temp7 = Mat::eye(4, 4, CV_64FC1);
	// temp7 = I4

	cam_A.copyTo(temp6(Range(0, 3), Range(0, 3)));
	invA.copyTo(temp7(Range(0, 3), Range(0, 3)));
	

	invH = temp7 * temp5 * temp6;

}

vector<string> CRobotTransE2H::SplitString(string str, string pattern)
{
	std::string::size_type pos;
	std::vector<std::string> result;

	str += pattern;//扩展字符串以方便操作

	int size = str.size();

	for (int i = 0; i<size; i++)
	{
		pos = str.find(pattern, i);
		if (pos<size)
		{
			std::string s = str.substr(i, pos - i);
			result.push_back(s);
			i = pos + pattern.size() - 1;
		}
	}

	return result;
}
