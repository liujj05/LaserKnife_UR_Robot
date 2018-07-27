#pragma once

#include <opencv2/opencv.hpp>
#include <float.h>

using namespace std;
using namespace cv;

class ICP2D
{
public:
	ICP2D();
	~ICP2D();

	int iter_max;			// 迭代次数上限
	double iter_thresh;		// 残差变化上限
	int binary_thresh;		// 二值化阈值


	Mat Image_ref;
	Mat Image_new;

	// 点云
	Mat ICP_ref_pts;
	Mat ICP_new_pts;

	// 降采样率
	int ref_down_sample_rate;
	int new_down_sample_rate;

	// ICP函数
	Mat R_res, t_res; // 汇总的最终结果
	void ICP_2D(void);			// ICP计算主程序
	void Draw_Contrary(void);	// 绘制前后二者的轮廓位置对比，基准用蓝色，新图用红色
private:

	void imPreProcess(void);

	flann::Index My_Kdtree;

};

