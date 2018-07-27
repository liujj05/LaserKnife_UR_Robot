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

	int iter_max;			// ������������
	double iter_thresh;		// �в�仯����
	int binary_thresh;		// ��ֵ����ֵ


	Mat Image_ref;
	Mat Image_new;

	// ����
	Mat ICP_ref_pts;
	Mat ICP_new_pts;

	// ��������
	int ref_down_sample_rate;
	int new_down_sample_rate;

	// ICP����
	Mat R_res, t_res; // ���ܵ����ս��
	void ICP_2D(void);

private:

	void imPreProcess(void);

	flann::Index My_Kdtree;

};

