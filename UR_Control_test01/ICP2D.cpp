#include "stdafx.h"
#include "ICP2D.h"


ICP2D::ICP2D()
{
}


ICP2D::~ICP2D()
{
}

void ICP2D::ICP_2D(void)
{
	imPreProcess();

	float pre_err = FLT_MAX;	// 迭代需要的误差量存储1
	float now_err = 0;			// 迭代需要的误差量存储2
	float delta_err;			// 收敛判定依据
	Mat res_ind, res_dist;		// K-D Tree 查找结果返回值
	int res_ind_int = 0;		// 查找索引提取
	float near_x, near_y;
	Mat nearest_pts_from_ref = Mat(ICP_new_pts.rows, 2, CV_32FC1);
	Mat mean_new = Mat(1, 2, CV_32FC1);		// ICP 计算过程中求解各个步骤的R、t的中间量，待匹配点云的重心
	Mat mean_near = Mat(1, 2, CV_32FC1);	// ICP 计算过程中求解各个步骤的R、t的中间量，迭代中点云的重心
	Mat AXY, BXY; // 中间量
	Mat H, U, S, Vt; // 中间量
	Mat Mid_eye = Mat::eye(2, 2, CV_32FC1);
	Mat temp_new_pts;
	Mat R, t; // 阶段计算结果
	
	R_res = Mat::eye(2, 2, CV_32FC1);
	t_res = Mat::zeros(2, 1, CV_32FC1);

	// 为了保留每个new_pts，复制矩阵 ICP_new_pts 至 ICP_new_pts_origin
	Mat ICP_new_pts_origin;
	ICP_new_pts.copyTo(ICP_new_pts_origin);


	// 2.2 迭代开始
	for (int iter_num = 0; iter_num < iter_max; iter_num++)
	{
		now_err = 0;


		for (int i = 0; i < ICP_new_pts.rows; i++)
		{
			My_Kdtree.knnSearch(ICP_new_pts.row(i), res_ind, res_dist, 1, flann::SearchParams(-1));
			res_ind_int = res_ind.at<int>(0, 0); // 确认过变量类型应该是对的
			near_x = ICP_ref_pts.at<float>(res_ind_int, 0);
			near_y = ICP_ref_pts.at<float>(res_ind_int, 1);
			nearest_pts_from_ref.at<float>(i, 0) = near_x;
			nearest_pts_from_ref.at<float>(i, 1) = near_y;

			now_err = now_err + sqrtf((ICP_new_pts.at<float>(i, 0) - near_x) * (ICP_new_pts.at<float>(i, 0) - near_x) +
				(ICP_new_pts.at<float>(i, 1) - near_y) * (ICP_new_pts.at<float>(i, 1) - near_y));
		}

#ifdef PRINT_OUT
		cout << "nearest: " << endl;
		cout << nearest_pts_from_ref << endl;
#endif

		delta_err = abs(now_err - pre_err);

		if (delta_err < iter_thresh)
		{
			break;
		}
		else
			pre_err = now_err;

		// 求重心，注意：cv::mean 的返回值是一个 cv::scalar 它由四个元素构成，但是我们只用到第一个，所以后面多了个[0]
		mean_new.at<float>(0, 0) = mean(ICP_new_pts.col(0))[0];
		mean_new.at<float>(0, 1) = mean(ICP_new_pts.col(1))[0];
		mean_near.at<float>(0, 0) = mean(nearest_pts_from_ref.col(0))[0];
		mean_near.at<float>(0, 1) = mean(nearest_pts_from_ref.col(1))[0];

#ifdef PRINT_OUT

		cout << "mean_new:" << endl;
		cout << mean_new << endl;

		cout << "mean_near" << endl;
		cout << mean_near << endl;

#endif

		// 所有点按重心归一化
		AXY = ICP_new_pts - repeat(mean_new, ICP_new_pts.rows, 1);
		BXY = nearest_pts_from_ref - repeat(mean_near, nearest_pts_from_ref.rows, 1);

#ifdef PRINT_OUT

		cout << "AXY:" << endl;
		cout << AXY << endl;

		cout << "BXY:" << endl;
		cout << BXY << endl;

#endif

		// 求出待SVD分解的H矩阵
		H = AXY.t() * BXY;
		SVD::compute(H, S, U, Vt);

#ifdef PRINT_OUT

		cout << "H:" << endl;
		cout << H << endl;

		cout << "U" << endl;
		cout << U << endl;

		cout << "Vt" << endl;
		cout << Vt << endl;

#endif

		Mid_eye.at<float>(1, 1) = determinant(Vt.t()*U.t());
		R = Vt.t() * Mid_eye * U.t();
		t = mean_near.t() - R * mean_new.t();

#ifdef PRINT_OUT

		cout << "R:" << endl;
		cout << R << endl;

		cout << "t" << endl;
		cout << t << endl;

#endif

		transpose((R * ICP_new_pts.t() + repeat(t, 1, ICP_new_pts.rows)), temp_new_pts);
		temp_new_pts.copyTo(ICP_new_pts);

#ifdef PRINT_OUT

		cout << "ICP_new_pts: " << endl;
		cout << ICP_new_pts << endl;

#endif
		R_res = R * R_res;
		t_res = R * t_res + t;
	}



	cout << "R_res:" << endl;
	cout << R_res << endl;

	cout << "t_res" << endl;
	cout << t_res << endl;

	cout << "now_err" << endl;
	cout << now_err << endl;
}

void ICP2D::imPreProcess(void)
{
	// Step1. 二值化
	Mat T_ref, T_new;
	threshold(Image_ref, T_ref, binary_thresh, 255, THRESH_BINARY);
	threshold(Image_new, T_new, binary_thresh, 255, THRESH_BINARY);

	// Step2. 边缘检测
	Mat E_ref, E_new;
	int edge_thresh = 50;
	Canny(T_ref, E_ref, edge_thresh, edge_thresh * 3);
	Canny(T_new, E_new, edge_thresh, edge_thresh * 3);

	// Step3. 边缘点云提取
	vector<Point> ref_pt_vec;
	vector<Point> new_pt_vec;
	for (int i = 0; i < E_ref.rows; i++)
	{
		for (int j = 0; j < E_ref.cols; j++)
		{
			if (E_ref.at<uchar>(i, j) != 0) // 这个地方这个char请验证
			{
				ref_pt_vec.push_back(Point(j, i));
			}
			if (E_new.at<uchar>(i, j) != 0) // 这个地方这个char请验证
			{
				new_pt_vec.push_back(Point(j, i));
			}
		}
	}

	// Step4. 降采样
	int ref_pt_num, new_pt_num;
	ref_pt_num = ref_pt_vec.size();
	new_pt_num = new_pt_vec.size();

	ICP_ref_pts = Mat(ref_pt_num / ref_down_sample_rate, 2, CV_32FC1);
	ICP_new_pts = Mat(new_pt_num / new_down_sample_rate, 2, CV_32FC1);

	for (int i = 0; i < ref_pt_num / ref_down_sample_rate; i++)
	{
		ICP_ref_pts.at<float>(i, 0) = ref_pt_vec[i * ref_down_sample_rate].x;
		ICP_ref_pts.at<float>(i, 1) = ref_pt_vec[i * ref_down_sample_rate].y;
	}
	for (int i = 0; i < new_pt_num / new_down_sample_rate; i++)
	{
		ICP_new_pts.at<float>(i, 0) = new_pt_vec[i * new_down_sample_rate].x;
		ICP_new_pts.at<float>(i, 1) = new_pt_vec[i * new_down_sample_rate].y;
	}


	// Step5. 建立K-D tree
	My_Kdtree.build(ICP_ref_pts, flann::KDTreeIndexParams(1), cvflann::FLANN_DIST_EUCLIDEAN);

	return;
}
