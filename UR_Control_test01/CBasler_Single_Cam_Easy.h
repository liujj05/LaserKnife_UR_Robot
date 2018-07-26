#pragma once

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

#include <opencv2/opencv.hpp>


using namespace Pylon;
using namespace std;
using namespace cv;

class CBasler_Single_Cam_Easy
{
public:
	CBasler_Single_Cam_Easy();
	~CBasler_Single_Cam_Easy();

	CInstantCamera camera;

	bool Init_Cam(void);			// 初始化单个相机
	void Release_Cam(void);			// 释放所有Pylon的变量
	bool Cap_single_image(void);	// 获取单帧图像

	Mat Current_Mat;

private:
	static const uint32_t c_countOfImagesToGrab = 1;
	CGrabResultPtr ptrGrabResult;
	bool if_init;
};

