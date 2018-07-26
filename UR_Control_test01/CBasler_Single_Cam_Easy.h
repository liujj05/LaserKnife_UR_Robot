#pragma once

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

using namespace Pylon;
using namespace std;

class CBasler_Single_Cam_Easy
{
public:
	CBasler_Single_Cam_Easy();
	~CBasler_Single_Cam_Easy();

	CInstantCamera camera;

	bool Init_Cam(void);			// ��ʼ���������
	void Release_Cam(void);			// �ͷ�����Pylon�ı���
	bool Cap_single_image(void);	// ��ȡ��֡ͼ��

private:
	static const uint32_t c_countOfImagesToGrab = 1;
	bool if_init;
};

