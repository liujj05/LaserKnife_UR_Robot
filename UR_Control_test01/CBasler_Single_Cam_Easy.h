#pragma once

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

using namespace Pylon;

class CBasler_Single_Cam_Easy
{
public:
	CBasler_Single_Cam_Easy();
	~CBasler_Single_Cam_Easy();

private:
	static const uint32_t c_countOfImagesToCrab = 1;
};

