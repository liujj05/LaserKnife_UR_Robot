#include "stdafx.h"
#include "CBasler_Single_Cam_Easy.h"


CBasler_Single_Cam_Easy::CBasler_Single_Cam_Easy()
{
	if_init = false;
}


CBasler_Single_Cam_Easy::~CBasler_Single_Cam_Easy()
{
}

bool CBasler_Single_Cam_Easy::Init_Cam(void)
{
	bool exitcode;

	PylonInitialize();

	try
	{
		camera.Attach(CTlFactory::GetInstance().CreateFirstDevice());
		//camera = CInstantCamera(CTlFactory::GetInstance().CreateFirstDevice());
#ifdef SHOW_MID_RES
		cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
#endif
		camera.MaxNumBuffer = 5;
		if_init = true;
		exitcode = true;
	}
	catch (const GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
		exitcode = false;
	}

	//PylonTerminate();
	return exitcode;
}

void CBasler_Single_Cam_Easy::Release_Cam(void)
{
	if_init = false;
	PylonTerminate();
}

bool CBasler_Single_Cam_Easy::Cap_single_image(void)
{
	bool exitCode;

	if (false == if_init)
	{
		cout << "Not init camera yet" << endl;
		return false;
	}

	try
	{
		camera.StartGrabbing(c_countOfImagesToGrab);
		CGrabResultPtr ptrGrabResult;
		while (camera.IsGrabbing())
		{
			// Wait for an image and then retrieve it. A timeout of 5000 ms is used.
			camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

			// Image grabbed successfully?
			if (ptrGrabResult->GrabSucceeded())
			{
				// Access the image data.
#ifdef SHOW_MID_RES
				cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
				cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
#endif
				const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
#ifdef SHOW_MID_RES
				cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;
#endif

				exitCode = true;
			}// if (ptrGrabResult->GrabSucceeded())
			else
			{
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
				exitCode = false;
			}
		}// while(camera.IsGrabbing())
	}
	catch (const GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
			<< e.GetDescription() << endl;
		exitCode = false;
	}
	return exitCode;
}

