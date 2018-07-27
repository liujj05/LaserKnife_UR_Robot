#include "stdafx.h"
#include "CBasler_Single_Cam_Easy.h"


CBasler_Single_Cam_Easy::CBasler_Single_Cam_Easy()
{
	if_init = false;
}


CBasler_Single_Cam_Easy::~CBasler_Single_Cam_Easy()
{
	// 释放相机
	if (if_init == true)
		Release_Cam();
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

		camera.StartGrabbing(c_countOfImagesToGrab);
		
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
				// const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
#ifdef SHOW_MID_RES
				cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;
#endif
				// 初始化 Current_Mat
				Mat Image_Captured = cv::Mat(ptrGrabResult->GetHeight(),
					ptrGrabResult->GetWidth(),
					CV_8UC1,
					(uint8_t *)ptrGrabResult->GetBuffer());
				Current_Mat = Image_Captured.clone();
				/*
				namedWindow("TestImage");
				imshow("TestImage", Image_Captured);
				waitKey(2000);
				destroyWindow("TestImage");
				*/
				exitcode = true;
			}// if (ptrGrabResult->GrabSucceeded())
			else
			{
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
				exitcode = false;
			}
		}// while(camera.IsGrabbing())


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
	// camera.StopGrabbing();
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

	// 读取新图像
	try
	{
		camera.StartGrabbing(c_countOfImagesToGrab);
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
				// const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
#ifdef SHOW_MID_RES
				cout << "Gray value of first pixel: " << (uint32_t)pImageBuffer[0] << endl << endl;
#endif
				// 更新 Mat
				//memcpy(Image_Captured.data, ptrGrabResult->GetBuffer(), Image_Captured.rows * Image_Captured.cols * sizeof(uchar));
				Mat Image_Captured = cv::Mat(ptrGrabResult->GetHeight(),
					ptrGrabResult->GetWidth(),
					CV_8UC1,
					(uint8_t *)ptrGrabResult->GetBuffer());

				Image_Captured.copyTo(Current_Mat);
				/*
				namedWindow("TestImage2");
				imshow("TestImage2", Image_Captured);
				waitKey(2000);
				destroyWindow("TestImage2");
				*/
				exitCode = true;
			}// if (ptrGrabResult->GrabSucceeded())
			else
			{
				cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
				exitCode = false;
			}
		}// while(camera.IsGrabbing())


		if_init = true;
		exitCode = true;
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

