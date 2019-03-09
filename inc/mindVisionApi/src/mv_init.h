#pragma once
#include <opencv2/opencv.hpp>
#include "CameraApi.h"
#include "CameraDefine.h"
#include <semaphore.h>

using namespace cv;

typedef void *PVOID;
//#define far
//typedef void far *LPVOID;

// camear num is bigger,the time is more
#define CAMERA_NUM 2

#define CAMERA_RED 0;
#define CAMERA_BULE 1;

// 
#define IMAGE_ROWS 960
#define IMAGE_COLS 1280

class MvInit
{
public:
	MvInit(int cameraId);
	~MvInit();
	CameraSdkStatus createCamera(int playgroundId);

	Mat getImage() 
	{
		sem_wait(&sems);
		return srcImage;
	}


	BOOL            m_bExit = FALSE;
	CameraHandle    m_hCamera;			 
	tSdkFrameHead   m_sFrInfo;		
	Mat srcImage;
	sem_t           sems;
	pthread_t       id;

private:
	UINT            m_threadID;		
	
	HANDLE          m_hDispThread;	
	int	            m_iDispFrameNum;	
	float           m_fDispFps;			
	float           m_fCapFps;			
	tSdkFrameStatistic  m_sFrameCount;
	tSdkFrameStatistic  m_sFrameLast;
	int					m_iTimeLast;
	char		    g_CameraName[64];
};

