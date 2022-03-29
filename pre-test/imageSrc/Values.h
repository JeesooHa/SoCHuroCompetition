#pragma once

///////////////////////////////////////////////////////////////////////////////
#define DBnum 4

static char* Obj[DBnum] = { "CH1.bmp", "JO4.bmp", "PE2.bmp", "OG5.bmp" };

/*
#define DBnum 20

static char* Obj[DBnum] = { "CH.bmp",  "JO.bmp",  "PE.bmp",  "OG.bmp",
"CH1.bmp", "JO1.bmp" ,"PE1.bmp", "OG1.bmp",
"CH2.bmp", "JO2.bmp", "PE2.bmp", "OG2.bmp",
"CH3.bmp", "JO3.bmp", "PE3.bmp", "OG3.bmp",
"CH4.bmp", "JO4.bmp", "PE4.bmp", "OG4.bmp" };
*/

int iLowH = 10;
int iHighH = 255;
int iLowS = 140; 
int iHighS = 255;
int iLowV = 0;
int iHighV = 255;

int hsvaluecnt = 0;
int old_hsvaluecnt = 0;
/////////////////////////////////////////////////////////////////////////////////////


void UpdateFPS()
{
	static float frameCount = 0.0f;
	static float timeElapsed = 0.0f;
	static int LastTime = GetTickCount();

	int curTime = GetTickCount();
	float timeDelta = (curTime - LastTime)*0.001f;

	timeElapsed += timeDelta;
	frameCount++;

	if (timeElapsed >= 1.0f)
	{
		float fps = (float)frameCount / timeElapsed;
		CString csTemp;
		cout <<"					=> FPS : "<< fps << endl;
		//csTemp.Format("\r\n!!!!!! FPS is %f \r\n", fps);
		OutputDebugString(csTemp);

		frameCount = 0;
		timeElapsed = 0.0f;
	}
	else
	{
		//딜레이를 주는 부분	
	}

	LastTime = curTime;
}