#pragma once

#include "MyImage.h"
#include <vfw.h>
#pragma comment (lib,"vfw32.lib")

// 카메라 캡쳐 클래스
class CWebCam
{
public:
	CWebCam();
	virtual ~CWebCam();

	bool StartCam(CWnd* wnd, int nWidth, int nHeight, BITMAPINFO& bmpInfo);
	void StopCam();
	BOOL StartRecord(const char* filename);
	BOOL StopRecord();
	BOOL SetCallBackOnFrame(LRESULT (*fpProc)(HWND, LPVIDEOHDR));
	//콜백함수 - 이벤트에 따라 시스템에서 자동적으로 함수를 호출하여 작업 수행

protected:
	HWND		m_hCam;
	HWND		SCam;
};

// AVI 파일 재생 클래스
class CAviPlayer
{
public:
	CAviPlayer();
	virtual ~CAviPlayer();

	bool LoadAvi(const char* filename, int& nFrame, int& nFrameRate, int& nWidth, int& nHeight);
	void CloseAvi();
	bool ShowFrame(CWnd* wnd, int idxFrame);
	bool GetFrame(CByteImage& image, int idxFrame);

protected:
	PAVIFILE	m_pAviFile;
	PGETFRAME	m_pGetFrame;

	int			m_nWidth;
	int			m_nHeight;
};

void YUY2ToRGB24(int nWidth, int nHeight, const BYTE* pYUY2, BYTE* pRGB24);