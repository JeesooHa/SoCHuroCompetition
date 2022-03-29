
// SocProjDlg.h : 헤더 파일
//

#pragma once

#include "imageSrc\VideoProcessing.h"
#include "afxwin.h"
#include "iostream"


// CSocProjDlg 대화 상자
class CSocProjDlg : public CDialogEx
{
// 생성입니다.
public:
	CSocProjDlg(CWnd* pParent = NULL);	// 표준 생성자입니다.

// 대화 상자 데이터입니다.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SOCPROJ_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 지원입니다.


// 구현입니다.
protected:
	HICON m_hIcon;

	// 생성된 메시지 맵 함수
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	// 카메라 캡쳐 관련 멤버 변수
	CWebCam		m_webCam;
public:

	/* Basic Vedio fuctions*/

	CComboBox path_sel;

	afx_msg void OnStnClickedStaticPreview();
	afx_msg void OnBnClickedButtonStartCam();
	afx_msg void OnBnClickedButtonStopCam();
	afx_msg void OnBnClickedButtonDbImageSave();

	BOOL b_ChkMed;
	afx_msg void OnBnClickedCheckMed();
	CString m_strEdit;

	BOOL HSV_check;
	afx_msg void OnBnClickedCheckHsv();
	CString state_HSV;

	BOOL Check_thres;
	afx_msg void OnBnClickedCheckThres();
	CString state_thres;

	BOOL check_contour;
	afx_msg void OnBnClickedCheckContour();
	CString state_contour;
	BOOL hist_check;
	CString state_hist;
	afx_msg void OnBnClickedChecHistogram();
	BOOL check_morpho;
	BOOL check_ROI;
	BOOL print_med;
	BOOL print_h;
	BOOL print_s;
	BOOL print_v;
	BOOL print_ori;
	BOOL print_thres;
	afx_msg void OnBnClickedSocStart();

};
