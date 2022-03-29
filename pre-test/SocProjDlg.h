
// SocProjDlg.h : ��� ����
//

#pragma once

#include "imageSrc\VideoProcessing.h"
#include "afxwin.h"
#include "iostream"


// CSocProjDlg ��ȭ ����
class CSocProjDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CSocProjDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_SOCPROJ_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;

	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	// ī�޶� ĸ�� ���� ��� ����
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
