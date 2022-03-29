
// SocProjDlg.cpp : ���� ����
//

#include "stdafx.h"
#include "SocProj.h"
#include "SocProjDlg.h"
#include "afxdialogex.h"

#include "imageSrc\ImageFrameWndManager.h"
#include "imageSrc\LoadImageFromFileDialog.h"
#include "imageSrc\Segmentation.h"

#include "imageSrc\ImageEnhancement.h"
#include "imageSrc\MyImageFunc.h"
#include "imageSrc\MorphologyFilter.h"
#include "imageSrc\labeling.h"
#include "imageSrc\Histogram.h"
#include "imageSrc\Values.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// ���� ���α׷� ������ ���Ǵ� CAboutDlg ��ȭ �����Դϴ�.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ȭ ���� �������Դϴ�.
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �����Դϴ�.

// �����Դϴ�.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CSocProjDlg ��ȭ ����



CSocProjDlg::CSocProjDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SOCPROJ_DIALOG, pParent)
	
	, b_ChkMed(FALSE)
	, m_strEdit(_T("OFF"))
	, HSV_check(FALSE)
	, state_HSV(_T("OFF"))
	, Check_thres(FALSE)
	, state_thres(_T("OFF"))
	, check_contour(FALSE)
	, state_contour(_T("OFF"))
	, hist_check(FALSE)
	, state_hist(_T("OFF"))
	, check_morpho(TRUE)
	, check_ROI(TRUE)
	, print_med(FALSE)
	, print_h(TRUE)
	, print_s(TRUE)
	, print_v(FALSE)
	, print_ori(FALSE)
	, print_thres(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSocProjDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_PATH_NAME, path_sel);
	DDX_Check(pDX, IDC_CHECK_MED, b_ChkMed);
	DDX_Text(pDX, IDC_EDIT1, m_strEdit);
	DDX_Check(pDX, IDC_CHECK_HSV, HSV_check);
	DDX_Text(pDX, IDC_EDIT2, state_HSV);
	DDX_Check(pDX, IDC_CHECK_THRES, Check_thres);
	DDX_Text(pDX, IDC_EDIT3, state_thres);
	DDX_Check(pDX, IDC_CHECK_CONTOUR, check_contour);
	DDX_Text(pDX, IDC_EDIT4, state_contour);
	DDX_Check(pDX, IDC_CHEC_HISTOGRAM, hist_check);
	DDX_Text(pDX, IDC_EDIT5, state_hist);
	DDX_Check(pDX, IDC_CHECK_MOR, check_morpho);
	DDX_Check(pDX, IDC_CHECK_ROI, check_ROI);
	DDX_Check(pDX, IDC_PRINT_MED, print_med);
	DDX_Check(pDX, IDC_PRINT_H, print_h);
	DDX_Check(pDX, IDC_PRINT_S, print_s);
	DDX_Check(pDX, IDC_PRINT_V, print_v);
	DDX_Check(pDX, IDC_PRINT_ORI, print_ori);
	DDX_Check(pDX, IDC_PRINT_IMG, print_thres);

}

BEGIN_MESSAGE_MAP(CSocProjDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()

	ON_BN_CLICKED(IDC_BUTTON_START_CAM, &CSocProjDlg::OnBnClickedButtonStartCam)
	ON_STN_CLICKED(IDC_STATIC_PREVIEW, &CSocProjDlg::OnStnClickedStaticPreview)
	ON_BN_CLICKED(IDC_BUTTON_STOP_CAM, &CSocProjDlg::OnBnClickedButtonStopCam)
	ON_BN_CLICKED(IDC_BUTTON_DB_IMAGE_SAVE, &CSocProjDlg::OnBnClickedButtonDbImageSave)
	ON_BN_CLICKED(IDC_CHECK_MED, &CSocProjDlg::OnBnClickedCheckMed)

	ON_BN_CLICKED(IDC_CHECK_HSV, &CSocProjDlg::OnBnClickedCheckHsv)
	ON_BN_CLICKED(IDC_CHECK_THRES, &CSocProjDlg::OnBnClickedCheckThres)
	ON_BN_CLICKED(IDC_CHECK_CONTOUR, &CSocProjDlg::OnBnClickedCheckContour)
	ON_BN_CLICKED(IDC_CHEC_HISTOGRAM, &CSocProjDlg::OnBnClickedChecHistogram)
	ON_BN_CLICKED(IDC_SOC_START, &CSocProjDlg::OnBnClickedSocStart)
	
END_MESSAGE_MAP()


// CSocProjDlg �޽��� ó����

BOOL CSocProjDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �ý��� �޴��� "����..." �޴� �׸��� �߰��մϴ�.

	// IDM_ABOUTBOX�� �ý��� ��� ������ �־�� �մϴ�.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �� ��ȭ ������ �������� �����մϴ�.  ���� ���α׷��� �� â�� ��ȭ ���ڰ� �ƴ� ��쿡��
	//  �����ӿ�ũ�� �� �۾��� �ڵ����� �����մϴ�.
	SetIcon(m_hIcon, TRUE);			// ū �������� �����մϴ�.
	SetIcon(m_hIcon, FALSE);		// ���� �������� �����մϴ�.

	// TODO: ���⿡ �߰� �ʱ�ȭ �۾��� �߰��մϴ�.

	//Save Image
	path_sel.AddString(_T(" "));
	path_sel.AddString(_T("ĥ�� ���̴�"));
	path_sel.AddString(_T("�Ҹ���"));
	path_sel.AddString(_T("���"));
	path_sel.AddString(_T("��¡�� ����"));
	path_sel.SetCurSel(0);



	return TRUE;  // ��Ŀ���� ��Ʈ�ѿ� �������� ������ TRUE�� ��ȯ�մϴ�.
}

void CSocProjDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// ��ȭ ���ڿ� �ּ�ȭ ���߸� �߰��� ��� �������� �׸�����
//  �Ʒ� �ڵ尡 �ʿ��մϴ�.  ����/�� ���� ����ϴ� MFC ���� ���α׷��� ��쿡��
//  �����ӿ�ũ���� �� �۾��� �ڵ����� �����մϴ�.

void CSocProjDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // �׸��⸦ ���� ����̽� ���ؽ�Ʈ�Դϴ�.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Ŭ���̾�Ʈ �簢������ �������� ����� ����ϴ�.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �������� �׸��ϴ�.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ����ڰ� �ּ�ȭ�� â�� ���� ���ȿ� Ŀ���� ǥ�õǵ��� �ý��ۿ���
//  �� �Լ��� ȣ���մϴ�.
HCURSOR CSocProjDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

/***************************** ī�޶� �⺻ �̺�Ʈ ********************************************/

// ī�޶� ���� �� ������ �����ϱ� ���� ���� ����
CByteImage gImageBuf;
BITMAPINFO gBmpInfo;
CByteImage Original;

float h;

//�ݹ� �Լ�
LRESULT ProcessCamFrame(HWND hWnd, LPVIDEOHDR lpVHdr)
{
	//1. gBmpInfo ������ �˻��Ͽ� ���� ������ ��ȯ ����
	if (gBmpInfo.bmiHeader.biCompression == BI_RGB) // RGB ���� <- ������ C270
	{
		memcpy(gImageBuf.GetPtr(), lpVHdr->lpData,
			3 * gBmpInfo.bmiHeader.biHeight*gBmpInfo.bmiHeader.biWidth);
	}
	else if (gBmpInfo.bmiHeader.biCompression == MAKEFOURCC('Y', 'U', 'Y', '2')) // 16��Ʈ ����
	{
		YUY2ToRGB24(gBmpInfo.bmiHeader.biWidth, gBmpInfo.bmiHeader.biHeight,
			lpVHdr->lpData, gImageBuf.GetPtr());
	}
	else
	{
		return FALSE;
	}

	
	//Original = gImageBuf;
	CByteImage c_imageOut(gImageBuf);
	CByteImage d_imageOut(gImageBuf);

	// ���� ����
	for (int r = 0; r < c_imageOut.GetHeight(); r++)
		for (int c = 0; c < c_imageOut.GetWidth(); c++) {
			d_imageOut.GetAt(c, r, 0) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 0);
			d_imageOut.GetAt(c, r, 1) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 1);
			d_imageOut.GetAt(c, r, 2) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 2);
		}
	Original = d_imageOut;


	UpdateFPS();


	return TRUE;
}


//�ݹ� �Լ�
LRESULT SOC_PROJECT(HWND hWnd, LPVIDEOHDR lpVHdr)
{
	//1. gBmpInfo ������ �˻��Ͽ� ���� ������ ��ȯ ����
	if (gBmpInfo.bmiHeader.biCompression == BI_RGB) // RGB ���� <- ������ C270
	{
		memcpy(gImageBuf.GetPtr(), lpVHdr->lpData,
			3 * gBmpInfo.bmiHeader.biHeight*gBmpInfo.bmiHeader.biWidth);
	}
	else if (gBmpInfo.bmiHeader.biCompression == MAKEFOURCC('Y', 'U', 'Y', '2')) // 16��Ʈ ����
	{
		YUY2ToRGB24(gBmpInfo.bmiHeader.biWidth, gBmpInfo.bmiHeader.biHeight,
			lpVHdr->lpData, gImageBuf.GetPtr());
	}
	else
	{
		return FALSE;
	}


	//////////////////////////////////////////////////////////////////////////////////////////////////
	//							�ٽ� �˰��� ����!!!!!!											//
	//////////////////////////////////////////////////////////////////////////////////////////////////

	//���� ���� ��
	//ShowImage(gImageBuf, "���� ����");

	//ShowImage(gImageBuf,"Ori");
	//_MeanFiltering(gImageBuf, gImageBuf);

	CByteImage Original;
	CDoubleImage HSV_image;
	CByteImage H_channel;
	CByteImage S_channel;
	CByteImage V_channel;

	//Original = gImageBuf;
	CByteImage c_imageOut(gImageBuf);
	CByteImage d_imageOut(gImageBuf);

	// ���� ����
	for (int r = 0; r < c_imageOut.GetHeight(); r++)
		for (int c = 0; c < c_imageOut.GetWidth(); c++) {
			d_imageOut.GetAt(c, r, 0) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 0);
			d_imageOut.GetAt(c, r, 1) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 1);
			d_imageOut.GetAt(c, r, 2) = c_imageOut.GetAt(c, c_imageOut.GetHeight() - 1 - r, 2);
		}
	Original = d_imageOut;

	//ShowImage(Original, "Original");

	HSV_image = RGB2HSV(Original);

	//ShowImage(HSV_image, "HSV_image");

	V_channel = (HSV_image.GetChannelImg(0) * 255);	// 0~255
	S_channel = (HSV_image.GetChannelImg(1) * 255);	// 0~255
	H_channel = (HSV_image.GetChannelImg(2)/2);	// 0~255
	//ShowImage(H_channel, "H ����");
	//ShowImage(S_channel, "S ����");
	//ShowImage(V_channel, "V ����");

	CByteImage H_Out;
	CByteImage S_Out;
	CByteImage V_Out;
	CDoubleImage Sum_HSV;
	CByteImage ThresImage;

	CByteImage Tmp;
	CByteImage Morpho;

	H_Out = CByteImage(H_channel.GetWidth(), H_channel.GetHeight());
	S_Out = CByteImage(S_channel.GetWidth(), S_channel.GetHeight());
	V_Out = CByteImage(V_channel.GetWidth(), V_channel.GetHeight());

	BinarizationLH(H_channel, H_Out, iLowH , iHighH );
	BinarizationLH(S_channel, S_Out, iLowS, iHighS);
	BinarizationLH(V_channel, V_Out, iLowV, iHighV);
	//ShowImage(H_Out, "H_bin");
	//ShowImage(S_Out, "S_bin");
	//ShowImage(V_Out, "V_bin");

	ThresImage = CDoubleImage(H_channel.GetWidth(), H_channel.GetHeight());
	

	for (int r = 0; r < ThresImage.GetHeight(); r++)
		for (int c = 0; c < ThresImage.GetWidth(); c++) {
			if (H_Out.GetAt(c, r) && S_Out.GetAt(c, r) && V_Out.GetAt(c, r))
				ThresImage.GetAt(c, r) = 255;
			else
				ThresImage.GetAt(c, r) = 0;
		}

	//ShowImage(ThresImage, "ThresImage");


	CByteImage maskDilate(3, 3);	// ��â ����ũ
	maskDilate.SetConstValue(0);

	Morpho = ThresImage;
	Tmp = Morpho;		// �ӽ� �Է� ����
	Morpho.SetConstValue(0);	// ��� ���� �ʱ�ȭ
	DilateG(Tmp, maskDilate, Morpho);


	CByteImage maskErode(5, 5);		// ħ�� ����ũ
	maskErode.SetConstValue(0);

	Tmp = Morpho;
	Morpho.SetConstValue(0);	// ��� ���� �ʱ�ȭ
	ErodeG(Tmp, maskErode, Morpho);

	ShowImage(Morpho, "�������� ����");

	CByteImage Contour;
	CByteImage ROI;

	Contour = Morpho;

	CByteImage img = Contour;
	CIntImage imgLabel;
	vector<LabelInfo> labels;
	int label_cnt = Labeling(img, imgLabel, labels);

	int contArea = 0;
	int maxArea = 0;
	int max_x1 = 0, max_x2 = 0, max_y1 = 0, max_y2 = 0;

	// ��ü�� ���δ� �簢�� �׸���
	for (LabelInfo& info : labels)
	{
		contArea = info.pixels.size();
		if (maxArea < contArea) {
			maxArea = contArea;
			max_x1 = info.minx;
			max_x2 = info.maxx;
			max_y1 = info.miny;
			max_y2 = info.maxy;
		}

		//�簢�� ǥ��
		for (int j = info.miny; j <= info.maxy; j++)
			img.GetAt(info.minx, j) = img.GetAt(info.maxx, j) = 128;
		for (int i = info.minx; i <= info.maxx; i++)
			img.GetAt(i, info.miny) = img.GetAt(i, info.maxy) = 128;

	}

	//cout << maxArea << endl;

	//ShowImage(img, "������ ����");

	//�ִ� ���� ���� �� ã��
	for (int j = max_y1; j <= max_y2; j++)
		img.GetAt(max_x1, j) = img.GetAt(max_x2, j) = 128;
	for (int i = max_x1; i <= max_x2; i++)
		img.GetAt(i, max_y1) = img.GetAt(i, max_y2) = 128;

	ROI = CByteImage(max_x2 - max_x1 + 1, max_y2 - max_y1 + 1, 3);
	ROI.SetConstValue(0);

	for (int y = max_y1; y <= max_y2; y++)
		for (int x = max_x1; x <= max_x2; x++) {
			ROI.GetAt(x - max_x1, y - max_y1, 0) = Original.GetAt(x, y, 0);
			ROI.GetAt(x - max_x1, y - max_y1, 1) = Original.GetAt(x, y, 1);
			ROI.GetAt(x - max_x1, y - max_y1, 2) = Original.GetAt(x, y, 2);
		}


	//���� �簢�� �׸���
	for (int y = max_y1; y <= max_y2; y++) {

		Original.GetAt(max_x1, y, 0) = Original.GetAt(max_x2, y, 0) = 0;
		Original.GetAt(max_x1, y, 1) = Original.GetAt(max_x2, y, 1) = 0;
		Original.GetAt(max_x1, y, 2) = Original.GetAt(max_x2, y, 2) = 255;
		Original.GetAt(max_x1 + 1, y, 0) = Original.GetAt(max_x2 + 1, y, 0) = 0;
		Original.GetAt(max_x1 + 1, y, 1) = Original.GetAt(max_x2 + 1, y, 1) = 0;
		Original.GetAt(max_x1 + 1, y, 2) = Original.GetAt(max_x2 + 1, y, 2) = 255;
	}

	for (int x = max_x1; x <= max_x2; x++) {
		Original.GetAt(x, max_y1, 0) = Original.GetAt(x, max_y2, 0) = 0;
		Original.GetAt(x, max_y1, 1) = Original.GetAt(x, max_y2, 1) = 0;
		Original.GetAt(x, max_y1, 2) = Original.GetAt(x, max_y2, 2) = 255;
		Original.GetAt(x, max_y1 + 1, 0) = Original.GetAt(x, max_y2 + 1, 0) = 0;
		Original.GetAt(x, max_y1 + 1, 1) = Original.GetAt(x, max_y2 + 1, 1) = 0;
		Original.GetAt(x, max_y1 + 1, 2) = Original.GetAt(x, max_y2 + 1, 2) = 255;
	}


	ShowImage(Original, "Ori");
	//ShowImage(ROI, "ROI");


	CByteImage InputImg = ROI;

	Histo hist1 = Histo();
	Histo hist2 = Histo();

	//ShowImage(Hist.m_imageIn, "Origan");
	hist1 = hist1.HS_histo(InputImg);


	//ShowImage(hist1.HS_Image, "HS_hist1");

	//ShowImage(hist1.H_hist, "H_hist1");
	//ShowImage(hist1.S_hist, "S_hist1");

	float h_max = -1;	int h_i = -1;


	char name[2];
	//Load image to compare
	for (int n = 0; n < DBnum; n++) {
		CByteImage OBJ;
		OBJ.LoadImage(Obj[n]);

		hist2 = hist2.HS_histo(OBJ);

		h = hist2._compareHist(hist1.H_histogram, hist2.H_histogram);

		/*
		if (n % 4 == 0) {
		cout << "ĥ�� : " << n << " " << h << endl;
		//ShowImage(hist2.HS_Image, *(Obj+n));

		}
		else if (n % 4 == 1) {
		cout << "�Ҹ��� : " << n << " " << h << endl;
		//ShowImage(hist2.HS_Image, *(Obj + n));

		}
		else if (n % 4 == 2) {
		cout << "��� : " << n << " " << h << endl;
		//ShowImage(hist2.HS_Image, *(Obj + n));

		}
		else if (n % 4 == 3) {
		cout << "��¡�� : " << n << " " << h << endl << endl;
		//ShowImage(hist2.HS_Image, *(Obj + n));

		}
		*/
		if (h > h_max) {
			h_max = h;
			h_i = n;
		}


	}


	if (h_max < 0.4|| maxArea<1200 ||((hsvaluecnt%4==1)&& maxArea<4000)) {//|| maxArea<2000
		cout << "ã�� ��" << endl;
		old_hsvaluecnt = hsvaluecnt;
		hsvaluecnt++;
	}
	else if (h_i % 4 == 0) {
		cout << "[[[ ĥ�� ]]] : " << " " << h_i << " " << h_max << endl;
	}
	else if (h_i % 4 == 1) {
		cout << "[[[ �Ҹ��� ]]] : " << " " << h_i << " " << h_max << endl;
	}
	else if (h_i % 4 == 2) {
		cout << "[[[ ��� ]]] : " << " " << h_i << " " << h_max << endl;
	}
	else if (h_i % 4 == 3) {
		cout << "[[[ ��¡�� ���� ]]] : " << " " << h_i << " " << h_max << endl;
	}

	//"CH.bmp", "JO.bmp", "PE.bmp", "OG.bmp" 

	//cout << hsvaluecnt << endl;
	
	
	 if ((hsvaluecnt % 4 == 1) && (hsvaluecnt != old_hsvaluecnt))	//������
		{
			iLowH = 10;
			iHighH = 30;
			iLowS = 150;
			iHighS = 255;

		}


	 else//���, ��¡�� ����, ĥ��
	 {
		 iLowH = 61;
		 iHighH = 141;
		 iLowS = 120;
		 iHighS = 255;

	 }
	 
	 

	 
	UpdateFPS();


	return TRUE;
}



void CSocProjDlg::OnBnClickedButtonStartCam()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	GetDlgItem(IDC_STATIC_PREVIEW)->SetWindowPos(NULL, 0, 0, 640, 480, SWP_NOZORDER | SWP_NOMOVE);
	m_webCam.StartCam(GetDlgItem(IDC_STATIC_PREVIEW), 640, 480, gBmpInfo);

	gImageBuf = CByteImage(gBmpInfo.bmiHeader.biWidth, gBmpInfo.bmiHeader.biHeight, 3);
	m_webCam.SetCallBackOnFrame(ProcessCamFrame);
	
}


void CSocProjDlg::OnStnClickedStaticPreview()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
}


void CSocProjDlg::OnBnClickedButtonStopCam()
{

	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	m_webCam.StopCam();
}


/************************* DB �̹��� ���� ********************************/


void CSocProjDlg::OnBnClickedButtonDbImageSave()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	char* path = NULL;

	CByteImage image = Original;
	
	int path_name = path_sel.GetCurSel();

	if (path_name == 1) {	// ��¡�� ����
		path = "OG.bmp";
	}
	else if (path_name == 2) {	// �Ҹ���
		path = "JO.bmp";
	}
	else if (path_name == 3) {	// ĥ�����̴�
		path = "CH.bmp";
	}
	else if (path_name == 4) {	//���
		path = "PE.bmp";
	}
	else
		path = NULL;

	if (path != NULL)
		image.SaveImage(path);
	
}

/************* Image Enhancement ************/

CByteImage after_image;
CByteImage before_image;
void CSocProjDlg::OnBnClickedCheckMed()
{
	before_image = Original;
	after_image = before_image;

	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData(TRUE);

	if (b_ChkMed == TRUE) {
		m_strEdit = "ON";

		if(print_med == true)
			ShowImage(before_image, "before");

		//_MeanFiltering(before_image, after_image);	//�߰��� ����
		_MedianFiltering(before_image, after_image);	//��հ� ����

		if (print_med == true)
			ShowImage(after_image, "after");
	}
	else {
		m_strEdit = "OFF";

		if (print_med == true){
			CloseImage("before");
			CloseImage("after");
		}

	}
	UpdateData(FALSE);
}

CDoubleImage HSV_image;
CByteImage H_channel;
CByteImage S_channel;
CByteImage V_channel;

void CSocProjDlg::OnBnClickedCheckHsv()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData(TRUE);

	if (HSV_check == TRUE) {
		state_HSV = "ON";
		//Original = gImageBuf;
		if (print_ori == true)
			ShowImage(Original, "Ori");
		HSV_image = RGB2HSV(Original);
		
		if (!HSV_image.IsEmpty()) {
			
			V_channel = (HSV_image.GetChannelImg(0)*255);	// 0~255
			S_channel = (HSV_image.GetChannelImg(1)*255);	// 0~255
			H_channel = (HSV_image.GetChannelImg(2)*(255 / 360.0));	// 0~255

			if(print_h==true)
				ShowImage(H_channel, "H ����");
			if (print_s == true)
				ShowImage(S_channel, "S ����");
			if (print_v == true)
				ShowImage(V_channel, "V ����");
		}
	}
	else {
		state_HSV = "OFF";
		if (print_ori == true)
			CloseImage("Ori");
		if (print_h == true)
			CloseImage("H ����");
		if (print_s == true)
			CloseImage("S ����");
		if (print_v == true)
			CloseImage("V ����");
	}
	UpdateData(FALSE);
}

CByteImage H_Out;
CByteImage S_Out;
CByteImage V_Out;
CDoubleImage Sum_HSV;
CByteImage ThresImage;

CDoubleImage Sum_RGB;
CByteImage Tmp;
CByteImage Morpho;

CByteImage maskDilate(3, 3);	// ��â ����ũ
CByteImage maskErode(5, 5);		// ħ�� ����ũ

void CSocProjDlg::OnBnClickedCheckThres()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData(TRUE);

	if (Check_thres == TRUE) {
		UpdateData(TRUE);
		state_thres = "ON";

		H_Out = CByteImage(H_channel.GetWidth(), H_channel.GetHeight());
		S_Out = CByteImage(S_channel.GetWidth(), S_channel.GetHeight());
		V_Out = CByteImage(V_channel.GetWidth(), V_channel.GetHeight());

		BinarizationLH(H_channel, H_Out, iLowH * (255.0 / 180.0), iHighH* (255.0 / 180.0));		//	// HS �� ���߿� ã��!!!!!!!!!!!!!!!!!!!!!!!
		BinarizationLH(S_channel, S_Out, iLowS, iHighS);
		BinarizationLH(V_channel, V_Out, iLowV, iHighV);

		if (print_thres == true) {
			ShowImage(H_Out, "H_bin");
			ShowImage(S_Out, "S_bin");
			ShowImage(V_Out, "V_bin");
		}
		Sum_HSV = CDoubleImage(H_channel.GetWidth(), H_channel.GetHeight(), 3);

		Sum_HSV.PutChannelImg(V_Out, 0);
		Sum_HSV.PutChannelImg(S_Out, 1);
		Sum_HSV.PutChannelImg(H_Out, 2);

		if (print_thres == true)
			ShowImage(Sum_HSV, "sum");

		Sum_RGB = CDoubleImage(H_channel.GetWidth(), H_channel.GetHeight(), 3);
		Sum_RGB = HSV2RGB(Sum_HSV);

		if (print_thres == true)
			ShowImage(Sum_RGB, "RGB");

		ThresImage = CDoubleImage(H_channel.GetWidth(), H_channel.GetHeight());
		ThresImage = RGB2Gray(Sum_RGB);


		ShowImage(ThresImage, "ThresImage");

		
		if (check_morpho == true)
		{

			maskDilate.SetConstValue(0);

			Morpho = ThresImage;
			Tmp = Morpho;				// �ӽ� �Է� ����
			Morpho.SetConstValue(0);	// ��� ���� �ʱ�ȭ
			DilateG(Tmp, maskDilate, Morpho);

			maskErode.SetConstValue(0);

			Tmp = Morpho;
			Morpho.SetConstValue(0);	// ��� ���� �ʱ�ȭ
			ErodeG(Tmp, maskErode, Morpho);

			ShowImage(Morpho, "�������� ����");
		}

		UpdateData(false);

	}
	else {
		UpdateData(TRUE);
		state_thres = "OFF";
		if (print_thres == true) {
			CloseImage("H_bin");
			CloseImage("S_bin");
			CloseImage("V_bin");
			CloseImage("sum");
			CloseImage("RGB");
		}
		if (check_morpho == true)
		{
			CloseImage("�������� ����");
		}
		CloseImage("ThresImage");

	}
	UpdateData(FALSE);
}

CByteImage Contour;
CByteImage ROI;
CByteImage ROIWnd;
void CSocProjDlg::OnBnClickedCheckContour()
{
	// TOO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.

	UpdateData(TRUE);

	if (check_morpho == true)
		Contour = Morpho;
	else
		Contour = ThresImage;

	if (check_contour == TRUE) {
		state_contour = "ON";

		CByteImage img = Contour;
		ShowImage(img, "img");
		CIntImage imgLabel;
		vector<LabelInfo> labels;
		int label_cnt = Labeling(img, imgLabel, labels);


		int contArea = 0;
		int maxArea = 0;
		int max_x1, max_x2, max_y1, max_y2;

		// ��ü�� ���δ� �簢�� �׸���
		//BYTE* ptr = img.GetPtr();
		for (LabelInfo& info : labels)
		{
			contArea = info.pixels.size();
			if (maxArea < contArea) {
				maxArea = contArea;
				max_x1 = info.minx;
				max_x2 = info.maxx;
				max_y1 = info.miny;
				max_y2 = info.maxy;
			}
			
			for (int j = info.miny; j <= info.maxy; j++)
				img.GetAt(info.minx, j) = img.GetAt(info.maxx, j) = 128;
			for (int i = info.minx; i <= info.maxx; i++)
				img.GetAt(i, info.miny) = img.GetAt(i, info.maxy) = 128;


		}
		//�ִ� ���� ���� �� ã��
		for (int j = max_y1; j <= max_y2; j++)
			img.GetAt(max_x1, j) = img.GetAt(max_x2, j) = 128;
		for (int i = max_x1; i <= max_x2; i++)
			img.GetAt(i, max_y1) = img.GetAt(i, max_y2) = 128;

		ShowImage(img, "img");
		

		//ROI ���
		UpdateData(TRUE);
		

		if (check_ROI == true) {
			ROI.SetConstValue(0);
			ROI = CByteImage(max_x2 - max_x1 + 1, max_y2 - max_y1 + 1,3);
			ROIWnd = CByteImage(360, 360, 3);

			for (int y = max_y1; y <= max_y2; y++)
				for (int x = max_x1; x <= max_x2; x++) {
					ROI.GetAt(x - max_x1, y - max_y1, 0) = Original.GetAt(x, y, 0);
					ROI.GetAt(x - max_x1, y - max_y1, 1) = Original.GetAt(x, y, 1);
					ROI.GetAt(x - max_x1, y - max_y1, 2) = Original.GetAt(x, y, 2);
				}	



			//���� �簢�� �׸���
			for (int y = max_y1; y <= max_y2; y++) {

				Original.GetAt(max_x1, y, 0) = Original.GetAt(max_x2, y, 0) = 0;
				Original.GetAt(max_x1, y, 1) = Original.GetAt(max_x2, y, 1) = 0;
				Original.GetAt(max_x1, y, 2) = Original.GetAt(max_x2, y, 2) = 255;
				Original.GetAt(max_x1 + 1, y, 0) = Original.GetAt(max_x2 + 1, y, 0) = 0;
				Original.GetAt(max_x1 + 1, y, 1) = Original.GetAt(max_x2 + 1, y, 1) = 0;
				Original.GetAt(max_x1 + 1, y, 2) = Original.GetAt(max_x2 + 1, y, 2) = 255;
			}

			for (int x = max_x1; x <= max_x2; x++) {
				Original.GetAt(x, max_y1, 0) = Original.GetAt(x, max_y2, 0) = 0;
				Original.GetAt(x, max_y1, 1) = Original.GetAt(x, max_y2, 1) = 0;
				Original.GetAt(x, max_y1, 2) = Original.GetAt(x, max_y2, 2) = 255;
				Original.GetAt(x, max_y1 + 1, 0) = Original.GetAt(x, max_y2 + 1, 0) = 0;
				Original.GetAt(x, max_y1 + 1, 1) = Original.GetAt(x, max_y2 + 1, 1) = 0;
				Original.GetAt(x, max_y1 + 1, 2) = Original.GetAt(x, max_y2 + 1, 2) = 255;
			}


			ShowImage(Original, "Ori");



		}
		else {
			ROI = Original;
		}
		
		//ShowImage(ROI, "ROI");
		//ShowImage(ROIWnd, "ROIWnd");
		UpdateData(FALSE);

	}
	else {
		state_contour = "OFF";
		CloseImage("img");
		CloseImage("Ori");
	}
	UpdateData(FALSE);
}


CByteImage H_Hist;
CByteImage S_Hist;

void CSocProjDlg::OnBnClickedChecHistogram()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
	UpdateData(TRUE);
	if (hist_check == TRUE) {
		state_hist = "ON";
	
		if (check_ROI == true) {
			CByteImage InputImg = ROI;

			Histo Hist;

			//ShowImage(Hist.m_imageIn, "Origan");
			Hist = Hist.HS_histo(InputImg);

			
			ShowImage(Hist.HS_Image, "HS_hist");
			
			//ShowImage(Hist.H_hist, "H_hist");
			//ShowImage(Hist.S_hist, "S_hist");
		}

	}
	else {
		state_hist = "OFF";

		CloseImage("HS_hist");
		CloseImage("H_hist");
		CloseImage("S_hist");
	}
	UpdateData(FALSE);
}


void CSocProjDlg::OnBnClickedSocStart()
{
	GetDlgItem(IDC_STATIC_PREVIEW)->SetWindowPos(NULL, 0, 0, 640, 480, SWP_NOZORDER | SWP_NOMOVE);
	m_webCam.StartCam(GetDlgItem(IDC_STATIC_PREVIEW), 640, 480, gBmpInfo);
	
	//ĸ�� ���� �̸����� ȭ�鿡 ��µǴ� �������� �ڵ� ����
	gImageBuf = CByteImage(gBmpInfo.bmiHeader.biWidth, gBmpInfo.bmiHeader.biHeight, 3);
	m_webCam.SetCallBackOnFrame(SOC_PROJECT);
}


