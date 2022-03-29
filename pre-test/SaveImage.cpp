// SaveImage.cpp : ���� �����Դϴ�.
//

#include "stdafx.h"
#include "SocProj.h"
#include "SaveImage.h"
#include "afxdialogex.h"


// CSaveImage ��ȭ �����Դϴ�.

IMPLEMENT_DYNAMIC(CSaveImage, CDialogEx)

CSaveImage::CSaveImage(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SAVE_IMAGE, pParent)
	, image_name(0)	//�ʱ� ������ �̹����� ĥ��(0)���� �ʱ�ȭ
{

}

CSaveImage::~CSaveImage()
{
}

void CSaveImage::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_CBIndex(pDX, IDC_IMAGE_NAME, image_name);
}


BEGIN_MESSAGE_MAP(CSaveImage, CDialogEx)
	ON_STN_CLICKED(IDC_DB_SAVED_IMAGE, &CSaveImage::OnStnClickedDbSavedImage)
	ON_BN_CLICKED(IDC_BUTTON_SELECT_IMAGE, &CSaveImage::OnBnClickedButtonSelectImage)
END_MESSAGE_MAP()


// CSaveImage �޽��� ó�����Դϴ�.

void CSaveImage::OnStnClickedDbSavedImage()
{
	// TODO: ���⿡ ��Ʈ�� �˸� ó���� �ڵ带 �߰��մϴ�.
}

