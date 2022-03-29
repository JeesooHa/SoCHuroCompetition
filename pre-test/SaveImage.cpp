// SaveImage.cpp : 구현 파일입니다.
//

#include "stdafx.h"
#include "SocProj.h"
#include "SaveImage.h"
#include "afxdialogex.h"


// CSaveImage 대화 상자입니다.

IMPLEMENT_DYNAMIC(CSaveImage, CDialogEx)

CSaveImage::CSaveImage(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_SAVE_IMAGE, pParent)
	, image_name(0)	//초기 저장할 이미지를 칠성(0)으로 초기화
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


// CSaveImage 메시지 처리기입니다.

void CSaveImage::OnStnClickedDbSavedImage()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}

