
// SocProj.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CSocProjApp:
// �� Ŭ������ ������ ���ؼ��� SocProj.cpp�� �����Ͻʽÿ�.
//

class CSocProjApp : public CWinApp
{
public:
	CSocProjApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CSocProjApp theApp;