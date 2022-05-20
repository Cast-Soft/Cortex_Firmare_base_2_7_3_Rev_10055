
// cs_management.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CCSMApp:
// See cs_management.cpp for the implementation of this class
//

class CCSMApp : public CWinApp
{
public:
	CCSMApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CCSMApp theApp;
