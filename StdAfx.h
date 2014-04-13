// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#if !defined(AFX_STDAFX_H__6C7696F1_02AC_4338_A257_08C263BC8D7E__INCLUDED_)
#define AFX_STDAFX_H__6C7696F1_02AC_4338_A257_08C263BC8D7E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <windows.h>
#define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#define VC_EXTRALEAN

#pragma warning(disable: 4786)

#include <cassert>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <utility>
#include <vector>
#include <string>

using namespace std;

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

#define SAFE_DELETE(p)  { if(p) { delete (p);     (p)=NULL; } }
#define SAFE_RELEASE(p) { if(p) { (p)->Release(); (p)=NULL; } }
//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_STDAFX_H__6C7696F1_02AC_4338_A257_08C263BC8D7E__INCLUDED_)
