// CarLoader.h: interface for the CarLoader class.
//
// Copyright (2005) Gideon Pertzov
//
// This source code is provided "AS IS" without express or implied warranties.
// You may use this source code in FREEWARE applications you distribute, 
// Provided that credit is given to the original author.
// You MAY NOT use the source code in ANY COMMERCIAL APPLICATION
// Without the written consent of the original author.
//
// http://www.gpdev.net
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CARLOADER_H__BB1F8E71_8ED5_4BEC_9401_A1005D560FE6__INCLUDED_)
#define AFX_CARLOADER_H__BB1F8E71_8ED5_4BEC_9401_A1005D560FE6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "XmlLoader.h"

class CarLoader  : public XmlLoader
{
// Methods
public:
	CarLoader();
	virtual ~CarLoader();

	double getPhysicsAttr(const char* attrName);
	const char*	getMeshPath();

protected:
	virtual int validateDoc();

// Members
protected:

	TiXmlElement*	_pVisuals;
	TiXmlElement*	_pPhysics;
};

#endif // !defined(AFX_CARLOADER_H__BB1F8E71_8ED5_4BEC_9401_A1005D560FE6__INCLUDED_)
