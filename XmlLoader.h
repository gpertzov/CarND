// XmlLoader.h: interface for the XmlLoader class.
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

#if !defined(AFX_XMLLOADER_H__A4ACA61E_FAEE_42E6_B0FE_7036B5B30B92__INCLUDED_)
#define AFX_XMLLOADER_H__A4ACA61E_FAEE_42E6_B0FE_7036B5B30B92__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "tinyxml\tinyxml.h"

// Abstract base class for inheriting XML file loaders
class XmlLoader  
{
// Methods
public:
	XmlLoader();
	virtual ~XmlLoader();

	int	load(const string& fullPath);

protected:

	virtual int validateDoc() = 0;
			int	getVector(TiXmlElement* pVector, double& x, double& y, double& z);
			int	getTriplet(TiXmlElement* pElem, double& a, double& b, double& c);
			int	getPair(TiXmlElement* pElem, double& a, double& b);

// Members
protected:

	TiXmlDocument	_xmlDoc;
	TiXmlElement*	_pRoot;
};

#endif // !defined(AFX_XMLLOADER_H__A4ACA61E_FAEE_42E6_B0FE_7036B5B30B92__INCLUDED_)
