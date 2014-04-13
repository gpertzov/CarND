// XmlLoader.cpp: implementation of the XmlLoader class.
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

#include "stdafx.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
#include "Mmgr\mmgr.h"
#endif

#include "XmlLoader.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

XmlLoader::XmlLoader() : _pRoot(NULL)
{

}

XmlLoader::~XmlLoader()
{

}

// load XML document
int	XmlLoader::load(const string& fullPath)
{
	if ( !_xmlDoc.LoadFile(fullPath.c_str()) )
		return -1;

	_pRoot = _xmlDoc.RootElement();
	if (_pRoot == NULL)
		return -1;

	return validateDoc();
}


// TODO: better use getTriplet instead
int	XmlLoader::getVector(TiXmlElement* pVector, double& x, double& y, double& z)
{
	if (pVector == NULL)
		return -1;

	double dval = 0;

	if (pVector->Attribute("x", &dval) == NULL)
		return -1;

	x = dval;

	if (pVector->Attribute("y", &dval) == NULL)
		return -1;

	y = dval;

	if (pVector->Attribute("z", &dval) == NULL)
		return -1;

	z = dval;

	return 0;
}



int	XmlLoader::getTriplet(TiXmlElement* pElem, double& a, double& b, double& c)
{
	if (pElem == NULL)
		return -1;

	const char* valStr = pElem->FirstChild()->Value();
	if (valStr == NULL)
		return -1;

	float x,y,z;
	if (sscanf(valStr, "%f,%f,%f", &x, &y, &z) != 3)
		return -1;

	a = x ;
	b = y ;
	c = z ;

	return 0;
}


int	XmlLoader::getPair(TiXmlElement* pElem, double& a, double& b)
{
	if (pElem == NULL)
		return -1;

	const char* valStr = pElem->FirstChild()->Value();
	if (valStr == NULL)
		return -1;

	float x, y;
	if (sscanf(valStr, "%f,%f", &x, &y) != 2)
		return -1;

	a = x;
	b = y;

	return 0;
}