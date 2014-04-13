// CarLoader.cpp: implementation of the CarLoader class.
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

#include "CarLoader.h"

#define CAR_FILE_CUR_HDR "NeuroDriverCarFile"
#define CAR_FILE_CUR_VER 2

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CarLoader::CarLoader() :	_pVisuals(NULL),
							_pPhysics(NULL)
{

}

CarLoader::~CarLoader()
{

}



int CarLoader::validateDoc()
{
	if (strcmp(_pRoot->Value(), CAR_FILE_CUR_HDR) != 0)
		return -1;

	// check version
	int ver;
	if (_pRoot->Attribute("version", &ver) == NULL)
		return -1;
	if (ver != CAR_FILE_CUR_VER)
		return -1;

	// get Visuals element
	_pVisuals = (TiXmlElement*)_pRoot->FirstChild("Visuals");
	if (_pVisuals == NULL)
		return -1;

	// get Physics element
	_pPhysics = (TiXmlElement*)_pRoot->FirstChild("Physics");
	if (_pPhysics == NULL)
		return -1;

	return 0;
}



double CarLoader::getPhysicsAttr(const char* attrName)
{
	if (_pPhysics == NULL)
		return 0;

	TiXmlElement* pElem = (TiXmlElement*)_pPhysics->FirstChild(attrName);
	if (pElem == NULL)
		return 0;

	double val = atof(pElem->FirstChild()->Value());
	
	return val;	
}


const char*	CarLoader::getMeshPath()
{
	if (_pVisuals == NULL)
		return NULL;

	TiXmlElement* pModelPath = (TiXmlElement*)_pVisuals->FirstChild("mesh");
	if (pModelPath == NULL)
		return NULL;

	return ( pModelPath->FirstChild()->Value() );
}