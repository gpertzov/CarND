// GenericPhysics.h: interface for the GenericPhysics class.
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

#if !defined(AFX_GENERICPHYSICS_H__00C8CD0B_122C_4A49_A110_710DF2FB7B6B__INCLUDED_)
#define AFX_GENERICPHYSICS_H__00C8CD0B_122C_4A49_A110_710DF2FB7B6B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "PhysicsServer.h"
#include "PhysicsClient.h"

class GenericPhysics : public PhysicsClient  
{
public:
	GenericPhysics();
	virtual ~GenericPhysics();

	// inherited from PhysicsClient
	virtual bool update(double step);
	virtual void odeDraw();

	// create a box with the specified dimensions
	bool create( double length, double width, double height, bool isStatic = false ); 
	
	void	setMass(double mass);
	void	setGeomData(const PhysGeomData* pData) { memcpy(&_geomData, pData, sizeof(PhysGeomData)); }
	void	setPosition(double x, double y, double z);
	void	setRotation(double x, double y, double z);
	void	addToEnvironment();

protected:

	dBodyID _body;
	dGeomID _geom;

	PhysGeomData _geomData;
};

#endif // !defined(AFX_GENERICPHYSICS_H__00C8CD0B_122C_4A49_A110_710DF2FB7B6B__INCLUDED_)
