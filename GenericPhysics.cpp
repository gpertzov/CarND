// GenericPhysics.cpp: implementation of the GenericPhysics class.
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

#include "PhysicsServer.h"
#include "GenericPhysics.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

GenericPhysics::GenericPhysics() :	_body(0),
									_geom(0)
{

}

GenericPhysics::~GenericPhysics()
{

}


// create a box with the specified dimensions
//	 
//	isStatic - specifies if object should be affected by physics forces
//
bool GenericPhysics::create( double length, double width, double height, bool isStatic /* (default: false )*/)
{
	// IMPORTANT NOTE: in ODE - 
	//			       Z-axis is the HEIGHT axis
	//				   Y-axis goes "into" the screen
	//				   X-axis goes from left to right (same as in graphics)

	if (_world == 0 || _space == 0)
		return false;

	// create geometry
	_geom = dCreateBox (0, length, width, height);
	ZeroMemory(&_geomData, sizeof(_geomData));
	dGeomSetData(_geom, (void*)&_geomData);
	
	// is not static - attach geom to body
	if ( !isStatic )
	{
		// create body
		_body = dBodyCreate (_world);

		// set body mass
		dMass m;
		dMassSetBox (&m, 1, length, width, height);
		dBodySetMass (_body, &m);
		dGeomSetBody (_geom, _body);
	}


	// handle creation hook (used only in NeuroDriver simulation)
	PHYSHOOK_CALLBACK	pCallback	= _hooks[PHYS_HOOK_CREATE].pCB;
	void*				pContext	= _hooks[PHYS_HOOK_CREATE].pContext;
	if (pCallback != NULL)
	{
		dReal aabb[6];
		dGeomGetAABB (_geom, aabb);
		const dReal *pos = dGeomGetPosition (_geom);
			
		PhysicsHookData data;
		ZeroMemory(&data, sizeof(data));
		data.geomId = (int)_geom;
		data.length	= aabb[1] - aabb[0];
		data.width	= aabb[3] - aabb[2];
		data.height = aabb[5] - aabb[4];
		data.posx = pos[0];
		data.posy = pos[2];
		data.posz = pos[1];

		pCallback(&data, pContext);
	}

	return true;
}


// update simulation state
bool GenericPhysics::update(double step)
{
	if (_body == 0 || _geom == 0)
		return false;

	// in GenericPhysics there is nothing to update so we just return //



	// handle update hook (used only in NeuroDriver simulation)
	PHYSHOOK_CALLBACK	pCallback	= _hooks[PHYS_HOOK_UPDATE].pCB;
	void*				pContext	= _hooks[PHYS_HOOK_UPDATE].pContext;
	if (pCallback != NULL)
	{
		const dReal *pos = dGeomGetPosition (_geom);

		PhysicsHookData data;
		ZeroMemory(&data, sizeof(data));
		data.geomId = (int)_geom;
		data.posx = pos[0];
		data.posy = pos[2];
		data.posz = pos[1];

		const dReal *rot = dGeomGetRotation (_geom);
		/* rot is a 4x3 rotation matrix: 
		 *	  X		  Y		  Z 
		 *	rot[0]	rot[1]	rot[2]	rot[3]
		 *	rot[4]	rot[5]	rot[6]	rot[7]	
		 *	rot[8]	rot[9]	rot[10]	rot[11]
		 * 	
		 *
		 * Because in ODE, Z is the height axis and in our world, Y is the height axis
		 * We switch between the Y and Z axes and get the following matrix:
		 *	rot[0]	rot[2]	rot[1]	rot[3]
		 *	rot[4]	rot[6]	rot[5]	rot[7]	
		 *	rot[8]	rot[10]	rot[9]	rot[11]
		 *
		 * Then we transpose the above matrix to match our Left-Handed coords and get:
		 *	rot[0]	rot[4]	rot[8]	
		 *	rot[2]	rot[6]	rot[10]
		 *	rot[1]	rot[5]	rot[9]
		 */
		
		data.rot._11 = rot[0];	data.rot._12 = rot[8];	data.rot._13 = rot[4];	data.rot._14 = 0;
		data.rot._21 = rot[2];	data.rot._22 = rot[10];	data.rot._23 = rot[6];	data.rot._24 = 0;
		data.rot._31 = rot[1];	data.rot._32 = rot[9];	data.rot._33 = rot[5];	data.rot._34 = 0;
		data.rot._41 = 0;		data.rot._42 = 0;		data.rot._43 = 0;		data.rot._44 = 1;

		pCallback(&data, pContext);
	}

	return true;
}


// set body mass
void GenericPhysics::setMass(double mass)
{
	if (_body == 0 || _geom == 0)
		return;

	dMass m;
	dBodyGetMass(_body, &m);
	dMassAdjust (&m, mass*PHYS_MASS_SCALE);
	dBodySetMass (_body, &m);
}


// set position (world coords)
void GenericPhysics::setPosition(double x, double y, double z)
{
	// NOTE: we switch the Y and Z axes because in ODE - Z is the height axis
	dGeomSetPosition(_geom, x, z, y);
}


// set rotation (Euler angles around x,y,z axes)
void GenericPhysics::setRotation(double x, double y, double z)
{
	// to convert from our axes and units to ODE we 
	// switch the Y and Z axes because in ODE - Z is the height axis
	
	// convert to radians
	double phi		= DEG2RAD(x);
	double theta	= DEG2RAD(z);
	double psi		= DEG2RAD(y);

	dMatrix3 R;
	dRFromEulerAngles (R, phi, theta, psi);
	dGeomSetRotation(_geom, R);
}


// add geometry to simulation space
void GenericPhysics::addToEnvironment()
{
	dSpaceAdd(_space, _geom);
}


// helper function for drawing car with ODE "drawstuff" library
void GenericPhysics::odeDraw()
{
#ifdef ND_USE_ODE_DRAW
	dVector3 dims;
	dGeomBoxGetLengths (_geom,dims);
	dsDrawBoxD (dGeomGetPosition(_geom),dGeomGetRotation(_geom),dims);
#endif
}