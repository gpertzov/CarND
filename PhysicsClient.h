// PhysicsClient.h: interface for the PhysicsClient class.
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

#if !defined(AFX_PHYSICSCLIENT_H__0B22E6D9_13F7_4828_851E_60A18893302D__INCLUDED_)
#define AFX_PHYSICSCLIENT_H__0B22E6D9_13F7_4828_851E_60A18893302D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "ode\ode.h"
#include "smatrix.h"
#include "Vector3.h"

// constants to convert degrees to radians and vice-versa
#define RAD2DEG(a) a*(180.0/M_PI)
#define DEG2RAD(a) a*(M_PI/180.0)

// Physics hooks defs //
enum PhysicsHookType
{
	PHYS_HOOK_CREATE = 0,
	PHYS_HOOK_UPDATE,
	
	PHYS_HOOK_MAX
};


// call-back structures
struct PhysicsHookData
{
	int		geomId;
	double	length;
	double	width;
	double	height;
	double	posx;
	double	posy;
	double	posz;
	sMATRIX rot; // rotation matrix
};

// hook callback prototype
typedef void (*PHYSHOOK_CALLBACK)(PhysicsHookData*, void*);


struct PhysicsHook
{
	PHYSHOOK_CALLBACK	pCB;
	void*				pContext;
};



// abstract base class for inheriting physics classes
class PhysicsClient  
{
public:
	PhysicsClient();
	virtual ~PhysicsClient();

	bool init(dWorldID world, dSpaceID space, dSpaceID envSpace);

	virtual bool update(double step) = 0;
			bool setHook(PhysicsHookType type, PHYSHOOK_CALLBACK hookCB, void* context);

	// for ODE debugging //
	virtual void odeDraw() = 0;

protected:
	dWorldID	_world;		// the world we live in
	dSpaceID	_space;		// the final frontier
	dSpaceID 	_envSpace;	// geometries that are considered part of the environment

	// for syncing physics entities with visual entities (used only in NeuroDriver simulation)
	PhysicsHook _hooks[PHYS_HOOK_MAX]; 
};

#endif // !defined(AFX_PHYSICSCLIENT_H__0B22E6D9_13F7_4828_851E_60A18893302D__INCLUDED_)
