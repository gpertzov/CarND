// PhysicsClient.cpp: implementation of the PhysicsClient class.
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

#include "PhysicsClient.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

PhysicsClient::PhysicsClient() :	_world(0),
									_space(0),
									_envSpace(0)
{
	ZeroMemory(&_hooks, sizeof(_hooks));
}


PhysicsClient::~PhysicsClient()
{

}


// this function is called by the PhysicsServer for each registered client
bool PhysicsClient::init(dWorldID world, dSpaceID space, dSpaceID envSpace)
{
	if (world == 0 || space == 0 || envSpace == 0)
		return false;

	_world			= world;
	_space			= space;
	_envSpace		= envSpace;

	return true;
}


// set physics hook (only used in NeuroDriver simulation)
bool PhysicsClient::setHook(PhysicsHookType type, PHYSHOOK_CALLBACK hookCB, void* context)
{
	if (type >= PHYS_HOOK_MAX)
		return false;

	_hooks[type].pCB		= hookCB;
	_hooks[type].pContext	= context;

	return true;
}
