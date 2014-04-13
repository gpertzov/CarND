// PhysicsServer.h: interface for the PhysicsServer class.
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

#if !defined(AFX_PHYSICSSERVER_H__91F24E34_DAFF_11D6_B5F1_94143056EB10__INCLUDED_)
#define AFX_PHYSICSSERVER_H__91F24E34_DAFF_11D6_B5F1_94143056EB10__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <list>
#include "ode\ode.h"
#include "PhysicsClient.h"

// global physics defs
#define PHYS_GRAVITY 9.81
#define PHYS_MASS_SCALE 0.1
#define PHYS_FORCE_SCALE (PHYS_MASS_SCALE * 0.2)
#define PHYS_STEP_SCALE 4.1

#define PHYS_UPDATE_RATE 80.0


// geometry attributes
struct PhysGeomData
{
	double slip; // slip amount (used for collisions)
};


// Singleton
class PhysicsServer  
{
public:
	virtual ~PhysicsServer();

	bool	init(int groundSizeX, int groundSizeY);
	void	shutdown();

	bool	registerClient(PhysicsClient* client);
	bool	update(double step);

public:
	static PhysicsServer* getInstance() { return &_instance; }

	static void collisionCallback (void *data, dGeomID o1, dGeomID o2);

protected:

	// ODE
	dWorldID		_world;
	dSpaceID		_space;
	dJointGroupID	_contactgroup;
	dSpaceID		_envSpace;
	dGeomID			_ground;
	dGeomID			_wall[4];

	PhysGeomData	_envData;

private:
	PhysicsServer(); // private c-tor

	void checkCollision(void* data, dGeomID o1, dGeomID o2);

private:	
	static PhysicsServer _instance;

private:

	list<PhysicsClient*> _clientList;
};

#endif // !defined(AFX_PHYSICSSERVER_H__91F24E34_DAFF_11D6_B5F1_94143056EB10__INCLUDED_)
