// PhysicsServer.cpp: implementation of the PhysicsServer class.
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
#include <cmath>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
//#define new DEBUG_NEW
#include "Mmgr\mmgr.h"
#endif

#include "PhysicsServer.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

// create static instance
PhysicsServer PhysicsServer::_instance;

// constructor
PhysicsServer::PhysicsServer() :	_world(0),
									_space(0),
									_contactgroup(0),
									_envSpace(0),
									_ground(0)
{
	ZeroMemory(_wall, sizeof(_wall));
}


PhysicsServer::~PhysicsServer()
{
	shutdown();
}


// initialize server (supply world dimensions)
bool PhysicsServer::init(int groundSizeX, int groundSizeY)
{
	if (groundSizeX <= 20 || groundSizeY <= 20)
		return false;

	// create global ODE world
	_world = dWorldCreate();

	// create global space
	_space = dHashSpaceCreate(0);

	// create group for handling collisions
	_contactgroup = dJointGroupCreate (0);

	// make ground a bit slippery :)
	_envData.slip = 0.15; 
	// TODO: let user specify ground surface properties

	// set world properties
	dWorldSetGravity (_world,0,0,-(PHYS_GRAVITY*PHYS_MASS_SCALE));
	dWorldSetERP (_world, 0.6);

	// create ground and walls (0,0,0 is the ground's center)
	// and put them in their own space
	_envSpace = dSimpleSpaceCreate(_space);
	_ground = dCreatePlane (_envSpace,0, 0, 1, 0);
	dGeomSetData(_ground, (void*)&_envData);

	_wall[0] = dCreatePlane(_envSpace,  1, 0,  0, -(groundSizeX/2.0) ); // a,b,c,d
	_wall[1] = dCreatePlane(_envSpace, -1, 0,  0, -(groundSizeX/2.0) ); // a,b,c,d
	_wall[2] = dCreatePlane(_envSpace,  0, 1,  0, -(groundSizeY/2.0) ); // a,b,c,d
	_wall[3] = dCreatePlane(_envSpace,  0, -1, 0, -(groundSizeY/2.0) ); // a,b,c,d

	return true;
}


// shutdown server (cleanup)
void PhysicsServer::shutdown()
{
	// clear client list
	_clientList.clear();

	// cleanup
	if(_contactgroup)
	{
		dJointGroupDestroy (_contactgroup);
		_contactgroup = NULL;
	}

	if (_space)
	{
		dSpaceDestroy (_space);
		_space = NULL;
	}

	if (_world)
	{
		dWorldDestroy (_world);
		_world = NULL;
	}
}


// register a physics client with the physics server
bool PhysicsServer::registerClient(PhysicsClient* client)
{
	if (client == NULL)
		return false;

	// init client
	bool success = client->init(_world, _space, _envSpace );

	// add client to list
	if (success)
		_clientList.push_back(client);

	return success;
}


// static callback for collision handling
void PhysicsServer::collisionCallback (void *data, dGeomID o1, dGeomID o2)
{
	assert( data != NULL);

	PhysicsServer* serverPtr = (PhysicsServer*)data;
	serverPtr->checkCollision(serverPtr, o1, o2);
}


// main collision handling function
void PhysicsServer::checkCollision(void *data, dGeomID o1, dGeomID o2)
{
	int i,n;
    if( dGeomIsSpace( o1 ) || dGeomIsSpace( o2 ) )
	{
		dSpaceCollide2( o1, o2, data, &collisionCallback );
	}
	else
	{
		int		mode	= 0;
		double	slip1	= 0;
		double	slip2	= 0;

		// get bodies
		dBodyID body1 = dGeomGetBody(o1);
		dBodyID body2 = dGeomGetBody(o2);

		// get geom data
		PhysGeomData* pData1 = (PhysGeomData*)dGeomGetData(o1);
		PhysGeomData* pData2 = (PhysGeomData*)dGeomGetData(o2);

		// set contact params according to geom data
		if (pData1 != NULL)
			slip1 = pData1->slip;
	
		if (pData2 != NULL)
			slip2 = pData2->slip;
	
		// set mode
		if (slip1 != 0)
			mode |= dContactSlip1;

		if (slip2 != 0)
			mode |= dContactSlip2;

		static const int N = 8; // max number of contact points
		dContact contact[N];
		n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
		if (n > 0) 
		{
			for (i=0; i<n; i++) 
			{
			  contact[i].surface.mode = mode; /*dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM;*/
			  contact[i].surface.mu = dInfinity;
			  contact[i].surface.slip1 = slip1;
			  contact[i].surface.slip2 = slip2;
//			  contact[i].surface.soft_erp = 0.7;
//			  contact[i].surface.soft_cfm = 0.1;
			  dJointID c = dJointCreateContact (_world, _contactgroup, &contact[i]);
			  dJointAttach (c, body1, body2);
			}
		}
	}

}



// update simulation state
bool PhysicsServer::update(double step)
{
	if (_world == 0 || _space == 0)
		return false;

	// handle collisions
	dSpaceCollide (_space, this, &collisionCallback);

    // update all clients
	list<PhysicsClient*>::const_iterator iter = _clientList.begin();
	while(iter != _clientList.end() )
	{
		(*iter)->update(step);
		iter++;
	}
 
	// step world
    dWorldStep (_world, step);

    // remove all contact joints
    dJointGroupEmpty (_contactgroup);

	return true;
}