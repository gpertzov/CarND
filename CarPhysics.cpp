// CarPhysics.cpp: implementation of the CarPhysics class.
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
#include "CarPhysics.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
CarPhysics::~CarPhysics()
{

}

CarPhysics::CarPhysics() 
{
	// reset all members
	_chassisHgt	= 0;
	_steering	= 0;
	_accel		= 0;
	_speed		= 0;
//	_brake		= 0;
	_traction	= 1;

	_gearMod	= 1;

	_carBody	= 0;

	_wheelBody[0] = 0; // front left wheel
	_wheelBody[1] = 0; // front right wheel
	
	_wheelBody[2] = 0; // rear left wheel
	_wheelBody[3] = 0; // rear right wheel

	_joint[0] = 0; // front left wheel joint
	_joint[1] = 0; // front right wheel joint
	_joint[2] = 0; // rear left wheel joint
	_joint[3] = 0; // rear right wheel joint

	_carSpace	= 0;

	_wheelGeom[0] = 0;  // wheel geometries
	_wheelGeom[1] = 0;
	_wheelGeom[2] = 0;
	_wheelGeom[3] = 0;
}


// set car parameters and create all bodies and geometries
bool CarPhysics::create( const CarData* carDataPtr )
{
	if ( carDataPtr == NULL)
		return false;

	// IMPORTANT NOTE: in ODE - 
	//			       Z-axis is the HEIGHT axis
	//				   Y-axis goes "into" the screen
	//				   X-axis goes from left to right

	// if init was not called (should have been called by PhysicsServer)
	if (_world == 0 || _space == 0)
		return false;

	// store car properties
	_pCarData = carDataPtr;

	// height of chassis only (not entire car height)
	_chassisHgt = 0.2; // TODO: maybe put this in Car file

	// for convenience
	double chassisMass	= _pCarData->mass;
	double len			= _pCarData->length;
	double wdt			= _pCarData->width;
	double hgt			= _pCarData->height;

	// car body
	_carBody = dBodyCreate (_world);

	// set body's mass
	dMass m;
	dMassSetBoxTotal(&m, chassisMass*PHYS_MASS_SCALE,len, wdt, _chassisHgt);
	dBodySetMass (_carBody, &m);

	// create space for car geoms
	_carSpace = dSimpleSpaceCreate(_space);
	
	// create car geoms //
	//
	//	This will define car's shape for collision detection
	//
	//  (TODO: it's probably better to base geoms on actual car mesh)

	// chassis
	dGeomID chassis = dCreateBox(0, len, wdt, _chassisHgt);
    dMatrix3 RI;
    dRSetIdentity (RI);
	addGeom(chassis, 0, 0, 0, RI);

	// fuselage (front)
	dGeomID fuseFront = dCreateBox(0, len*0.8, wdt*1.1, hgt*0.8);
	dMatrix3 R2;
	dRFromEulerAngles(R2, 0, DEG2RAD(-5), 0);
	addGeom(fuseFront, (len*0.2), 0, _pCarData->chassisOffset, R2);

	// fuselage (rear)
	dGeomID fuseRear = dCreateBox(0, len*0.6, wdt*1.3, hgt*0.8);
	dMatrix3 R1;
	dRFromEulerAngles(R1, 0, DEG2RAD(5), 0);
	addGeom(fuseRear, (-len*0.3), 0, _pCarData->chassisOffset, R1);


	// wheels
	for (int i = 0; i != 4; ++i)
	{
		_wheelBody[i] = dBodyCreate (_world);

		// rotate wheel 90deg around X-axis so it "stands"
		dQuaternion q;
		dQFromAxisAndAngle (q, 1, 0, 0, M_PI*0.5);
		dBodySetQuaternion (_wheelBody[i], q);

		// set wheel mass
		dMassSetSphereTotal (&m, _pCarData->wheelMass*PHYS_MASS_SCALE, _pCarData->wheelRadius);
		dBodySetMass (_wheelBody[i], &m);

		// wheel geometry
		_wheelGeom[i] = dCreateSphere (0, _pCarData->wheelRadius);
		_wheelData.slip = WHEEL_SLIP; 
		dGeomSetData(_wheelGeom[i], (void*)&_wheelData);
		dGeomSetBody (_wheelGeom[i], _wheelBody[i]);

		// add wheel geom to car space
		dSpaceAdd(_carSpace, _wheelGeom[i]);
	}

	// position car correctly before setting body/wheels hinges
	positionCar(0, 0, 0);
	
	// front and back wheel hinges
	for (i = 0; i != 4; i++)
	{
		// create double-axis hinge
		_joint[i] = dJointCreateHinge2 (_world, 0);

		// attach hinge to wheel
		dJointAttach(_joint[i], _carBody, _wheelBody[i]);

		const dReal *a = dBodyGetPosition (_wheelBody[i]);

		// anchor is at wheel position
		dJointSetHinge2Anchor (_joint[i], a[0], a[1], a[2]);
		
		// axis between car and wheel (steering axis)
		dJointSetHinge2Axis1 (_joint[i], 0, 0, 1);

		// wheel rotation axis
		dJointSetHinge2Axis2 (_joint[i], 0, 1, 0);
	}

	// set joints suspension (TODO: read suspension values from Car file)
	for (i = 0; i != 2; ++i) 
	{
		// Front wheels
		dJointSetHinge2Param (_joint[i], dParamSuspensionERP,0.6);
		dJointSetHinge2Param (_joint[i], dParamSuspensionCFM,0.05);
	}

	for (i = 2; i != 4; ++i) 
	{
		// Rear wheels
		dJointSetHinge2Param (_joint[i], dParamSuspensionERP,0.6);
		dJointSetHinge2Param (_joint[i], dParamSuspensionCFM,0.05);
	}


	// lock rear wheels along the steering axis
	for (i = 2; i != 4; ++i)
	{
		// set stops to make sure wheels always stay in alignment
		dJointSetHinge2Param (_joint[i], dParamLoStop, 0);
		dJointSetHinge2Param (_joint[i], dParamHiStop, 0);
	}


	// reset real-time parameters
	_steering	= 0;
	_accel		= 0;
//	_brake		= 0;
	_speed		= 0;
	_traction	= 1;

	// handle creation hook (used only in NeuroDriver simulation)
	PHYSHOOK_CALLBACK	pCallback	= _hooks[PHYS_HOOK_CREATE].pCB;
	void*				pContext	= _hooks[PHYS_HOOK_CREATE].pContext;
	if (pCallback != NULL)
	{
		PhysicsHookData data;
		getHookData(_carGeoms[0], _carBody, &data);
		pCallback(&data, pContext);

		// wheels
		for (int i = 0; i != 4; ++i)
		{
			getHookData(_wheelGeom[i], _wheelBody[i], &data);
			pCallback(&data, pContext);

		}
	}

	return true;
}




// update simulation state
bool CarPhysics::update(double step)
{
	if (_world == 0 || _space == 0)
		return false;

	// simulate drag/friction as a factor of speed
	dReal drag = 1.5 * _speed * PHYS_FORCE_SCALE;

    // motor
	_speed += _accel - drag;

	if (_speed < 0) _speed = 0;


	// The following determines the car's drive:
	//
	//		Drive		startWheel		endWheel
	//		------------------------------------
	//		front			0				2	
	//		rear			2				4
	//		4WD				0				4
	//
	int startWheel	= 2; 
	int endWheel	= 4;


	dReal totalForce = _pCarData->power * PHYS_FORCE_SCALE;
	if (_accel < 0)
	{
		totalForce = 30; // TODO: put braking force in Car file
	}
	dReal force	 = totalForce/(endWheel - startWheel); // force per wheel

	// apply force to wheels
	for(int i = startWheel; i != endWheel; ++i)
	{
		dJointSetHinge2Param (_joint[i], dParamVel2, - _gearMod * _speed);
		dJointSetHinge2Param (_joint[i], dParamFMax2, force);
	}

    // steering //

	// compute current wheel angle
	dReal langle = dJointGetHinge2Angle1 (_joint[0]);

	// compute velocity needed to bring wheel to desired angle
    dReal vl = (_steering * STEER_LIMIT_ANGLE) - langle;
    if (vl > 0.1) vl = 0.1;
    if (vl < -0.1) vl = -0.1;
    vl *= 10.0;

    dJointSetHinge2Param (_joint[0],dParamVel, vl);
    dJointSetHinge2Param (_joint[0],dParamFMax, 0.2);
    dJointSetHinge2Param (_joint[0],dParamLoStop, -STEER_LIMIT_ANGLE);
    dJointSetHinge2Param (_joint[0],dParamHiStop, STEER_LIMIT_ANGLE);
    dJointSetHinge2Param (_joint[0],dParamFudgeFactor, 0.1);


	dReal rangle = dJointGetHinge2Angle1 (_joint[1]);

    dReal vr = (_steering*STEER_LIMIT_ANGLE) - rangle;
    if (vr > 0.1) vr = 0.1;
    if (vr < -0.1) vr = -0.1;
    vr *= 10.0;

    dJointSetHinge2Param (_joint[1],dParamVel, vr);
    dJointSetHinge2Param (_joint[1],dParamFMax, 0.2);
    dJointSetHinge2Param (_joint[1],dParamLoStop, -STEER_LIMIT_ANGLE);
    dJointSetHinge2Param (_joint[1],dParamHiStop, STEER_LIMIT_ANGLE);
    dJointSetHinge2Param (_joint[1],dParamFudgeFactor, 0.1);


	// handle update hook (used only in NeuroDriver simulation)
	PHYSHOOK_CALLBACK	pCallback	= _hooks[PHYS_HOOK_UPDATE].pCB;
	void*				pContext	= _hooks[PHYS_HOOK_UPDATE].pContext;
	if (pCallback != NULL)
	{
		// car geom
		PhysicsHookData data;
		getHookData(_carGeoms[0], _carBody, &data);
		pCallback(&data, pContext);

		// wheels
		for (int i = 0; i != 4; ++i)
		{
			getHookData(_wheelGeom[i], _wheelBody[i], &data);
			pCallback(&data, pContext);

		}

	}

	return true;
}


// set gear ( 1 = Drive, 0 = Neutral, -1 = Reverse )
void CarPhysics::setGear(int gear)
{
	if (gear < -1 || gear > 1)
		return;

	// TODO: add Gear Ratios to Car file, take _gearMod from an array of ratios
	if (gear == -1)
		_gearMod = -0.1;
	else
		_gearMod = gear;
}


// return speed
double  CarPhysics::getSpeed()
{
	assert( _carBody != NULL );

	static Vector3 velocity;
	const dReal* vel = dBodyGetLinearVel(_carBody);
	velocity.set(vel[0], vel[1], vel[2]);

	return (velocity.mag() * 3.6);
}


// get car position
const Vector3& CarPhysics::getPosition()
{
	const double *pos = getGeomPosition(_carGeoms[0]);

	// NOTE: we switch the Y and Z axes because in ODE, Z is the height axis
	static Vector3 retPos;
	retPos.set(pos[0], pos[2], pos[1]);

	return (retPos);
}


// get car rotation (returns a 4x3 rotation matrix)
const double* CarPhysics::getRotation()
{
	return getGeomRotation(_carGeoms[0]);
}


// set car's position
void CarPhysics::setPosition(double x, double y, double z)
{
	// NOTE: we switch the Y and Z axes because in ODE - Z is the height axis
	positionCar(x, z, y);
}


// set car rotation (Euler angles around x,y,z axes)
void CarPhysics::setRotation(double x, double y, double z)
{
	// reset all velocities	
	dBodySetLinearVel(_carBody,0,0, 0);
	dBodySetAngularVel(_carBody,0,0,0);
	for(int i = 0; i != 4; ++i)
	{
		 dBodySetLinearVel(_wheelBody[i],0,0, 0);
		 dBodySetAngularVel(_wheelBody[i],0,0,0);
	}

	// to convert from our axes and units to ODE we
	// switch the Y and Z axes because in ODE, Z is the height axis

	// convert to radians
	double phi		= DEG2RAD(x);
	double theta	= DEG2RAD(z);
	double psi		= DEG2RAD(y);

	dMatrix3 R;
	dRFromEulerAngles (R, phi, theta, psi);
	dBodySetRotation(_carBody, R);
}



// position entire car (global world coordinates)
void CarPhysics::positionCar(double x, double y, double z)
{
	dMass m;

	// reset all velocities	
	dBodySetLinearVel(_carBody,0,0, 0);
	dBodySetAngularVel(_carBody,0,0,0);
	for(int i = 0; i != 4; ++i)
	{
		 dBodySetLinearVel(_wheelBody[i],0,0, 0);
		 dBodySetAngularVel(_wheelBody[i],0,0,0);
	}
	
	dBodySetPosition(_carBody, x, y, z + _pCarData->chassisOffset);

	// position wheels relative to chassis 
	// TODO: take offsets from vehicle mesh or Car file
	double xoff = _pCarData->length*0.4;
	double yoff = _pCarData->width*0.5;
	double zoff = _pCarData->wheelRadius;

	// front left
	dBodySetPosition (_wheelBody[0],  xoff,  yoff, zoff);
	//front right
	dBodySetPosition (_wheelBody[1],  xoff, -yoff, zoff);
	// rear left
	dBodySetPosition (_wheelBody[2], -xoff,  yoff, zoff);
	// rear right
	dBodySetPosition (_wheelBody[3], -xoff, -yoff, zoff);
}


// get position of a specific geom
const double* CarPhysics::getGeomPosition(dGeomID geom)
{
	const dReal *pos = dGeomGetPosition (geom);
	if (dGeomGetClass(geom) != dGeomTransformClass)
		return pos;

	// handle transformed geom
	const dReal *R	= dGeomGetRotation (geom);

	dGeomID		g2		= dGeomTransformGetGeom (geom);
	const dReal *pos2	= dGeomGetPosition (g2);
	const dReal *R2		= dGeomGetRotation (g2);

	static dVector3 actual_pos;
	dMULTIPLY0_331 (actual_pos,R,pos2);
	actual_pos[0] += pos[0];
	actual_pos[1] += pos[1];
	actual_pos[2] += pos[2];

	return actual_pos;

}


// get rotation of a specific geom (returns a 4x3 rotation matrix)
const double* CarPhysics::getGeomRotation(dGeomID geom)
{
	const dReal *R	= dGeomGetRotation (geom);
	if (dGeomGetClass(geom) != dGeomTransformClass)
		return R;

	// handle transformed geom
	dGeomID		g2		= dGeomTransformGetGeom (geom);
	const dReal *pos2	= dGeomGetPosition (g2);
	const dReal *R2		= dGeomGetRotation (g2);
	static dMatrix3 actual_R;	
	dMULTIPLY0_333 (actual_R, R, R2);

	double RR[12];
	memcpy(RR, actual_R, sizeof(dMatrix3));

	return actual_R;
}



//	add geom to car geoms
//
//		offsets and rotation matrix (R) are relative to car body (chasis)
void CarPhysics::addGeom(dGeomID geom, dReal xoff, dReal yoff, dReal zoff, dMatrix3 R)
{
	// create geom transform to encapsulate new geom
	dGeomID xform = dCreateGeomTransform(_carSpace);
	dGeomTransformSetCleanup(xform, 1);
	dGeomTransformSetInfo(xform, 0);

	// set encapsulated geom
	dGeomTransformSetGeom(xform, geom);

	// attach transform to car body
	dGeomSetBody(xform, _carBody);

	// set translation/rotation of encapsulated geom
	dGeomSetPosition(geom, xoff, yoff, zoff);
	dGeomSetRotation(geom, R);
	
	// insert new transform to car geoms vector
	_carGeoms.push_back(xform);
}



// get physics hook data (used only in NeuroDriver simulation)
void CarPhysics::getHookData(dGeomID geomID, dBodyID bodyID, PhysicsHookData* pData)
{
		dReal aabb[6];
		dGeomGetAABB (geomID, aabb);
		const dReal *pos = getGeomPosition (geomID);

		ZeroMemory(pData, sizeof(PhysicsHookData));
		pData->geomId = (int)geomID;
		pData->length	= aabb[1] - aabb[0];
		pData->width	= aabb[3] - aabb[2];
		pData->height = aabb[5] - aabb[4];
		pData->posx = pos[0];
		pData->posy = pos[2];
		pData->posz = pos[1];

		const dReal *rot = getGeomRotation (geomID);
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
		
		pData->rot._11 = rot[0];	pData->rot._12 = rot[8];	pData->rot._13 = rot[4];	pData->rot._14 = 0;
		pData->rot._21 = rot[2];	pData->rot._22 = rot[10];	pData->rot._23 = rot[6];	pData->rot._24 = 0;
		pData->rot._31 = rot[1];	pData->rot._32 = rot[9];	pData->rot._33 = rot[5];	pData->rot._34 = 0;
		pData->rot._41 = 0;			pData->rot._42 = 0;			pData->rot._43 = 0;			pData->rot._44 = 1;
}



// helper function for drawing car with ODE "drawstuff" library
void CarPhysics::odeDraw()
{
#ifdef ND_USE_ODE_DRAW
	dVector3 sides;
	for (int i = 0; i != _carGeoms.size(); ++i)
	{
	    dGeomBoxGetLengths (dGeomTransformGetGeom(_carGeoms[i]), sides);	
		dsDrawBoxD (getGeomPosition(_carGeoms[i]), getGeomRotation(_carGeoms[i]), sides);
	}
	
	for (i=0; i != 4; i++) 
		dsDrawCylinderD (getGeomPosition(_wheelGeom[i]), getGeomRotation(_wheelGeom[i]), 0.3f, _pCarData->wheelRadius);
#endif
}

