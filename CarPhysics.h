// CarPhysics.h: interface for the CarPhysics class.
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

#if !defined(AFX_CARPHYSICS_H__44BC0541_DBD8_11D6_B5F1_C1F5B9BD9913__INCLUDED_)
#define AFX_CARPHYSICS_H__44BC0541_DBD8_11D6_B5F1_C1F5B9BD9913__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "Vector3.h"
#include "PhysicsServer.h"
#include "PhysicsClient.h"


#define STEER_LIMIT_ANGLE	1.0		// max steering angle (in radians)
#define PHYS_STEERING_SCALE 1.0		// 
#define WHEEL_SLIP			0.15	// slip amount when wheels collide with something

// struct for storing car properties
struct CarData  
{
	// dimensions
	double length;
	double width;
	double height;

	double wheelRadius;	// radius of each wheel
	double wheelMass;	// mass of each wheel
	double chassisOffset;// chassis height above ground 
	double massOffset;	// -- NOT USED -- mass offset relative to center of body

	double mass;	// mass
    double power;	// max engine torque (Newton*m)
	double handling;// -- NOT USED -- value in range: 0.0 - 1.0	
};


class CarPhysics : public PhysicsClient  
{
public:
	CarPhysics();
	virtual ~CarPhysics();

	// inherited from PhysicsClient
	virtual bool update(double step);
	virtual void odeDraw();
	
	bool create( const CarData* carDataPtr );

	void setSteering(double steer) { _steering = steer; }
	void setAccelleration(double acl) { _accel = acl; }
	void setGear(int gear); 

	double			getSpeed();
	double			getTraction() { return _traction; } // -- NOT USED -- 1.0=full traction   0.0=no traction
	const Vector3&	getPosition();
	const double*	getRotation();

	void			setPosition(double x, double y, double z);
	void			setRotation(double x, double y, double z);	

protected:
		void positionCar(double x, double y, double z); // helper function for positioning entire car

		const double*	getGeomPosition(dGeomID geom); // returns position for a specific geom
		const double*	getGeomRotation(dGeomID geom); // returns 4x3 rotation matrix for a specific geom

		// add geom to car geoms
		void addGeom(dGeomID geom, dReal xoff, dReal yoff, dReal zoff, dMatrix3 R);

		// for NeuroDriver simulation (used for syncing physics and visual objects)
		void getHookData(dGeomID geomID, dBodyID bodyID, PhysicsHookData* pData);

protected:

const	CarData*	_pCarData;	// car attributes
		double		_steering;	// current steering (-1=left  1=right)
		double		_accel;		// current acceleration
//		double		_brake;		// current braking power (0=none  1=full)
		double		_speed;		// current car speed
		double		_traction;	// not used
		double		_chassisHgt; // height of chassis

		double		_gearMod; // gear modifier, used for gear switching (transmission)

		/* ODE Stuff */

		// Bodies
		dBodyID _carBody;
		dBodyID	_wheelBody[4];

		// Joints
		dJointID _joint[4]; // wheel joints

		// Geometries
		dSpaceID 		_carSpace;		// space for car geometries
		vector<dGeomID>	_carGeoms;		// vector for car geometries (actually dGeomTransforms)
		dGeomID			_wheelGeom[4];	// geom for each wheel
		PhysGeomData	_wheelData;		// geom data for wheels
};

#endif // !defined(AFX_CARPHYSICS_H__44BC0541_DBD8_11D6_B5F1_C1F5B9BD9913__INCLUDED_)
