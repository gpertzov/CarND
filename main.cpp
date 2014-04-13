// main.cpp - Test NeuroDriver physics layer (built on top of ODE)
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

#include "GenericPhysics.h"
#include "CarPhysics.h"
#include "CarLoader.h"
#include "PhysicsServer.h"

/*
 * Important note about co-ordinate systems:
 *	
 *	Throughout the simulation we use the following coordinate system: 
 *
 *	X = Right
 *	Y = Up
 *	Z = "Into" screen
 *	
 *	Y    Z
 *	|    /
 *	|  / 
 *	|/
 *	+------ X
 *																		        X  Y  Z			
 *	So to denote a 3D point which is 5 units above the origin (0,0,0) - we use (0, 5, 0)
 *	
 *
 *	However, the ODE drawstuff library uses a slightly different system:
 *
 *	Z    Y
 *	|    /
 *	|  / 
 *	|/
 *	+------ X
 *
 *	(Notice how the Z and Y axes are switched)
 *														X  Y  Z
 *  So for ODE, the same 3D point is denoted like this: (0, 0, 5)
 *  (Z is the height axis instead of Y)
 *
 *	
 *
 */


/* globals */

// simulation
static const double	physDelta	= 1.0 / PHYS_UPDATE_RATE;
static const double	physStep	= 0.07 * (50 / PHYS_UPDATE_RATE);
double	physTime	= 0;
double	deltaTime	= 0;
double	throttle	= 0;
double	brake		= 0;
double	steering	= 0;

// ground dimensions
#define groundSizeX 200
#define	groundSizeY	200

// car physics
CarPhysics		*pCarPhys	= NULL;

// walls
GenericPhysics	*wall[4];
#define WALL_WIDTH 1
#define WALL_HEIGHT 5

// ramp
GenericPhysics* ramp = NULL;

// boxes
#define NUM_BOXES	20
GenericPhysics	*box[NUM_BOXES];
#define BOX_SIZE 2

// initial viewpoint 
// (0,0,0) is the center of our "playground"
static float xyz[3]			= { -6.0f , 6.0f, -6.0f};	// position
static float hpr[3]			= { 10.0f, -30.0f, 0.0f};	// yaw, pitch, roll angles (Degrees)	

static float car_alpha = 1.0f;

// camera modes
enum 
{
	E_CAM_MANUAL = 0,
	E_CAM_LOOKAT = 1
} eCameraMode;
static int cam_mode = E_CAM_LOOKAT;


// set camera viewpoint
inline void odeSetViewpoint(double x, double y, double z,
							double yaw, double pitch, double roll)
{
	static float ode_xyz[3];

	xyz[0] = x;
	xyz[1] = y; 
	xyz[2] = z;

	hpr[0] = yaw;
	hpr[1] = pitch;
	hpr[2] = roll;

	// convert to ODE coordinate system by switching the Y and Z values	
	ode_xyz[0] = xyz[0];
	ode_xyz[1] = xyz[2];
	ode_xyz[2] = xyz[1];
	dsSetViewpoint (ode_xyz,hpr);
}


// start simulation
static void start()
{
	odeSetViewpoint(xyz[0], xyz[1], xyz[2], hpr[0], hpr[1], hpr[2]);
  
	printf("CarND [http://www.gpdev.net]\n\n");
	printf(	"Controls:\n\n"
			" W\t- accelerate\n"
			" S\t- brake\n"
			" A\t- left\n"
			" D\t- right\n"
			" Q\t- Drive gear\n"
			" Z\t- Reverse gear\n"
			" SPACE\t- hand-brake\n\n"
			" C\t- toggle camera mode\n"
			" ESC\t- reset car to initial position");
}



// set viewpoint to "look-at" a specific point
static void cameraLookAt(	double eyex, double eyey, double eyez,
							double centerx, double centery, double centerz,
							double upx, double upy, double upz )
{
	// Forward vector
	Vector3 vecN(centerx - eyex, centery - eyey, centerz - eyez);
	vecN.normalize();

	// Up vector
	Vector3 vecU(upx, upy, upz);
	vecU.normalize();

	// Side vector
	Vector3 vecV = vecU.cross(vecN);

	// recompute Up vector
	vecU = vecN.cross(vecV);
/*
	printf ("N: (%2.2f, %2.2f, %2.2f)\nU: (%2.2f, %2.2f, %2.2f)\nV: (%2.2f, %2.2f, %2.2f)\n",
			 vecN.x(), vecN.y(), vecN.z(), vecU.x(), vecU.y(), vecU.z(), vecV.x(), vecV.y(), vecV.z());
*/
	// get angles from vectors
	float pitch	= atan2(-vecN.y(), sqrt(vecN.x() * vecN.x() + vecN.z() * vecN.z()));
	float yaw	= atan2(vecN.x(), vecN.z());

	hpr[0] = -RAD2DEG(yaw)+90;
	hpr[1] = -RAD2DEG(pitch);
	odeSetViewpoint(xyz[0], xyz[1], xyz[2], hpr[0], hpr[1], hpr[2]);
}


// helper function for getting key status
inline bool isKeyPressed(int vKey)
{
	return ( (GetAsyncKeyState(vKey) & 0x8000) != 0 );
}


// process user input
static void processInput(double delta)
{

	// General Controls //


	if ( isKeyPressed(VK_ESCAPE) )
	{
		// reset car position/orientation
		pCarPhys->setPosition(0,0,0);
		pCarPhys->setRotation(0,0,0);
		return;
	}


	// Car Controls //

	// throttle
	if( isKeyPressed('W') )
		throttle += 0.25 * delta;
	else
		throttle -= 2.5  * delta;


	// brake
	if( isKeyPressed('S') )
		brake += 0.25  * delta;
	else if ( isKeyPressed(VK_SPACE)) // hand-brake
		brake = 1;
	else
		brake -= 2.5 * delta;


	// steer left
	if( isKeyPressed('A') )
	{
		steering -= delta;
	}
	else // steer right
	if( isKeyPressed('D') ) 
	{
		steering += delta;
	}
	else // "auto-center"
		steering -=  steering * 12 * delta; 


	// gear up (Drive)
	if ( isKeyPressed('Q') )
	{
		pCarPhys->setGear(1);
		car_alpha = 1.0f;
	}


	// gear down (Reverse)
	if ( isKeyPressed('Z') )
	{
		pCarPhys->setGear(-1);
		car_alpha = 0.5f;
	}



	// enforce bounds
	if (throttle > 1)
		throttle = 1;
	if (throttle < 0)
		throttle = 0;

	if (brake > 1)
		brake = 1;
	if (brake < 0)
		brake = 0;

	if (steering < -1)
		steering = -1;

	if (steering > 1)
		steering = 1;
}



// see also processInput() above
static void command (int cmd)
{
	// toggle camera mode
	if ( cmd == 'C' || cmd == 'c' )
		cam_mode = 1 - cam_mode;
}


// simulation loop
static void simLoop (int pause)
{
	// draw car
	dsSetColorAlpha(0.0f, 0.7f, 0.4f, car_alpha);
	dsSetTexture (DS_NONE);
	pCarPhys->odeDraw();

	// draw ramp
	dsSetColorAlpha (1.0f, 0.0f, 0.0f, 0.7f );
	ramp->odeDraw();

	// draw walls
	dsSetColorAlpha (0.0f, 0.0f, 0.0f, 1.0f);
	dsSetTexture (DS_WOOD);
	for(int i = 0; i != 4; ++i)
		wall[i]->odeDraw();

	// draw boxes
	dsSetColorAlpha (0.7f, 0.3f, 0.0f, 1.0f );
	for(i = 0; i != NUM_BOXES; ++i)
		box[i]->odeDraw();

	if (cam_mode == E_CAM_LOOKAT)
	{
		Vector3 pos = pCarPhys->getPosition();
		cameraLookAt(xyz[0], xyz[1], xyz[2], pos.x(), pos.y(), pos.z(), 0, 1, 0);
	}

	// compute delta time since last call
	static double lastTime = timeGetTime() * 0.001;
	double curTime = timeGetTime() * 0.001;
	deltaTime = (curTime - lastTime);
	physTime += deltaTime;

	if (!pause)
	{
		// process user input
		processInput(deltaTime);

		// set car controls
		pCarPhys->setSteering(steering);
		pCarPhys->setAccelleration(throttle - brake);

		// update physics
		if ( physTime >= physDelta)
		{
			PhysicsServer::getInstance()->update(physStep);
			physTime -= physDelta;
		}

	} // if !pause

	lastTime = curTime;
}



int main (int argc, char **argv)
{

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "textures";

	// init physics sever
	bool success = PhysicsServer::getInstance()->init( groundSizeX, groundSizeY );
	if (!success)
	{
		printf("Error initializing PhysicsServer\n");
		return -1;
	}

	// load car file
	CarLoader loader;
	if (loader.load("car.xml") < 0)
	{
		printf("Error loading car data\n");
		return -1;
	}

	// set car physics properties
	CarData carData;
	carData.length			= loader.getPhysicsAttr("length");
	carData.width			= loader.getPhysicsAttr("width");
	carData.height			= loader.getPhysicsAttr("height");
	carData.wheelRadius		= loader.getPhysicsAttr("wheelRadius");
	carData.wheelMass		= loader.getPhysicsAttr("wheelMass");
	carData.chassisOffset	= loader.getPhysicsAttr("chassisOffset");
	carData.massOffset		= loader.getPhysicsAttr("massOffset"); // not used
	carData.mass			= loader.getPhysicsAttr("mass");
	carData.power			= loader.getPhysicsAttr("power");
	carData.handling		= loader.getPhysicsAttr("handling"); // not used
	
	// create car physics instance
	pCarPhys = new CarPhysics;
	if (!pCarPhys)
	{
		printf("Error creating car physics\n");
		return -1;
	}

	// register car physics with physics server
	PhysicsServer::getInstance()->registerClient(pCarPhys);

	// create car physics according to car data
	if (!(pCarPhys->create( &carData )) )
	{
		printf("Error creating car physics\n");
		SAFE_DELETE(pCarPhys);
		return -1;
	}

	// position car at world's origin (0,0,0)
	pCarPhys->setPosition(0, 1, 0); // 1 unit above ground


	// create walls
	for (int i=0; i != 4; ++i)
	{
		wall[i] = new GenericPhysics;
		PhysicsServer::getInstance()->registerClient(wall[i]);
	}
	
	
	wall[0]->create(groundSizeX*0.95, WALL_WIDTH, WALL_HEIGHT, true); // Right
	wall[0]->setRotation(0, 90, 0);
	wall[0]->setPosition((groundSizeX*0.5)-WALL_WIDTH, 0, 0);

	wall[1]->create(groundSizeX*0.95, WALL_WIDTH, WALL_HEIGHT, true); // Left
	wall[1]->setRotation(0, 90, 0);
	wall[1]->setPosition(-(groundSizeX*0.5)+WALL_WIDTH, 0, 0);

	wall[2]->create(groundSizeY*0.95, WALL_WIDTH, WALL_HEIGHT, true); // Front
	wall[2]->setPosition(0, 0, (groundSizeY*0.5)-WALL_WIDTH);

	wall[3]->create(groundSizeY*0.95, WALL_WIDTH, WALL_HEIGHT, true); // Back
	wall[3]->setPosition(0, 0, -(groundSizeY*0.5)+WALL_WIDTH);

	for (i=0; i != 4; ++i)
	{
		wall[i]->addToEnvironment();
	}

	// create ramp
	ramp = new GenericPhysics;
	PhysicsServer::getInstance()->registerClient(ramp);
	ramp->create(20, 5, 0.4, true);

	PhysGeomData rampData;
	rampData.slip = 0.05;
	ramp->setGeomData(&rampData);
	ramp->setPosition(25, 1.2, 0);
	ramp->setRotation(0, 0, 10);
	ramp->addToEnvironment();

	
	// randomly place some boxes
	srand( (unsigned)time( NULL ) );
	PhysGeomData boxData;
	boxData.slip = 0.1;
	for(i = 0; i != NUM_BOXES; ++i)
	{
		double x = (rand() % (groundSizeX/2)) - (groundSizeX/4);
		double z = (rand() % (groundSizeY/2)) - (groundSizeY/4);

		box[i] = new GenericPhysics;
		PhysicsServer::getInstance()->registerClient(box[i]);
		box[i]->create(BOX_SIZE, BOX_SIZE, BOX_SIZE);
		box[i]->setMass(2);
		box[i]->setGeomData(&boxData);
		
		box[i]->setPosition(x, BOX_SIZE, z);
		box[i]->addToEnvironment();
	}


	// run simulation
	dsSimulationLoop (argc,argv,800,600,&fn);
  

	// cleanup
	PhysicsServer::getInstance()->shutdown();
	SAFE_DELETE(pCarPhys);
	SAFE_DELETE(ramp);
	for(i = 0; i != 4; ++i)
		SAFE_DELETE(wall[i]);
	for(i = 0; i != NUM_BOXES; ++i)
		SAFE_DELETE(box[i]);
	
  return 0;
}
