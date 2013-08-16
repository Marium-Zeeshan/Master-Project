/*
 * MultipleObjects.cpp
 *
 *  Created on: Aug 8, 2013
 *      Author: marium
 */

/*
 * Basic.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: marium
 */

///create 125 (5x5x5) dynamic object
/*
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5
*/

//maximum number of objects (and allow user to shoot additional boxes)
//#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)


///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "MultipleObjects.h"
#include "GlutStuff.h"

#include "btBulletDynamicsCommon.h"

#include <stdio.h>
#include "GLDebugDrawer.h"

#include <iostream>
#include <stdlib.h>

static GLDebugDrawer gDebugDraw;

static double timeStep = 0.0;


void MultipleObjects::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		/*int maxSimSubSteps = m_idle ? 1 : 1;
		if (m_idle)
			ms = 1.0f/420.f;


		int numSimSteps = m_dynamicsWorld->stepSimulation(ms,maxSimSubSteps);
		*/

		//Information of the simulation
		std::cout<<"===================================================="<<std::endl;
		std::cout<<"INFORMATION OF SIMULATION"<<std::endl;
		std::cout<<"===================================================="<<std::endl;
		std::cout<<" Time Step "<< timeStep++ <<std::endl;
		std::cout<<" Gravity  ("<< m_dynamicsWorld->getGravity().getX()<<","<<m_dynamicsWorld->getGravity().getY()<<","<<m_dynamicsWorld->getGravity().getZ()<<")"<<std::endl;

		//std::vector<Triplet> vector1,vector2;

		for(int i=0;i<masses.size();i++){
			masses[i].first->getMotionState()->getWorldTransform(masses[i].second);

			if(masses[i].first->isActive()){
				std::cout<< "Mass" << i <<std::endl;
				std::cout<< "("
					<<masses[i].second.getOrigin().getX()
					<<","
					<< masses[i].second.getOrigin().getY()
					<< ","
					<< masses[i].second.getOrigin().getZ()
					<< ")"
					<< std::endl;
				myfile<<masses[i].second.getOrigin().getX()<<","<<masses[i].second.getOrigin().getY()<<","<<masses[i].second.getOrigin().getZ()<<",";
				//vector1.push_back(Triplet());
				//vector1.push_back({masses[i].second.getOrigin().getX(),masses[i].second.getOrigin().getY(),masses[i].second.getOrigin().getZ()});
				}
			else{ //for non-active masses

				//vector1.push_back(Triplet());
				//vector1.push_back({0,0,0});
				myfile<<"NaN,NaN,NaN,";
			}
			//vector2.insert( vector2.end(),vector1.begin(),vector1.end() );
			//vector1.clear();
			//std::cout<<"@@"<< vector2.front().one_<<vector2.front().two_<<vector2.front().three_<<std::endl;

			//writing in CSV
			//if(myfile.is_open()){
				//myfile<<masses[i].second.getOrigin().getX()<<","<<masses[i].second.getOrigin().getY()<<","<<masses[i].second.getOrigin().getZ()<<",";
			//}

		}
		myfile<<std::endl;

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
		std::cout<<"===================================================="<<std::endl;
	}

	renderme();
	glFlush();
	swapBuffers();

}

void MultipleObjects::getResults(std::vector<std::pair<btRigidBody*,btTransform> > _massInformation){
	/*for (int i=0 ; i<300 ; i++) {
		m_dynamicsWorld->stepSimulation(1/60.f,10);

	    _massInformation->getMotionState()->getWorldTransform(trans);

	                std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
	        }

*/

}

void MultipleObjects::setSimulationTime(double _time){
	time = _time;
	timeStep = time;

}

double MultipleObjects::getSimulationTime(void){
	time = timeStep;
	return time;
}

void MultipleObjects::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

/*btRigidBody* addSphereShape(float rad,float x,float y,float z,float mass){
	//add static objects
	//Body : sphere
	btTransform t;
	t.setIdentity(); //no rotation at all

	t.setOrigin(btVector3(x,y,z));
	btSphereShape* sphere = new btSphereShape(rad);//set the plane at x-z position
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		sphere->calculateLocalInertia(mass,inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,sphere,inertia);
	btRigidBody* body = new btRigidBody(info);

	m_dynamicsWorld->addRigidBody(body);
	m_collisionShapes.push_back(body);

return body;

}
*/

void MultipleObjects::addGround(){
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(60.),btScalar(60.),btScalar(60.)));
	groundShape->initializePolyhedralFeatures();
	//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));


	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		body->setGravity(btVector3(0,-9.81,0));

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);

	}

}


void	MultipleObjects::initPhysics()
{


	///create a few basic rigid bodies
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
	groundShape->initializePolyhedralFeatures();
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));


	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}

	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//SPHERES
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		btSphereShape* sphere = new btSphereShape(4);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(sphere);

		/// Create Dynamic Objects
		//btTransform startTransform;
		//startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(10,10,0);
		if (isDynamic)
			sphere->calculateLocalInertia(mass,localInertia);

/*
		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(2.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));
*/

		btTransform t;
		t.setIdentity(); //no rotation at all
		//t.getBasis().setEulerZYX(0,0,0);
		t.setOrigin(btVector3(0,-3,0));


		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		body->setRestitution(btScalar(1));
		m_dynamicsWorld->addRigidBody(body);


		/////////////////////////////////////////////////
		////TEST
		////////////////////////////////////////////////
		// Add another sphere
		/*
		btSphereShape* SphereShape = new btSphereShape(3);

		btTransform capsuleTransform;
		capsuleTransform.setIdentity();
		capsuleTransform.setOrigin(btVector3(0,5,6));

		btRigidBody *capsuleBody = localCreateRigidBody(btScalar(1),capsuleTransform,SphereShape);
		btTransform fixedpoint;
		fixedpoint.setIdentity();
		btGeneric6DofConstraint *joint0 = new btGeneric6DofConstraint( *capsuleBody, fixedpoint, false );

		m_dynamicsWorld->addConstraint( joint0, true );
		//m_dynamicsWorld->addRigidBody(SphereShape);

*/



		//Add another Sphere


		btSphereShape* sphere2 = new btSphereShape(1);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(sphere2);

		btScalar	mass2(0.f);

		bool isDynamictwo = (mass2 != 0.f);

		btVector3 localInertia2(0,0,0);
		if (isDynamictwo)
			sphere->calculateLocalInertia(mass2,localInertia2);

		btTransform t2;
		t2.setIdentity(); //no rotation at all
		//t2.getBasis().setEulerZYX(0,0,0);
		t2.setOrigin(btVector3(10,5,10));


		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btDefaultMotionState* myMotionState2 = new btDefaultMotionState(t2);
		btRigidBody::btRigidBodyConstructionInfo rbInfotwo(mass2,myMotionState2,sphere2,localInertia);
		btRigidBody* bodytwo = new btRigidBody(rbInfotwo);
		m_dynamicsWorld->addRigidBody(bodytwo);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			//CONSTRAINT
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		/*btVector3 sliderWorldPos(0,10,0);
        btVector3 sliderAxis(1,0,0);

        btScalar angle=0.f;//SIMD_RADS_PER_DEG * 10.f;
        btMatrix3x3 sliderOrientation(btQuaternion(sliderAxis ,angle));
        t.setOrigin(sliderWorldPos);
        //trans.setBasis(sliderOrientation);
        btTransform sliderTransform = t;
*/
		btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*bodytwo, *body, t2, t,true);
		pGen6DOFSpring->setLinearUpperLimit(btVector3(5., 0., 0.));
		pGen6DOFSpring->setLinearLowerLimit(btVector3(-5., 0., 0.));

		pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
		pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.5f));


		pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

		pGen6DOFSpring->enableSpring(0, true);
		pGen6DOFSpring->setStiffness(0, 39.478f);
		pGen6DOFSpring->setDamping(0, 0.5f);
		pGen6DOFSpring->enableSpring(5, true);
		pGen6DOFSpring->setStiffness(5, 39.478f);
		pGen6DOFSpring->setDamping(0, 0.3f);
		pGen6DOFSpring->setEquilibriumPoint();

		m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

			//	}
		//	}
	//}

	}


}

std::pair <btRigidBody*,btTransform> MultipleObjects::addMass(double x,double y,double z,double radius,bool fixed,bool activate){
	//Adding sphere as mass
	btSphereShape* sphere = new btSphereShape(radius);
	m_collisionShapes.push_back(sphere);

	btScalar mass(1.0);

	if(fixed)
		mass = 0.;


	bool isDynamic = (mass != 0.f);
	btVector3 localInertia(0,0,0);

	if(isDynamic)
		sphere->calculateLocalInertia(mass,localInertia);

	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x,y,z));

	btDefaultMotionState* myMotionState = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,sphere,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	body->setRestitution(btScalar(1));//set as elastic
	if (activate)
		activate = DISABLE_DEACTIVATION;

	body->activate(activate);

	masses.push_back(std::make_pair(body,t));

	return std::make_pair(body,t);

}


void MultipleObjects::addConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2){
	btGeneric6DofSpringConstraint* pGen6DOFSpring = new btGeneric6DofSpringConstraint(*body1, *body2, t1, t2,true);
	pGen6DOFSpring->setLinearUpperLimit(btVector3(5., 0., 0.));
	pGen6DOFSpring->setLinearLowerLimit(btVector3(-5., 0., 0.));

	pGen6DOFSpring->setAngularLowerLimit(btVector3(0.f, 0.f, -1.5f));
	pGen6DOFSpring->setAngularUpperLimit(btVector3(0.f, 0.f, 1.5f));

	pGen6DOFSpring->setDbgDrawSize(btScalar(5.f));

	pGen6DOFSpring->enableSpring(0, true);
	pGen6DOFSpring->setStiffness(0, 39.478f);
	pGen6DOFSpring->setDamping(0, 0.5f);
	pGen6DOFSpring->enableSpring(5, true);
	pGen6DOFSpring->setStiffness(5, 39.478f);
	pGen6DOFSpring->setDamping(0, 0.3f);
	pGen6DOFSpring->setEquilibriumPoint();

	m_dynamicsWorld->addConstraint(pGen6DOFSpring, true);

}


void MultipleObjects::Interface(){
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(SCALING*50.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);

	//m_dynamicsWorld->setGravity(btVector3(0,-9.81,0));

	//Opening file
	myfile.open("MassPositions.csv");

	addGround();
// Entering the data

	int NumberofMasses;
	std::cout<< "Enter number of masses"<<std::endl;
	std::cin>>NumberofMasses;


	double mass,x,y,z;
	std::string fixed,activate,connection;

	for(int i = 0;i<NumberofMasses;i++){

		std::cout<<"Enter mass value of mass "<<i<<std::endl;
		std::cin>>mass;
		//std::cout<<<<masses.size()<<std::endl;
		std::cout<<"Enter x coordinate value of mass "<<i<<std::endl;
		std::cin>>x;
		std::cout<<"Enter y coordinate value of mass "<<i<<std::endl;
		std::cin>>y;
		std::cout<<"Enter z coordinate value of mass "<<i<<std::endl;
		std::cin>>z;
		std::cout<<"Mass is fixed? true or false!!"<<std::endl;
		std::cin>>fixed;
		std::cout<<"Do you want to activate this mass in the simulation? t or f"<<std::endl;
		std::cin>>activate;

		bool fixedMass,activateMass,connectionSpring;

		{
		if(fixed == "t")
			fixedMass = true;
		else
			fixedMass = false;

		if(activate == "t")
			activateMass = true;
		else
			activateMass = false;
		}

		std::pair<btRigidBody*,btTransform> Latest = addMass(x,y,z,mass,fixedMass,activateMass);
		myfile<<"Mass Number "<<i<<","<<","<<",";

		m_dynamicsWorld->addRigidBody(Latest.first);

		if(masses.size()>1){
			std::cout<<"Do you want to connect this mass to any other previous mass(true/false)"<<std::endl;
			std::cin>>connection;

			if(connection == "t")
				connectionSpring = true;
			else
				connectionSpring = false;

			if(connectionSpring){
				addConnection(Latest);
			}
		}
	}
	myfile<<std::endl;
}
/*
 * This function is used to connect one mass to another via spring connection
 */
void MultipleObjects::addConnection(std::pair<btRigidBody*,btTransform> MassTransformPair){
	std::string multiple;
	//bool selection = true;

	/*std::cout<<"Do you want to add multiple connections t/f"<<std::endl;
	std::cin>>multiple;

	if(multiple == "t")
		selection = true;
	else
		selection = false;

*/
	int massNo;
	bool connection = true;

	//if(selection){
		while(connection == true){

			std::cout<<"Enter mass number which you wanted to connect with this mass"<<std::endl;
			std::cin>>massNo;
			//std::vector<btRigidBody>::iterator iter;
			//for (iter = masses.begin();iter!= masses.end();iter++){
				//if(iter->getMa)
			if(massNo > (masses.size()-1)){
				std::cout<<"This mass does not exist, Try again"<<std::endl;
				continue;
			}
			addConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);


			std::cout<<"Do you want more connections t/f"<<std::endl;
			std::cin>>multiple;

			if(multiple == "t")
				connection = true;
			else
				connection = false;
	}

	//}
/*	else // for single connection
	{
		addConstraint(MassTransformPair.first,MassTransformPair.second,masses[massNo].first,masses[massNo].second);
	}
*/
}

void	MultipleObjects::clientResetScene()
{
	exitPhysics();
	Interface();
}


void	MultipleObjects::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_dynamicsWorld;

	delete m_solver;

	delete m_broadphase;

	delete m_dispatcher;

	delete m_collisionConfiguration;

	myfile.close();


}







