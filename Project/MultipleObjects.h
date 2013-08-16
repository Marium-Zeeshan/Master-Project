/*
 * MultipleObjects.h
 *
 *  Created on: Aug 8, 2013
 *      Author: marium
 */

#ifndef MULTIPLEOBJECTS_H_
#define MULTIPLEOBJECTS_H_

#ifndef MULTIPLEOBJECTS_DEMO_H
#define MULTIPLEOBJECTS_DEMO_H


#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

#include <vector>
#include <sstream>
#include <fstream>


#define PlatformDemoApplication GlutDemoApplication


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;


struct Triplet
{
  int  one_, two_, three_;
};


class MultipleObjects : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup

	std::ofstream myfile;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	public:

	MultipleObjects()
	{
	}

	virtual ~MultipleObjects()
	{
		exitPhysics();
	}

	void	initPhysics();

	void	exitPhysics();

	void Interface();

	void addConstraint(btRigidBody* body1,btTransform t1,btRigidBody* body2,btTransform t2);

	void getResults(std::vector<std::pair<btRigidBody*,btTransform> > _massInformation);

	void setSimulationTime(double _time);

	double getSimulationTime();

	std::pair<btRigidBody*,btTransform> addMass(double x,double y,double z,double radius,bool fixed,bool activate);

	void addConnection(std::pair<btRigidBody*,btTransform> MassTransformPair);

	void addGround();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void	clientResetScene();

	static DemoApplication* Create()
	{
		MultipleObjects* demo = new MultipleObjects;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

private:
	std::vector<std::pair<btRigidBody*,btTransform> > masses;

	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	double time;

	std::vector<Triplet> coordinateToFile;

};

#endif //BASIC_DEMO_H



#endif /* MULTIPLEOBJECTS_H_ */
