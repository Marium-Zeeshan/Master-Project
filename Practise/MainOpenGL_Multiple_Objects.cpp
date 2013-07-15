#include <iostream>
#include <SDL.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <vector>
#include "btBulletDynamicsCommon.h"
#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <windowsx.h>

GLUquadricObj* quad;
//just for now!!! make it globally just for the sake of simplicity
btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionconfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;
std::vector<btRigidBody*> bodies;


btRigidBody* addSphereShape(float rad,float x,float y,float z,float mass){
	//add static objects 
	//Body : sphere
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(x,y,z));
	btSphereShape* sphere  = new btSphereShape(rad);//set the plane at x-z position
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		sphere->calculateLocalInertia(mass,inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,sphere,inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;

}
//********************* Cylinder **************************
// In cylinder half of the dimensions value is to be used

btRigidBody* addCylinderShape(float d,float h,float x,float y,float z,float mass){
	//Body : Cylinder
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(x,y,z));
	btCylinderShape* cylinder  = new btCylinderShape(btVector3(d/2.0,h/2.0,d/2.0));//set the plane at x-z position
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		cylinder->calculateLocalInertia(mass,inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,cylinder,inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;

}

void renderCylinder(btRigidBody* cylinder)
{
	if(cylinder->getCollisionShape()->getShapeType()!= CYLINDER_SHAPE_PROXYTYPE)	//cylinder should have to be cylinder_shape_proxytype
		return;

	glColor3f(1,0,0);
	btVector3 extent = ((btCylinderShape*) cylinder->getCollisionShape())->getHalfExtentsWithoutMargin();
	btTransform t;
	cylinder->getMotionState()->getWorldTransform(t); //position of the current sphere
	float mat[16];//4X4
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current cylinder
	//Since in openGL the centre of the cylinder is at the bottom part of the cylinder where as in bullet physics the centre should be at the cnetre of mass.
	//to resolve this, just translate the object transformation
	glTranslatef(0,extent.y(),0);
	//rotate the cylinder, it is important because bullet physics put the cyliner on y-axis where as opengl put it on x-axis
	glRotatef(90,1,0,0);
	gluCylinder(quad,extent.x(),extent.x(),extent.y()*2.0,20, 20); //rendering cylinder ; 20 and 20 is resolution ; multiply by 2 to get the full height
	glPopMatrix();

}

//################# Cone ############################
// In cone half of the dimensions value is to be used

btRigidBody* addConeShape(float d,float h,float x,float y,float z,float mass){
	//Body : Cylinder
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(x,y,z));
	btConeShape* cone  = new btConeShape(d,h);//dia, and full height
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		cone->calculateLocalInertia(mass,inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,cone,inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;

}

void renderCone(btRigidBody* cone)
{
	if(cone->getCollisionShape()->getShapeType()!= CONE_SHAPE_PROXYTYPE)	//cone should have to be cone_shape_proxytype
		return;

	glColor3f(1,0,0);
	float r = ((btConeShape*) cone->getCollisionShape())->getRadius();
	float h = ((btConeShape*) cone->getCollisionShape())->getHeight();

	btTransform t;
	cone->getMotionState()->getWorldTransform(t); //position of the current sphere
	float mat[16];//4X4
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current cone
	//Since in openGL the centre of the cone is at the bottom part of the cone where as in bullet physics the centre should be at the centre of mass.
	//to resolve this, just translate the object transformation
	glTranslatef(0,h/2.0,0);//divide by 2 because it is the full height
	//rotate the cone, it is important because bullet physics put the cone on y-axis where as opengl put it on x-axis
	glRotatef(90,1,0,0);
	gluCylinder(quad,0,r,h,20, 20); //rendering cone ; 20 and 20 is resolution ; top of the cone is zero hence zero is placed, whereas rand h are full originally
	glPopMatrix();

}

//################# Box ############################
// In cone half of the dimensions value is to be used

btRigidBody* addBoxShape(float width,float height,float depth,float x,float y,float z,float mass){
	//Body : Cylinder
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(x,y,z));
	btBoxShape* box  = new btBoxShape(btVector3(width/2.0,height/2.0,depth/2.0));
	btVector3 inertia(0,0,0);
	if(mass!= 0.0) //dynamic
		box->calculateLocalInertia(mass,inertia);

	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass,motion,box,inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	return body;

}

void renderBox(btRigidBody* box)
{
	if(box->getCollisionShape()->getShapeType()!= BOX_SHAPE_PROXYTYPE)	//box should have to be box_shape_proxytype
		return;

	glColor3f(1,0,0);
	btVector3 extent = ((btBoxShape*) box->getCollisionShape())->getHalfExtentsWithoutMargin();
	
	btTransform t;
	box->getMotionState()->getWorldTransform(t); //position of the current box
	float mat[16];//4X4
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current cone
	
	//draw a box for each of the faces(do it just as the OpenGl as Bullet and OpenGL works the same for box)
	glBegin(GL_QUADS);
		glVertex3f(-extent.x(),extent.y(),-extent.z());
		glVertex3f(-extent.x(),-extent.y(),-extent.z());
		glVertex3f(-extent.x(),-extent.y(),extent.z());
		glVertex3f(-extent.x(),extent.y(),extent.z());
	glEnd();

	glBegin(GL_QUADS);
		glVertex3f(extent.x(),extent.y(),-extent.z());
		glVertex3f(extent.x(),-extent.y(),-extent.z());
		glVertex3f(extent.x(),-extent.y(),extent.z());
		glVertex3f(extent.x(),extent.y(),extent.z());
	glEnd();

	glBegin(GL_QUADS);
		glVertex3f(-extent.x(),extent.y(),extent.z());
		glVertex3f(-extent.x(),-extent.y(),extent.z());
		glVertex3f(extent.x(),-extent.y(),extent.z());
		glVertex3f(extent.x(),extent.y(),extent.z());
	glEnd();

	glBegin(GL_QUADS);
		glVertex3f(-extent.x(),extent.y(),-extent.z());
		glVertex3f(-extent.x(),-extent.y(),-extent.z());
		glVertex3f(extent.x(),-extent.y(),-extent.z());
		glVertex3f(extent.x(),extent.y(),-extent.z());
	glEnd();

	glBegin(GL_QUADS);
		glVertex3f(-extent.x(),extent.y(),-extent.z());
		glVertex3f(-extent.x(),extent.y(),extent.z());
		glVertex3f(extent.x(),extent.y(),extent.z());
		glVertex3f(extent.x(),extent.y(),-extent.z());
	glEnd();

	glBegin(GL_QUADS);
		glVertex3f(-extent.x(),-extent.y(),-extent.z());
		glVertex3f(-extent.x(),-extent.y(),extent.z());
		glVertex3f(extent.x(),-extent.y(),extent.z());
		glVertex3f(extent.x(),-extent.y(),-extent.z());
	glEnd();
	glPopMatrix();

}

void init(float angle)
{
	collisionconfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionconfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionconfig);
	world->setGravity(btVector3(0,-10,0));//gravity in the negative direction
	

	//Add plane
	btTransform t;
	t.setIdentity(); //no rotation at all
	t.setOrigin(btVector3(0,0,0));
	btStaticPlaneShape* plane  = new btStaticPlaneShape(btVector3(0,1,0),0);//set the plane at x-z position, 0 = original position
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(0.0,motion,plane);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	bodies.push_back(body);
	
	addSphereShape(1.0,0.0,20,0,1.0); //1Kg of mass (dynamic body)
	addCylinderShape(2,5,0,30,0,1.0); //dia,height,x,y,z,mass
	addConeShape(2,5,0,30,0,1.0); //rad,height,x,y,z,mass
	addBoxShape(10,2,3,0,40,0,1.0); //size of box,position,mass

	glClearColor(0,0,0,1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(angle,640.0/480,1,1000);
	glMatrixMode(GL_MODELVIEW);
	quad  = gluNewQuadric();
	glEnable(GL_DEPTH_TEST);
}

	
void renderSphere(btRigidBody* sphere)
{
	if(sphere->getCollisionShape()->getShapeType()!= SPHERE_SHAPE_PROXYTYPE)	//sphere should have to be sphere_shape_proxytype
		return;

	glColor3f(1,0,0);
	float r = ((btSphereShape*) sphere->getCollisionShape())->getRadius();
	btTransform t;
	sphere->getMotionState()->getWorldTransform(t); //position of the current sphere
	float mat[16];//4X4
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current sphere
	gluSphere(quad,r,20,20); //rendering sphere ; 20 and 20 is resolution
	glPopMatrix();

}

void renderPlane(btRigidBody* plane)
{
	if(plane->getCollisionShape()->getShapeType()!= STATIC_PLANE_PROXYTYPE)	//plane should have to be static
		return;

	glColor3f(0.8,0.8,0.8); //light grey
	
	btTransform t;
	plane->getMotionState()->getWorldTransform(t); //position of the current plane
	float mat[16];//4X4 matrix
	t.getOpenGLMatrix(mat);
	glPushMatrix();
	glMultMatrixf(mat); //translation, rotation to the current plane
	glBegin(GL_QUADS);
		glVertex3f(-1000,0,1000);
		glVertex3f(-1000,0,-1000);//left closer vertex
		glVertex3f(1000,0,-1000);
		glVertex3f(1000,0,1000);
	glEnd();
	glPopMatrix();

}

void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	
	//go thorugh all the objects in the scene
	for(int i=0;i<bodies.size();i++){
		if(bodies[i]->getCollisionShape()->getShapeType() == STATIC_PLANE_PROXYTYPE)
			renderPlane(bodies[i]);
		else if(bodies[i]->getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE)
			renderSphere(bodies[i]);
		else if(bodies[i]->getCollisionShape()->getShapeType() == CYLINDER_SHAPE_PROXYTYPE)
			renderCylinder(bodies[i]);
		else if(bodies[i]->getCollisionShape()->getShapeType() == CONE_SHAPE_PROXYTYPE)
			renderCone(bodies[i]);
		else if(bodies[i]->getCollisionShape()->getShapeType() == BOX_SHAPE_PROXYTYPE)
			renderBox(bodies[i]);
	}

	
	
}

int _tmain() //used this because it can trasnlate to either main() or wmain()
{
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_SetVideoMode(640,480,32,SDL_OPENGL);
	Uint32 start;
	SDL_Event event;
	bool running = true;
	float angle = 50;
	init(angle);
	while(running)
	{
		start = SDL_GetTicks();
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
			case SDL_QUIT:
				running = false;
				break;
			case SDL_KEYDOWN:
				switch(event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					running = false;
					break;
				case SDLK_y:
					break;
				case SDLK_SPACE:
					break; 
				}
				break;
			case SDL_KEYUP:
				switch(event.key.keysym.sym)
				{
				}
				break;
			case SDL_MOUSEBUTTONDOWN:
				break;
			}
		}
		world->stepSimulation(1.0/60.0);
		display();
		SDL_GL_SwapBuffers();
		if(1000.0/60 >SDL_GetTicks()-start)
			SDL_Delay(1000.0/60 - (SDL_GetTicks() - start));
	}

	//render all the bodies
	for(int i=0;i<bodies.size();i++){
		world->removeCollisionObject(bodies[i]);
		btMotionState* motionState = bodies[i]->getMotionState();
		btCollisionShape* shape = bodies[i]->getCollisionShape();//due their dynamical allocation
		delete bodies[i];
		delete shape;
		delete motionState;
	}

	delete dispatcher;
	delete collisionconfig;
	delete solver;
	delete world;
	delete broadphase;
SDL_Quit();
gluDeleteQuadric(quad);

return 0;
}

