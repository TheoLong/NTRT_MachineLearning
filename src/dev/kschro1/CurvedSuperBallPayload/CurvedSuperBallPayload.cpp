/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file CurvedSuperBallPayload.cpp
 * @brief Contains the implementation of class CurvedSuperBallPayload.
 * $Id$
 */

// This module
#include "CurvedSuperBallPayload.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "tgcreator/tgSphereInfo.h"
#include "core/abstractMarker.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <iostream>

namespace
{
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Note: This current model of the SUPERball rod is 1.5m long by 3 cm radius,
    // which is 0.00424 m^3.
    // For SUPERball v1.5, mass = 3.5kg per strut, which comes out to 
    // 0.825 kg / (decimeter^3).

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double stiffness_in;
        double damping;
        double damping_in;
        double rod_length;
        double rod_space; 
	double height; 
        double density_pay;
        double radius_pay; 
        double density_ball;
        double radius_ball; 
        double friction;
        double rollFriction;
        double restitution;
        double pretension;
        bool   history; 
        double maxTens;
        double targetVelocity;
    } c =
   {
     4.50,	    	// density (X*1000 kg / m^3)
     0.3/2,	     	// radius (X*0.1 m)
     15000.0,   	// stiffness of outer muscles (kg / sec^2)
     10000.0,   	// stiffness of inner muscles (kg/sec^2)
     00.0,   		// damping of outer muscles (kg / sec)
     0.0, 	    	// damping of inner muscles (kg/sec)
     40.0,     		// rod_length (X*0.1 m)
     10.0,	      	// rod_space (X*0.1 m)
     10.0,	      	// height (X*0.1 m)
     0.750,        	// payload density (X*1000 kg/m^3)
     5.0,        	// payload radius (X*0.1 m)
     4.5,        	// ball density (X*1000 kg/m^3)
     0.9,        	// ball radius (X*0.1 m)
     1.0,      		// friction (unitless)
     0.1,     		// rollFriction (unitless)
     0.0,      		// restitution (?)
     1000.0,        	// pretension (X*0.1 N)
     false,     	// history
     1000000,   	// maxTens (X*0.1 N)
     0    		// targetVelocity (X*0.1 m/sec^2 IDK what this does)
  };
    int NumSegments=6;	// Always chose EVEN numbers (unitless)
} 

CurvedSuperBallPayload::CurvedSuperBallPayload() : tgModel() 
{
}

CurvedSuperBallPayload::~CurvedSuperBallPayload()
{
}

//Node numbers seen from Front (multiply by NumSegments)
// ----11-------5------
// ---------3----------
// 0------------------1
// ---------9----------
// ----10-------4------
//
//Node numbers seen from Back
// ----11-------5------
// ---------2----------
// 6------------------7
// ---------8----------
// ----10-------4------
//


void CurvedSuperBallPayload::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;
 
/*
* the equation y=ax^2+bx+c
* can be found from three points were
* a=-(X1(Y2-Y3)-X2(Y1-Y3))/(X1*X2*(X1-X2))
* b=(X1^2(Y2-Y3)-X2^2(Y1-Y3))/(X1*X2*(X1-X2))
* c=Y3
* 
* NOTE:  this calculation is only valid if point3 is the Y intercept. 
*/

	double 	x, y, firstx=-half_length, firsty=c.rod_space, secondx=half_length, 
		secondy=c.rod_space, thridy=c.rod_space+c.height;

	double 	a=-(firstx*(secondy-thridy)-secondx*(firsty-thridy))/(firstx*secondx*(firstx-secondx));
	double 	b=(firstx*firstx*(secondy-thridy)-secondx*secondx*(firsty-thridy))/(firstx*secondx*\
			(firstx-secondx));
	double 	c=thridy;

	int 	d, e, f;
	int 	i, j, k, count=0;
/*
*The third ForLoop creates the coordinate points of the first bar. 
*The first two ForLoop then rotate that inital bar around to create the other 5 bars.
*/

for(k=1;k>=-1;k=k-2)
   {
      for(j=0;j>-3;j--)
       {
	d=j;
	e=j+1;
	f=j+2;
	if(d>-1){}
	else{
	d=j+3;}
	if(e>-1){}
	else{
	e=j+4;}
std::cout<<j<<std::endl;
std::cout<<d<<std::endl;
std::cout<<e<<std::endl;
std::cout<<f<<std::endl;
	    for(i=0;i<=NumSegments;i++)
	        {
	         x=firstx+(i*2*half_length/NumSegments);
		 y=(a*(x*x)+b*x+c)*k;
			
		 nodePositions.push_back(btVector3(x, y, 0));
		 s.addNode(nodePositions[count][d],nodePositions[count][e],nodePositions[count][f]);
		 count++;
			
		}
	}
    }			


}

  //  Payload
void CurvedSuperBallPayload::addSphere(tgStructure& s)
{
    		s.addNode(0,	    0,		   0, "payload"); 

		s.addNode(-c.rod_length/2,	 c.rod_space,    0, "ball");
		s.addNode( c.rod_length/2,	 c.rod_space,    0, "ball");
		s.addNode(-c.rod_length/2,	-c.rod_space,    0, "ball");
		s.addNode( c.rod_length/2,      -c.rod_space,    0, "ball");
		s.addNode(0, -c.rod_length/2,	 c.rod_space,  "ball");
		s.addNode(0,  c.rod_length/2,	 c.rod_space,  "ball");
		s.addNode(0, -c.rod_length/2,	-c.rod_space,  "ball");
		s.addNode(0,  c.rod_length/2,	-c.rod_space,  "ball");
		s.addNode(-c.rod_space, 0,  c.rod_length/2,  "ball");
		s.addNode( c.rod_space, 0,  c.rod_length/2,  "ball");
		s.addNode(-c.rod_space, 0, -c.rod_length/2,  "ball");
		s.addNode( c.rod_space, 0, -c.rod_length/2,  "ball");
 
}

void CurvedSuperBallPayload::addRods(tgStructure& s)
{
  // Struts
	int 	i, j, count=0;
	for(j=1;j<=6;j++)
	 {
	  for(i=0;i<NumSegments;i++)
	   {
	    s.addPair( count,  count+1, "rod");

	    count++;

	   }
	  count++;
	  }
}
/*
void CurvedSuperBallPayload::addMarkers(tgStructure &s)
{
    std::vector<tgRod *> rods=find<tgRod>("rod");

	for(int i=0;i<12;i++)
	{
		const btRigidBody* bt = rods[rodNumbersPerNode[i]]->getPRigidBody();
		btTransform inverseTransform = bt->getWorldTransform().inverse();
		btVector3 pos = inverseTransform * (nodePositions[i]);
		abstractMarker tmp=abstractMarker(bt,pos,btVector3(0.08*i,1.0 - 0.08*i,.0),i);
		this->addMarker(tmp);
	}
}
*/
void CurvedSuperBallPayload::addMuscles(tgStructure& s)
{

  // Outer Cables
    s.addPair(0+0*NumSegments, 1+2*NumSegments,  "muscle"); //0,3
    s.addPair(0+0*NumSegments, 4+5*NumSegments,  "muscle"); //0,9
    s.addPair(0+0*NumSegments, 5+5*NumSegments,  "muscle"); //0,10
    s.addPair(0+0*NumSegments, 5+6*NumSegments,  "muscle"); //0,11
    s.addPair(1+2*NumSegments, 5+6*NumSegments,  "muscle"); //3,11
    s.addPair(1+2*NumSegments, 2+3*NumSegments,  "muscle"); //3,5
    s.addPair(0+1*NumSegments, 1+2*NumSegments,  "muscle"); //1,3
    s.addPair(0+1*NumSegments, 2+3*NumSegments,  "muscle"); //1,5
    s.addPair(0+1*NumSegments, 2+2*NumSegments,  "muscle"); //1,4
    s.addPair(0+1*NumSegments, 4+5*NumSegments,  "muscle"); //1,9
    s.addPair(4+5*NumSegments, 2+2*NumSegments,  "muscle"); //9,4
    s.addPair(4+5*NumSegments, 5+5*NumSegments,  "muscle"); //9,10
    s.addPair(3+3*NumSegments, 1+1*NumSegments,  "muscle"); //6,2
    s.addPair(3+3*NumSegments, 4+4*NumSegments,  "muscle"); //6,8
    s.addPair(3+3*NumSegments, 5+5*NumSegments,  "muscle"); //6,10
    s.addPair(3+3*NumSegments, 5+6*NumSegments,  "muscle"); //6,11
    s.addPair(1+1*NumSegments, 5+6*NumSegments,  "muscle"); //2,11
    s.addPair(1+1*NumSegments, 2+3*NumSegments,  "muscle"); //2,5
    s.addPair(3+4*NumSegments, 1+1*NumSegments,  "muscle"); //7,2
    s.addPair(3+4*NumSegments, 2+3*NumSegments,  "muscle"); //7,5
    s.addPair(3+4*NumSegments, 2+2*NumSegments,  "muscle"); //7,4
    s.addPair(3+4*NumSegments, 4+4*NumSegments,  "muscle"); //7,8
    s.addPair(4+4*NumSegments, 2+2*NumSegments,  "muscle"); //8,4
    s.addPair(4+4*NumSegments, 5+5*NumSegments,  "muscle"); //8,10

    // Payload Muscles
    s.addPair(0+0*NumSegments, (NumSegments+1)*6, "muscle_in"); //0
    s.addPair(0+1*NumSegments, (NumSegments+1)*6, "muscle_in"); //1
    s.addPair(1+1*NumSegments, (NumSegments+1)*6, "muscle_in"); //2
    s.addPair(1+2*NumSegments, (NumSegments+1)*6, "muscle_in"); //3
    s.addPair(2+2*NumSegments, (NumSegments+1)*6, "muscle_in"); //4
    s.addPair(2+3*NumSegments, (NumSegments+1)*6, "muscle_in"); //5
    s.addPair(3+3*NumSegments, (NumSegments+1)*6, "muscle_in"); //6
    s.addPair(3+4*NumSegments, (NumSegments+1)*6, "muscle_in"); //7
    s.addPair(4+4*NumSegments, (NumSegments+1)*6, "muscle_in"); //8
    s.addPair(4+5*NumSegments, (NumSegments+1)*6, "muscle_in"); //9
    s.addPair(5+5*NumSegments, (NumSegments+1)*6, "muscle_in"); //10
    s.addPair(5+6*NumSegments, (NumSegments+1)*6, "muscle_in"); //11

}

void CurvedSuperBallPayload::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					    c.history, c.maxTens, c.targetVelocity);

    tgBasicActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
                        c.maxTens, c.targetVelocity);
     
    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay);
    const tgSphere::Config ballConfig(c.radius_ball, c.density_ball);
   
    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addSphere(s);
    addRods(s);
    addMuscles(s);

//    // Add a rotation. This is needed if the ground slopes too much,
//    // otherwise  glitches put a rod below the ground.
//    btVector3 rotationPoint = btVector3(0, 0, 0); // origin
//    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
//    double rotationAngle = M_PI/2;
//    s.addRotation(rotationPoint, rotationAxis, rotationAngle);

    s.move(btVector3(0,30,0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("payload", new tgSphereInfo(sphereConfig));
    spec.addBuilder("ball", new tgSphereInfo(ballConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("muscle_in", new tgBasicActuatorInfo(muscleInConfig));


    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);

    //map the rods and add the markers to them
    //addMarkers(s);

    btVector3 location(0,0.0,0);
    btVector3 rotation(0.0,0,0);
  	btVector3 speed(0,0,0);
    this->moveModel(location,rotation,speed);
}

void CurvedSuperBallPayload::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void CurvedSuperBallPayload::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& CurvedSuperBallPayload::getAllMuscles() const
{
    return allMuscles;
}
    
void CurvedSuperBallPayload::teardown()
{
    tgModel::teardown();
}

void CurvedSuperBallPayload::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
{
    std::vector<tgRod *> rods=find<tgRod>("rod");

	btQuaternion initialRotationQuat;
	initialRotationQuat.setEuler(rotationVector[0],rotationVector[1],rotationVector[2]);
	btTransform initialTransform;
	initialTransform.setIdentity();
	initialTransform.setRotation(initialRotationQuat);
	initialTransform.setOrigin(positionVector);
	for(int i=0;i<rods.size();i++)
	{
			rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
			rods[i]->getPRigidBody()->setWorldTransform(initialTransform * rods[i]->getPRigidBody()->getWorldTransform());
	}
}
