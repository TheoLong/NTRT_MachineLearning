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
 * @file SensoredSuperBallPayload.cpp
 * @brief Contains the implementation of class SensoredSuperBallPayload.
 * $Id$
 */

// This module
#include "SensoredSuperBallPayload.h"
// This library
#include "SensoredSuperBallPayloadController.h"

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

#include "core/tgSubject.h"
#include "core/tgObserver.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>

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
        double density_pay;
        double radius_pay;  
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
     0.3/2.0,     	// radius (X*0.1 m)
     15000.0,   	// stiffness of outer muscles (kg / sec^2)
     10000.0,   	// stiffness of inner muscles (kg/sec^2)
     00.0,   		// damping of outer muscles (kg / sec)
     0.0, 	    	//damping of inner muscles (kg/sec)
     40.0,     		// rod_length (X*0.1 m)
     10.0,	      	// rod_space (X*0.1 m)
     0.750,        	//payload density (X*1000 kg/m^3)
     2.50,        	//payload radius (X*0.1 m)
     1.0,      		// friction (unitless)
     0.1,     		// rollFriction (unitless)
     0.0,      		// restitution (?)
     1000.0,        	// pretension (X*0.1 N)
     false,     	// history
     1000000,   	// maxTens (X*0.1 N)
     0    		// targetVelocity (X*0.1 m/sec^2 IDK what this does)
  };
} 

/*returns the number of the rod for a given node */
const int rodNumbersPerNode[13]={0,1,2,3,3,4,0,1,2,5,5,4,6};

/*returns the node that is at the other end of the given node */
const int otherEndOfTheRod[13]={6,7,8,4,3,11,0,1,2,10,9,5,12};

/*returns the node that is at the parallel rod
 * and at the same end of the given node
 */
const int parallelNode[13]={1,0,5,9,10,2,7,6,11,3,4,8,12};

SensoredSuperBallPayload::SensoredSuperBallPayload() : tgModel() 
{
}

SensoredSuperBallPayload::~SensoredSuperBallPayload()
{
}

//Node numbers seen from Front
// -----0-------1------
// ---------2----------
// 3------------------4
// ---------5----------
// -----6-------7------
//
//Node numbers seen from Back
// -----0-------1------
// ---------8----------
// 9-----------------10
// ---------11---------
// -----6-------7------
//


void SensoredSuperBallPayload::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;
/*
    nodePositions.push_back(btVector3(-half_length,   c.rod_space, 0));            // 0
    nodePositions.push_back(btVector3( half_length,   c.rod_space, 0));            // 1
    nodePositions.push_back(btVector3(0,            half_length,   -c.rod_space)); // 2
    nodePositions.push_back(btVector3(-c.rod_space, 0,           -half_length));   // 3
    nodePositions.push_back(btVector3( c.rod_space, 0,           -half_length));   // 4
    nodePositions.push_back(btVector3(0,           -half_length,   -c.rod_space)); // 5
    nodePositions.push_back(btVector3(-half_length,  -c.rod_space, 0));            // 6
    nodePositions.push_back(btVector3( half_length,  -c.rod_space, 0));            // 7
    nodePositions.push_back(btVector3(0,            half_length,    c.rod_space)); // 8
    nodePositions.push_back(btVector3(-c.rod_space, 0,            half_length));   // 9
    nodePositions.push_back(btVector3( c.rod_space, 0,            half_length));   // 10
    nodePositions.push_back(btVector3(0,           -half_length,    c.rod_space)); // 11

*/
    nodePositions.push_back(btVector3(-c.rod_space,  -half_length, 0));            // 0
    nodePositions.push_back(btVector3(-c.rod_space,   half_length, 0));            // 1
    nodePositions.push_back(btVector3( c.rod_space,  -half_length, 0));            // 2
    nodePositions.push_back(btVector3( c.rod_space,   half_length, 0));            // 3
    nodePositions.push_back(btVector3(0,           -c.rod_space,   -half_length)); // 4
    nodePositions.push_back(btVector3(0,           -c.rod_space,    half_length)); // 5
    nodePositions.push_back(btVector3(0,            c.rod_space,   -half_length)); // 6
    nodePositions.push_back(btVector3(0,            c.rod_space,    half_length)); // 7
    nodePositions.push_back(btVector3(-half_length, 0,            c.rod_space));   // 8
    nodePositions.push_back(btVector3( half_length, 0,            c.rod_space));   // 9
    nodePositions.push_back(btVector3(-half_length, 0,           -c.rod_space));   // 10
    nodePositions.push_back(btVector3( half_length, 0,           -c.rod_space));   // 11

    for(int i=0;i<12;i++)
    {
		s.addNode(nodePositions[i][0],nodePositions[i][1],nodePositions[i][2]);
    }
}

  //  Payload
void SensoredSuperBallPayload::addSphere(tgStructure& s)
{
    		s.addNode(0,	    0,		   0, "payload"); // 12
}

void SensoredSuperBallPayload::addRods(tgStructure& s)
{
  // Struts
/*
    s.addPair( 0,  6, "r1 rod");
    s.addPair( 1,  7, "r2 rod");
    s.addPair( 2,  8, "r3 rod");
    s.addPair( 3,  4, "r4 rod");
    s.addPair( 5, 11, "r5 rod");
    s.addPair( 9, 10, "r6 rod");
*/
    s.addPair( 0,  1, "r1 rod");
    s.addPair( 2,  3, "r2 rod");
    s.addPair( 4,  5, "r3 rod");
    s.addPair( 6,  7, "r4 rod");
    s.addPair( 8,  9, "r5 rod");
    s.addPair(10, 11, "r6 rod");
}

void SensoredSuperBallPayload::addMarkers(tgStructure &s)
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

void SensoredSuperBallPayload::addMuscles(tgStructure& s)
{
  // Outer Cables
    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");
    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");
    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");
    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");
    s.addPair(4, 2,  "muscle");
    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");
    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");
    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");
    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");

    // Payload Muscles
    s.addPair(0, 12, "muscle_in");
    s.addPair(1, 12, "muscle_in");
    s.addPair(2, 12, "muscle_in");
    s.addPair(3, 12, "muscle_in");
    s.addPair(4, 12, "muscle_in");
    s.addPair(5, 12, "muscle_in");
    s.addPair(6, 12, "muscle_in");
    s.addPair(7, 12, "muscle_in");
    s.addPair(8, 12, "muscle_in");
    s.addPair(9, 12, "muscle_in");
    s.addPair(10, 12, "muscle_in");
    s.addPair(11, 12, "muscle_in");
}

void SensoredSuperBallPayload::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension,
					    c.history, c.maxTens, c.targetVelocity);

    tgBasicActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
                        c.maxTens, c.targetVelocity);
     
    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay);
   
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

    //s.move(btVector3(0,30,0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("payload", new tgSphereInfo(sphereConfig));
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
    addMarkers(s);

    btVector3 location(0,0.0,0);
    btVector3 rotation(0.0,0,0);
  	btVector3 speed(0,31,50);
    this->moveModel(location,rotation,speed);
}

void SensoredSuperBallPayload::step(double dt)
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

void SensoredSuperBallPayload::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& SensoredSuperBallPayload::getAllMuscles() const
{
    return allMuscles;
}
    
void SensoredSuperBallPayload::teardown()
{
    tgModel::teardown();
}

void SensoredSuperBallPayload::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
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
