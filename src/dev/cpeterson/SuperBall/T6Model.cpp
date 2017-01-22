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
 * @file T6Model.cpp
 * @brief Contains the implementation of class T6Model.
 * $Id$
 */

// This module
#include "T6Model.h"
// This library
//#include "core/tgSpringCableActuator.h"
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
     8.148,    // density (kg / length^3)
     0.3/2.0,     // radius (length)
     15000.0,   // stiffness of outer muscles (kg / sec^2)
     10000.0,    // stiffness of inner muscles (kg/sec^2)
     1,    // damping of outer muscles (kg / sec)
     1,     //damping of inner muscles (kg/sec)
     20.0,     // rod_length (length)
     5.0,      // rod_space (length)
     0.75,       //payload density (kg/lenght^3)    //0.75
     2.5,        //payload radius (length) (X*0.1m)
     1.0,      // friction (unitless)
     0.1,     // rollFriction (unitless)
     0.0,      // restitution (?)
     0.0,        // pretension (force)
     false,     // history
     1000000,   // maxTens
     1.0    // targetVelocity  
  };
} // namespace


///*
 //* helper arrays for node and rod numbering schema
 //*/
 
////returns the number of the rod for a given node
//const int rodNumbersPerNode[13]={0,1,2,3,3,4,0,1,2,5,5,4,6};

////returns the node that is at the other end of the given node
//const int otherEndOfTheRod[13]={6,7,8,4,3,11,0,1,2,10,9,5,12};

///*returns the node that is at the parallel rod
 //* and at the same end of the given node
 //*/
//const int parallelNode[13]={1,0,5,9,10,2,7,6,11,3,4,8,12};
//*/

T6Model::T6Model() : tgModel() 
{
    //data observer
    //tgDataObserver("Data_test");
}

T6Model::~T6Model()
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

void T6Model::addNodes(tgStructure& s)
{
    const double half_length = c.rod_length / 2;

    // Nodes for struts
    s.addNode(-c.rod_space,  -half_length, 0);            // 0
    s.addNode(-c.rod_space,   half_length, 0);            // 1
    s.addNode( c.rod_space,  -half_length, 0);            // 2
    s.addNode( c.rod_space,   half_length, 0);            // 3
    s.addNode(0,           -c.rod_space,   -half_length); // 4
    s.addNode(0,           -c.rod_space,    half_length); // 5
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11

    //Node for payload
    s.addNode(0,0,0, "payload"); //12
}

void T6Model::addRods(tgStructure& s)
{
    // Struts
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");
}

/*
void T6Model::addMarkers(tgStructure &s)
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

void T6Model::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 4,  "alpha muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "bravo muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "charlie muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "delta muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");

    s.addPair(4, 10, "echo muscle");
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



void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.history, 
						c.maxTens, c.targetVelocity);

    tgBasicActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
                        c.maxTens, c.targetVelocity);
            
	
    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay);

    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    //// Initial Location
    
    //btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    //btVector3 rotationAxis1 = btVector3(0, 1, 0);  // x-axis
    //double rotationAngle1 = 0.0;          //0.4636; 
    //s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);

    s.move(btVector3(0, 12, 0));   //12




    // Add a rotation to land the struture on a V.
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(1, 0, 0);  // x-axis
    double rotationAngle1 = 0.4636; //M_PI/2;
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);
/*    // Add a rotation to move structure towards triangle.
    btVector3 rotationPoint2 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis2 = btVector3(0, 1, 0);  // z-axis
    double rotationAngle2 = 1.991; 
    s.addRotation(rotationPoint2, rotationAxis2, rotationAngle2);
    // Add a rotation to land the struture on a triangle.
    btVector3 rotationPoint3 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis3 = btVector3(-1, 0, 0);  // x-axis
    double rotationAngle3 = 0.58895; 
    s.addRotation(rotationPoint3, rotationAxis3, rotationAngle3);
*/
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
    //addMarkers(s);

    btVector3 location(0,10.0,0);
    btVector3 rotation(0.0,0.6,0.8);
  	btVector3 speed(-20,-60,0);
    //this->moveModel(location,rotation,speed); 
}

void T6Model::step(double dt)
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

void T6Model::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& T6Model::getAllMuscles() const
{
    return allMuscles;
}
    
void T6Model::teardown()
{
    tgModel::teardown();
}

//Initial Velocity

void T6Model::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
{
    std::vector<tgBaseRigid *> rods = tgCast::filter<tgModel, tgBaseRigid> (getDescendants());
	
	for(int i=0;i<rods.size();i++)
	{
			rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
	}
}


