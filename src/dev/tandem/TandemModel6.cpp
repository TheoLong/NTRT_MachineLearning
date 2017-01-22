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
 * @file TandemModel6.cpp
 * @brief Contains the implementation of class TandemModel6.
 * $Id$
 */

// This module
#include "TandemModel6.h"
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
     0.75,       //payload density (kg/lenght^3)
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

TandemModel6::TandemModel6() : tgModel() 
{
    //data observer
    //tgDataObserver("Data_test");
}

TandemModel6::~TandemModel6()
{
}

void TandemModel6::addNodes(tgStructure& s)
{
    // Nodes for struts
    s.addNode(-0.326950,        0.588880,       -1.178110); //0
    s.addNode(0.000,            1.330870,       -0.575360); //1
    s.addNode(0.07503,          1.361590,        0.093289); //2
    s.addNode(-0.579090,        1.279080,       -0.235300); //3
    s.addNode(-0.489130,        1.210210,        0.428660); //4
    s.addNode(-0.537720,        0.405620,        1.037780); //5
    s.addNode(0.346510,         0.577580,       -1.178110); //6
    s.addNode(1.152570,         0.665440,       -0.575360); //7
    s.addNode(1.216690,         0.615820,        0.093289); //8
    s.addNode(0.818170,         1.141050,       -0.235300); //9
    s.addNode(0.803510,         1.028710,        0.428660); //10
    s.addNode(0.082420,         0.668490,        1.037780); //11
    s.addNode(0.673450,        -0.011290,       -1.178110); //12
    s.addNode(1.152570,        -0.665430,       -0.575360); //13
    s.addNode(1.141660,        -0.745770,        0.093289); //14
    s.addNode(1.397260,        -0.138030,       -0.235290); //15
    s.addNode(1.292640,        -0.181500,        0.428660); //16
    s.addNode(0.620140,         0.262870,        1.037780); //17
    s.addNode(0.326950,        -0.588880,       -1.178110); //18
    s.addNode(0.000010,        -1.330870,       -0.575360); //19
    s.addNode(-0.075040,       -1.361590,        0.093289); //20
    s.addNode(0.579090,        -1.279080,       -0.235290); //21
    s.addNode(0.489130,        -1.210210,        0.428660); //22
    s.addNode(0.537720,        -0.405620,        1.037780); //23
    s.addNode(-0.346510,       -0.577580,       -1.178110); //24
    s.addNode(-1.152570,       -0.665430,       -0.575360); //25
    s.addNode(-1.216690,       -0.615810,        0.093289); //26
    s.addNode(-0.818170,       -1.141040,       -0.235290); //27
    s.addNode(-0.803510,       -1.028700,        0.428660); //28
    s.addNode(-0.082420,       -0.668480,        1.037780); //29
    s.addNode(-0.673450,        0.011290,       -1.178110); //30
    s.addNode(-1.152570,        0.665430,       -0.575360); //31
    s.addNode(-1.141660,        0.745780,        0.093289); //32
    s.addNode(-1.397260,        0.138030,       -0.235300); //33
    s.addNode(-1.292640,        0.181510,        0.428660); //34
    s.addNode(-0.620140,       -0.262870,        1.037780); //35

    //Node for payload
    s.addNode(0,0,0, "payload"); //36
}

void TandemModel6::addRods(tgStructure& s)
{
    // Struts
    s.addPair( 0,  8, "rod");
    s.addPair( 6,  14, "rod");
    s.addPair( 12,  20, "rod");
    s.addPair( 18,  26, "rod");
    s.addPair( 24,  32, "rod");
    s.addPair(30, 32, "rod");
    s.addPair(4, 7, "rod");
    s.addPair(10, 13, "rod");
    s.addPair(16, 19, "rod");
    s.addPair(22, 25, "rod");
    s.addPair(28, 31, "rod");
    s.addPair(34, 1, "rod");
    s.addPair(3, 35, "rod");
    s.addPair(9, 7, "rod");
    s.addPair(15, 11, "rod");
    s.addPair(21, 17, "rod");
    s.addPair(27, 23, "rod");
    s.addPair(33, 29, "rod");

}

/*
void TandemModel6::addMarkers(tgStructure &s)
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

void TandemModel6::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 1,  "1 A muscle");
    s.addPair(1, 2,  "2 A muscle");
    s.addPair(1, 3,  "3 A muscle");
    s.addPair(2, 4,  "4 A muscle");
    s.addPair(3, 4,  "5 A muscle");
    s.addPair(4, 5,  "6 A muscle");
    s.addPair(0, 6,  "7 A muscle");
    s.addPair(2, 9,  "8 A muscle");
    s.addPair(5, 11, "9 A muscle");
                          
    s.addPair(6, 7,  "1 B muscle");
    s.addPair(7, 8,  "2 B muscle");
    s.addPair(7, 9,  "3 B muscle");
    s.addPair(8, 10, "4 B muscle");
    s.addPair(9, 10, "5 B muscle");
    s.addPair(10, 11,"6 B muscle");
    s.addPair(6, 12, "7 B muscle");
    s.addPair(8, 15, "8 B muscle");
    s.addPair(11, 17,"9 B muscle");
    
    s.addPair(12, 13,  "1 C muscle");
    s.addPair(13, 14,  "2 C muscle");
    s.addPair(13, 15,  "3 C muscle");
    s.addPair(14, 16,  "4 C muscle");
    s.addPair(15, 16,  "5 C muscle");
    s.addPair(16, 17,  "6 C muscle");
    s.addPair(12, 18,  "7 C muscle");
    s.addPair(14, 21,  "8 C muscle");
    s.addPair(17, 23,  "9 C muscle");
                            
    s.addPair(18, 19,  "1 D muscle");
    s.addPair(19, 20,  "2 D muscle");
    s.addPair(19, 21,  "3 D muscle");
    s.addPair(20, 22,  "4 D muscle");
    s.addPair(21, 22,  "5 D muscle");
    s.addPair(22, 23,  "6 D muscle");
    s.addPair(18, 24,  "7 D muscle");
    s.addPair(20, 27,  "8 D muscle");
    s.addPair(23, 29,  "9 D muscle");
                            
    s.addPair(24, 25,  "1 E muscle");
    s.addPair(25, 26,  "2 E muscle");
    s.addPair(25, 27,  "3 E muscle");
    s.addPair(26, 28,  "4 E muscle");
    s.addPair(27, 28,  "5 E muscle");
    s.addPair(28, 29,  "6 E muscle");
    s.addPair(24, 30,  "7 E muscle");
    s.addPair(26, 33,  "8 E muscle");
    s.addPair(29, 35,  "9 E muscle");

    s.addPair(30, 31,  "1 F muscle");
    s.addPair(31, 32,  "2 F muscle");
    s.addPair(31, 33,  "3 F muscle");
    s.addPair(32, 34,  "4 F muscle");
    s.addPair(33, 34,  "5 F muscle");
    s.addPair(34, 35,  "6 F muscle");
    s.addPair(30, 0,   "7 F muscle");
    s.addPair(32, 3,   "8 F muscle");
    s.addPair(35, 5,   "9 F muscle");






    // Payload Muscles
    s.addPair(29,  36, "upper E muscle");
    s.addPair(23,  36, "upper D muscle");
    s.addPair(17,  36, "upper C muscle");
    s.addPair(11,  36, "upper B muscle");
    s.addPair(5,   36, "upper A muscle");
    s.addPair(35,  36, "upper F muscle");
    s.addPair(12,  36, "lower C muscle");
    s.addPair(18,  36, "lower D muscle");
    s.addPair(24,  36, "lower E muscle");
    s.addPair(30,  36, "lower F muscle");
    s.addPair(0,   36, "lower A muscle");
    s.addPair(6,   36, "lower B muscle");


}



void TandemModel6::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.history, 
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

    s.move(btVector3(0, 12, 0));   //(X*0.1 m)

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

    //btVector3 location(0,10.0,0);
    //btVector3 rotation(0.0,0.6,0.8);
  	//btVector3 speed(-20,-60,0);
    //this->moveModel(location,rotation,speed); 
}

void TandemModel6::step(double dt)
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

void TandemModel6::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& TandemModel6::getAllMuscles() const
{
    return allMuscles;
}
    
void TandemModel6::teardown()
{
    tgModel::teardown();
}

//Initial Velocity

void TandemModel6::moveModel(btVector3 positionVector,btVector3 rotationVector,btVector3 speedVector)
{
    std::vector<tgBaseRigid *> rods = tgCast::filter<tgModel, tgBaseRigid> (getDescendants());
	
	for(int i=0;i<rods.size();i++)
	{
			rods[i]->getPRigidBody()->setLinearVelocity(speedVector);
	}
}
