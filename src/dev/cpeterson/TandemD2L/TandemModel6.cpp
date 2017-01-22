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
        double outer_pretension;
        double inner_pretension;
        bool   history; 
        double maxTens;
        double targetVelocity;
    } c =
   {
     8.148,    // density (kg / length^3)
     0.3/2.0,     // radius (length)
     100000.0,   // stiffness of outer muscles (kg / sec^2)
     100000.0,    // stiffness of inner muscles (kg/sec^2)
     200.0,    // damping of outer muscles (kg / sec)
     200.0,     //damping of inner muscles (kg/sec)
     25.0,     // rod_length (length)
     5.0,      // rod_space (length)
     0,       //payload density (kg/lenght^3)    0.05
     5.0,        //payload radius (length) (X*0.1m)
     3.0,      // friction (unitless)
     0.1,     // rollFriction (unitless)
     0.0,      // restitution (?)
     0.0,        // outer pretension (force)
     0.0,        //inner pretension
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
    const double half_length = c.rod_length / 2;

    // Nodes for struts
    s.addNode( -3.1471,        3.9118,        0.00);   //0
    s.addNode(  0.00,         20.3936,       11.0338); //1
    s.addNode( -0.1587,       18.2072,       15.5505); //2
    s.addNode( -4.2291,       17.9233,       12.1378); //3
    s.addNode( -4.8601,       19.3148,       16.9203); //4
    s.addNode( -3.9973,        3.0377,       27.3093); //5
    s.addNode(  1.8142,        4.6813,        0.00);   //6
    s.addNode( 17.6613,       10.1969,       11.0338); //7
    s.addNode( 15.6885,        9.241,        15.5505); //8
    s.addNode( 13.4075,       12.6242,       12.1378); //9
    s.addNode( 14.2971,       13.8663,       16.9203); //10
    s.addNode(  0.632,         4.9807,       27.3093); //11
    s.addNode(  4.9612,        0.7696,        0.00);   //12
    s.addNode( 17.6613,      -10.1968,       11.0339); //13
    s.addNode( 15.8471,       -8.9662,       15.5506); //14
    s.addNode( 17.6365,       -5.2991,       12.1379); //15
    s.addNode( 19.157,        -5.4485,       16.9204); //16
    s.addNode(  4.6292,        1.9429,       27.3093); //17
    s.addNode(  3.1471,       -3.9118,        0.00);   //18
    s.addNode( -0.0001,      -20.3936,       11.0338); //19
    s.addNode(  0.1587,      -18.2072,       15.5505); //20
    s.addNode(  4.229,       -17.9233,       12.1378); //21
    s.addNode(  4.8601,      -19.3148,       16.9203); //22
    s.addNode(  3.9974,       -3.0377,       27.3093); //23
    s.addNode( -1.8142,       -4.6813,        0.00);   //24
    s.addNode(-17.6613,      -10.1969,       11.0338); //25
    s.addNode(-15.6885,       -9.241,        15.5505); //26
    s.addNode(-13.4075,      -12.6242,       12.1378); //27
    s.addNode(-14.2971,      -13.8663,       16.9203); //28
    s.addNode( -0.632,        -4.9807,       27.3093); //29
    s.addNode( -4.9612,       -0.7696,        0.00);   //30
    s.addNode(-17.6613,       10.1968,       11.0339); //31
    s.addNode(-15.8471,        8.9662,       15.5506); //32
    s.addNode(-17.6365,        5.2992,       12.1379); //33
    s.addNode(-19.157,         5.4485,       16.9204); //34
    s.addNode( -4.6293,       -1.9429,       27.3093); //35

    //Node for payload
    s.addNode(0,0,13.6546, "payload"); //36
}

void TandemModel6::addRods(tgStructure& s)
{
    // Struts
    s.addPair( 0,  8,  "rod");
    s.addPair( 6, 14,  "rod");
    s.addPair(12, 20,  "rod");
    s.addPair(18, 26,  "rod");
    s.addPair(24, 32,  "rod");
    s.addPair(30,  2,  "rod");
    s.addPair( 4,  7,  "rod");
    s.addPair(10, 13,  "rod");
    s.addPair(16, 19,  "rod");
    s.addPair(22, 25,  "rod");
    s.addPair(28, 31,  "rod");
    s.addPair(34,  1,  "rod");
    s.addPair( 3, 35,  "rod");
    s.addPair( 9,  5,  "rod");
    s.addPair(15, 11,  "rod");
    s.addPair(21, 17,  "rod");
    s.addPair(27, 23,  "rod");
    s.addPair(33, 29,  "rod");

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
    s.addPair(0, 1,   "one one_A muscle");
    s.addPair(1, 2,   "two two_A muscle");
    s.addPair(1, 3,   "three three_A muscle");
    s.addPair(2, 4,   "four four_A muscle");
    s.addPair(3, 4,   "five five_A muscle");
    s.addPair(4, 5,   "six six_A muscle");
    s.addPair(0, 6,   "seven seven_A muscle bottom");
    s.addPair(2, 9,   "eight eight_A muscle");
    s.addPair(5, 11,  "nine nine_A muscle top");
                           
    s.addPair(6, 7,   "one one_B muscle");
    s.addPair(7, 8,   "two two_B muscle");
    s.addPair(7, 9,   "three three_B muscle");
    s.addPair(8, 10,  "four four_B muscle");
    s.addPair(9, 10,  "five five_B muscle");
    s.addPair(10, 11, "six six_B muscle");
    s.addPair(6, 12,  "seven seven_B muscle bottom");
    s.addPair(8, 15,  "eight eight_B muscle");
    s.addPair(11, 17, "nine nine_B muscle top");
                      
    s.addPair(12, 13, "one one_C muscle");
    s.addPair(13, 14, "two two_C muscle");
    s.addPair(13, 15, "three three_C muscle");
    s.addPair(14, 16, "four four_C muscle");
    s.addPair(15, 16, "five five_C muscle");
    s.addPair(16, 17, "six six_C muscle");
    s.addPair(12, 18, "seven seven_C muscle bottom");
    s.addPair(14, 21, "eight eight_C muscle");
    s.addPair(17, 23, "nine nine_C muscle top");
                             
    s.addPair(18, 19, "one one_D muscle");
    s.addPair(19, 20, "two two_D muscle");
    s.addPair(19, 21, "three three_D muscle");
    s.addPair(20, 22, "four four_D muscle");
    s.addPair(21, 22, "five five_D muscle");
    s.addPair(22, 23, "six six_D muscle");
    s.addPair(18, 24, "seven seven_D muscle bottom");
    s.addPair(20, 27, "eight eight_D muscle");
    s.addPair(23, 29, "nine nine_D muscle top");
                             
    s.addPair(24, 25, "one one_E muscle");
    s.addPair(25, 26, "two two_E muscle");
    s.addPair(25, 27, "three three_E muscle");
    s.addPair(26, 28, "four four_E muscle");
    s.addPair(27, 28, "five five_E muscle");
    s.addPair(28, 29, "six six_E muscle");
    s.addPair(24, 30, "seven seven_E muscle bottom");
    s.addPair(26, 33, "eight eight_E muscle");
    s.addPair(29, 35, "nine nine_E muscle top");
                      
    s.addPair(30, 31, "one one_F muscle");
    s.addPair(31, 32, "two two_F muscle");
    s.addPair(31, 33, "three three_F muscle");
    s.addPair(32, 34, "four four_F muscle");
    s.addPair(33, 34, "five five_F muscle");
    s.addPair(34, 35, "six six_F muscle");
    s.addPair(30, 0,  "seven seven_F muscle bottom");
    s.addPair(32, 3,  "eight eight_F muscle");
    s.addPair(35, 5,  "nine nine_F muscle top");






    // Payload Muscles
    s.addPair(29,  36, "upper_E innerMuscle upper");
    s.addPair(23,  36, "upper_D innerMuscle upper");
    s.addPair(17,  36, "upper_C innerMuscle upper");
    s.addPair(11,  36, "upper_B innerMuscle upper");
    s.addPair(5,   36, "upper_A innerMuscle upper");
    s.addPair(35,  36, "upper_F innerMuscle upper");
    s.addPair(12,  36, "lower_C innerMuscle lower");
    s.addPair(18,  36, "lower_D innerMuscle lower");
    s.addPair(24,  36, "lower_E innerMuscle lower");
    s.addPair(30,  36, "lower_F innerMuscle lower");
    s.addPair(0,   36, "lower_A innerMuscle lower");
    s.addPair(6,   36, "lower_B innerMuscle lower");
    
    
    s.addPair(2,   36,  "lateral_A innerMuscle");
    s.addPair(8,   36,  "lateral_B innerMuscle");
    s.addPair(14,  36,  "lateral_C innerMuscle");
    s.addPair(20,  36,  "lateral_D innerMuscle");
    s.addPair(26,  36,  "lateral_E innerMuscle");
    s.addPair(32,  36,  "lateral_F innerMuscle");
    s.addPair(4,   36,  "lateral_A innerMuscle");
    s.addPair(10,  36,  "lateral_B innerMuscle");
    s.addPair(16,  36,  "lateral_C innerMuscle");
    s.addPair(22,  36,  "lateral_D innerMuscle");
    s.addPair(28,  36,  "lateral_E innerMuscle");
    s.addPair(34,  36,  "lateral_F innerMuscle");
    


}



void TandemModel6::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    tgBasicActuator::Config innerMuscleConfig(c.stiffness, c.damping, c.inner_pretension, c.history, 
						c.maxTens, c.targetVelocity);

    tgBasicActuator::Config outerMuscleConfig(c.stiffness, c.damping, c.outer_pretension, c.history, 
						c.maxTens, c.targetVelocity);
						
    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay);

    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    //// Initial Location
    
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(1, 0, 0);
    double rotationAngle1 = -M_PI/2;          //0.4636; 
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);

    s.move(btVector3(0, 20, 0));   //(X*0.1 m)     15
/*
    // Add a rotation to land the struture on a V.
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(1, 0, 0);  // x-axis
    double rotationAngle1 = 0.4636; //M_PI/2;
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);
  */  
    
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
    spec.addBuilder("muscle", new tgBasicActuatorInfo(outerMuscleConfig));
    spec.addBuilder("innerMuscle", new tgBasicActuatorInfo(innerMuscleConfig));
    
    
    
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
/*
    btVector3 location(0,10.0,0);
    btVector3 rotation(0.0,0.6,0.8);
  	btVector3 speed(-20,-180,0);
    this->moveModel(location,rotation,speed); 
    */
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
