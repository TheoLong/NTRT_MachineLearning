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
     20.0,     // rod_length (length)
     5.0,      // rod_space (length)
     0.5,       //payload density (kg/lenght^3)    0.05
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
    s.addNode(-3.26950,        5.88880,       -11.78110); //0
    s.addNode(0.000,            13.30870,       -5.75360); //1
    s.addNode(0.7503,          13.61590,        0.93289); //2
    s.addNode(-5.79090,        12.79080,       -2.35300); //3
    s.addNode(-4.89130,        12.10210,        4.28660); //4
    s.addNode(-5.37720,        4.05620,        10.37780); //5
    s.addNode(3.46510,         5.77580,       -11.78110); //6
    s.addNode(11.52570,         6.65440,       -5.75360); //7
    s.addNode(12.16690,         6.15820,        0.93289); //8
    s.addNode(8.18170,         11.41050,       -2.35300); //9
    s.addNode(8.03510,         10.28710,        4.28660); //10
    s.addNode(0.82420,         6.68490,        10.37780); //11
    s.addNode(6.73450,        -0.11290,       -11.78110); //12
    s.addNode(11.52570,        -6.65430,       -5.75360); //13
    s.addNode(11.41660,        -7.45770,        0.93289); //14
    s.addNode(13.97260,        -1.38030,       -2.35290); //15
    s.addNode(12.92640,        -1.81500,        4.28660); //16
    s.addNode(6.20140,         2.62870,        10.37780); //17
    s.addNode(3.26950,        -5.88880,       -11.78110); //18
    s.addNode(0.00010,        -13.30870,       -5.75360); //19
    s.addNode(-0.75040,       -13.61590,        0.93289); //20
    s.addNode(5.79090,        -12.79080,       -2.35290); //21
    s.addNode(4.89130,        -12.10210,        4.28660); //22
    s.addNode(5.37720,        -4.05620,        10.37780); //23
    s.addNode(-3.46510,       -5.77580,       -11.78110); //24
    s.addNode(-11.52570,       -6.65430,       -5.75360); //25
    s.addNode(-12.16690,       -6.15810,        0.93289); //26
    s.addNode(-8.18170,       -11.41040,       -2.35290); //27
    s.addNode(-8.03510,       -10.28700,        4.28660); //28
    s.addNode(-0.82420,       -6.68480,        10.37780); //29
    s.addNode(-6.73450,        0.11290,       -11.78110); //30
    s.addNode(-11.52570,        6.65430,       -5.75360); //31
    s.addNode(-11.41660,        7.45780,        0.93289); //32
    s.addNode(-13.97260,        1.38030,       -2.35300); //33
    s.addNode(-12.92640,        1.81510,        4.28660); //34
    s.addNode(-6.20140,       -2.62870,        10.37780); //35

    //Node for payload
    s.addNode(0,0,0, "payload"); //36
}

void TandemModel6::addRods(tgStructure& s)
{
    // Struts
    s.addPair(0,  8, "rod");
    s.addPair(6,  14, "rod");
    s.addPair(12,  20, "rod");
    s.addPair(18,  26, "rod");
    s.addPair(24,  32, "rod");
    s.addPair(30, 2, "rod");
    s.addPair(4, 7, "rod");
    s.addPair(10, 13, "rod");
    s.addPair(16, 19, "rod");
    s.addPair(22, 25, "rod");
    s.addPair(28, 31, "rod");
    s.addPair(34, 1, "rod");
    s.addPair(3, 35, "rod");
    s.addPair(9, 5, "rod");
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
    s.addPair(0, 1,   "one_A muscle");
    s.addPair(1, 2,   "two_A muscle");
    s.addPair(1, 3,   "three_A muscle");
    s.addPair(2, 4,   "four_A muscle");
    s.addPair(3, 4,   "five_A muscle");
    s.addPair(4, 5,   "six_A muscle");
    s.addPair(0, 6,   "seven_A muscle bottom");
    s.addPair(2, 9,   "eight_A muscle");
    s.addPair(5, 11,  "nine_A muscle top");
                           
    s.addPair(6, 7,   "one_B muscle");
    s.addPair(7, 8,   "two_B muscle");
    s.addPair(7, 9,   "three_B muscle");
    s.addPair(8, 10,  "four_B muscle");
    s.addPair(9, 10,  "five_B muscle");
    s.addPair(10, 11, "six_B muscle");
    s.addPair(6, 12,  "seven_B muscle bottom");
    s.addPair(8, 15,  "eight_B muscle");
    s.addPair(11, 17, "nine_B muscle top");
                      
    s.addPair(12, 13, "one_C muscle");
    s.addPair(13, 14, "two_C muscle");
    s.addPair(13, 15, "three_C muscle");
    s.addPair(14, 16, "four_C muscle");
    s.addPair(15, 16, "five_C muscle");
    s.addPair(16, 17, "six_C muscle");
    s.addPair(12, 18, "seven_C muscle bottom");
    s.addPair(14, 21, "eight_C muscle");
    s.addPair(17, 23, "nine_C muscle top");
                             
    s.addPair(18, 19, "one_D muscle");
    s.addPair(19, 20, "two_D muscle");
    s.addPair(19, 21, "three_D muscle");
    s.addPair(20, 22, "four_D muscle");
    s.addPair(21, 22, "five_D muscle");
    s.addPair(22, 23, "six_D muscle");
    s.addPair(18, 24, "seven_D muscle bottom");
    s.addPair(20, 27, "eight_D muscle");
    s.addPair(23, 29, "nine_D muscle top");
                             
    s.addPair(24, 25, "one_E muscle");
    s.addPair(25, 26, "two_E muscle");
    s.addPair(25, 27, "three_E muscle");
    s.addPair(26, 28, "four_E muscle");
    s.addPair(27, 28, "five_E muscle");
    s.addPair(28, 29, "six_E muscle");
    s.addPair(24, 30, "seven_E muscle bottom");
    s.addPair(26, 33, "eight_E muscle");
    s.addPair(29, 35, "nine_E muscle top");
                      
    s.addPair(30, 31, "one_F muscle");
    s.addPair(31, 32, "two_F muscle");
    s.addPair(31, 33, "three_F muscle");
    s.addPair(32, 34, "four_F muscle");
    s.addPair(33, 34, "five_F muscle");
    s.addPair(34, 35, "six_F muscle");
    s.addPair(30, 0,  "seven_F muscle bottom");
    s.addPair(32, 3,  "eight_F muscle");
    s.addPair(35, 5,  "nine_F muscle top");






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

    s.move(btVector3(0, 12, 0));   //(X*0.1 m)     15
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
