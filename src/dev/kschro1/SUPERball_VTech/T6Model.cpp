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

#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"
#include "tgcreator/tgSphereInfo.h"
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
       // double rod_length;
       //double rod_space; 
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
     8.148,	    	// density (X*1000 kg / m^3)
     0.3/2.0,     	// radius (X*0.1 m)
     15000.0,   	// stiffness of outer muscles (kg / sec^2)
     10000.0,   	// stiffness of inner muscles (kg/sec^2)
     00.0,   		// damping of outer muscles (kg / sec)
     0.0, 	    	// damping of inner muscles (kg/sec)
     //40.0,     		// rod_length (X*0.1 m)
     //10.0,	      	// rod_space (X*0.1 m)
     0.750,        	// payload density (X*1000 kg/m^3)
     5.0,        	// payload radius (X*0.1 m)
     1.0,      		// friction (unitless)
     0.1,     		// rollFriction (unitless)
     0.0,      		// restitution (?)
     1000.0,        	// pretension (X*0.1 N)
     false,     	// history
     1000000,   	// maxTens (X*0.1 N)
     0    		// targetVelocity (X*0.1 m/sec^2 IDK what this does)
  };
} // namespace

T6Model::T6Model() : tgModel() 
{
    //data observer
    // m_dataObserver("Data_test");
}

T6Model::~T6Model()
{
}

void T6Model::addNodes(tgStructure& s)
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

void T6Model::addRods(tgStructure& s)
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


void T6Model::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 1,  "A muscle");       //1 
    s.addPair(1, 2,  "A muscle");       //2 
    s.addPair(1, 3,  "A muscle");       //3 
    s.addPair(2, 4,  "A muscle");       //4 
    s.addPair(3, 4,  "A muscle");       //5 
    s.addPair(4, 5,  "A muscle");       //6 
    s.addPair(0, 6,  "A muscle");       //7 
    s.addPair(2, 9,  "A muscle");       //8 
    s.addPair(5, 11, "A muscle");       //9 
                                        //  
    s.addPair(6, 7,  "B muscle");       //1 
    s.addPair(7, 8,  "B muscle");       //2 
    s.addPair(7, 9,  "B muscle");       //3 
    s.addPair(8, 10, "B muscle");       //4 
    s.addPair(9, 10, "B muscle");       //5 
    s.addPair(10, 11,"B muscle");       //6 
    s.addPair(6, 12, "B muscle");       //7 
    s.addPair(8, 15, "B muscle");       //8 
    s.addPair(11, 17,"B muscle");       //9 
                                        //
    s.addPair(12, 13,"C muscle");       //1 
    s.addPair(13, 14,"C muscle");       //2 
    s.addPair(13, 15,"C muscle");       //3 
    s.addPair(14, 16,"C muscle");       //4 
    s.addPair(15, 16,"C muscle");       //5 
    s.addPair(16, 17,"C muscle");       //6 
    s.addPair(12, 18,"C muscle");       //7 
    s.addPair(14, 21,"C muscle");       //8 
    s.addPair(17, 23,"C muscle");       //9 
                                        //  
    s.addPair(18, 19,"D muscle");       //1 
    s.addPair(19, 20,"D muscle");       //2 
    s.addPair(19, 21,"D muscle");       //3 
    s.addPair(20, 22,"D muscle");       //4 
    s.addPair(21, 22,"D muscle");       //5 
    s.addPair(22, 23,"D muscle");       //6 
    s.addPair(18, 24,"D muscle");       //7 
    s.addPair(20, 27,"D muscle");       //8 
    s.addPair(23, 29,"D muscle");       //9 
                                        //  
    s.addPair(24, 25,"E muscle");       //1 
    s.addPair(25, 26,"E muscle");       //2 
    s.addPair(25, 27,"E muscle");       //3 
    s.addPair(26, 28,"E muscle");       //4 
    s.addPair(27, 28,"E muscle");       //5 
    s.addPair(28, 29,"E muscle");       //6 
    s.addPair(24, 30,"E muscle");       //7 
    s.addPair(26, 33,"E muscle");       //8 
    s.addPair(29, 35,"E muscle");       //9 
                                        //
    s.addPair(30, 31,"F muscle");       //1 
    s.addPair(31, 32,"F muscle");       //2 
    s.addPair(31, 33,"F muscle");       //3 
    s.addPair(32, 34,"F muscle");       //4 
    s.addPair(33, 34,"F muscle");       //5 
    s.addPair(34, 35,"F muscle");       //6 
    s.addPair(30, 0, "F muscle");       //7 
    s.addPair(32, 3, "F muscle");       //8 
    s.addPair(35, 5, "F muscle");       //9 

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

void T6Model::setup(tgWorld& world)
{

    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
    
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension * c.stiffness / c.stiffness_in, c.history, c.maxTens, c.targetVelocity);

    //tgSpringCableActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
   //                     c.maxTens, c.targetVelocity);
            
	
    const tgSphere::Config sphereConfig(c.radius_pay, c.density_pay);

    // Start creating the structure
    tgStructure s;
    addNodes(s);
    addRods(s);
    addMuscles(s);
    // Initial Location
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(0, 1, 0);  // x-axis
    double rotationAngle1 = 0.4636; 
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);

    s.move(btVector3(0, 31, 50));   //(X*0.1 m)



/*
    // Add a rotation to land the struture on a V.
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(1, 0, 0);  // x-axis
    double rotationAngle1 = 0.4636; //M_PI/2;
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);
    // Add a rotation to move structure towards triangle.
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
    //spec.addBuilder("muscle_in", new tgBasicActuatorInfo(muscleInConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
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

const std::vector<tgSpringCableActuator*>& T6Model::getAllMuscles() const
{
    return allMuscles;
}
    
void T6Model::teardown()
{
    tgModel::teardown();
}
