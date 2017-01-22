/*
 * Copyright © 2015, United States Government, as represented by the
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
 * @file TandemController.cpp
 * @brief Preferred Length Controller for TandemModel6
 * @author Christopher Peterson
 * @version 1.0.0
 * $Id$
 */

// This module
#include "TandemController.h"
// This application
#include "TandemModel6.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

# define M_PI 3.14159265358979323846 
# define END_TIME 10000

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
TandemController::TandemController(const double initialLength, double timestep) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    dt(timestep) {}

//Fetch all the muscles and set their preferred length
void TandemController::onSetup(TandemModel6& subject) {
	this->m_totalTime=0.0;
    const double Active_length = 6;


	const std::vector<tgBasicActuator*> activeMuscle = subject.find<tgBasicActuator>("active");

    for (size_t i=0; i<activeMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = activeMuscle[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(Active_length, dt);
    }
}

// Set target length of each muscle, then move motors accordingly
void TandemController::onStep(TandemModel6& subject, double dt) {
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;

	

    setMuscleTargetLength(subject, dt, "eight_A",   6,  1,   8, 4, 3);
    setMuscleTargetLength(subject, dt, "eight_D",   6,  12,  8, 4, 3);
    
    setMuscleTargetLength(subject, dt, "lateral_A", 6,  1,   8, 4,  3);
    setMuscleTargetLength(subject, dt, "lateral_D", 6, 12,   8, 4,  3);
    setMuscleTargetLength(subject, dt, "bottom",     6,  1,   8, 4, 10);
    
    
    setMuscleTargetLength(subject, dt, "upper",     6,  1, 30, 4,  3);
    setMuscleTargetLength(subject, dt, "lower",     6,  12, 30, 4,  3);
    
    
    /*
    setMuscleTargetLength(subject, dt, "one_F",     6,  1, 30, 4,  3);
    setMuscleTargetLength(subject, dt, "one_A",     6,  1, 50, 4,  10);
    
    setMuscleTargetLength(subject, dt, "lateral_A", 6, 11, 20, 4,  3);
    setMuscleTargetLength(subject, dt, "lateral_D", 6,  1, 20, 4,  3);
    setMuscleTargetLength(subject, dt, "top",      6,  2,  1, 4, 30);
    */
    
    //moveAllMotors(subject, dt);
    //updateActions(dt);
}
 
void TandemController::setMuscleTargetLength(TandemModel6& subject, double dt, std::string tag, double originalLength, double endLength, 
	int startTime, int rampTime, int holdTime) {
    //const double mean_activeMuscle_length = originalLength; //original muscle length
    

    double newLength = 0; //variable declaration
    const double amplitude    = (originalLength - endLength); //
    const std::vector<tgBasicActuator*> Muscle = subject.find<tgBasicActuator>(tag);
    for (size_t i=0; i<Muscle.size(); i++) {
		tgBasicActuator * const pMuscle = Muscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        newLength = originalLength;
       
		if(m_totalTime > startTime && m_totalTime < startTime + rampTime) {
			newLength = originalLength - amplitude * (m_totalTime - startTime) / rampTime;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else if(m_totalTime > startTime + rampTime && m_totalTime < startTime + rampTime + holdTime){
			newLength = endLength;
		}
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}


//Move motors for all the muscles
void TandemController::moveAllMotors(TandemModel6& subject, double dt) {
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
     
}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > TandemController::transformActions(vector< vector <double> > actions)
{
	double min=6;
	double max=11;
	double range=max-min;
	double scaledAct;
	for(unsigned i=0;i<actions.size();i++) {
		for(unsigned j=0;j<actions[i].size();j++) {
			scaledAct=actions[i][j]*(range)+min;
			actions[i][j]=scaledAct;
		}
	}
	return actions;
}

//Pick particular muscles (according to the structure's state) and apply the given actions one by one
void TandemController::applyActions(TandemModel6& subject, vector< vector <double> > act)
{
	//Get All the muscles of the subject
	const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
	//Check if the number of the actions match the number of the muscles
	if(act.size() != muscles.size()) {
		cout<<"Warning: # of muscles: "<< muscles.size() << " != # of actions: "<< act.size()<<endl;
		return;
	}
	//Apply actions (currently in a random order)
	for (size_t i = 0; i < muscles.size(); ++i)	{
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		cout<<"i: "<<i<<" length: "<<act[i][0]<<endl;
		pMuscle->setControlInput(act[i][0]);
	}
}
