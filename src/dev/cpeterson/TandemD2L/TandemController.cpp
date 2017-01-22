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
# define END_TIME 1000000

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

	//setMuscleTargetLength(subject, dt, tag, startTime, rampTime, holdTime)

    setMuscleTargetLength(subject, dt, "one",    5,  5,  20);
    setMuscleTargetLength(subject, dt, "two",    5,  5,  20);
    setMuscleTargetLength(subject, dt, "three",  5,  5,  20);
    setMuscleTargetLength(subject, dt, "four",   5,  5,  20);
    setMuscleTargetLength(subject, dt, "five",   5,  5,  20);
    setMuscleTargetLength(subject, dt, "six",    5,  5,  20);
    setMuscleTargetLength(subject, dt, "seven",  5,  5,  20);
    setMuscleTargetLength(subject, dt, "eight",  5,  5,  20);
    setMuscleTargetLength(subject, dt, "nine",   5,  5,  20);
    
}
 
     /*TODO: check tags
    set coefficients of quadratic equation
    */
 
 
void TandemController::setMuscleTargetLength(TandemModel6& subject, double dt, std::string tag, 
	int startTime, int rampTime, int holdTime) {
    //const double mean_activeMuscle_length = originalLength; //original muscle length
    
    double a = 0;
    double b = 0;
    double c = 0;
    double newLength = 0; //variable declaration
    double endLength = 0;
    int switchTag = 0;
   
   
    const std::vector<tgBasicActuator*> Muscle = subject.find<tgBasicActuator>(tag);
    for (size_t i=0; i<Muscle.size(); i++) {
		
		    if(tag == "one") {
				switchTag = 1;
			} else if(tag == "two") {
				switchTag = 2;
			} else if(tag == "three") {
				switchTag = 3;
			} else if(tag == "four") {
				switchTag = 4;
			} else if(tag == "five") {
				switchTag = 5;
			} else if(tag == "six") {
				switchTag = 6;
			} else if(tag == "seven") {
				switchTag = 7;
			} else if(tag == "eight") {
				switchTag = 8;
			} else if(tag == "nine"){ 
				switchTag = 9;	
			} else {
				switchTag = 0;
			}
    
		switch(switchTag) {
			case 1 : { a = 0.025131792;
					   b = -0.354974508;
					   c = 21.230349851;
					   endLength = 20.082308383;
					   newLength = c;
					   cout<<"case 1"<<endl;
					   break;
				     }
			
			case 2 : { a = 0.006282948;
					 b = -0.088743627;
					 c = 5.307587463;
					 endLength = 5.020577096;
					 newLength = c;
					 cout<<"case 2"<<endl;
					 break;
				 }
		
			case 3 : { a = 0.006282948;
					   b = -0.088743627;
					   c = 5.307587463;
					   endLength = 5.020577096;
					   newLength = c;
					   cout<<"case 3"<<endl;
					 break;
				 }
		
			case 4 : { a = 0.006282948;
					  b = -0.088743627;
					  c = 5.307587463;
					  endLength = 5.020577096;
					  newLength = c;
					  cout<<"case 4"<<endl;
					 break;
				 }
		
			case 5 : { a = 0.006282948;
					  b = -0.088743627;
					  c = 5.307587463;
					  endLength = 5.020577096;
					  newLength = c;
					  cout<<"case 5"<<endl;
					 break;
				 }
		
			case 6 : { a = 0.00952436;
					 b = -0.791671571;
					 c = 15.130201576;
					 endLength = 19.329221819;
					 newLength = c;
					 cout<<"case 6"<<endl;
					 break;
				 }
		
			case 7 : { a = 0.006282948;
					   b = -0.088743627;
					   c = 5.307587463;
					   endLength = 5.020577096;
					   newLength = c;
					   cout<<"case 7"<<endl;
					 break;
				 }
		
			case 8 : { a = 0.018848844;
					   b = -0.266230881;
					   c = 15.922762388;
					   endLength = 15.061731287;
					   newLength = c;
					   cout<<"case 8"<<endl;
					 break;
				 }
		
			case 9 : { a = 0.006282948;
					  b = -0.088743627;
					  c = 5.307587463;
					  endLength = 5.020577096;
					  newLength = c;
					  cout<<"case 9"<<endl;
					 break;
				 }
		}
		
		tgBasicActuator * const pMuscle = Muscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        cout<<m_totalTime<<endl;
       
		if(m_totalTime < startTime && m_totalTime < startTime + rampTime) {
			newLength = a * (m_totalTime - startTime) * (m_totalTime - startTime) + b * (m_totalTime - startTime) + c;
		} else if(m_totalTime > startTime + rampTime + holdTime) { // > END_TIME
			m_totalTime = 0;
		} else if(m_totalTime > startTime + rampTime + 4){    // && m_totalTime < startTime + rampTime + holdTime
			newLength = endLength;
		}
		pMuscle->setControlInput(newLength, dt);
    }
} 
 
 
 
 
 
 
/*
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
*/

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
