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
 * @file T6Controller.cpp
 * @brief Preferred Length Controller for T6Model
 * @author Christopher Peterson
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6Controller.h"
// This application
#include "T6Model.h"
// This library
#include "core/tgBasicActuator.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

# define M_PI 3.14159265358979323846 
# define END_TIME 30

using namespace std;

//Constructor using the model subject and a single pref length for all muscles.
T6Controller::T6Controller(const double initialLength, double timestep) :
    m_initialLengths(initialLength),
    m_totalTime(0.0),
    dt(timestep) {}

//Fetch all the muscles and set their preferred length
void T6Controller::onSetup(T6Model& subject) {
	this->m_totalTime=0.0;
    const double Active_length = 12.247;


	const std::vector<tgBasicActuator*> activeMuscle = subject.find<tgBasicActuator>("active");

    for (size_t i=0; i<activeMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = activeMuscle[i];
		assert(pMuscle != NULL);
		pMuscle->setControlInput(Active_length, dt);
    }
}

// Set target length of each muscle, then move motors accordingly
void T6Controller::onStep(T6Model& subject, double dt) {
    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;

	
    setAlphaMuscleTargetLength(subject, dt);
    setBravoMuscleTargetLength(subject, dt);
    setCharlieMuscleTargetLength(subject, dt);
    setDeltaMuscleTargetLength(subject, dt);
    setEchoMuscleTargetLength(subject, dt);
    //moveAllMotors(subject, dt);
    //updateActions(dt);
}
 
void T6Controller::setAlphaMuscleTargetLength(T6Model& subject, double dt) {
    const double mean_activeMuscle_length = 12.247; //TODO: define according to vars
    double newLength = 0;
    const double amplitude    = mean_activeMuscle_length/2;
    const double angular_freq = 2;
    const double phase = M_PI/2;
    const double dcOffset     = mean_activeMuscle_length;
    const std::vector<tgBasicActuator*> alphaMuscle = subject.find<tgBasicActuator>("alpha");

    for (size_t i=0; i<alphaMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = alphaMuscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        //newLength = dcOffset - amplitude*m_totalTime/5;
        newLength = dcOffset;
        if(newLength < dcOffset/8) {
            newLength = dcOffset/8;
        }
/*
        if(m_totalTime > 10) {    //15
            m_totalTime = 0;
        }
  */
		if(m_totalTime > 4 && m_totalTime < 8) {
			newLength = dcOffset - amplitude*m_totalTime/4;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else {
			newLength = dcOffset;
		}
        
        /*
        else if(m_totalTime > 10) {
			m_totalTime = 0;
		}
        */
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}

void T6Controller::setBravoMuscleTargetLength(T6Model& subject, double dt) {
    const double mean_activeMuscle_length = 12.247; //TODO: define according to vars
    double newLength = 0;
    const double amplitude    = mean_activeMuscle_length/2;
    const double angular_freq = 2;
    const double phase = M_PI/2;
    const double dcOffset     = mean_activeMuscle_length;
    const std::vector<tgBasicActuator*> bravoMuscle = subject.find<tgBasicActuator>("bravo");

    for (size_t i=0; i<bravoMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = bravoMuscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        //newLength = dcOffset - amplitude*m_totalTime/5;
        newLength = dcOffset;
        if(newLength < dcOffset/8) {
            newLength = dcOffset/8;
        }
/*
        if(m_totalTime > 10) {    //15
            m_totalTime = 0;
        }
  */
		if(m_totalTime > 8 && m_totalTime < 12) {
			newLength = dcOffset - amplitude*(m_totalTime-4)/4;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else {
			newLength = dcOffset;
		}
        
        /*
        else if(m_totalTime > 10) {
			m_totalTime = 0;
		}
        */
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}

void T6Controller::setCharlieMuscleTargetLength(T6Model& subject, double dt) {
    const double mean_activeMuscle_length = 12.247; //TODO: define according to vars
    double newLength = 0;
    const double amplitude    = mean_activeMuscle_length/2;
    const double angular_freq = 2;
    const double phase = M_PI/2;
    const double dcOffset     = mean_activeMuscle_length;
    const std::vector<tgBasicActuator*> charlieMuscle = subject.find<tgBasicActuator>("charlie");

    for (size_t i=0; i<charlieMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = charlieMuscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        //newLength = dcOffset - amplitude*m_totalTime/5;
        newLength = dcOffset;
        if(newLength < dcOffset/8) {
            newLength = dcOffset/8;
        }
/*
        if(m_totalTime > 10) {    //15
            m_totalTime = 0;
        }
  */
		if(m_totalTime > 12 && m_totalTime < 16) {
			newLength = dcOffset - amplitude*(m_totalTime-8)/4;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else {
			newLength = dcOffset;
		}
        
        /*
        else if(m_totalTime > 10) {
			m_totalTime = 0;
		}
        */
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}

void T6Controller::setDeltaMuscleTargetLength(T6Model& subject, double dt) {
    const double mean_activeMuscle_length = 12.247; //TODO: define according to vars
    double newLength = 0;
    const double amplitude    = mean_activeMuscle_length/2;
    const double angular_freq = 2;
    const double phase = M_PI/2;
    const double dcOffset     = mean_activeMuscle_length;
    const std::vector<tgBasicActuator*> deltaMuscle = subject.find<tgBasicActuator>("delta");

    for (size_t i=0; i<deltaMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = deltaMuscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        //newLength = dcOffset - amplitude*m_totalTime/5;
        newLength = dcOffset;
        if(newLength < dcOffset/8) {
            newLength = dcOffset/8;
        }
/*
        if(m_totalTime > 10) {    //15
            m_totalTime = 0;
        }
  */
		if(m_totalTime > 16 && m_totalTime < 20) {
			newLength = dcOffset - amplitude*(m_totalTime-12)/4;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else {
			newLength = dcOffset;
		}
        
        /*
        else if(m_totalTime > 10) {
			m_totalTime = 0;
		}
        */
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}

void T6Controller::setEchoMuscleTargetLength(T6Model& subject, double dt) {
    const double mean_activeMuscle_length = 12.247; //TODO: define according to vars
    double newLength = 0;
    const double amplitude    = mean_activeMuscle_length/2;
    const double angular_freq = 2;
    const double phase = M_PI/2;
    const double dcOffset     = mean_activeMuscle_length;
    const std::vector<tgBasicActuator*> echoMuscle = subject.find<tgBasicActuator>("echo");

    for (size_t i=0; i<echoMuscle.size(); i++) {
		tgBasicActuator * const pMuscle = echoMuscle[i];
		assert(pMuscle != NULL);
        cout <<"t: " << pMuscle->getCurrentLength() << endl;
        //newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        //newLength = dcOffset - amplitude*m_totalTime/5;
        newLength = dcOffset;
        if(newLength < dcOffset/8) {
            newLength = dcOffset/8;
        }
/*
        if(m_totalTime > 10) {    //15
            m_totalTime = 0;
        }
  */
		if(m_totalTime > 20 && m_totalTime < 24) {
			newLength = dcOffset - amplitude*(m_totalTime-16)/4;
		} else if(m_totalTime > END_TIME) {
			m_totalTime = 0;
		} else {
			newLength = dcOffset;
		}
        
        /*
        else if(m_totalTime > 10) {
			m_totalTime = 0;
		}
        */
        
        std::cout<<"calculating activeMuscle target length:" << newLength << "\n";
        std::cout<<"m_totalTime: " << m_totalTime << "\n";
		pMuscle->setControlInput(newLength, dt);
        cout <<"t+1: " << pMuscle->getCurrentLength() << endl;
    }
}


//Move motors for all the muscles
void T6Controller::moveAllMotors(T6Model& subject, double dt) {
    const std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
		tgBasicActuator * const pMuscle = muscles[i];
		assert(pMuscle != NULL);
		pMuscle->moveMotors(dt);
	}
     
}

//Scale actions according to Min and Max length of muscles.
vector< vector <double> > T6Controller::transformActions(vector< vector <double> > actions)
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
void T6Controller::applyActions(T6Model& subject, vector< vector <double> > act)
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
