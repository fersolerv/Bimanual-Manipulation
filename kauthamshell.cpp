/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Alexander Perez, Jan Rosell, Nestor Garcia Hidalgo */



#include <planner/omplg/omplplanner.h>
#include "kauthamshell.h"
#include "util/kthutil/kauthamexception.h"
#include <iostream>
#include "UR5_kinematics.h"
//#include "rob.h"

namespace ob = ompl::base;
namespace Kautham {

bool kauthamshell::openProblem(istream* inputfile, vector <string> def_path) {
    try {
        //delete _problem;
        _problem = new Problem();
        if (_problem->setupFromFile(inputfile,def_path)) {
            _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
            return true;
        } else  {
            delete _problem;
            _problem = NULL;
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        delete _problem;
        _problem = NULL;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        delete _problem;
        _problem = NULL;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        delete _problem;
        _problem = NULL;
        return false;
    }
}

bool kauthamshell::openProblem(string problemfilename, vector <string> def_path) {
    try {
        std::cout << "Kautham is opening a problem file: " << problemfilename << endl;
        //delete _problem;
        _problem = new Problem();
        if (_problem->setupFromFile(problemfilename,def_path)) {
            _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
            return true;
        } else {
            delete _problem;
            _problem = NULL;
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        delete _problem;
        _problem = NULL;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        delete _problem;
        _problem = NULL;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        delete _problem;
        _problem = NULL;
        return false;
    }
}

bool kauthamshell::checkCollision(vector<KthReal> smpcoords, bool *collisionFree) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }
        if (smpcoords.size() != _problem->wSpace()->getNumRobControls()) {
            cout << "Sample has dimension " << smpcoords.size() << " and should have dimension "
                 << _problem->wSpace()->getNumRobControls() << endl;
            return false;
        }
        Sample* smp = new Sample(_problem->wSpace()->getNumRobControls());
        string msg;
        if (smp->setCoords(smpcoords)) {
            *collisionFree = !_problem->wSpace()->collisionCheck(smp,&msg);
            if(msg.empty()) {
                std::cout<<"Response for collision checking service is: Collision Free"<<std::endl;
            } else {
                std::cout<<"Response for collision checking service is: "<<msg<<std::endl;
            } //*collisionFree = !_problem->wSpace()->collisionCheck(smp);
            return true;
        } else {
            cout << "Sample has dimension " << smpcoords.size() << " and should have dimension "
                 << _problem->wSpace()->getNumRobControls() << endl;
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setRobotsConfig(vector<KthReal> smpcoords) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        Sample* smp = new Sample(_problem->wSpace()->getNumRobControls());
        if (smp->setCoords(smpcoords)) {
            _problem->wSpace()->moveRobotsTo(smp);

            //EUROC
            std::cout<<"Robot moved to: (";
            for(unsigned i=0; i<smpcoords.size(); i++)
            {
                std::cout<<smpcoords[i]<<" ";
            }
            std::cout<<std::endl;
            int s = _problem->wSpace()->getRobot(0)->getAttachedObject()->size();
            std::cout<<"Number of attached objets = "<<s<<std::endl;
            list<attObj>::iterator it = _problem->wSpace()->getRobot(0)->getAttachedObject()->begin();
            for( it = _problem->wSpace()->getRobot(0)->getAttachedObject()->begin();
                 it != _problem->wSpace()->getRobot(0)->getAttachedObject()->end();
                 ++it)
            {
                float x,y,z;
                string obsname = (*it).obs->getName();

                x = (*it).obs->getCurrentPos()->getSE3().getPos()[0];
                y = (*it).obs->getCurrentPos()->getSE3().getPos()[1];
                z = (*it).obs->getCurrentPos()->getSE3().getPos()[2];
                std::cout<<"Object "<<obsname<<" is at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
            }
            //EUROC



            return true;
        } else {
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setObstaclesConfig(vector<KthReal> smpcoords) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        Sample* smp = new Sample(_problem->wSpace()->getNumObsControls());
        if (smp->setCoords(smpcoords)) {
            _problem->wSpace()->moveObstaclesTo(smp);
            return true;
        } else {
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setQuery(vector<KthReal> init, vector<KthReal> goal) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        int d = _problem->wSpace()->getNumRobControls();
        SampleSet* samples = _problem->getSampleSet();
        samples->clear();

        string msg_init, msg_goal;

        //init
        Sample* smp = new Sample(d);
        smp->setCoords(init);
        if(_problem->wSpace()->collisionCheck(smp, &msg_init)) {
            std::cout<<"Init in collision: (";
            for(unsigned k=0;k<init.size();k++) std::cout<<init[k]<<" ";
            std::cout<<std::endl;
            std::cout<<msg_init<<std::endl;
            return false;
        }
        samples->add(smp);

        //goal
        smp = new Sample(d);
        smp->setCoords(goal);
        if(_problem->wSpace()->collisionCheck(smp, &msg_goal)) {
            std::cout<<"Goal in collision: (";
            for(unsigned k=0;k<goal.size();k++) std::cout<<goal[k]<<" ";
            std::cout<<std::endl;
            std::cout<<msg_goal<<std::endl;
            return false;
        }
        samples->add(smp);

        _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
        _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

        return true;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setInit(vector<KthReal> init) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        int d = _problem->wSpace()->getNumRobControls();
        SampleSet* samples = _problem->getSampleSet();
        Sample *goal = new Sample(samples->getSampleAt(1));
        samples->clear();

        //init
        Sample* smp = new Sample(d);
        smp->setCoords( init );
        if(_problem->wSpace()->collisionCheck(smp)) return false;
        samples->add(smp);

        //goal
        samples->add(goal);

        _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
        _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

        return true;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setGoal(vector<KthReal> goal) {
    try {

        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        int d = _problem->wSpace()->getNumRobControls();
        SampleSet* samples = _problem->getSampleSet();
        Sample *init = new Sample(samples->getSampleAt(0));
        samples->clear();

        //init
        samples->add(init);

        //goal
        Sample* smp = new Sample(d);
        smp->setCoords(goal);
        if (_problem->wSpace()->collisionCheck(smp)) {
            cout<<"Sample in collision"<<endl;
            return false;
        }
        samples->add(smp);
        _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
        _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));
        return true;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setInitObs(vector<KthReal> initObs) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        int d = _problem->wSpace()->getNumObsControls();
        Sample* smp = new Sample(d);
        smp->setCoords(initObs);
        _problem->wSpace()->setInitObsSample(smp);
        return (!_problem->wSpace()->collisionCheck(_problem->getSampleSet()->getSampleAt(0)) &&
                !_problem->wSpace()->collisionCheck(_problem->getSampleSet()->getSampleAt(1)));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::clearSampleSet() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        SampleSet* samples = _problem->getSampleSet();
        Sample *init = new Sample(samples->getSampleAt(0));
        Sample *goal = new Sample(samples->getSampleAt(1));
        samples->clear();

        samples->add(init);
        samples->add(goal);

        _problem->getPlanner()->setInitSamp(samples->getSampleAt(0));
        _problem->getPlanner()->setGoalSamp(samples->getSampleAt(1));

        return true;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setRobControls(istream* inputfile, vector<KthReal> init, vector<KthReal> goal) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setRobotControls(inputfile)) return false;
        return (setQuery(init,goal));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setRobControls(string controlsFile, vector<KthReal> init, vector<KthReal> goal) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setRobotControls(controlsFile)) return false;
        return (setQuery(init,goal));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setDefaultRobControls(vector<KthReal> init, vector<KthReal> goal) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setDefaultRobotControls()) return false;
        return (setQuery(init,goal));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setObsControls(istream* inputfile, vector<KthReal> initObs) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setObstacleControls(inputfile)) return false;
        return (setInitObs(initObs));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setObsControls(string controlsFile, vector<KthReal> initObs) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setObstacleControls(controlsFile)) return false;
        return (setInitObs(initObs));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setPlannerByName(string name) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (_problem->createPlanner(name)) {
            _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
            return true;
        } else {
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setPlanner(istream* inputfile) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        _problem->resetPlanner();
        if (_problem->createPlannerFromFile(inputfile)) {
            _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
            return true;
        } else {
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setPlanner(string problemfilename) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (_problem->createPlannerFromFile(problemfilename))  {
            _problem->getPlanner()->setInitSamp(_problem->getSampleSet()->getSampleAt(0));
            _problem->getPlanner()->setGoalSamp(_problem->getSampleSet()->getSampleAt(1));
            return true;
        } else {
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setPlannerParameter(string parameter, string value) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        return (_problem->getPlanner()->setParametersFromString(parameter+"|"+value));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::setFixedObsControls() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (!_problem->setFixedObstacleControls()) return false;
        vector<KthReal> coords;
        coords.resize(0);
        return (setInitObs(coords));
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::solve() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        return _problem->getPlanner()->solveAndInherit();
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::solve_savePaths(unsigned int robot, ofstream &outRr, ofstream &outRl ){

    bool _solved = false;

    if (_problem->getPlanner()->solveAndInherit()){
        _solved = true;

        og::SimpleSetupPtr ss = ((omplplanner::omplPlanner *)_problem->getPlanner())->SimpleSetupPtr();
        std::vector< ob::State * >  pathstates = ss->getSolutionPath().getStates();
        ob::State *state;
        Sample *sample;

        int d = _problem->wSpace()->getNumRobControls();

        vector < Sample* > setOfSamples;
        setOfSamples.clear();

        if(!pathstates.empty()){
            for (uint k = 0; k< pathstates.size();k++){
                //create a sample
                sample = new Sample(d);
                //set a mapped configuration
                sample->setMappedConf(_problem->getPlanner()->initSamp()->getMappedConf());

                //create a state from pathstates "k"
                state = pathstates.at(k);
                //convert the ompl state to kautham sample
                ((Kautham::omplplanner::omplPlanner*)_problem->getPlanner())->omplState2smp(state,sample);
                //adding the sample to the set of samples
                setOfSamples.push_back(sample);
            }
        }

        for (uint i = 0 ; i < setOfSamples.size() ; i++){
            vector < RobConf > configuration;
            configuration = setOfSamples.at(i)->getMappedConf();

            //if(robot == 1){
            //Rr
            for (uint r = (configuration.size()/2) ; r < configuration.size() ; ++r) {
                RobConf robotConfiguration = configuration.at(r);
                uint SE3Dim = robotConfiguration.getSE3().getCoordinates().size();
                uint RnDim = robotConfiguration.getRn().getCoordinates().size();

                for (unsigned j = 0; j <  SE3Dim; j++) {
                    outRr << robotConfiguration.getSE3().getCoordinates().at(j) << " ";
                }

                for (unsigned j = 0; j <  RnDim; j++) {
                    outRr << robotConfiguration.getRn().getCoordinates().at(j) << " ";
                }


            }
            outRr <<std::endl;
            if(i == setOfSamples.size()-1){
                outRr << 1 << std::endl;
                outRr << 1 << std::endl;
            }
            else{
                outRr << 0 << std::endl;
                outRr << 0 << std::endl;
            }
            //}
            //else{
            //Rl
            for (uint r = 0 ; r < (configuration.size()/2) ; ++r) {
                RobConf robotConfiguration = configuration.at(r);
                uint SE3Dim = robotConfiguration.getSE3().getCoordinates().size();
                uint RnDim = robotConfiguration.getRn().getCoordinates().size();

                for (unsigned j = 0; j <  SE3Dim; j++) {
                    outRl << robotConfiguration.getSE3().getCoordinates().at(j) << " ";
                }

                for (unsigned j = 0; j <  RnDim; j++) {
                    outRl << robotConfiguration.getRn().getCoordinates().at(j) << " ";
                }


            }
            outRl <<std::endl;
            if(i == setOfSamples.size()-1){
                outRl << 1 << std::endl;
                outRl << 1 << std::endl;
            }
            else{
                outRl << 0 << std::endl;
                outRl << 0 << std::endl;
            }
            //}
        }
    }
    return _solved;
}

double kauthamshell::getLastPlanComputationTime() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return -1.0;
        }

        Planner *_planner = _problem->getPlanner();
        if (_planner != NULL) {
            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
            case IOCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#if defined(KAUTHAM_USE_OMPL)
            case OMPLPLANNER:
                return ((omplplanner::omplPlanner*)_planner)->SimpleSetup()->getLastPlanComputationTime();
                break;
            case OMPLCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#if defined(KAUTHAM_USE_ODE)
            case ODEPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#endif
            case NOFAMILY:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            default:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            }
            return -1.0;
        } else {
            cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            return -1.0;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return -1.0;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return -1.0;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return -1.0;
    }
}

int kauthamshell::getNumEdges() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return -1;
        }

        Planner *_planner = _problem->getPlanner();
        if (_planner != NULL) {
            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
            case IOCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#if defined(KAUTHAM_USE_OMPL)
            case OMPLPLANNER: {
                ob::PlannerDataPtr pdata;
                pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)_planner)->ss->getSpaceInformation()));
                ((omplplanner::omplPlanner*)_planner)->ss->getPlanner()->getPlannerData(*pdata);
                return pdata->numEdges();
                break;
            }
            case OMPLCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#if defined(KAUTHAM_USE_ODE)
            case ODEPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#endif
            case NOFAMILY:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            default:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            }
            return -1;
        } else {
            cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            return -1;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return -1;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return -1;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return -1;
    }
}

int kauthamshell::getNumVertices() {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return -1;
        }

        Planner *_planner = _problem->getPlanner();
        if (_planner != NULL) {
            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
            case IOCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#if defined(KAUTHAM_USE_OMPL)
            case OMPLPLANNER: {
                ob::PlannerDataPtr pdata;
                pdata = ((ob::PlannerDataPtr) new ob::PlannerData(((omplplanner::omplPlanner*)_planner)->ss->getSpaceInformation()));
                ((omplplanner::omplPlanner*)_planner)->ss->getPlanner()->getPlannerData(*pdata);
                return pdata->numVertices();
                break;
            }
            case OMPLCPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#if defined(KAUTHAM_USE_ODE)
            case ODEPLANNER:
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
#endif
#endif
            case NOFAMILY:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            default:
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            }
            return -1;
        } else {
            cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            return -1;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return -1;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return -1;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return -1;
    }
}

bool kauthamshell::connect(vector<float> smpcoords1, vector<float> smpcoords2) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        unsigned dim = _problem->getDimension();
        if (smpcoords1.size() != dim || smpcoords2.size() != dim) {
            cout << "The samples should have dimension " << dim << endl;
        }

        Planner *_planner = _problem->getPlanner();
        if (_planner != NULL) {
            Sample *fromSample = new Sample(dim);
            fromSample->setCoords(smpcoords1);
            Sample *toSample = new Sample(dim);
            toSample->setCoords(smpcoords2);

            switch ((int)_planner->getFamily()) {
#if defined(KAUTHAM_USE_IOC)
            case IOCPLANNER: {

                ((IOC::iocPlanner*)_planner)->getLocalPlanner()->setInitSamp(fromSample);
                ((IOC::iocPlanner*)_planner)->getLocalPlanner()->setGoalSamp(toSample);

                if (((IOC::iocPlanner*)_planner)->getLocalPlanner()->canConect()) {
                    cout << "The samples can be connected." << endl;
                    return true;
                } else {
                    cout << "The samples can not be connected." << endl;
                    return false;
                }
                break;
            }
#endif
#if defined(KAUTHAM_USE_OMPL)
            case OMPLPLANNER: {
                ((omplplanner::omplPlanner*)_planner)->SimpleSetup()->setup();

                ob::ScopedState<ob::CompoundStateSpace> fromState(((omplplanner::omplPlanner*)_planner)->getSpace());
                ((omplplanner::omplPlanner*)_planner)->smp2omplScopedState(fromSample,&fromState);

                ob::ScopedState<ob::CompoundStateSpace> toState(((omplplanner::omplPlanner*)_planner)->getSpace());
                ((omplplanner::omplPlanner*)_planner)->smp2omplScopedState(toSample,&toState);

                bool connected = ((ob::MotionValidator*)((ob::SpaceInformation*)((omplplanner::omplPlanner*)
                                                                                 _planner)->
                                                         SimpleSetup()->getSpaceInformation().get())->
                                  getMotionValidator().get())->checkMotion(fromState.get(),toState.get());
                if (connected) {
                    cout << "The samples can be connected." << endl;
                    return true;
                } else {
                    cout << "The samples can not be connected." << endl;
                    return false;
                }
                break;
            }
            case OMPLCPLANNER: {
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
            }
#if defined(KAUTHAM_USE_ODE)
            case ODEPLANNER: {
                cout << "This function is not implemeted yet for this planner family" << endl;
                break;
            }
#endif
#endif
            case NOFAMILY: {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            }
            default: {
                cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
                break;
            }
            }
            return false;
        } else {
            cout << "The planner is not configured properly!!. Something is wrong with your application." << endl;
            return false;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::solve(ostream &graphVizPlannerDataFile) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        bool ret = false;
        if (_problem->getPlanner()->getFamily()==OMPLPLANNER) {
            ret = _problem->getPlanner()->solveAndInherit();
            if (ret) {

                omplplanner::omplPlanner *p = (omplplanner::omplPlanner *)_problem->getPlanner();

                ob::PlannerDataPtr pdata;
                pdata = ((ob::PlannerDataPtr) new ob::PlannerData(p->ss->getSpaceInformation()));

                p->ss->getPlanner()->getPlannerData(*pdata);
                pdata->printGraphviz(graphVizPlannerDataFile);
            }
        }

        return ret;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::getPath(ostream &path) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        bool ret = false;

        if (_problem->getPlanner()->getFamily()==OMPLPLANNER) {
            ret = _problem->getPlanner()->solveAndInherit();
            if (ret) {
                ((omplplanner::omplPlanner*)_problem->getPlanner())->SimpleSetup()->
                        getSolutionPath().printAsMatrix(path);
            }
        }

        return ret;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

int kauthamshell::addRobot(string robFile, KthReal scale, vector<KthReal> home, vector< vector<KthReal> > limits,
                           vector< vector<KthReal> > mapMatrix, vector<KthReal> offMatrix) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return -1;
        }

        if (!_problem->addRobot2WSpace(robFile,scale,home,limits)) return (-1);
        int index = _problem->wSpace()->getNumRobots()-1;
        Robot *rob = _problem->wSpace()->getRobot(index);
        int numCntr = _problem->wSpace()->getNumRobControls();
        int numDOF = 6+rob->getNumJoints();
        KthReal **MapMatrix = new KthReal*[numDOF];
        KthReal *OffMatrix = new KthReal[numDOF];
        for (int i = 0; i < numDOF; ++i) {
            OffMatrix[i] = offMatrix.at(i);
            MapMatrix[i] = new KthReal[numCntr];
            for (int j = 0; j < numCntr; ++j) {
                MapMatrix[i][j] = mapMatrix.at(i).at(j);
            }
        }
        rob->setMapMatrix(MapMatrix);
        rob->setOffMatrix(OffMatrix);

        return index;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return -1;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return -1;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return -1;
    }
}

int kauthamshell::addObstacle(string obsFile, KthReal scale, vector<KthReal> home) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return -1;
        }

        if (!_problem->addObstacle2WSpace(obsFile,scale,home)) return (-1);
        int index = _problem->wSpace()->getNumObstacles()-1;
        return index;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return -1;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return -1;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return -1;
    }
}

bool kauthamshell::removeRobot(unsigned index) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }
        if (index < 0 && index >= _problem->wSpace()->getNumRobots()) return false;
        else
            _problem->wSpace()->removeRobot(index);
            return true;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::removeObstacle(unsigned index) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }

        if (index < 0 && index >= _problem->wSpace()->getNumObstacles()) {
            return false;
        } else {
            _problem->wSpace()->removeObstacle(index);
            return true;
        }
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::attachObstacle2RobotLink(int robot, int link, int obs) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }
        bool ret = _problem->wSpace()->attachObstacle2RobotLink(robot,link,obs);
        float x,y,z;
        x = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[0];
        y = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[1];
        z = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[2];
        std::cout<<"Object "<<obs<<" attached at position ("<<x<<","<<y<<","<<z<<")"<<std::endl;
        if(ret) std::cout<<"attachfunction returned TRUE\n";
        else std::cout<<"attachfunction returned FALSE\n";
        return (ret);
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

bool kauthamshell::detachObstacle(uint obs) {
    try {
        if (_problem == NULL || !problemOpened()) {
            cout << "The problem is not opened" << endl;
            return false;
        }
        bool ret = _problem->wSpace()->detachObstacle(obs);
        float x,y,z;
        x = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[0];
        y = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[1];
        z = _problem->wSpace()->getObstacle(obs)->getCurrentPos()->getSE3().getPos()[2];
        cout<<"Object " << obs << " detached at position ("<< x << "," << y << "," << z << ")" << endl;
        if(ret) std::cout<<"detachfunction returned TRUE\n";
        else std::cout<<"detachfunction returned FALSE\n";
        return ret;
    } catch (const KthExcp& excp) {
        cout << "Error: " << excp.what() << endl << excp.more() << endl;
        return false;
    } catch (const exception& excp) {
        cout << "Error: " << excp.what() << endl;
        return false;
    } catch(...) {
        cout << "Something is wrong with the problem. Please run the "
             << "problem with the Kautham2 application at less once in order "
             << "to verify the correctness of the problem formulation.\n";
        return false;
    }
}

//Added //////////////////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////////////////////////

bool kauthamshell::ArmInverseKinematics(uint _dim, mt::Transform tfinal, vector<double> &controls) {

    bool ik = false;
    bool armType = true;
    double result [6];
    double controls_tmp [6];

    controls_tmp[0] = controls.at(0);
    controls_tmp[1] = controls.at(1);
    controls_tmp[2] = controls.at(2);
    controls_tmp[3] = controls.at(3);
    controls_tmp[4] = controls.at(4);
    controls_tmp[5] = controls.at(5);
    /*
        if(handType == AHL){
            armType = true;
        }
    */

    if(UR5_inv_kin(tfinal, armType, result)) {
        UR5_controls(controls_tmp,result);
        vector<KthReal> cords(22);
        for (uint i = 0; i < 6; i++) {
            cords[i] = controls_tmp[i];
            controls.at(i)= controls_tmp[i];
        }
        cords[6] = 0.0;
        cords[7] = 0.0;
        cords[8] = 0.0;
        cords[9] = 0.0;
        cords[10] = 0.0;
        cords[11] = 0.0;
        cords[12] = 0.0;
        cords[13] = 0.0;
        cords[14] = 0.0;
        cords[15] = 0.0;
        cords[16] = 0.0;
        cords[17] = 0.0;
        cords[18] = 0.0;
        cords[19] = 0.0;
        cords[20] = 0.0;
        cords[21] = 0.0;

        Sample* smp = new Sample(_dim);
        smp->setCoords(cords);

        //cout << "Sample Grasp: ";
        //for (unsigned i = 0; i < _dim; i++)
        //    cout << smp->getCoords().at(i) << " ";
        //cout << endl;

        if (_problem->wSpace()->collisionCheck(smp))
            cout << "Sample in collision" << endl;
            ik = false;
        else ik = true;
        
    }
    else ik = false;
    return ik;
}

bool kauthamshell::loadProblem(std::string absPath,std::string Path) {
    bool respond = false;
    string dir;
    vector<string> def_path;
    def_path.clear();
    ifstream inputfile;

    def_path.push_back(Path);
    dir = absPath.substr(0, absPath.find_last_of("/")+1);
    def_path.push_back(dir);
    def_path.push_back(dir+"../../models/");
    inputfile.open(absPath.c_str());

    if(openProblem(&inputfile, def_path)){
        respond = true;
    }
    return respond;
}

bool kauthamshell::isGraspableObject(int numObject, 
                                     uint NumRobot, 
                                     uint _dim, 
                                     std::vector<float> Position, 
                                     Sample &smp, 
                                     vector<float> &ControlConf, 
                                     vector<float> HandConf) {

    mt::Transform T1 = _problem->wSpace()->getObstacle(numObject)->getHomeTransform();
    mt::Transform TworldTOrobot = _problem->wSpace()->getRobot(NumRobot)->getHomeTransform();
    mt::Unit3 axis(Position.at(0), Position.at(1), Position.at(2));
    mt::Scalar angle = Position.at(3);
    mt::Rotation rot(axis, angle);
    mt::Point3 tr(Position.at(4), Position.at(5), Position.at(6));
    mt::Transform T2(rot, tr);
    // /////////////////////////////////////////////////////////////////////////
    mt::Unit3 axis1(1, 0, 0);
    mt::Scalar angle1 = -1.5708;
    mt::Rotation rot1(axis1, angle1);
    mt::Point3 tr1(0, 0, 0);
    mt::Transform T2b(rot1, tr1);
    // /////////////////////////////////////////////////////////////////////////////
    mt::Transform T3 = T1 * T2;
    mt::Transform Tfinal = TworldTOrobot.inverse() * T3;
    std::cout << "Wrist position relative to the Arm Base " << Tfinal << std::endl;
    vector<KthReal> cords(7);
    vector<KthReal> ControlConf_tmp(7);
    ControlConf_tmp.clear();
    ControlConf_tmp.resize(7);
    cords[0] = 0.0;
    cords[1] = 0.0;
    cords[2] = 0.0;
    cords[3] = 0.0;
    cords[4] = 0.0;
    cords[5] = 0.0;
    cords[6] = 0.0;

    ControlConf_tmp.at(0) = 0.0;
    ControlConf_tmp.at(1) = 0.0;
    ControlConf_tmp.at(2) = 0.0;
    ControlConf_tmp.at(3) = 0.0;
    ControlConf_tmp.at(4) = 0.0;
    ControlConf_tmp.at(5) = 0.0;
    ControlConf_tmp.at(6) = 0.0;

    vector < double > controls;
    controls.clear();
    for(uint i = 0; i < 6; i++) {
        controls.push_back(0.0);
    }

    bool invKinSolved = ArmInverseKinematics(_dim, Tfinal, controls);
    if (invKinSolved == true) {
        for (uint i = 0 ; i < 6; i++) {
            cords[i] = controls.at(i);
            ControlConf_tmp.at(i) = controls.at(i);
        }

        float temporal;
        ControlConf_tmp.resize(_dim);
        for (uint k = 6; k < ControlConf_tmp.size(); k++) {
            ControlConf_tmp.at(k) = HandConf.at(k-6);

            if((k == 7) || (k == 11) || (k == 15) || (k == 20) || (k == 8) || (k == 9) || (k == 12) || (k == 13) || (k == 16) || (k == 17) || (k == 21)){
                temporal = ControlConf_tmp.at(k) = HandConf.at(k-6) - 0.25;
                if(temporal < 0.0) {
                    ControlConf_tmp.at(k) = 0.0;
                }
                else ControlConf_tmp.at(k) = temporal;
            }
            else {
                if(k > 5) ControlConf_tmp.at(k) = HandConf.at(k-6); 
            }
            temporal = 0.0;
        }

        cords.resize(_dim);
        cords[6] = 0.0;
        cords[7] = 0.0;
        cords[8] = 0.0;
        cords[9] = 0.0;
        cords[10] = 0.0;
        cords[11] = 0.0;
        cords[12] = 0.0;
        cords[13] = 0.0;
        cords[14] = 0.0;
        cords[15] = 0.0;
        cords[16] = 0.0;
        cords[17] = 0.0;
        cords[18] = 0.0;
        cords[19] = 0.0;
        cords[20] = 0.0;
        cords[21] = 0.0;
        ControlConf.resize(_dim);
        ControlConf = ControlConf_tmp;
        return true;
    }
    return false;
}

bool kauthamshell::plan2Grasp(vector<float> goal, Sample* smp, unsigned int robot, ofstream &outRr, ofstream &outRl) {
    if(setGoal(goal)) {
        if(solve_savePaths(robot, outRr, outRl)) {
            cout << "Path found to grasp the object" << endl;
            smp->setCoords(goal);
            _problem->wSpace()->moveRobotsTo(smp);
        }
        else
            cout << "Path not found to grasp the object" << endl;
    }
}

bool kauthamshell::plan2Move(vector<float> init, vector<float> goal, Sample* smp, unsigned int robot, ofstream &outRr, ofstream &outRl) {

    setInit(init);
    setGoal(goal);

    if(solve_savePaths(robot, outRr, outRl)) {
        cout << "Path found to move the object" << endl;
        smp->setCoords(goal);
        _problem->wSpace()->moveRobotsTo(smp);
    }
    else
        cout << "Path not found to move the object" << endl;
}

bool kauthamshell::return2Home(vector<float> init, vector<float> home, unsigned int robot, ofstream &outRr, ofstream &outRl, Sample* smp) {
    if(setInit(init) && setGoal(home) && solve_savePaths(robot, outRr, outRl)) {
        smp->setCoords(home);
        _problem->wSpace()->moveRobotsTo(smp);
        cout << "Path found to return to home" << endl;
    }
    else
        cout << "Path not fount to return to home" << endl;
    }
}

bool kauthamshell::ChangeProblem(const char *absPath, vector <float> right, vector <float> left) {

    pugi::xml_document doc;
    //Load .xml file
    pugi::xml_parse_result result = doc.load_file(absPath);
    if (!result) {
        std::cout << "Parse error: " << result.description()
                  << ", character pos = " << result.offset<<std::endl;
    }
    else
        std::cout << "Problem file loaded"<<std::endl;
    //Convert float values to strings
    stringstream query;
    //right arm
    query << right[0] << " ";
    query << right[1] << " ";
    query << right[2] << " ";
    query << right[3] << " ";
    query << right[4] << " ";
    query << right[5] << " ";
    //left arm
    query << left[0] << " ";
    query << left[1] << " ";
    query << left[2] << " ";
    query << left[3] << " ";
    query << left[4] << " ";
    query << left[5] << " ";
    //right hand
    query << right[6] << " ";
    query << right[7] << " ";
    query << right[8] << " ";
    query << right[9] << " ";
    query << right[10] << " ";
    query << right[11] << " ";
    query << right[12] << " ";
    query << right[13] << " ";
    query << right[14] << " ";
    query << right[15] << " ";
    query << right[16] << " ";
    query << right[17] << " ";
    query << right[18] << " ";
    query << right[19] << " ";
    query << right[20] << " ";
    query << right[21] << " ";
    //left hand
    query << left[6] << " ";
    query << left[7] << " ";
    query << left[8] << " ";
    query << left[9] << " ";
    query << left[10] << " ";
    query << left[11] << " ";
    query << left[12] << " ";
    query << left[13] << " ";
    query << left[14] << " ";
    query << left[15] << " ";
    query << left[16] << " ";
    query << left[17] << " ";
    query << left[18] << " ";
    query << left[19] << " ";
    query << left[20] << " ";
    query << left[21];

    //Get the node to change data
    doc.child("Problem").child("Planner").child("Queries").child("Query").child("Init").text() = query.str().c_str();
    doc.child("Problem").child("Planner").child("Queries").child("Query").child("Goal").text() = query.str().c_str();

    //Save .xml file
    if(doc.save_file(absPath))
        cout << "Problem file changed and saved\n" << endl;
    else
        cout << "Problem file NOT changed\n" << endl;
}
}
