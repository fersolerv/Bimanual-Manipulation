#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <mt/transform.h>
#include <Inventor/SoDB.h>
#include "kauthamshell.h"
#include "rob.h"
#include "../../problem/robot.h"
#include <stdlib.h>
#define GRASPSIZEl 200
#define GRASPSIZEr 200

using namespace std;
using namespace Kautham;

int main(){

    SoDB::init();
    kauthamshell *kth;
    kth = new kauthamshell();
    Rob *Rr; //Right Rob = 0
    Rob *Rl; //Left Rob = 1

    //Choose a problem and the path:
    string absPath,Path;
    const char *absPath2;
    absPath = "/home/users/fernando.soler/PHD/Experiments/xml/UR5_AH_LR_BOTH_ARMS.xml";
    absPath2 = "/home/users/fernando.soler/PHD/Experiments/xml/UR5_AH_LR_BOTH_ARMS2.xml";
    Path = "/home/users/fernando.soler/PHD/Experiments/models";

    //Load problem:
    if(kth->loadProblem(absPath,Path)){
        cout << "THE PROBLEM FILE (" << absPath << ") HAS BEEN LOADED SUCCESSFULLY\n";
    }
    else{
        cout << "THE PROBLEM FILE HAS NOT BEEN LOADED. "
             << "Please take care with the problem definition \n";
        return -1;
    }

    //Choose The Grasps Files, Robot Configurations, and Controls files path for the left and right robots:
    //Right Robot
    string ArmRightPath = "/home/users/fernando.soler/PHD/Experiments/grasps/Right_Grasp_NEW_KAUTHAM.txt";
    string HandRightPath = "/home/users/fernando.soler/PHD/Experiments/grasps/Right_Grasps_NEWConfigurations.txt";
    string ControlPathR = "/home/users/fernando.soler/PHD/Experiments/models/controls/UR5_AH_R.cntr";
    const char *ControlPathR2 = "/home/users/fernando.soler/PHD/Experiments/models/controls/UR5_AH_R2.cntr";

    //Left Robot
    string ArmLeftPath = "/home/users/fernando.soler/PHD/Experiments/grasps/Left_Grasp_NEW_KAUTHAM.txt";
    string HandLeftPath = "/home/users/fernando.soler/PHD/Experiments/grasps/Left_Grasps_NEWConfigurations.txt";
    string ControlPathL = "/home/users/fernando.soler/PHD/Experiments/models/controls/UR5_AH_L.cntr";
    const char *ControlPathL2 = "/home/users/fernando.soler/PHD/Experiments/models/controls/UR5_AH_L2.cntr";

    Rr = new Rob(ArmRightPath, HandRightPath, ControlPathR, 0, GRASPSIZEr);
    Rl = new Rob(ArmLeftPath, HandLeftPath, ControlPathL, 1, GRASPSIZEl);

    //Rl->printSetGrasp(Rl->SetGrasp);
    //Rr->printSetGrasp(Rr->SetGrasp);

    kth->setRobControls(ControlPathR, Rr->ControlConf, Rr->ControlConf);
    unsigned int obj_init = 1, obj_goal = 2, obj_exch = 3;

    SampleSet* _samples;
    uint _dim;
    _samples = kth->_problem->getSampleSet();
    _dim = _samples->getSampleAt(0)->getDim();  //_dim tiene el tamaño de un robot
    vector <float> grasp;
    grasp.resize(7, 0.0);

    Sample* grasp_smp = new Sample(_dim);
    Sample* rob_homeL = new Sample(_dim);
    Sample* rob_homeR = new Sample(_dim);
    Sample* rob_conf = new Sample(_dim);
    Sample* rob_init = new Sample(_dim);
    Sample* rob_goal = new Sample(_dim);
    Sample* rob_exch = new Sample(_dim);

    //GRASPS FOR THE RIGHT HAND ///////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < Rr->SetGrasp.size(); i++) {
        bool reach_init = true, reach_exch = true;
        for (unsigned int j = 0; j < Rr->SetGrasp.at(i).ArmConf.size(); j++) {
            grasp.at(j) = Rr->SetGrasp.at(i).ArmConf.at(j);
        }

        // Compute all the grasp that are reachable at Initial Pose of the object
        Rr->CompleteConf.clear();
        if (kth->isGraspableObject(obj_init, Rr->RobType, _dim, grasp, *grasp_smp,Rr->CompleteConf, Rr->SetGrasp.at(i).HandConf)) {
            Rr->graspReachebles.first = Rr->CompleteConf;
            //cout << "Is graspable at INIT\n";
            //graspInfo.at(it).graspSmp.push_back(grasp_smp);
        }
        else {
            Rr->graspReachebles.first.resize(0);
            reach_init = false;
            //cout<<"No graspable\n";
            delete grasp_smp;
            grasp_smp = new Sample(_dim);
        }

        // Compute all the grasps that are reachable at Exchange Pose of the object
        Rr->CompleteConf.clear();
        if (kth->isGraspableObject(obj_exch, Rr->RobType, _dim, grasp, *grasp_smp, 
                                   Rr->CompleteConf, Rr->SetGrasp.at(i).HandConf)) {
            Rr->graspReachebles.second = Rr->CompleteConf;
            //cout << "Is graspable at EXCH\n";
            //graspInfo.at(it).graspSmp.push_back(grasp_smp);
        }
        else {
            Rr->graspReachebles.second.resize(0);
            reach_exch = false;
            //cout<<"No graspable \n";
            delete grasp_smp;
        }

        if(((reach_init) && (reach_exch)) /*|| ((!reach_init) && (reach_exch)) || ((reach_init) && (!reach_exch))*/) {
            Rr->SetGraspReachable.push_back(Rr->graspReachebles);
            // Rr->printCompatibleGrasps2(Rr->graspReachebles);  //Print the pair of grasp compatible a diferent object's conf
            //cout << endl;
        }
        Rr->graspReachebles.first.clear();
        Rr->graspReachebles.second.clear();
        grasp_smp = new Sample(_dim);
    }

    //GRASPS FOR THE LEFT HAND ///////////////////////////////////////////////////////////////
    kth->setRobControls(Rl->ControlsPath, Rl->ControlConf, Rl->ControlConf);

    for (unsigned int i = 0; i < Rl->SetGrasp.size(); i++){
        bool reach_exch = true, reach_goal = true;
        for (unsigned int j = 0; j < Rl->SetGrasp.at(i).ArmConf.size(); j++){
            grasp.at(j) = Rl->SetGrasp.at(i).ArmConf.at(j);
        }
        // Compute all the grasp that are reachable at Exch Pose of the object
        Rl->CompleteConf.clear();
        if (kth->isGraspableObject(obj_exch, Rl->RobType, _dim, grasp, *grasp_smp, 
                                   Rl->CompleteConf, Rl->SetGrasp.at(i).HandConf)){
            Rl->graspReachebles.first = Rl->CompleteConf;
            //cout << "Is graspable\n";
        }
        else {
            Rl->graspReachebles.first.resize(0);
            reach_exch = false;
            //cout<<"No graspable\n";
            delete grasp_smp;
            grasp_smp = new Sample(_dim);
        }

        // Compute all the grasps that are reachable at Goal Pose of the object
        Rl->CompleteConf.clear();
        if (kth->isGraspableObject(obj_goal, Rl->RobType, _dim, grasp, *grasp_smp, 
                                   Rl->CompleteConf, Rl->SetGrasp.at(i).HandConf)) {
            Rl->graspReachebles.second = Rl->CompleteConf;
            //cout << "Is graspable\n";
        }
        else {
            Rl->graspReachebles.second.resize(0);
            reach_goal = false;
            //cout<<"No graspable \n";
            delete grasp_smp;
        }

        if(((reach_exch) && (reach_goal)) /*|| ((!reach_exch) && (reach_goal)) || ((reach_exch) && (!reach_goal))*/) {
            Rl->SetGraspReachable.push_back(Rl->graspReachebles);
            //Rl->printCompatibleGrasps3(Rl->graspReachebles);  //Print the pair of grasp compatible a diferent object's conf
        }
        Rl->graspReachebles.first.clear();
        Rl->graspReachebles.second.clear();
        grasp_smp = new Sample(_dim);
    }

    // MOTION PLANNER  ///////////////////////////////////////////////////////////////////////////////
    kth->removeObstacle(obj_exch);
    kth->removeObstacle(obj_goal);
    //RIGHT
    kth->setRobControls(ControlPathR, Rr->ControlConf, Rr->ControlConf);
    for (unsigned int i =1; i < Rr->SetGraspReachable.size(); i++) {

        if((Rr->SetGraspReachable.at(i).first.size()) == (Rr->SetGraspReachable.at(i).second.size())) {
            Rr->ShowInit2Exch(Rr->SetGraspReachable.at(i).first, Rr->SetGraspReachable.at(i).second);

            if(kth->plan2Grasp(Rr->SetGraspReachable.at(i).first, rob_init, Rr->RobType, Rr->out, Rl->out)) {
                cout << "with the Right hand " << endl;
                kth->attachObstacle2RobotLink(Rr->RobType, 7, obj_init);

                if(kth->plan2Move(Rr->SetGraspReachable.at(i).first, Rr->SetGraspReachable.at(i).second,
                                  rob_exch, Rr->RobType, Rr->out, Rl->out)){
                    cout << "with the Right hand " << endl;
                    cout << endl;

                    //LEFT
                    Rl->ChangeControl(ControlPathL2, Rr->SetGraspReachable.at(i).second);
                    if(kth->setRobControls(ControlPathL2, Rl->ControlConf, Rl->ControlConf)){
                        cout << "Left controls set" << endl;

                        for(unsigned int j = 1; j < Rl->SetGraspReachable.size(); j++) {

                            if((Rl->SetGraspReachable.at(j).first.size()) == (Rl->SetGraspReachable.at(j).second.size())) {
                                Rl->ShowExch2Goal(Rl->SetGraspReachable.at(j).first, Rl->SetGraspReachable.at(j).second);

                                if(kth->plan2Grasp(Rl->SetGraspReachable.at(j).first, rob_exch, Rl->RobType, Rr->out, Rl->out)) {
                                    cout << "with the Left hand " << endl;
                                    cout << endl;

                                    //Right
                                    Rr->ChangeControl(ControlPathR2, Rl->SetGraspReachable.at(j).first);
                                    kth->setRobControls(ControlPathR2, Rr->SetGraspReachable.at(i).second, Rr->SetGraspReachable.at(i).second);
                                    kth->detachObstacle(obj_init);

                                    //Left
                                    Rl->ChangeControl(ControlPathL2, Rr->ControlConf);
                                    kth->setRobControls(ControlPathL2, Rl->SetGraspReachable.at(j).first, Rl->SetGraspReachable.at(j).first);
                                    //kth->attachObstacle2RobotLink(Rl->RobType, 7, obj_init);

                                    if(kth->plan2Move(Rl->SetGraspReachable.at(j).first, Rl->SetGraspReachable.at(j).second, 
                                                      rob_goal, Rl->RobType, Rr->out, Rl->out)) {
                                        cout << "with the Left hand." << endl;
                                        cout << endl;
                                        //kth->detachObstacle(obj_init);

                                        if(kth->return2Home(Rl->SetGraspReachable.at(j).second, Rl->ControlConf, 
                                                            Rl->RobType, Rr->out, Rl->out, rob_homeL)) {
                                            cout << "with the LEFT arm." << endl;
                                            cout << endl;

                                            //RIGHT
                                            Rr->ChangeControl(ControlPathR2, Rl->ControlConf);
                                            kth->setRobControls(ControlPathR2, Rr->SetGraspReachable.at(i).second, Rr->SetGraspReachable.at(i).second);

                                            if(kth->return2Home(Rr->SetGraspReachable.at(i).second, Rr->ControlConf, 
                                                                Rr->RobType, Rr->out, Rl->out, rob_homeR)) {
                                                cout << "with the RIGHT arm." << endl;
                                                cout << endl;
                                            }
                                        }
                                    }


                                    //                                    kth->closeProblem();
                                    //                                    kth->ChangeProblem(absPath2, Rr->SetGraspReachable.at(i).second, Rl->SetGraspReachable.at(j).first);

                                    //                                    //Load problem:
                                    //                                    if(kth->loadProblem(absPath2,Path)){
                                    //                                        cout << "THE PROBLEM FILE (" << absPath2 << ") HAS BEEN LOADED SUCCESSFULLY\n";
                                    //                                    }
                                    //                                    else{
                                    //                                        cout << "THE PROBLEM FILE HAS NOT BEEN LOADED. "
                                    //                                             << "Please take care with the problem definition \n";
                                    //                                        return -1;
                                    //                                    }

                                    //                                    if(kth->setRobControls(ControlPathL2, Rl->ControlConf, Rl->ControlConf)){
                                    //                                        cout << "Left controls set" << endl;

                                    //                                        kth->attachObstacle2RobotLink(Rl->RobType, 7, obj_exch);
                                    //                                        kth->removeObstacle(obj_init);
                                    //                                        kth->removeObstacle(obj_goal);

                                    //                                        if(kth->plan2Move(Rl->SetGraspReachable.at(j).first, Rl->SetGraspReachable.at(j).second, rob_goal, Rl->RobType, Rr->out, Rl->out)){
                                    //                                            cout << "with the Left hand." << endl;
                                    //                                            break;

                                    //                                        }

                                    //                                    }


                                }
                            }
                            break;
                        }
                    }
                }
            }
        }
        break;
    }
}
