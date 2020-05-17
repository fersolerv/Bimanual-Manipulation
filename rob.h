#ifndef ROB_H
#define ROB_H

#include <iostream>
#include <vector>
#include <string>
#include <string.h>
#include <fstream>
#include <sstream>
#include "std_msgs/String.h"

#define HANDJOINTS 16

//#include <mt/transform.h>

using namespace std;

class Rob{

public:
    //Constructors
    Rob();
    Rob(std::string Apath, std::string Hpath,std::string Controls, unsigned int rob, unsigned int GSize);
    //virtual ~Rob();

    //Variables
    unsigned int RobType;
    unsigned int GraspSize;
    std::string HandPath;
    std::string ArmPath;
    std::string ControlsPath;

    //Controls configurations
    std::vector<float> ControlConf;

    //Planning configurations
    std::vector<float> InitConf;
    std::vector<float> GoalrolConf;
    std::vector<double> lowerLimits;
    std::vector<double> upperLimits;
    struct grasp{
        vector <float> ArmConf;
        vector <float> HandConf; //Control values of kautham
    };
    std::vector<grasp> SetGrasp;
    std::vector<float> CompleteConf;
    std::vector<std::pair < std::vector<float> , std::vector<float> > > SetGraspReachable;
    std::pair < std::vector<float> , std::vector<float> > graspReachebles;

    std::ofstream out;

    //Functions
    bool to_double(std::string str, double *arr, int num_to_read);
    void loadGrasps();
    void printSetGrasp(vector <grasp> Grasps);
    void printCompleteConf(vector < vector <float> > Grasps);
    void printCompatibleGrasps(std::pair < std::vector<float> , std::vector<float> > GraspPair);
    void printCompatibleGrasps2(std::pair < std::vector<float> , std::vector<float> > GraspPair);
    void printCompatibleGrasps3(std::pair < std::vector<float> , std::vector<float> > GraspPair);
    void RlConfigurations();
    void RrConfigurations();
    void loadjointlimits();
    bool ShowInit2Goal(std::vector <float> InitConf, std::vector <float> GoalConf);
    bool ShowInit2Exch(std::vector <float> InitConf, std::vector <float> ExchConf);
    bool ShowExch2Goal(std::vector <float> ExchConf, std::vector <float> GoalConf);
    bool showConf(std::vector <float> Conf);
    bool ChangeControl(const char *absPath, vector <float> offset);
};

#endif // Rob_H
