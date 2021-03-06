#if !defined(_KAUTHAMSHELL_H)
#define _KAUTHAMSHELL_H
#include <problem/problem.h>

using namespace std;
using namespace Kautham;

namespace Kautham {
    class kauthamshell {
    public:
        inline kauthamshell() {_problem = NULL;} //! Constructor
        inline ~kauthamshell(){} //! Destructor
        inline bool problemOpened() {
            return(_problem != NULL);
        } //! Informs wether there is a problem opened;
        inline void closeProblem() { if(problemOpened()) delete _problem;}
        bool openProblem(istream* inputfile, vector<string> def_path = vector<string>());
        bool openProblem(string problemfilename, vector<string> def_path);
        bool checkCollision(vector<KthReal> smpcoords, bool *collisionFree);
        bool setRobotsConfig(vector<KthReal> smpcoords);
        bool setObstaclesConfig(vector<KthReal> smpcoords);
        bool setQuery(vector<KthReal> init, vector<KthReal> goal);
        bool setInit(vector<KthReal> init);
        bool setGoal(vector<KthReal> goal);
        bool setInitObs(vector<KthReal> initObs);
        bool clearSampleSet();
        bool setRobControls(istream* inputfile, vector<KthReal> init, vector<KthReal> goal);
        bool setRobControls(string controlsFile, vector<KthReal> init, vector<KthReal> goal);
        bool setDefaultRobControls(vector<KthReal> init, vector<KthReal> goal);
        bool setObsControls(istream* inputfile, vector<KthReal> initObs);
        bool setObsControls(string controlsFile, vector<KthReal> initObs);
        bool setFixedObsControls();
        bool setPlannerByName(string name);
        bool setPlanner(istream* inputfile);
        bool setPlanner(string problemfilename);
        bool setPlannerParameter(string parameter, string value);
        bool solve(ostream &graphVizPlannerDataFile);
        bool getPath(ostream &path);
        bool solve();
        double getLastPlanComputationTime();
        int getNumEdges();
        int getNumVertices();
        bool connect(vector<KthReal> smpcoords1, vector<KthReal> smpcoords2);
        int addRobot(string robFile, 
                     KthReal scale, 
                     vector<KthReal> home, 
                     vector<vector<KthReal>> limits, 
                     vector<vector<KthReal>> mapMatrix, 
                     vector<KthReal> offMatrix);
        bool removeRobot(unsigned index);
        int addObstacle(string obsFile, KthReal scale, vector<KthReal> home);
        bool removeObstacle(unsigned index);
        bool attachObstacle2RobotLink(int robot, int link, int obs);
        bool detachObstacle(uint obs);

        //Added
        bool ArmInverseKinematics(uint _dim,mt::Transform tfinal, vector < double > &controls);
        
        bool loadProblem(std::string absPath,std::string Path);
        
        bool isGraspableObject(int numObject,
                               uint NumRobot, 
                               uint _dim, 
                               std::vector<float> Position, 
                               Sample &smp, 
                               vector<float> &ControlConf, 
                               vector<float> HandConf);

        bool solve_savePaths(unsigned int robot, ofstream &outRr, ofstream &outRl );
        
        bool plan2Grasp(vector<float> goal, 
                        Sample* smp, 
                        unsigned int robot, 
                        ofstream &outRr, 
                        ofstream &outRl);
        
        bool plan2Move(vector<float> init, 
                       vector<float> goal, 
                       Sample* smp, 
                       unsigned int robot, 
                       ofstream &outRr, 
                       ofstream &outRl);
        
        bool return2Home(vector<float> init, 
                         vector<float> home, 
                         unsigned int robot, 
                         ofstream &outRr, 
                         ofstream &outRl, 
                         Sample* smp);
        
        bool ChangeProblem(const char *absPath, vector<float> right, vector<float> left);

    //private:
        Problem *_problem;

    };
}
#endif  //_KAUTHAMSHELL_H
