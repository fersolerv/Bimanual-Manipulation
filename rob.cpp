#include "rob.h"
#include "kauthamshell.h"

//Constructor default
Rob::Rob(){
}

//Constructor
Rob::Rob(std::string Apath, std::string Hpath, std::string Controls, 
         unsigned int rob, unsigned int GSize) {
    ArmPath = Apath;
    HandPath = Hpath;
    ControlsPath = Controls;
    RobType = rob;
    GraspSize = GSize;

    SetGraspReachable.clear();
    graspReachebles.first.clear();
    graspReachebles.second.clear();
    SetGrasp.clear();
    CompleteConf.clear();
    SetGraspReachable.clear();
    ControlConf.clear();

    //Planning configurations
    InitConf.clear();
    GoalrolConf.clear();
    lowerLimits.resize(HANDJOINTS);
    upperLimits.resize(HANDJOINTS);
    loadjointlimits();
    loadGrasps();

    if(RobType == 0) {
        RrConfigurations();
        out.open("/home/fernando/PHD/Experiments/plans/Plan_Rr.txt",ios::app);
    }
    else {
        RlConfigurations();
        out.open("/home/fernando/PHD/Experiments/plans/Plan_Rl.txt",ios::app);
    }


}

//Convert to double
bool Rob::to_double(std::string str, double *arr, int num_to_read) {
     std::istringstream strs(str);
     int chars_to_read = strs.str().size();
     int num_read = 0;
     while (chars_to_read > 0 && num_read < num_to_read) {
         getline(strs,str,' ');
         if (str.size() > 0) {
             arr[num_read] = std::atof(str.c_str());
             num_read++;
         }
         chars_to_read -= str.size() + 1;
     }
     return (num_read == num_to_read && chars_to_read == -1);
}

//Load grasps from a file
void Rob::loadGrasps() {

    string armline,handline;
    double armvalues[7], handvalues[16];

    SetGrasp.resize(GraspSize);

    std::ifstream armfile(ArmPath.c_str());
    std::ifstream handfile(HandPath.c_str());

    if ((armfile.is_open())&&(handfile.is_open())) {
        cout << "Your GRASP file is OPEN."<< endl;
        for (unsigned int i = 0; i < GraspSize; i++) {
            std::getline (armfile,armline);
            std::getline (handfile,handline);
            to_double(armline,armvalues,7);
            to_double(handline,handvalues,HANDJOINTS);

            SetGrasp.at(i).ArmConf.resize(7);
            SetGrasp.at(i).HandConf.resize(HANDJOINTS);

            for (unsigned int j = 0; j < SetGrasp.at(i).ArmConf.size(); j++) { 
                SetGrasp.at(i).ArmConf.at(j) = armvalues[j];
            }

            for (unsigned int j = 0 ;j < SetGrasp.at(i).HandConf.size(); j++) {
                SetGrasp.at(i).HandConf.at(j) = ((handvalues[j]-lowerLimits.at(j))/(upperLimits.at(j)-lowerLimits.at(j)));
                //cout<<SetGrasp.at(i).HandConf.at(j)<<" ";
            }
            //cout<<endl;

        }
        armfile.close();
        handfile.close();
    }
    else {
        cout << "Your grasp file can NOT be opened. Check what is wrong."<< endl;
    }
}

void Rob::printSetGrasp(vector<grasp> Grasps) {
    cout << "Printing grasps" << endl;
    for(unsigned int i = 0; i < Grasps.size(); i++) {
        cout<<"Arm: ";
        for(unsigned int j = 0; j < Grasps.at(i).ArmConf.size(); j++) {
            cout<< Grasps.at(i).ArmConf.at(j)<< " ";
        }
        cout<<endl;
        cout<<"Hand: ";
        for(unsigned int j = 0; j < Grasps.at(i).HandConf.size(); j++) {
            cout<< Grasps.at(i).HandConf.at(j)<< " ";
        }
        cout<<endl;
    }
}

void Rob::printCompleteConf(vector< vector <float> > Grasps) {
    cout << "Printing grasps" << endl;
    for(unsigned int i = 0; i < Grasps.size(); i++) {
        cout << "Configuration: ";
        for(unsigned int j = 0; j < Grasps.at(i).size(); j++) {
            cout << Grasps.at(i).at(j) << " ";
        }
        cout<<endl;
    }
}

void Rob::printCompatibleGrasps(std::pair < std::vector<float> , std::vector<float> > GraspPair) {
    cout << "Init: ";
    for(unsigned int i = 0; i < GraspPair.first.size(); i++) {
        cout << GraspPair.first.at(i) << " ";
    }
    cout << endl;
    cout << "Goal: ";
    for(unsigned int i =0; i < GraspPair.second.size(); i++) {
        cout << GraspPair.second.at(i) << " ";
    }
    cout << endl;
}

void Rob::printCompatibleGrasps2(std::pair < std::vector<float> , std::vector<float> > GraspPair) {
    cout << "Init: ";
    for(unsigned int i = 0; i < GraspPair.first.size(); i++) {
        cout << GraspPair.first.at(i) << " ";
    }
    cout << endl;
    cout << "Exchange: ";
    for(unsigned int i =0; i < GraspPair.second.size(); i++) {
        cout << GraspPair.second.at(i) << " ";
    }
    cout << endl;
}

void Rob::printCompatibleGrasps3(std::pair < std::vector<float> , std::vector<float> > GraspPair) {
    cout << "Exchange: ";
    for(unsigned int i = 0; i < GraspPair.first.size(); i++) {
        cout << GraspPair.first.at(i) << " ";
    }
    cout << endl;
    cout << "Goal: ";
    for(unsigned int i =0; i < GraspPair.second.size(); i++) {
        cout << GraspPair.second.at(i) << " ";
    }
    cout << endl;
}

void Rob::RlConfigurations() {
    ControlConf.push_back(0.4);
    ControlConf.push_back(0.4);
    ControlConf.push_back(0.35);
    ControlConf.push_back(0.4);
    ControlConf.push_back(0.5);
    ControlConf.push_back(0.65);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
} 

void Rob::RrConfigurations() {
    ControlConf.push_back(0.6);
    ControlConf.push_back(0.35);
    ControlConf.push_back(0.65);
    ControlConf.push_back(0.35);
    ControlConf.push_back(0.5);
    ControlConf.push_back(0.35);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
    ControlConf.push_back(0.50);
}

void Rob::loadjointlimits() {
    //Lower limits
            //Index
    lowerLimits[0] = -0.47;
    lowerLimits[1] = -0.196;
    lowerLimits[2] = -0.174;
    lowerLimits[3] = -0.227;

        //Middle
    lowerLimits[4] = lowerLimits[0];
    lowerLimits[5] = lowerLimits[1];
    lowerLimits[6] = lowerLimits[2];
    lowerLimits[7] = lowerLimits[3];


        //Ring
    lowerLimits[8] = lowerLimits[0];
    lowerLimits[9] = lowerLimits[1];
    lowerLimits[10] = lowerLimits[2];
    lowerLimits[11] = lowerLimits[3];


        //Thumb
    lowerLimits[12] = 0.263;
    lowerLimits[13] = -0.105;
    lowerLimits[14] = -0.189;
    lowerLimits[15] = -0.162;

    //Upper limit

        //Index
    upperLimits[0] = 0.47;
    upperLimits[1] = 1.61;
    upperLimits[2] = 1.709;
    upperLimits[3] = 1.618;

        //Middle
    upperLimits[4] = upperLimits[0];
    upperLimits[5] = upperLimits[1];
    upperLimits[6] = upperLimits[2];
    upperLimits[7] = upperLimits[3];

        //Ring
    upperLimits[8] = upperLimits[0];
    upperLimits[9] = upperLimits[1];
    upperLimits[10] = upperLimits[2];
    upperLimits[11] = upperLimits[3];

        //Thumb
    upperLimits[12] = 1.396;
    upperLimits[13] = 1.163;
    upperLimits[14] = 1.644;
    upperLimits[15] = 1.719;
}

bool Rob::ShowInit2Goal(std::vector <float> InitConf, std::vector <float> GoalConf) {
    cout << endl;
    cout<<"INITIAL CONFIGURATION: ";
    for (unsigned int j = 0; j < InitConf.size(); j++) {
        cout << InitConf.at(j) <<" ";
    }
    cout<<endl;
    cout<<"GOAL CONFIGURATION: ";
    for (unsigned int j = 0; j < GoalConf.size(); j++) {
        cout << GoalConf.at(j) << " ";
    }
    cout<<endl;
}

bool Rob::ShowInit2Exch(std::vector <float> InitConf, std::vector <float> ExchConf) {
    cout << endl;
    cout<< "INITIAL CONFIGURATION: ";
    for (unsigned int j = 0;  j < InitConf.size();  j++) {
        cout << InitConf.at(j) <<" ";
    }
    cout << endl;
    cout << "EXCHANGE CONFIGURATION: ";
    for (unsigned int j = 0; j < ExchConf.size();  j++) {
        cout << ExchConf.at(j) << " ";
    }
    cout<<endl;
}

bool Rob::ShowExch2Goal(std::vector <float> ExchConf, std::vector <float> GoalConf) {
    cout << endl;
    cout << "EXCHANGE CONFIGURATION: ";
    for (unsigned int j = 0; j <ExchConf.size(); j++) {
        cout << ExchConf.at(j) << " ";
    }
    cout << endl;
    cout << "GOAL CONFIGURATION: ";
    for (unsigned int j = 0; j < GoalConf.size(); j++) {
        cout << GoalConf.at(j) << " ";
    }
    cout << endl;
}

bool Rob::showConf(std::vector <float> Conf) {
    cout << endl;
    cout<< "CONFIGURATION: ";
    for (unsigned int j = 0;  j < Conf.size();  j++) {
        cout << Conf.at(j) <<" ";
    }
    cout << endl;
}

bool Rob::ChangeControl(const char *absPath, vector <float> offset) {

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(absPath);

    if (!result) {
    std::cout << "Parse error: " << result.description()
              << ", character pos= " << result.offset<<std::endl;
    }
    else {
    std::cout << "File loaded" << std::endl;
    }

    int j = 0;
    int k = 0;
    for (pugi::xml_node tool = doc.child("ControlSet").child("Offset").child("DOF"); tool; tool = tool.next_sibling("DOF")){

        pugi::xml_attribute attr = tool.attribute("value");
        attr.set_value(offset[j]);
     j++;
    k++;
    }
    doc.save_file(absPath);
}
