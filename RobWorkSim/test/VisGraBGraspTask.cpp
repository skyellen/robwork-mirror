#include "VisGraBGraspTask.hpp"

#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <rw/rw.hpp>
USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace std;
using namespace boost::numeric;
using namespace boost::property_tree;

////////////////// GRASP TASK LOADING STUFF
#define TO_METER_FROM_DOCUNIT 1.0/1000.0
#define TO_DOCUNIT_FROM_METER 1000.0

namespace {


    struct compareElemStrings: public std::binary_function<std::string, std::string, bool> {

        compareElemStrings(){};

        bool operator()(const std::string& s1, const std::string& s2) const{
            // first we extract the name without namespaces (xmlns)
            std::string s1_tmp = s1;
            std::string s2_tmp = s2;

            size_t found = s1.find_last_of(':');
            if(found!=std::string::npos){
                s1_tmp = s1.substr(found+1);
            }

            found = s2.find_last_of(':');
            if(found!=std::string::npos){
                s2_tmp = s2.substr(found+1);
            }
            //std::cout << s1_tmp << "  " << s2_tmp << std::endl;
            return std::less<std::string>()(s1_tmp,s2_tmp);
        }

    };

}

typedef boost::property_tree::basic_ptree<std::string, std::string, compareElemStrings> PTree;

namespace {
    typedef boost::property_tree::basic_ptree<std::string, std::string, compareElemStrings>::iterator CI;
    typedef PTree::assoc_iterator OCI;


    struct ParserState {
    public:
        ParserState(std::string file):
            dwcfile(file),targetNr(0)
        {
        }

        const std::string dwcfile, dir;
        int targetNr;
    };

    bool isName(const std::string& elemName, const std::string& matchName){
        // first we extract the name without namespaces (xmlns)
        std::string elem = elemName;
        size_t found = elemName.find_last_of(':');
        if(found!=std::string::npos){
            elem = elemName.substr(found+1);
        }
        return elem == matchName;
    }

    bool has_child(PTree& tree, const std::string& name){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if(isName(p->first, name))
                return true;
        }
        return false;
    }

    std::pair<bool, double> toDouble(const std::string& str)
    {
        std::pair<bool, double> nothing(false, 0);
        istringstream buf(str);
        double x;
        buf >> x;
        if (!buf) return nothing;
        string rest;
        buf >> rest;
        if (buf) return nothing;
        else return make_pair(true, x);
    }

    std::vector<double> readArray(PTree& tree){
        istringstream buf(tree.get_value<string>());
        std::vector<double> values;

        std::string str;
        while( buf >> str ){
            const pair<bool, double> okNum = toDouble(str);
            if (!okNum.first)
                RW_THROW("Number expected. Got \"" << str << "\" ");
            values.push_back(okNum.second);
        }
        return values;
    }

    Q readQ(PTree& tree){
        //Log::debugLog()<< "ReadQ" << std::endl;
        std::vector<double> arr = readArray(tree);
        Q q(arr.size());
        for(size_t i=0;i<q.size();i++){
            q[i] = arr[i];
        }
        return q;
    }

    Vector3D<> readVector3D(PTree& tree){
        //Log::debugLog()<< "ReadVector3D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=3)
            RW_THROW("Unexpected sequence of values, must be length 3");
        return Vector3D<>(q[0],q[1],q[2]);
    }

    Transform3D<> readPose(PTree& tree, ParserState& state){
        // position
        Vector3D<> pos = readVector3D( tree.get_child("position") ) * TO_METER_FROM_DOCUNIT;
        Rotation3D<> rot;
        for (CI p1 = tree.begin(); p1 != tree.end(); ++p1) {
            //std::cout << p1->first << std::endl;
            if (isName(p1->first,"quaternion")) {
                std::vector<double> vals = readArray(p1->second);
                if(vals.size()!=4)
                    RW_THROW("quaternion is wrongly dimensioned!");
                rot = Quaternion<>(vals[1], vals[2],vals[3], vals[0]).toRotation3D();
            } else if (isName(p1->first,"rotmatrix")) {
                std::vector<double> vals = readArray(p1->second);
                if(vals.size()!=9)
                    RW_THROW("rotmatrix is wrongly dimensioned!");
                rot = Rotation3D<>(
                        vals[0], vals[1], vals[2],
                        vals[3], vals[4], vals[5],
                        vals[6], vals[7], vals[8]);
            } else {
                //RW_THROW("Unknown element!" << p1->first);
            }
        }

        //ctask->getPropertyMap().set<std::string>("GripperName", gripperType);
        return Transform3D<>(pos,rot);
    }


    VisGraBGraspTask::Ptr readExperiments(PTree& data, ParserState& state){

        rwlibs::task::CartesianTask::Ptr roottask = ownedPtr( new rwlibs::task::CartesianTask() );
        // this is a container for experiments
        string experimentsId = data.get_child("<xmlattr>").get<std::string>("id");


        // parse the gripper used
        std::map<std::string, boost::tuple<Q,Q,Q> > preshapes;
        PTree &gripper_tree = data.get_child("gripper");
        string gripperName = gripper_tree.get<string>("name");
        for (CI p = gripper_tree.begin(); p != gripper_tree.end(); ++p) {
            // each experiment is a VisGraBGraspTask
            if(isName(p->first, "preshape")){
                std::string id = p->second.get_child("<xmlattr>").get<std::string>("id");
                Q params1 = readQ(p->second.get_child("qopen"));
                Q params2 = readQ(p->second.get_child("qclose"));
                Q params3 = readQ(p->second.get_child("maxtau"));
                preshapes[id] = boost::make_tuple(params1,params2,params3);
            }
        }
        string tcpframe = gripper_tree.get<string>("tcpframe");
        string controllername = gripper_tree.get<string>("graspcontroller", "GraspController");

        roottask->getPropertyMap().set<std::string>("Gripper", gripperName);
        roottask->getPropertyMap().set<std::string>("TCP", tcpframe);
        roottask->getPropertyMap().set<std::string>("ControllerName", controllername);
        // add stuff to the RobWork task description


        // now parse the list of grasps
        for (CI p = data.begin(); p != data.end(); ++p) {
            // each experiment is a VisGraBGraspTask
            std::string id;
            Transform3D<> pose;
            Q openQ;
            Q closeQ;
            Q tau;
            if(isName(p->first, "grasp")){
                id = p->second.get_child("<xmlattr>").get<std::string>("id");
                pose = readPose(p->second.get_child("pose"), state);
                openQ = readQ(p->second.get_child("qopen") );
                closeQ = readQ(p->second.get_child("qclose") );
                tau = readQ(p->second.get_child("maxtau") );
            } else if(p->first=="preshapegrasp") {
                id = p->second.get_child("<xmlattr>").get<std::string>("id");
                pose = readPose(p->second.get_child("pose"), state);
                id = p->second.get_child("preshape").get_child("<xmlattr>").get<std::string>("id");
                if(preshapes.find(id)==preshapes.end())
                    RW_THROW("Error: Preshape id "<< id << " is not defined in gripper "<< gripperName);
                boost::tie(openQ,closeQ,tau) = preshapes[id];
            } else if(p->first=="contactgrasp") {
                id = p->second.get_child("<xmlattr>").get<std::string>("id");
                // here we need to do some inverse kinematics
                RW_THROW("Not impl yet");
            } else {
                continue;
            }

            // create the task or add the target to an existing task
            rwlibs::task::CartesianTask::Ptr ctask = ownedPtr( new rwlibs::task::CartesianTask() );

            ctask->getPropertyMap().set<Transform3D<> >("Retract",Transform3D<>( Vector3D<>(0,0,0.1)) );
            ctask->getPropertyMap().set<Q>("OpenQ", openQ);
            ctask->getPropertyMap().set<Q>("CloseQ", closeQ);
            ctask->getPropertyMap().set<Q>("TauMax", tau);
            ctask->getPropertyMap().set<string>("VisGraBID", experimentsId + "." + id);

            ctask->addTargetByValue( pose );


        }

        return ownedPtr( new VisGraBGraspTask(roottask) );
    }
}


std::vector<VisGraBGraspTask::Ptr> VisGraBGraspTask::load(const std::string& filename){
    std::string file = IOUtil::getAbsoluteFileName(filename);
    std::string firstelem = IOUtil::getFirstXMLElement(file);
    //std::cout << "FIRST ELEMENT: " << firstelem << std::endl;

    std::vector<VisGraBGraspTask::Ptr> tasks;

    if(firstelem=="CartesianTask"){
        rwlibs::task::CartesianTask::Ptr grasptask;
        XMLTaskLoader loader;
        loader.load( file );
        grasptask = loader.getCartesianTask();
        tasks.push_back( ownedPtr( new VisGraBGraspTask(grasptask) ) );

    } else {
        try {
            ParserState state(file);

            //state.dir = StringUtil::getDirectoryName(file);
            PTree tree;
            read_xml(file, tree);

            PTree& visgrab_tree = tree.get_child("visgrab");
            //PTree& exp_tree = visgrab_tree.get_child("experiments");

            for (CI p = visgrab_tree.begin(); p != visgrab_tree.end(); ++p) {
                if(isName(p->first, "experiments")){
                    VisGraBGraspTask::Ptr gtask = readExperiments(p->second, state);
                    tasks.push_back(gtask);
                }
            }


            //rw::loaders::XML::printTree(tree, std::cout);
        } catch (const ptree_error& e) {
            // Convert from parse errors to RobWork errors.
            RW_THROW(e.what());
        }

    }

    return tasks;
}

