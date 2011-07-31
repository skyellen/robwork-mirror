
#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <csignal>
#include <sys/stat.h>

#include <rw/rw.hpp>
#include <rwlibs/task.hpp>

#include <vector>

#include <rw/geometry/STLFile.hpp>
#include <rw/geometry/Triangle.hpp>
#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <rw/math/Vector3D.hpp>

#include <rwsim/dynamics/ContactManifold.hpp>
#include <rw/loaders/rwxml/XML.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>


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

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

using namespace boost::numeric;
using namespace boost::property_tree;

namespace {
    typedef PTree::iterator CI;


    struct ParserState {
    public:
        ParserState(std::string file):
            dwcfile(file)
        {
        }
        const std::string dwcfile, dir;
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


    rwlibs::task::CartesianTask::Ptr readGrasp(PTree& tree, ParserState& state){
        rwlibs::task::CartesianTask::Ptr ctask = ownedPtr( new rwlibs::task::CartesianTask() );

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            //std::cout << p->first << "\n";
            if ( isName(p->first, "gripper") ) {
                string gripperType = p->second.get_child("<xmlattr>").get<std::string>("type");
                Q params = readQ(p->second.get_child("params"));
                ctask->getPropertyMap().set<std::string>("GripperName", gripperType);

            } else if(isName(p->first, "pose") ){
                // position
                PTree& pos_tree = p->second.get_child("position");
                std::string posdomain = pos_tree.get_child("<xmlattr>").get<std::string>("domain","R3");
                Vector3D<> pos = readVector3D( pos_tree.get_child("euclidean") );
                // orientation
                //std::cout << "---------------------------" << std::endl;
                PTree& rot_tree = p->second.get_child("orientation");
                std::string rotdomain = pos_tree.get_child("<xmlattr>").get<std::string>("domain","SO3");
                Rotation3D<> rot;
                for (CI p1 = rot_tree.begin(); p1 != rot_tree.end(); ++p1) {
                    if (isName(p1->first,"quaternion")) {
                        std::vector<double> vals = readArray(p1->second);
                        if(vals.size()!=4)
                            RW_THROW("rotmatrix is wrongly dimensioned!");
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
                ctask->addTargetByValue(Transform3D<>(pos/1000.0,rot));
            } else if(isName(p->first, "outcome")){
                // TODO: convert from UIBK to RW format
            } else if(isName(p->first, "density")){
            } else if(isName(p->first, "<xmlattr>")){

            } else {
                RW_THROW("Unknown element!" << p->first);
            }

        }
        return ctask;
    }

    rwlibs::task::CartesianTask::Ptr readGrasps(PTree& data, ParserState& state){
        rwlibs::task::CartesianTask::Ptr grasptasks = ownedPtr( new rwlibs::task::CartesianTask() );

        for (CI p = data.begin(); p != data.end(); ++p) {
            if (isName(p->first, "grasp")) {
                rwlibs::task::CartesianTask::Ptr gtask = readGrasp(p->second, state);
                grasptasks->addTask(gtask);
            } else if(p->first=="<xmlattr>") {
            } else {
                RW_THROW("Unknown element!" << p->first);
            }
        }

        // get tasks from state

        return grasptasks;
    }

}



rwlibs::task::CartesianTask::Ptr  readGraspTask(const std::string& filename){
    std::string file = IOUtil::getAbsoluteFileName(filename);

    try {
        ParserState state(file);

        //state.dir = StringUtil::getDirectoryName(file);
        rwlibs::task::CartesianTask::Ptr grasptask;

        PTree tree;
        read_xml(file, tree);

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            //std::cout << p->first << "\n";
            if ( isName(p->first, "grasps") ) {
                grasptask = readGrasps(p->second, state);
            }
        }


        //rw::loaders::XML::printTree(tree, std::cout);


        return grasptask;

    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}


void writeHeader(std::ostream& out){
    out << "<?xml version=\"1.0\"?> \n"
        << "<grasps xmlns=\"http://iis.uibk.ac.at/ns/grasping\">\n";

}

void writeGripper(std::ostream& out, std::string name, Q params ){
    out << "<gripper type=\""<< name << "\">\n"
           " <params>";
    for(size_t i=0;i<params.size();i++) out << params[i] << " ";
    out << " </params>\n"
        << "</gripper>\n";
}

void writePose(std::ostream& out, const Transform3D<>& pose){
    Quaternion<> quat(pose.R());
    out << "<pose>\n"
            << " <position domain=\"R3\">\n"
            << "  <euclidean>" << pose.P()[0] << " " << pose.P()[1] << " " << pose.P()[2] << "</euclidean>\n"
            << " </position>\n"
            << " <orientation domain=\"SO3\">\n"
            << "  <quaternion format=\"wxyz\">" << quat(0) << " " << quat(1) << " "<< quat(2) << "</quaternion>\n"
            << "  <rotmatrix>"
            << pose.R()(0,0) << " " << pose.R()(1,0) << " " << pose.R()(2,0) << "   "
            << pose.R()(0,1) << " " << pose.R()(1,1) << " " << pose.R()(2,1) << "   "
            << pose.R()(0,2) << " " << pose.R()(1,2) << " " << pose.R()(2,2) << "   "
            << "  </rotmatrix>\n"
            << " </orientation>\n"
            << "</pose>\n";
}

void writeRW(const std::string& taskFile, const std::string& outfile){
    rwlibs::task::CartesianTask::Ptr task = readGraspTask(taskFile);
    XMLTaskSaver saver;
    saver.save(task, outfile);
}

void writeUIBK(const std::string& taskFile, const std::string& outfile){
    XMLTaskLoader loader;
    loader.load( taskFile );

    rwlibs::task::CartesianTask::Ptr task = loader.getCartesianTask();
    Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
    Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    Vector3D<> approach = task->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
    Q openQ = task->getPropertyMap().get<Q>("OpenQ");
    Q closeQ = task->getPropertyMap().get<Q>("CloseQ");

    std::ofstream fstr(outfile.c_str());
    writeHeader(fstr);

    BOOST_FOREACH( rwlibs::task::CartesianTarget::Ptr target, task->getTargets() ){
        Transform3D<> trans = wTe_n * target->get();
        fstr << "<grasp>\n";
        writeGripper(fstr, "Schunk Hand", Q::zero(2));
        writePose(fstr, trans);
        //writeOutcome(fstr, );
        fstr << "</grasp>\n";
    }

}


void writeLua(const std::string& tasksDir, const std::string& outfile){
    std::ofstream fstr(outfile.c_str());
    fstr << "-- set name of movable base frame \n"
         << "base = findFrame(\"Base\") \n"
         << "-- set name of tcp frame \n"
         << "tcp = findFrame(\"TCP\") \n"
         << "-- set the name of the sub directory where images should be put (the directory must exist)\n"
         << "dir = \"./\"\n"
         << "---------------------------------\n\n\n"
         << "state = rwstudio:getState()\n"
         << "pTw = rw.inverse( rw.worldTframe(base:getParent(state),state) )\n"
         << "tcpTbase = rw.frameTframe(tcp, base, state)\n\n\n"

         ;



    std::vector<std::string> files = IOUtil::getFilesInFolder(tasksDir, false, true, "*.xml" );
    // for each file we add the target to the lua script

    BOOST_FOREACH(const std::string& taskfile, files){
        int fcount = 0;
        rwlibs::task::CartesianTask::Ptr task = readGraspTask(taskfile);
        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, task->getTasks()){
            BOOST_FOREACH(rwlibs::task::CartesianTarget::Ptr target, subtask->getTargets()){
                Transform3D<> t3d = target->get();

                std::string tfile = taskfile;
                const std::string::size_type pos = taskfile.find_last_of("/\\");
                if (pos != std::string::npos)
                    tfile = taskfile.substr(pos + 1);


                fstr << "wTbase = rw.Transform3D(rw."<< t3d.P() << ",rw." << t3d.R() << ") * tcpTbase\n"
                     << "pTbase = pTw * wTbase\n"
                     << "setTransform(base, pTbase )\n"
                     << "rw.sleep(0.5)\n"
                     << "filename = dir .. \"/" << tfile << "_" << fcount <<  ".png\"\n"
                     << "rwstudio:saveViewGL(filename)\n\n"
                     ;
                fcount++;
            }
        }
    }
}

int main(int argc, char** argv)
{
    string convtype="", taskFile = "", outfile = "";
    if(argc!=4){
        std::cout << "Usage: \n"
                  << "Convert from RW to UIBK format \n"
                  << "    \"ConvertSimTask toUIBK taskFile outfile\" \n"
                  << "Convert from UIBK to RW format \n"
                  << "    \"ConvertSimTask toRW taskFile outfile\" \n"
                  << "Convert from multiple UIBK files to Lua script\n"
                  << "    \"ConvertSimTask toLua tasksDir outfile\n"
                  ;
        return 0;
    }

    convtype = std::string(argv[1]);
    taskFile = std::string(argv[2]);
    outfile = std::string(argv[3]);

    if(convtype=="toRW"){
        writeRW(taskFile, outfile);
    } else if(convtype=="toUIBK"){
        writeUIBK(taskFile,outfile);
    } else if(convtype=="toLua"){
        writeLua(taskFile, outfile);
    } else {
        RW_WARN("Unknown conversion!");
    }
    return (0);
}
