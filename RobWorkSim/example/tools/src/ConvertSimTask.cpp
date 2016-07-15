
#include <iostream>
#include <vector>
#include <string>

#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
//#include <rw/loaders/rwxml/XML.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/GraspTask.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
//#include <boost/optional.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rwlibs::task;
using namespace boost::property_tree;

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


    rwlibs::task::CartesianTask::Ptr readGrasp(PTree& tree, ParserState& state){
        rwlibs::task::CartesianTask::Ptr ctask = ownedPtr( new rwlibs::task::CartesianTask() );
        std::vector<double> qualities;
        CartesianTarget::Ptr target;
        int status = GraspResult::UnInitialized;
        //for (OCI p = tree.ordered_begin(); p != tree.not_found(); ++p) {
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

                target = ctask->addTargetByValue(Transform3D<>(pos/1000.0,rot));
                state.targetNr--;

            } else if(isName(p->first, "outcome")){
                //string gripperType = p->second.get_child("<xmlattr>").get<std::string>("type");
                if( has_child(p->second, "success") ){
                    status = GraspResult::Success;
                    if(has_child(p->second.get_child("success"),"<xmlattr>")){
                        double squal = p->second.get_child("success").get_child("<xmlattr>").get<double>("quality",0.0);
                        qualities.push_back(squal);
                    } else {
                        qualities.push_back(0.0);
                    }
                } else if(has_child(p->second,"failure") ){
                    status = GraspResult::ObjectDropped;
                    std::string cause = p->second.get_child("failure").get_child("<xmlattr>").get<std::string>("cause");
                    if(cause=="POSE"){
                        status = GraspResult::PoseEstimateFailure;
                    } else if(cause=="PLAN"){
                        status = GraspResult::InvKinFailure;
                    } else if(cause=="SERVO"){
                        status = GraspResult::CollisionInitially;
                    } else if(cause=="GRASP"){
                        status = GraspResult::ObjectMissed;
                    } else if(cause=="LIFT"){
                        status = GraspResult::ObjectDropped;
                    }
                } else {
                    status = GraspResult::UnInitialized;
                }
                /*
                element success {
                  attribute quality { xsd:float {minInclusive="0" maxInclusive = "1.0"} }?
                } |
                element failure {
                  attribute cause { ("POSE"|"PLAN"|"SERVO"|"GRASP"|"LIFT") }
                  # POSE:  no correct object pose recovered
                  # PLAN:  no collision-free inverse-kinematics solution
                  # SERVO: error (collision, ...) during servoing
                  # GRASP: error (collision, whatever) during grasping
                  # LIFT:  error during lifting (object dropped, ...)
                }
                 */
                // TODO: convert from UIBK to RW format
            } else if(isName(p->first, "density")){
                std::string densityStr = p->second.get_value<string>();
                double qual = toDouble(densityStr).second;
                qualities.push_back(qual);
                qualities.push_back(state.targetNr);
            } else if(isName(p->first, "<xmlattr>")){

            } else {
                RW_THROW("Unknown element!" << p->first);
            }

        }

        Q qqual(qualities.size(), &qualities[0]);
        target->getPropertyMap().set<Q>("QualityAfterLifting", qqual);
        target->getPropertyMap().set<int>("TestStatus", (int)status);

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
    return NULL;
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
            << "  <quaternion format=\"wxyz\">" << quat(3) << " " << quat(0) << " " << quat(1) << " "<< quat(2) << "</quaternion>\n"
            << "  <rotmatrix>"
            << pose.R()(0,0) << " " << pose.R()(0,1) << " " << pose.R()(0,2) << "   "
            << pose.R()(1,0) << " " << pose.R()(1,1) << " " << pose.R()(1,2) << "   "
            << pose.R()(2,0) << " " << pose.R()(2,1) << " " << pose.R()(2,2) << "   "
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
    //Unused: Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
    //Unused: Vector3D<> approach = task->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
    //Unused: Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
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

void mergeRW(const std::string& tasksDir, const std::string& outfile){
    std::vector<std::string> files = IOUtil::getFilesInFolder(tasksDir, false, true, "*.xml" );
    RW_WARN("1");
    if(files.size()==0)
        return;
    RW_WARN("1");
    rwlibs::task::CartesianTask::Ptr mergedtask;
    {
        XMLTaskLoader loader;
        loader.load(files[0]);
        mergedtask = loader.getCartesianTask();
    }
    for(std::size_t i=1;i<files.size();i++){
        RW_WARN("1");
        const std::string& taskfile = files[i];
        RW_WARN("1"<<taskfile);
        //Unused: int fcount = 0;
        rwlibs::task::CartesianTask::Ptr task;
        {
            XMLTaskLoader loader;
            loader.load(taskfile);
            task = loader.getCartesianTask();
        }
        RW_WARN("1");
        //BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr subtask, task->getTasks()){
            RW_WARN("1");
            if(task==NULL)
                continue;
            BOOST_FOREACH(rwlibs::task::CartesianTarget::Ptr target, task->getTargets()){
                RW_WARN("1");
                mergedtask->addTarget(target);
                RW_WARN("1");
            }
        //}
        RW_WARN("1");
    }
    RW_WARN("1");
    XMLTaskSaver saver;
    saver.save(mergedtask, outfile);

}

int main(int argc, char** argv)
{
    string convtype="", taskFile = "", outfile = "";
    if(argc<4){
        std::cout << "Usage: \n"
                  << "Convert from RW to UIBK format \n"
                  << "    \"ConvertSimTask toUIBK taskFile outfile\" \n"
                  << "Convert from UIBK to RW format \n"
                  << "    \"ConvertSimTask toRW taskFile outfile\" \n"
                  << "Convert from multiple UIBK files to Lua script\n"
                  << "    \"ConvertSimTask toLua tasksDir outfile\n"
                  << "Convert multiple rwTasks to one\n"
                  << "    \"ConvertSimTask mergeRW tasksDir outfile\n"
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
    } else if(convtype=="mergeRW"){
        mergeRW(taskFile, outfile);
    } else {
        RW_WARN("Unknown conversion!");
    }
    return (0);
}
