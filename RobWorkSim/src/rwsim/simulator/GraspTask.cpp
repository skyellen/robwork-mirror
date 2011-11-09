#include "GraspTask.hpp"

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


namespace {
    void writeOutcome( std::ostream& out, rwlibs::task::CartesianTarget::Ptr target){
        int status = target->getPropertyMap().get<int> ("TestStatus");
        Q quality = target->getPropertyMap().get<Q>("QualityAfterLifting", Q::zero(1));

        out << "    <outcome>\n";
        // if success we write all informal qualities

        if(status==GraspTask::Success || status==GraspTask::ObjectSlipped ){
            for(size_t i=0;i<quality.size();i++){
                out << "     <informal uri=\"rwgq" << i << "\" quality=\""<< quality[i] << "\" />\n";
            }
            if(quality[0]>0.01){ // we use a weak fixed quality rule here
                out << "     <success quality=\"" << quality[0] <<"\" />\n";
            } else {
                out << "     <failure cause=\"OBJECTSLIPPED\" />\n";
            }
        } else {
            out << "     <failure cause=\"";
            switch(status){
            case(GraspTask::ObjectDropped): out << "OBJECTDROPPED"; break;
            case(GraspTask::UnInitialized): out << "UNINITIALIZED"; break;
            case(GraspTask::CollisionInitially): out << "COLLISIONINITIALLY"; break;
            case(GraspTask::ObjectMissed): out << "OBJECTMISSED"; break;
            case(GraspTask::InvKinFailure): out << "INVKINFAILURE"; break;
            case(GraspTask::TimeOut): out << "TIMEOUT"; break;
            case(GraspTask::SimulationFailure): out << "SIMULATIONFAILURE"; break;
            default:
                RW_THROW("Not supposed to go here!");
                break;
            }
            out << "\" />\n";
        }

        out << "    </outcome>\n";
    }

    void writePose(std::ostream& out, const Transform3D<>& pose){
        Quaternion<> quat(pose.R());
        out << "<pose>\n"
                << " <position domain=\"R3\">\n"
                << "  <euclidean>" << pose.P()[0]*1000.0 << " " << pose.P()[1]*1000.0 << " " << pose.P()[2]*1000.0 << "</euclidean>\n"
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

    void writeUIBK(GraspTask::Ptr gtask, const std::string& outfile){

        rwlibs::task::CartesianTask::Ptr grasptask = gtask->getRootTask();

        std::ofstream fstr(outfile.c_str());
        fstr << "<?xml version=\"1.0\"?> \n"
             << "<experiments xmlns=\"http://iis.uibk.ac.at/ns/grasping\">\n";
        fstr << "<notes> Generated with RobWork, rwsim::simulator::GraspTask </notes>\n";

        BOOST_FOREACH(rwlibs::task::CartesianTask::Ptr task, grasptask->getTasks() ){

            Transform3D<> wTe_n = task->getPropertyMap().get<Transform3D<> >("Nominal", Transform3D<>::identity());
            Transform3D<> wTe_home = task->getPropertyMap().get<Transform3D<> >("Home", Transform3D<>::identity());
            Vector3D<> approach = task->getPropertyMap().get<Vector3D<> >("Approach", Vector3D<>(0,0,0));
            Transform3D<> approachDef = Transform3D<>( approach, Rotation3D<>::identity());
            Q openQ = task->getPropertyMap().get<Q>("OpenQ");
            Q closeQ = task->getPropertyMap().get<Q>("CloseQ");
            std::string objectId = task->getPropertyMap().get<std::string>("Object",std::string("Undefined"));
            std::string gripperId = task->getPropertyMap().get<std::string>("Gripper");
            std::string controllerName = task->getPropertyMap().get<std::string>("ControllerName", "GraspController");
            std::string tcpName = task->getPropertyMap().get<std::string>("TCP");

            fstr << " <experiment>\n";
            fstr << "  <notes></notes>\n";
            fstr << "  <object type=\"" << objectId << "\">\n";
            fstr << "   <notes> </notes>\n";
            fstr << "  </object>\n";
            fstr << "  <gripper type=\""<< gripperId << "\">\n";
            fstr << "   <notes> GraspController:"<<controllerName << " TCP:" << tcpName <<" CloseQ:"<< closeQ << "</notes>\n";
            fstr << "   <params>";
            for(size_t i=0;i<openQ.size();i++)
                fstr << openQ[i] << " ";
            fstr << "   </params>\n"
                 << "  </gripper>\n";
            fstr << "  <grasps>\n";
            fstr << "   <notes>  </notes>\n"; // don't have any notes yet

            // we don't add predictiondef
            BOOST_FOREACH( rwlibs::task::CartesianTarget::Ptr target, task->getTargets() ){

                Transform3D<> trans = wTe_n * target->get();
                fstr << "   <grasp>\n";
                writePose(fstr, trans);
                writeOutcome(fstr, target);
                fstr << "   </grasp>\n";
            }
            fstr << "  </grasps>\n";
            fstr << " </experiment>\n";

        }


        // and end with an experiments tag
        fstr << "</experiments>\n";
    }

}


void GraspTask::saveUIBK(GraspTask::Ptr task, const std::string& name ){

    writeUIBK(task, name);
}

void GraspTask::saveRWTask(GraspTask::Ptr task, const std::string& name ){
    std::ofstream outfile(name.c_str());
    try {
        XMLTaskSaver saver;
        saver.save(task->getRootTask(), outfile );
    } catch (const Exception& exp) {
        RW_THROW("Unable to save task: " << exp.what());
    }
    outfile.close();
}

void GraspTask::saveText(GraspTask::Ptr gtask, const std::string& name ){
   std::ofstream outfile(name.c_str());
   if(!outfile.is_open())
       RW_THROW("Could not open file: " << name);
   int gripperDim = 0;
   std::string sep(";");
  // outfile << "// Description: {target.pos(3), target.rpy(3), TestStatus(1), GripperConfiguration("<<gripperDim<<"), "
  //         "GripperTObject.pos, GripperTObject.rpy, ObjectTtcpBefore.pos, ObjectTtcpBefore.rpy, ObjectTtcpAfter.pos, ObjectTtcpAfter.rpy}\n";
   outfile << "// One grasp per line, Line Description (quat is xyzw encoded): target.pos(3), target.quat(4), TestStatus(1), "
              "GripperTObject.pos(3); GripperTObject.quat(4); "
              "ObjectTtcpBefore.pos(3); ObjectTtcpBefore.quat(4); "
              "ObjectTtcpAfter.pos(3); ObjectTtcpAfter.quat(4); "
              "GripperConfiguration(x)\n";

   outfile << "// TestStatus enum { UnInitialized=0, Success=1, CollisionInitially=2, ObjectMissed=3, ObjectDropped=4, ObjectSlipped=5, TimeOut=6, SimulationFailure=7}\n";

   std::string gripperID = gtask->getGripperID();
   std::string tcpID = gtask->getTCPID();
   std::string graspcontrollerID = gtask->getGraspControllerID();
   CartesianTask::Ptr root = gtask->getRootTask();

   BOOST_FOREACH(CartesianTask::Ptr task, root->getTasks()){
       Q openQ = task->getPropertyMap().get<Q>("OpenQ");
       Q closeQ = task->getPropertyMap().get<Q>("CloseQ");

       std::vector<CartesianTarget::Ptr> targets = task->getTargets();
       //outfile<<"{" << task->getId() << "}\n";
       BOOST_FOREACH(CartesianTarget::Ptr target, targets) {
          Transform3D<> ttrans = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
          if( MetricUtil::dist2(ttrans.P(), Vector3D<>(0,0,0))<0.00001 ){
              ttrans = target->get();
          }

          const Vector3D<>& pos = ttrans.P();
          Quaternion<> quat(ttrans.R());

          int status = target->getPropertyMap().get<int>("TestStatus", GraspTask::UnInitialized);

          outfile<<pos(0)<<sep<<pos(1)<<sep<<pos(2)<<sep<<quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep<<status<<sep;

          Transform3D<> t3d = target->getPropertyMap().get<Transform3D<> >("GripperTObject0", Transform3D<>::identity());
          //RPY<> rpyObj(t3d.R());
          quat = Quaternion<>(t3d.R());
          outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
                  << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

          t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpTarget", Transform3D<>::identity() );
          quat = Quaternion<>(t3d.R());
          outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
                  << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

          t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpApproach", Transform3D<>::identity() );
          quat = Quaternion<>(t3d.R());
          outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
                  << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

          t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpGrasp", Transform3D<>::identity() );
          quat = Quaternion<>(t3d.R());
          outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
                  << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

          t3d = target->getPropertyMap().get<Transform3D<> >("ObjectTtcpLift", Transform3D<>::identity() );
          quat = Quaternion<>(t3d.R());
          outfile << t3d.P()[0] << sep << t3d.P()[1] << sep <<t3d.P()[2] << sep
                  << quat.getQx()<<sep<<quat.getQy()<<sep<<quat.getQz()<<sep<<quat.getQw()<<sep;

          Q distance = target->getPropertyMap().get<Q>("GripperConfigurationPost", Q::zero(gripperDim));
          for(size_t i=0;i<distance.size();i++)
              outfile << distance[i] << sep;

          outfile << "\n";
       }
   }
   outfile.close();
}


////////////////// GRASP TASK LOADING STUFF

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

    rwlibs::task::CartesianTarget::Ptr readGrasp(PTree& tree, ParserState& state){
        rwlibs::task::CartesianTarget::Ptr target = ownedPtr( new rwlibs::task::CartesianTarget(Transform3D<>()) );

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            //std::cout << p->first << std::endl;
            if(isName(p->first, "pose") ){
                // position
                PTree& pos_tree = p->second.get_child("position");
                std::string posdomain = pos_tree.get_child("<xmlattr>").get<std::string>("domain","R3");
                Vector3D<> pos = readVector3D( pos_tree.get_child("euclidean") )/1000.0;

                // orientation
                //std::cout << "---------------------------" << std::endl;
                PTree& rot_tree = p->second.get_child("orientation");
                std::string rotdomain = pos_tree.get_child("<xmlattr>").get<std::string>("domain","SO3");
                Rotation3D<> rot;
                for (CI p1 = rot_tree.begin(); p1 != rot_tree.end(); ++p1) {
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
                target->get() = Transform3D<>(pos,rot);
            } else if(isName(p->first, "outcome")){
                std::vector<double> qualities;
                int status = GraspTask::UnInitialized;
                //string gripperType = p->second.get_child("<xmlattr>").get<std::string>("type");
                if( has_child(p->second, "success") ){
                    status = GraspTask::Success;
                    if(has_child(p->second.get_child("success"),"<xmlattr>")){
                        double squal = p->second.get_child("success").get_child("<xmlattr>").get<double>("quality",0.0);
                        qualities.push_back(squal);
                    } else {
                        qualities.push_back(0.0);
                    }
                } else if(has_child(p->second,"failure") ){
                    status = GraspTask::ObjectDropped;
                    std::string cause = p->second.get_child("failure").get_child("<xmlattr>").get<std::string>("cause");

                    if(cause=="UNINITIALIZED"){
                        status = GraspTask::UnInitialized;
                    } else if(cause=="COLLISIONINITIALLY"){
                        status = GraspTask::CollisionInitially;
                    } else if(cause=="TIMEOUT"){
                        status = GraspTask::TimeOut;
                    } else if(cause=="OBJECTMISSED"){
                        status = GraspTask::ObjectMissed;
                    } else if(cause=="OBJECTDROPPED"){
                        status = GraspTask::ObjectDropped;
                    } else if(cause=="OBJECTSLIPPED"){
                        status = GraspTask::ObjectSlipped;
                    } else if(cause=="SIMULATIONFAILURE"){
                        status = GraspTask::SimulationFailure;
                    } else if(cause=="POSEESTIMATEFAILURE"){
                        status = GraspTask::PoseEstimateFailure;
                    } else if(cause=="INVKINFAILURE"){
                        status = GraspTask::InvKinFailure;
                    }
                } else {

                }
                // todo: get all informal quality measures

                Q qqual(qualities.size(), &qualities[0]);
                target->getPropertyMap().set<Q>("QualityAfterLifting", qqual);
                target->getPropertyMap().set<int>("TestStatus", (int)status);
                // TODO: convert from UIBK to RW format
            }
        }
        return target;
    }

    rwlibs::task::CartesianTask::Ptr readExperiment(PTree& tree, ParserState& state){

        //std::cout << "experiment" << std::endl;
        rwlibs::task::CartesianTask::Ptr ctask = ownedPtr( new rwlibs::task::CartesianTask() );
        std::vector<double> qualities;

        GraspTask::Status status = GraspTask::UnInitialized;
        //for (OCI p = tree.ordered_begin(); p != tree.not_found(); ++p) {
        for (CI p = tree.begin(); p != tree.end(); ++p) {

            //std::cout << p->first << "\n";
            if ( isName(p->first, "gripper") ) {
                string gripperType = p->second.get_child("<xmlattr>").get<std::string>("type");
                Q params = readQ(p->second.get_child("params"));
                ctask->getPropertyMap().set<std::string>("Gripper", gripperType);
                // TODO: get notes
            } else if(isName(p->first, "object") ){
                string objectName = p->second.get_child("<xmlattr>").get<std::string>("type");
                ctask->getPropertyMap().set<std::string>("Object", objectName);
                // TODO: get notes
            } else if(isName(p->first, "predictiondef") ){

            } else if(isName(p->first, "grasps") ){
                for (CI p1 = p->second.begin(); p1 != p->second.end(); ++p1) {
                    if(isName(p1->first, "grasp")){
                        CartesianTarget::Ptr target = readGrasp(p1->second, state);
                        ctask->addTarget(target);
                    } else if(isName(p1->first, "notes") ){
                        // TODO: add notes
                    }
                }
            } else if(isName(p->first, "<xmlattr>")){
                // todo: uri
            } else if(isName(p->first, "notes")){
            } else {
                RW_THROW("Unknown element!" << p->first);
            }
            //std::cout << "read experiment end" << std::endl;
        }

        return ctask;
    }

    rwlibs::task::CartesianTask::Ptr readGrasps(PTree& data, ParserState& state){
        rwlibs::task::CartesianTask::Ptr grasptasks = ownedPtr( new rwlibs::task::CartesianTask() );

        for (CI p = data.begin(); p != data.end(); ++p) {
            if (isName(p->first, "grasp")) {
                rwlibs::task::CartesianTarget::Ptr target = readGrasp(p->second, state);
                grasptasks->addTarget(target);
            } else if(p->first=="<xmlattr>") {
            } else {
                RW_THROW("Unknown element!" << p->first);
            }
        }

        // get tasks from state

        return grasptasks;
    }

    rwlibs::task::CartesianTask::Ptr readExperiments(PTree& data, ParserState& state){
        // this is a container for experiments
        rwlibs::task::CartesianTask::Ptr grasptasks = ownedPtr( new rwlibs::task::CartesianTask() );
        //std::cout << "read experiments" << std::endl;
        for (CI p = data.begin(); p != data.end(); ++p) {
            // each experiment is a GraspTask
            if(isName(p->first, "experiment")){
                grasptasks->addTask( readExperiment(p->second, state) );
            } else if(p->first=="notes") {
                // grasptasks->getPropertyMap().set<std::string>( );
            } else if(p->first=="<xmlattr>") {
                // uri
            }
        }
        //std::cout << "read experiments end" << std::endl;
        return grasptasks;

    }
}


GraspTask::Ptr GraspTask::load(const std::string& filename){


    std::string file = IOUtil::getAbsoluteFileName(filename);
    std::string firstelem = IOUtil::getFirstXMLElement(file);
    //std::cout << "FIRST ELEMENT: " << firstelem << std::endl;

    rwlibs::task::CartesianTask::Ptr grasptask;

    if(firstelem=="CartesianTask"){
        XMLTaskLoader loader;
        loader.load( file );
        grasptask = loader.getCartesianTask();
    } else {

        try {
            ParserState state(file);

            //state.dir = StringUtil::getDirectoryName(file);
            PTree tree;
            read_xml(file, tree);

            for (CI p = tree.begin(); p != tree.end(); ++p) {
                //std::cout << p->first << "\n";
                if ( isName(p->first, "experiments") ) {
                    grasptask = readExperiments(p->second, state);
                }
            }
            //rw::loaders::XML::printTree(tree, std::cout);
        } catch (const ptree_error& e) {
            // Convert from parse errors to RobWork errors.
            RW_THROW(e.what());
        }

    }

    GraspTask::Ptr gtask = ownedPtr( new GraspTask(grasptask) );
    return gtask;
}

void GraspTask::setGripperID(const std::string& id){
    _task->getPropertyMap().set<std::string>("Gripper",id);
}
void GraspTask::setTCPID(const std::string& id){
    _task->getPropertyMap().set<std::string>("TCP",id);
}
void GraspTask::setGraspControllerID(const std::string& id){
    _task->getPropertyMap().set<std::string>("ControllerName",id);
}

std::string GraspTask::getGripperID(){
    return _task->getPropertyMap().get<std::string>("Gripper");
}
std::string GraspTask::getTCPID(){
    return _task->getPropertyMap().get<std::string>("TCP");
}
std::string GraspTask::getGraspControllerID(){
    return _task->getPropertyMap().get<std::string>("ControllerName","GraspController");
}


