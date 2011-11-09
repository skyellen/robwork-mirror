#include "VisGraBBenchmark.hpp"

#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include "SDHInvKinSolver.hpp"
#include <boost/filesystem.hpp>
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
            dwcfile(file),targetNr(0),nrGrasps(0),solFound(0)
        {
        }

        const std::string dwcfile, dir;
        SDHInvKinSolver _invkin;
        int solFound;
        int nrGrasps;

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


    GraspUnknownObjectStereoBM::Ptr readGraspUnknownObjectStereoBM(PTree& data, ParserState& state){
        GraspUnknownObjectStereoBM::Ptr benchmark = ownedPtr( new GraspUnknownObjectStereoBM() );

        for (CI p = data.begin(); p != data.end(); ++p) {
            if(isName(p->first, "scene")){
                std::string id = p->second.get_child("<xmlattr>").get<std::string>("id");

                //! the info, encoded as SceneID, SceneLocator, SceneSet, SceneSetID, scenetextureinfo, sceneObjects
                std::vector<std::string> vals, objects;
                PTree& info = p->second.get_child("info");
                //vals.push_back( info.get<std::string>("sceneID") );
                vals.push_back( id );
                vals.push_back( info.get<std::string>("sceneLocator") );
                vals.push_back( info.get<std::string>("sceneSet") );
                vals.push_back( info.get<std::string>("sceneSetID") );
                vals.push_back( info.get<std::string>("textureCondition") );
                benchmark->_sidToIndex[vals[0]] = benchmark->_sceneInfos.size();
                benchmark->_sceneInfos.push_back( boost::make_tuple(vals[0], vals[1], vals[2], vals[3], vals[4], objects)  );

                vals.clear();
                PTree& input = p->second.get_child("input");
                vals.push_back( input.get<std::string>("leftImg") );
                vals.push_back( input.get<std::string>("rightImg") );
                vals.push_back( input.get<std::string>("disp") );
                vals.push_back( input.get<std::string>("pointCloud") );
                benchmark->_sceneInput.push_back( boost::make_tuple(vals[0], vals[1], vals[2], vals[3])  );

                vals.clear();
                PTree& sim = p->second.get_child("simulator");
                vals.push_back( sim.get<std::string>("wc") );
                vals.push_back( sim.get<std::string>("dwc") );
                vals.push_back( sim.get<std::string>("collision") );
                benchmark->_sceneSimData.push_back( boost::make_tuple(vals[0], vals[1], vals[2])  );


            }
        }
        return benchmark;
    }
}

VisGraBBenchmark::Ptr VisGraBBenchmark::load(const std::string& filename){
    std::string file = IOUtil::getAbsoluteFileName(filename);
    std::string firstelem = IOUtil::getFirstXMLElement(file);

    VisGraBBenchmark::Ptr benchmark;

    if(!boost::filesystem3::exists( file ))
        RW_THROW("File does not exist!");

    try {
        ParserState state(file);

        PTree tree;
        read_xml(file, tree);

        PTree& visgrab_tree = tree.get_child("visgrab");

        for (CI p = visgrab_tree.begin(); p != visgrab_tree.end(); ++p) {
            if(isName(p->first, "benchmark")){
                std::string type = p->second.get_child("<xmlattr>").get<std::string>("type");
                if(isName(type,"GraspUnknownObject")){

                    GraspUnknownObjectStereoBM::Ptr gbm = readGraspUnknownObjectStereoBM(p->second, state);
                    boost::filesystem3::path p(file);
                    gbm->fullpathBenchmark = p.parent_path().string();
                    return gbm;
                } else {
                    RW_THROW("Unknown benchmark type: \"" << type << "\"");
                }
            }
        }

        //rw::loaders::XML::printTree(tree, std::cout);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }

    return benchmark;
}

