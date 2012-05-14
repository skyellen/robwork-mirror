/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/


#include "XMLRWLoader.hpp"

#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>

#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/JointDevice.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/task/Trajectory.hpp>
#include <rw/task/Task.hpp>

#include <rw/loaders/rwxml/XML.hpp>

#include <direct.h> // for getcwd
#include <iostream> // for cout and cin

typedef boost::property_tree::ptree PTree;


using namespace std;
using namespace rw::loaders;

using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;

using namespace boost::numeric;
using namespace boost::property_tree;

namespace {


    void getCurrentPath(char* buffer, int maxLength){
        getcwd(buffer, maxLength);
    }

    string quote(const string& str) { return StringUtil::quote(str); }
    typedef PTree::const_iterator CI;

    typedef enum{FIXED, MOVABLE,
                 REVOLUTE, PRISMATIC,
                 DEPENDREVOLUTE, DEPENDPRISMATIC,
                } DFrameType;

    typedef pair<double,double> PosLimit;
    typedef vector<string> Scope;

    struct DummyFrame {
        string name, refname;
        DFrameType type;

        PosLimit posLimit;
        double velLimit;
        double accLimit;
        rw::math::Transform3D<> transform;

        PropertyMap properties;
        bool isEnd, isDaf;

        // For dependent joints
        bool _isDepend;
        double _gain, _offset;
        std::string _dependsOn;
    };

    typedef vector<DummyFrame> DFrameList;
    typedef vector<DummyFrame*> DFramePtrList;

    struct ParserState {
        string wcfile,workDir;

        StateStructure *sstruct;
        DFrameList dframes;
        map<string, DFramePtrList> ddevToFrames;
        map<string, string> frameToDdev;
        map<string, DummyFrame*> nameToFrame;
    };

    bool isFrameNode(const std::string& name){
        if (    name == "Frame" ||
                name == "Joint" ||
                name == "DHJoint"){
            return true;
        }else {
            return false;
        }
    }

    bool isGroupNode(const std::string& name){
        if (    name == "Device" ||
                name == "SerialDevice" ||
                name == "ParallelDevice" ||
                name == "TreeDevice" ||
                name == "MobileDevice" ||
                name == "ConveorDevice"){
            return true;
        }else {
            return false;
        }
    }



    boost::optional<double> toDouble(const std::string& str)
    {
        istringstream buf(str);
        double x;
        buf >> x;
        if (!buf) return boost::optional<double>();

        string rest;
        buf >> rest;
        if (buf) return boost::optional<double>();
        else return boost::optional<double>(x);
    }

    std::vector<double> readArray(const PTree& tree){
        istringstream buf(tree.get_own<string>());
        std::vector<double> values;

        std::string str;
        while (buf >> str) {
            boost::optional<double> okNum = toDouble(str);
            if (!okNum)
                RW_THROW("Number expected. Got " << quote(str));
            values.push_back(*okNum);
        }
        return values;
    }

    Q readQ(const PTree& tree){
        std::vector<double> arr = readArray(tree);
        Q q(arr.size());
        for(int i=0;i<q.size();i++){
            q[i] = arr[i];
        }
        return q;
    }

    ublas::matrix<float> readMatrix(const PTree& tree, std::pair<int,int> dim){
        Q q = readQ(tree);
        if(q.size()!=dim.first*dim.second)
            RW_THROW("Unexpected sequence of values, must be length " << dim.first*dim.second);

        ublas::matrix<double> values(dim.first, dim.second);
        for(int y=0;y<dim.second;y++)
            for(int x=0;x<dim.first;x++)
                values(x,y) = (float)q(y*dim.first + x);

        return values;
    }

    Vector3D<> readVector3D(const PTree& tree){
        std::cout << "ReadVector3D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=3)
            RW_THROW("Unexpected sequence of values, must be length 3");
        return Vector3D<>(q[0],q[1],q[2]);
    }

    RPY<> readRPY(const PTree& tree){
        Q q = readQ(tree);
        if(q.size()!=3)
            RW_THROW("Unexpected sequence of values in RPY, must be length 3");
        return RPY<>(q[0],q[1],q[2]);
    }

    Transform3D<> readDH(const PTree& tree){
        Transform3D<> t3d;
        string type = tree.get_child("<xmlattr>").get("type","Craigh");
        if(type == "Craigh"){

        } else if (type == "Schilling"){

        } else {
            RW_THROW("Unsupported DH parameter type: " << StringUtil::quote(type));
        }
        return t3d;
    }

    // read a transform from a node. can be RPY/Pos, 4*3 Matrix, DH
    Transform3D<> readTransform(const PTree& tree){
        Transform3D<> t3d = Transform3D<>::identity();
        boost::optional<const PTree&> rpyDef = tree.get_child_optional("RPY");
        if( rpyDef ){
            t3d.R() = readRPY(*rpyDef).toRotation3D();
        }
        boost::optional<const PTree&> posDef = tree.get_child_optional("Pos");
        if( posDef ){
            t3d.P() = readVector3D(*posDef);
        }
        boost::optional<const PTree&> matDef = tree.get_child_optional("Matrix");
        if( matDef ){
            ublas::matrix<double> m = readMatrix(*matDef, make_pair(3,4));
            t3d.R() = Rotation3D<>(m(0,0),m(0,1),m(0,2),
                                   m(1,0),m(1,1),m(1,2),
                                   m(2,0),m(2,1),m(2,2));
            t3d.P() = Vector3D<>(m(0,3),m(1,3),m(2,3));
        }
        boost::optional<const PTree&> dhDef = tree.get_child_optional("DH");
        if( dhDef ){
            t3d = readDH(*dhDef);
        }

        return t3d;
    }

    // properties in the scope of a framenode
    void readFrameProperties(const PTree& tree, ParserState& pstate){

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Property") {

            }
        }
    }

    // properties in the scope of a groupnode
    void readProperties(const PTree& tree, ParserState& pstate){

        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Property") {

            }
        }
    }

    void readModels(const PTree& tree, ParserState& pstate){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Frame") {
            } else if (p->first == "ParallelDevice") {
            } else if (p->first == "TreeDevice") {
            } else if (p->first == "MobileDevice") {
            } else if (p->first == "ConveorDevice") {

            }
        }
    }

    void readFrame(const PTree& tree, const string& elemname, ParserState& pstate){
        const PTree& att = tree.get_child("<xmlattr>");
        DummyFrame dframe;
        string name = att.get<string>("name");
        string refframename = att.get<string>("refframe","");
        if(refframename==""){
            // get the name of the last frame added to the state
            refframename = pstate.dframes.back().name;
        }
        string type = att.get<string>("type","Fixed");

        if(elemname=="DAF"){
            dframe.isDaf = true;
        } else if( elemname=="EndEffector" ){
            dframe.isEnd = true;
        }

        dframe.transform = readTransform(tree);

    }

    void readFrames(const PTree& tree, ParserState& pstate){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if ( isFrameNode(p->first) ) {
                readFrame(p->second, p->first, pstate);
            } else if ( isGroupNode(p->first) ) {
                readFrames(p->second, pstate);
            }
        }
    }

    void readDevices(const PTree& tree, ParserState& pstate){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "SerialDevice") {
            } else if (p->first == "ParallelDevice") {
            } else if (p->first == "TreeDevice") {
            } else if (p->first == "MobileDevice") {
            } else if (p->first == "ConveorDevice") {

            }
        }
    }

    void readSensor(const PTree& tree, ParserState& pstate){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Camera") {
            } else if (p->first == "Scanner1D") {
            } else if (p->first == "Scanner2D") {
            } else if (p->first == "Scanner3D") {
            }
        }
    }

    WorkCell* readWorkCell(PTree& tree, ParserState& pstate)
    {
        // get name of WorkCell
        string name = tree.get_child("<xmlattr>").get<string>("name");

        pstate.sstruct = new StateStructure();
        DummyFrame dframe;
        dframe.name = "WORLD";
        pstate.dframes.push_back(dframe);
        // first handle all includes and defines
        //readInclude(tree, pstate);
        // secondly read all frames into dummy structure
        readFrames(tree, pstate);
        // next create all frames
        createFrames(pstate);
        // now read and create devices
        //readDevices(tree, pstate);
        // and models
        //readModels(tree, pstate);
        // and sensors
        //readSensors(tree, pstate);


/*
        std::cout << "load all elements" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            std::cout << p->first << std::endl;
            if (p->first == "Frame") {
            } else if (p->first == "CollisionModel") {
            } else if (p->first == "DrawableModel") {
            } else if (p->first == "Camera") {
            } else if (p->first == "KinematicDevice") {
            } else if (p->first == "RigidDevice") {
            } else if (p->first == "ContactMap") {
            } else if (p->first == "ContactModel") {
            } else if (p->first == "TactileArraySensor") {
            } else if (p->first == "<xmlattr>") {
            } else if (p->first == "<xmlcomment>") {
            } else {
                RW_THROW(
                    "Unsupported element "
                    << quote(p->first) );
            }
        }
        //  create the dynamic workcell
*/

        return NULL;
    }
}

std::auto_ptr<WorkCell> XMLRWLoader::load(const string& file)
{
    char dirbuff[1024];
    getCurrentPath(dirbuff, 1024);
    string workDir = string(dirbuff);

    WorkCell *wc;
    try {
        ParserState pstate;
        pstate.workDir = workDir;
        // handle if file is not absolute path
        if(StringUtil::isAbsoluteFileName(file)){
            pstate.wcfile = file;
        } else {
            // get current directory
            pstate.wcfile = pstate.workDir + file;
        }

        PTree tree;
        read_xml(pstate.wcfile, tree);

        // XML::printTree(tree);
        wc = readWorkCell(tree.get_child("WorkCell"), pstate);

    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }

    return std::auto_ptr<WorkCell>(wc);
}
