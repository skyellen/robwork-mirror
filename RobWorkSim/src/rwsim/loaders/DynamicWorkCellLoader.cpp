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


#include "DynamicWorkCellLoader.hpp"

#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/loaders/rwxml/DependencyGraph.hpp>
#include <rw/models/JointDevice.hpp>

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/Log.hpp>
#include <rw/common/Ptr.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/Constants.hpp>

#include <rwsim/dynamics/Body.hpp>
#include <rwsim/dynamics/Constraint.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidJoint.hpp>
#include <rwsim/dynamics/ControllerModel.hpp>
#include <rwsim/dynamics/SensorModel.hpp>

#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>

#include <rwsim/dynamics/MaterialDataMap.hpp>
#include <rwsim/dynamics/ContactDataMap.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>

#include <rwsim/sensor/TactileArraySensor.hpp>
#include <rwsim/sensor/BodyContactSensor.hpp>
#include <rwsim/sensor/SimulatedFTSensor.hpp>

#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>

#include <rw/geometry/GeometryUtil.hpp>
#include <rw/common/PropertyMap.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/models/RigidObject.hpp>

#include <rwlibs/simulation/SimulatedController.hpp>
#include <rwlibs/simulation/SimulatedSensor.hpp>
#include <rwlibs/control/JointController.hpp>

#include <rw/geometry/Geometry.hpp>
#include <rw/loaders/GeometryFactory.hpp>
#include <rw/geometry/IndexedTriMesh.hpp>
#include <rw/geometry/TriangleUtil.hpp>

#include <rwsim/control/PDController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwsim/control/VelRampController.hpp>
//#include <rwsim/control/TrajectoryController.hpp>
#include <rwsim/control/SuctionCupController.hpp>
#include <rwsim/control/PoseController.hpp>

typedef boost::property_tree::ptree PTree;

using namespace std;
using namespace rwsim::loaders;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::control;

using namespace rw::loaders;

using namespace rw::math;
using namespace rw::common;
using namespace rw::sensor;
using namespace rw::geometry;
using namespace rwlibs::control;

using namespace rw::models;
using namespace rw::kinematics;

using namespace rwlibs::simulation;

using namespace boost::property_tree;

//#define RW_DEBUGS(str) std::cout << str << std::endl
#define RW_DEBUGS(str)

namespace
{
    struct FrictionDataTmp {
        FrictionData data;
        std::string matA,matB;
    };

    struct ContactDataTmp {
        ContactDataMap::NewtonData data;
        std::string objA,objB;
    };

    typedef PTree::iterator CI;

    struct ParserState {
    public:
        ParserState(std::string file):
            dwcfile(file),
            wc(NULL)
        {
        }
        const std::string dwcfile;
        rw::kinematics::State rwstate;
        rw::models::WorkCell::Ptr wc;
        rwsim::dynamics::DynamicWorkCell::Ptr dwc;

        double defaultcolmargin;
        string defaultRestModel;
        string defaultContactModel;
        bool autoDisable;
        MaterialDataMap materialData;
        std::string defaultMaterial;
        ContactDataMap contactData;
        std::string defaultObjectType;

        PropertyMap engineProps;

        //std::vector<Body::Ptr> bodies;
        //std::vector<Body::Ptr> allbodies;
        //std::vector<DynamicDevice::Ptr> devices;
        //std::vector<SimulatedController::Ptr> controllers;
        //std::vector<SimulatedSensor::Ptr> sensors;
        Vector3D<> gravity;

        std::vector<FrictionDataTmp> fdatas;

        std::vector<ContactDataTmp> cdatas;

        std::vector<Frame*> deviceBases;
        std::string dir;

        DependencyGraph graph;
       /* PTree& top();
        void push(PTree* tree);
        void pop();
        bool hasNext();
        CI next();
        */
    };

    string quote(const string& str) { return StringUtil::quote(str); }


/*
    std::vector<Geometry::Ptr> loadGeometrySingle(Frame &f, const State& rwstate){
        std::vector<Geometry::Ptr> geoms;
        Frame *frame = &f;
        // std::vector<Face<float> > faces;
        Log::debugLog()<< "- for all nodes: " << std::endl;
        if( frame==NULL )
            return geoms;
        // check if frame has collision descriptor
        if( CollisionModelInfo::get(frame).size()==0 )
            return geoms;
        // get the geo descriptor
        std::string geofile = CollisionModelInfo::get(frame)[0].getGeoString();
        std::string geoname = CollisionModelInfo::get(frame)[0].getName();
        Transform3D<> fTgeo = CollisionModelInfo::get(frame)[0].getTransform();

        Geometry::Ptr geo = GeometryFactory::getGeometry(geofile);
        geo->setTransform(fTgeo);
        geo->setName( geoname );
        geoms.push_back(geo);
        return geoms;
    }
*/
    /*
    std::vector<Geometry::Ptr> loadGeometry(Frame *bodyFrame, std::vector<Frame*> frames, const State& rwstate){
        std::vector<Geometry::Ptr> geoms;
        BOOST_FOREACH(const Frame *frame, frames){
            if( frame==NULL )
                continue;
            // check if frame has collision descriptor
            if( CollisionModelInfo::get(frame).size()==0 )
                continue;
            Transform3D<> pTf = Kinematics::frameTframe(bodyFrame, frame, rwstate);
            // get the geo descriptor
            std::string geofile = CollisionModelInfo::get(frame)[0].getGeoString();
            std::string geoname = CollisionModelInfo::get(frame)[0].getName();
            Transform3D<> geomt3d = CollisionModelInfo::get(frame)[0].getTransform();
            Transform3D<> fTgeo = pTf * CollisionModelInfo::get(frame)[0].getTransform();
            Geometry::Ptr geo = GeometryFactory::getGeometry(geofile);
            geo->setTransform(fTgeo);
            geo->setName( geoname );
            geoms.push_back(geo);
        }
        return geoms;
    }
*/
//    std::vector<Geometry*> loadGeometry(Frame &parent, const State& rwstate){
//        std::vector<Frame*> frames = DynamicUtil::getAnchoredFrames( parent, rwstate);
//        return loadGeometry(&parent, frames,rwstate);
//        //std::vector<Face<float> > faces;
//    }

//    std::vector<Geometry*> loadGeometryChildren(Frame &parent, const State& rwstate){
//        std::vector<Frame*> frames = DynamicUtil::getAnchoredChildFrames( &parent, rwstate);
//        std::cout << "Frames : " << frames.size() << std::endl;
//        return loadGeometry(&parent, frames, rwstate);
        //std::vector<Face<float> > faces;
//    }

    DynamicDevice::Ptr findDynamicDevice(ParserState& state, Device* dev){
    	return state.dwc->findDevice(dev->getName());
        /*
    	BOOST_FOREACH(DynamicDevice::Ptr ddev, state.devices){
            if(ddev->getKinematicModel() == dev){
                return ddev;
            }
        }
        RW_THROW("No such dynamic device!");
        return NULL;
        */
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
/*
    template<class ARR>
    ARR readFixedArray(PTree& tree, ARR &values, int size){
        istringstream buf(tree.get_own<string>());

        std::string str;
        for(int i=0; i<size; i++){
            if( buf >> str ){
                const pair<bool, double> okNum = toDouble(str);
                if (!okNum.first)
                    RW_THROW("Number expected. Got " << quote(str));
                values[i] = okNum.second;
            } else {
                return ARR;
            }
        }
    }
*/
    std::vector<double> readArray(PTree& tree){
        RW_DEBUGS( "ReadArray: " << tree.get_value<string>() );
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

    rw::math::Q getValueAsQ(const std::string& strlist ){
        std::istringstream buf(strlist);
        std::vector<double> values;
        std::string str;
        while( buf >> str ){
        	double val = boost::lexical_cast<double>(str);
            values.push_back(val);
        }

        return rw::math::Q(values.size(), &values[0]);
    }


    Q readQ(PTree& tree){
        Log::debugLog()<< "ReadQ" << std::endl;
        std::vector<double> arr = readArray(tree);
        Q q(arr.size());
        for(size_t i=0;i<q.size();i++){
            q[i] = arr[i];
        }
        return q;
    }


    bool readBool(PTree& tree){
        Log::debugLog()<< "ReadBool" << std::endl;
        string str= tree.get_value<string>();
        string strup = StringUtil::toUpper(str);
        if(strup=="TRUE" || strup=="1" || strup=="ON"){
            return true;
        } else if(strup=="FALSE" || strup=="0" || strup=="OFF"){
            return false;
        }
        RW_THROW("The input \"" << str << "\" is not a valid boolean!");
        return false;
    }

    Vector3D<> readVector3D(PTree& tree){
        Log::debugLog()<< "ReadVector3D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=3)
            RW_THROW("Unexpected sequence of values, must be length 3");
        return Vector3D<>(q[0],q[1],q[2]);
    }


    Vector2D<> readVector2D(PTree& tree){
        Log::debugLog()<< "ReadVector2D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=2)
            RW_THROW("Unexpected sequence of values, must be length 2");
        return Vector2D<>(q[0],q[1]);
    }

    Eigen::MatrixXf readMatrix(PTree& tree, std::pair<int,int> dim){
        Log::debugLog()<< "ReadVector2D" << std::endl;
        Q q = readQ(tree);
        if(q.size()!=(size_t)(dim.first*dim.second))
            RW_THROW("Unexpected sequence of values, must be length " << dim.first*dim.second);

        Eigen::MatrixXf values(dim.first, dim.second);
        for(int y=0;y<dim.second;y++)
            for(int x=0;x<dim.first;x++)
                values(x,y) = (float)q(y*dim.first + x);

        return values;
    }

    Eigen::MatrixXd readMatrixD(PTree& tree, std::pair<int,int> dim){
        Log::debugLog()<< "ReadVector2D" << std::endl;
        Q q = readQ(tree);
        if(q.size() != (size_t)(dim.first*dim.second))
            RW_THROW("Unexpected sequence of values, must be length " << dim.first*dim.second);

        Eigen::MatrixXd values(dim.first, dim.second);
        for(int y=0; y < dim.second; y++) {
            for(int x=0; x < dim.first; x++) {
                values(x,y) = (double) q(y*dim.first + x);
            }
        }

        return values;
    }

    InertiaMatrix<> readInertia(PTree& tree){
        Q q = readQ(tree);
        if(q.size()==3){
            return InertiaMatrix<>(q[0],q[1],q[2]);
        } else if( q.size()==9){
            return InertiaMatrix<>(q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8]);
        }
        RW_THROW("Inertia needs either 3 or 9 arguments, it got " << q.size() );
    }


    Transform3D<> readTransform(PTree& tree){
        Transform3D<> t3d;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "RPY" ){
                Vector3D<> v=readVector3D(p->second);
                t3d.R() = RPY<>(v[0],v[1],v[2]).toRotation3D();
            } else if( p->first == "Pos" ){
                Vector3D<> v=readVector3D(p->second);
                t3d.P() = v;
            } else if( p->first == "Rotation3D" ){
                Q q= readQ(p->second);
                if( q.size()!=9)
                    RW_THROW("Rotation3D must have 9 elements!");
                t3d.R() = Rotation3D<>(q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8]);
            }
        }
        return t3d;
    }


    void readProperties(PTree& tree, PropertyMap& map){
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if(p->first == "Property") {
                std::string name = p->second.get_child("<xmlattr>").get<std::string>("name");
                //std::string desc = p->second.get_child_optional("Description").get<std::string>("desc","");
                std::string type = p->second.get_child("<xmlattr>").get<std::string>("type","string");
                if( type=="string" ){
                    std::string value = p->second.get_value<std::string>();
                    map.add<std::string>(name, "", value);
                } else if( type=="int" ){
                    int value = p->second.get_value<int>();
                    map.add<int>(name, "", value);
                } else if( type=="float" ){
                    double value = p->second.get_value<double>();
                    map.add<double>(name, "", value);
        		} else if( type=="Q" ){
        			rw::math::Q value = getValueAsQ(p->second.get_value<std::string>());
        			map.add<rw::math::Q>(name, "", value);
                } else {
                    RW_THROW("DynamicWorkCellLoader: Unknown engine property type: " << StringUtil::quote(type) );
                }

            }
        }
    }

    Body::Ptr getBodyFromAttr(PTree& tree, ParserState &state, const std::string& attr){
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Body::Ptr b = state.dwc->findBody( refframeName );
        if(b==NULL)
            RW_THROW("Body " << quote(refframeName) << " does not exist in workcell!");
        return b;
    }

    Body::Ptr getBodyFromAttr(PTree& tree, ParserState &state, const std::string& attr, const std::string& prefix){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = prefix+tree.get_child("<xmlattr>").get<std::string>(attr);
        Body::Ptr b = state.dwc->findBody( refframeName );
        if(b==NULL)
            RW_THROW("Body " << quote(refframeName) << " does not exist in workcell!");
        return b;
    }


    Object::Ptr getObjectFromAttr(PTree& tree, ParserState &state, const std::string& attr){
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Object::Ptr obj = state.wc->findObject(refframeName);
        if( !obj )
            RW_THROW("Object " << quote(refframeName) << " does not exist in workcell!");
        return obj;
    }

    Object::Ptr getObjectFromAttr(PTree& tree, ParserState &state, const std::string& attr, const std::string& prefix){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Object::Ptr obj = state.wc->findObject(prefix+refframeName);
        if( !obj )
            RW_THROW("Object " << quote(refframeName) << " does not exist in workcell!");
        return obj;
    }

    Frame *getFrameFromAttr(PTree& tree, ParserState &state, const std::string& attr){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Frame* frame = state.wc->findFrame(refframeName);
        if( !frame )
            RW_THROW("Frame " << quote(refframeName) << " does not exist in workcell!");
        return frame;
    }

    Frame *getFrameFromAttr(PTree& tree, ParserState &state, const std::string& attr, const std::string& prefix){
        Log::debugLog()<< "getFrameFromAttr" << std::endl;
        string refframeName = tree.get_child("<xmlattr>").get<std::string>(attr);
        Frame* frame = state.wc->findFrame(prefix+refframeName);
        if( !frame )
            RW_THROW("Frame " << quote(refframeName) << " does not exist in workcell!");
        return frame;

    }
/*
    CompositeRigidBody* readCompositeRigidBody(PTree& tree, const std::string& prefix, ParserState &state){


    }
  */
    RigidBody::Ptr readRigidBody(PTree& tree, const std::string& prefix, ParserState &state){
        Log::debugLog()<< "ReadRigidBody" << std::endl;

        string refframeName = tree.get_child("<xmlattr>").get<std::string>("frame");
        RigidObject::Ptr obj = state.wc->findObject(prefix+refframeName).cast<RigidObject>();
        if( !obj ){
            // create one without any geometry
            MovableFrame* baseframe = state.wc->findFrame<MovableFrame>(prefix+refframeName);
            if(baseframe==NULL)
                RW_THROW("no frame with name " << prefix+refframeName);
            obj = ownedPtr(new RigidObject(baseframe));
            state.wc->add( obj );
        }

        //Object::Ptr obj = getObjectFromAttr(tree, state, "frame", prefix);
        // check if frame is actually a moveable frame
        MovableFrame *mframe = dynamic_cast<MovableFrame*>( obj->getBase() );
        if( !mframe )
            RW_THROW("Object "<< quote(obj->getName())<< " is not a movable frame!");
        BodyInfo info;

        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", state.defaultMaterial);
        info.integratorType = tree.get<string>("Integrator");
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );
        info.objects.push_back(obj);
        // check if the body has multiple objects associated
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "Associate" ){
                std::string objname = p->second.get_child("<xmlattr>").get<std::string>("object");
                Object::Ptr assobj = state.wc->findObject(prefix+objname);
                if(assobj==NULL)
                    assobj = state.wc->findObject(objname);
                if(assobj==NULL)
                    RW_THROW("Associated Object does not exist: \"" << objname << "\"");
                info.objects.push_back(assobj);
            }
        }

        std::vector<Geometry::Ptr> geoms;
        BOOST_FOREACH(Object::Ptr assobj, info.objects){
            BOOST_FOREACH(Geometry::Ptr g, assobj->getGeometry(state.rwstate)){
                geoms.push_back(g);
            }
        }


        // time to load the geometry
        Log::debugLog()<< "load geom" << std::endl;
        //info.frames = GeometryUtil::getAnchoredFrames( *mframe, state.rwstate);
        //std::vector<Geometry::Ptr> geometry = loadGeometry(mframe, info.frames, state.rwstate);

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
        	if(obj->getGeometry().size()!=0){
        	    if( tree.get_optional<string>("COG") ){
        	        // if COG specified then use it and calculate inertia

        	        info.masscenter = readVector3D( tree.get_child("COG") );
        	        Transform3D<> ref(info.masscenter);
        	        info.inertia = GeometryUtil::estimateInertia(info.mass, geoms, mframe,state.rwstate, ref);
        	    } else {
        	        boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, obj->getGeometry(), mframe,state.rwstate);
        	    }
        	} else {
        		RW_WARN("No geomtry present to generate Inertia from. Default masscenter and inertia is used.");
        		info.masscenter = Vector3D<>(0,0,0);
        		info.inertia = InertiaMatrix<>::makeSolidSphereInertia(info.mass, 0.0001);
        	}
        }


        readProperties(tree, mframe->getPropertyMap());

        //info.print();
        Log::debugLog()<< "Creating rigid body" << std::endl;
        RigidBody::Ptr body = ownedPtr( new RigidBody(info, obj) );
        state.dwc->addBody( body );
        return body;
    }

    RigidBody::Ptr readRigidBody(PTree& tree, ParserState &state){
    	return readRigidBody(tree,"",state);
    }

    FixedBody::Ptr readFixedBody(PTree& tree, ParserState &state){

        Object::Ptr bodyObj = getObjectFromAttr(tree, state, "frame");

        BodyInfo info;
        info.material = tree.get<string>("MaterialID", state.defaultMaterial );
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );

        //info.frames = std::vector<Frame*>(1, bodyObj->getName());
        //std::vector<Geometry::Ptr> geoms = loadGeometrySingle(*refframe, state.rwstate);
        FixedBody::Ptr body = ownedPtr( new FixedBody(info, bodyObj) );

        readProperties(tree, bodyObj->getBase()->getPropertyMap());

        state.dwc->addBody( body );
        return body;
    }

    FixedBody::Ptr readFixedBase(PTree& tree, ParserState &state, JointDevice *dev){
        // Log::infoLog() << "ReadFixedBase" << std::endl;
        Object::Ptr obj = getObjectFromAttr(tree, state, "frame", dev->getName()+".");

        BodyInfo info;
        info.material = tree.get<string>("MaterialID", state.defaultMaterial );
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );

        //info.frames = DynamicUtil::getAnchoredChildFrames( refframe, state.rwstate, state.deviceBases);
        //std::vector<Geometry::Ptr> geoms = loadGeometry(refframe, info.frames, state.rwstate);
        FixedBody::Ptr body = ownedPtr( new FixedBody(info, obj) );
        //Log::infoLog() << "ReadFixedBody end" << std::endl;
        state.dwc->addBody( body );
        return body;
    }


    JointDevice::Ptr getJointDeviceFromAttr(PTree& tree, ParserState &state){
        Log::debugLog()<< "Device from attr" << std::endl;
        string deviceName = tree.get_child("<xmlattr>").get<std::string>("device");
        Device::Ptr device = state.wc->findDevice(deviceName).get();
        if( device==NULL )
            RW_THROW("Device " << quote(deviceName) << " does not exist in workcell!");
        JointDevice::Ptr jdev = device.cast<JointDevice>();
        if(jdev==NULL)
            RW_THROW("Device " << quote(deviceName) << " is not a JointDevice!");
        return jdev;
    }

    Device::Ptr getDeviceFromAttr(PTree& tree, ParserState &state){
        Log::debugLog()<< "Device from attr" << std::endl;
        string deviceName = tree.get_child("<xmlattr>").get<std::string>("device");
        Device::Ptr device = state.wc->findDevice(deviceName);
        if( device==NULL )
            RW_THROW("Device " << quote(deviceName) << " does not exist in workcell!");
        return device;
    }

    int getJointIdx(const std::string& name, JointDevice *device){
        std::string fullname = device->getName() + string(".")+name;
        for(size_t i=0; i<device->getJoints().size(); i++){
            //Log::debugLog()<< device->getActiveJoint(i)->getName() << std::endl;
            if( device->getJoints()[i]->getName()== fullname)
                return (int)i;
        }
        RW_THROW("Joint "+quote(fullname)+"does not exist!");
        return -1;
    }

    KinematicBody::Ptr readKinematicBody(PTree& tree,
    		const std::string& frameAttr,
    		const std::string& prefix,
    		ParserState &state){
        Log::debugLog()<< "ReadKinematicBody" << std::endl;
        Frame *frame = getFrameFromAttr(tree, state, frameAttr, prefix);

        RW_DEBUGS("Trying to find object: \"" << frame->getName() << "\"");
        RigidObject::Ptr obj = state.wc->findObject( frame->getName() ).cast<RigidObject>();
        if(obj==NULL){
            // TODO: unfortunately the robwork kinematic loader does not fully support the
            // Object loading yet. So we need to create an object for this particular frame
            RW_DEBUGS("Adding new object to state!");
            obj = ownedPtr( new RigidObject(frame) );
            state.wc->add(obj);
        }

        //Object::Ptr obj = getObjectFromAttr(tree, state, frameAttr, prefix);
        RW_DEBUGS("Reading movable frame!");
        MovableFrame *refframe = dynamic_cast<MovableFrame*>( obj->getBase() );
        if(refframe==NULL) RW_THROW("The body frame of a Kinematic body must be a movable frame type!");
        string materialId = tree.get<string>("MaterialID");
        BodyInfo info;
        info.material = tree.get<string>("MaterialID",state.defaultMaterial);
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );

        //info.frames = DynamicUtil::getAnchoredChildFrames( refframe, state.rwstate, state.deviceBases);
        //std::vector<Geometry::Ptr> geoms = loadGeometry(refframe, info.frames, state.rwstate);
        TimerUtil::sleepMs(500);
        RW_DEBUGS("Creating kinematic body!");

        KinematicBody::Ptr body = ownedPtr( new KinematicBody(info, obj) );
        RW_DEBUGS("Adding kinematic body!");
        state.dwc->addBody( body );

        RW_DEBUGS("Added kinematic body!");
        //info.print();
        return body;
    }

    KinematicBody::Ptr readKinematicBody(PTree& tree,
    		const std::string& frameAttr,
    		ParserState &state)
    {
    	return readKinematicBody(tree,frameAttr,"",state);
    }


    Body::Ptr readRefBody(PTree& tree, ParserState &state)
    {
        string refbodyName = tree.get_child("<xmlattr>").get<std::string>("body");
        return state.dwc->findBody( refbodyName );
    }


    std::pair<BodyInfo, Object::Ptr> readRigidJoint(PTree& tree, ParserState &state, JointDevice *device ){
        Log::debugLog()<< "ReadRigidJoint" << std::endl;
        //string refjointName = tree.get_child("<xmlattr>").get<std::string>("joint");
        //Object::Ptr obj = state.wc->findObject(device->getName()+string(".")+refjointName);

        Frame *frame = getFrameFromAttr(tree, state, "joint", device->getName()+".");
        RigidObject::Ptr obj = state.wc->findObject( frame->getName() ).cast<RigidObject>();
        if(obj==NULL){
            // TODO: unfortunately the robwork kinematic loader does not fully support the
            // Object loading yet. So we need to create an object for this particular frame
            obj = ownedPtr( new RigidObject(frame) );
            state.wc->add(obj);
        }


        if( obj==NULL )
            return std::pair<BodyInfo, Object::Ptr>(BodyInfo(), NULL);

        BodyInfo info;
        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", state.defaultMaterial);
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
        	if(obj->getGeometry().size()!=0){
        		if( tree.get_optional<string>("COG") ){
        			// if COG specified then use it and calculate inertia

        			info.masscenter = readVector3D( tree.get_child("COG") );
        			Transform3D<> ref(info.masscenter);
        			info.inertia = GeometryUtil::estimateInertia(info.mass, obj->getGeometry(), obj->getBase(),state.rwstate, ref);
        		} else {
        			boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, obj->getGeometry(), obj->getBase(),state.rwstate);
        		}
        	} else {
        		RW_THROW("No geometry present to generate Inertia from Object: \"" << obj->getName() << "\"");
        	}
        }
        return std::make_pair(info, obj);
    }

    std::pair<BodyInfo, Object::Ptr> readLink(PTree& tree, ParserState &state, JointDevice *device, bool kinematic=false){
        Log::debugLog()<< "ReadLink" << std::endl;

        string refframeName = tree.get_child("<xmlattr>").get<std::string>("object");
        RigidObject::Ptr obj = state.wc->findObject(device->getName()+"."+refframeName).cast<RigidObject>();
        if( !obj ){
            // create one without any geometry
            Frame* baseframe = state.wc->findFrame(device->getName()+"."+refframeName);
            if(baseframe==NULL)
                RW_THROW("no frame with name " << device->getName()+"." + refframeName);
            obj = ownedPtr(new RigidObject(baseframe));
            state.wc->add( obj );
        }

        BodyInfo info;
        info.mass = tree.get<double>("Mass");
        info.material = tree.get<string>("MaterialID", state.defaultMaterial);
        info.objectType = tree.get<string>("ObjectID", state.defaultObjectType );


        readProperties(tree, obj->getBase()->getPropertyMap());

        info.objects.push_back(obj);
        // check if the body has multiple objects associated
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "Associate" ){
                std::string objname = p->second.get_child("<xmlattr>").get<std::string>("object");
                Object::Ptr assobj = state.wc->findObject(device->getName()+string(".")+objname);
                if(assobj==NULL)
                    assobj = state.wc->findObject(objname);
                if(assobj==NULL)
                    RW_THROW("Associated Object does not exist: \"" << objname << "\"");
                info.objects.push_back(assobj);
            }
        }

        std::vector<Geometry::Ptr> geoms;
        BOOST_FOREACH(Object::Ptr assobj, info.objects){
            BOOST_FOREACH(Geometry::Ptr g, assobj->getGeometry(state.rwstate)){
                geoms.push_back(g);
            }
        }

        boost::optional<string> def = tree.get_optional<string>("EstimateInertia");
        if(!def){
            info.masscenter = readVector3D( tree.get_child("COG") );
            info.inertia = readInertia( tree.get_child("Inertia") );
        } else {
            if(obj->getGeometry().size()!=0){
        		if( tree.get_optional<string>("COG") ){
        			// if COG specified then use it and calculate inertia

        			info.masscenter = readVector3D( tree.get_child("COG") );
        			Transform3D<> ref(info.masscenter);
        			info.inertia = GeometryUtil::estimateInertia(info.mass, geoms, obj->getBase(),state.rwstate, ref);
        		} else {
                	boost::tie(info.masscenter,info.inertia) = GeometryUtil::estimateInertiaCOG(info.mass, geoms, obj->getBase(),state.rwstate);
                }
            } else {
                RW_THROW("No geometry present to generate Inertia from Object: \"" << obj->getName() << "\"");
            }
        }
        return std::make_pair(info, obj);
    }
/*
    BeamJoint* readBeamJoint(PTree& tree, ParserState &state, JointDevice *device ){

    }
*/

    void readSpringJointController(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
        std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");

        Device::Ptr dev = getDeviceFromAttr(tree, state);

        if(dev==NULL)
            RW_THROW("No valid is referenced by the SpringJointController.");

        //bool useSyncPD = readBool( tree.get_child("Sync") );
        //JointController::ControlMode controlType = readControlMode( tree.get_child("<xmlattr>"), "type" );
        std::vector<double> params_tmp = readArray( tree.get_child("SpringParams") );
        double dt = tree.get<double>("TimeStep");

        RW_ASSERT(params_tmp.size()>1);
        SpringJointController::SpringParam sparam;
        std::vector<SpringJointController::SpringParam> params;
        for(size_t i=0;i<params_tmp.size()/3;i++){
            sparam.elasticity = params_tmp[3*i];
            sparam.dampening = params_tmp[3*i+1];
            sparam.offset = params_tmp[3*i+2];
            params.push_back( sparam );
        }

        DynamicDevice::Ptr ddev = findDynamicDevice(state, dev.get());
        RigidDevice::Ptr rdev = ddev.cast<RigidDevice>();
        if(rdev==NULL)
            RW_THROW("Spring controller can only control RigidDevice's!");
        SpringJointController::Ptr controller = ownedPtr(new SpringJointController(controllername, rdev, params, dt) );
        state.dwc->addController( controller );
    }


    void readSerialDeviceController(PTree& tree, ParserState &state){
        // configurations include
    	// possible force/torque sensor, for cartesean force control

    	Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
        std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");
        Device::Ptr dev = getDeviceFromAttr(tree, state);

        if(dev==NULL)
            RW_THROW("No valid is referenced by the SerialDeviceController.");

        RW_WARN("Find device");
        DynamicDevice::Ptr ddev = findDynamicDevice(state, dev.get());

        RW_WARN("Creating controller");
        SerialDeviceController::Ptr controller = ownedPtr(new SerialDeviceController(controllername, ddev) );
        RW_WARN("Push back controller");
        state.dwc->addController( controller );
    }



    SuctionCup::Ptr readSuctionCup(PTree& tree, ParserState &state){
        string deviceName = tree.get_child("<xmlattr>").get<std::string>("name");
        double radius = tree.get<double>("Radius");
        double height = tree.get<double>("Height");

        Q sc1 = readQ( tree.get_child("SpringParamsOpen") );
        if( sc1.size()!=5 )
            RW_THROW("There must be 5 parametes in SpringConstant1");

        Q sc2 = readQ( tree.get_child("SpringParamsClosed") );
        if( sc2.size()!=5 )
            RW_THROW("There must be 5 parametes in SpringConstant2");

        Body::Ptr base = NULL;
        RigidBody::Ptr end = NULL;
        Transform3D<> offset;
        std::string framePrefix("");
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "FixedBase" ){
                base = readFixedBody(p->second, state);
            } else if( p->first == "KinematicBase" ){
                base = readKinematicBody(p->second, "frame", framePrefix, state);
            } else if( p->first == "RigidBase" ){
                base = readRigidBody(p->second, framePrefix, state);
            } else if( p->first == "End") {
                end = readRigidBody(p->second,  framePrefix, state);
            } else if( p->first == "Offset" ){
                offset = readTransform(p->second);
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                //RW_THROW("Unknown element");
            }
        }

        if(base==NULL)
            RW_THROW("A SuctionCup must define a Base!");
        if(end==NULL)
            RW_THROW("A SuctionCup must define an end RigidBody!");

        state.dwc->addBody( base );
        state.dwc->addBody( end );
        SuctionCup::Ptr scup = ownedPtr( new SuctionCup(deviceName, base, end, offset, radius, height, sc1, sc2) ) ;
        state.wc->addDevice(scup->getKinematicModel());
        state.dwc->addDevice( scup );
        return scup;
    }



    KinematicDevice::Ptr readKinematicDevice(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadKinematicBody" << std::endl;
        JointDevice::Ptr device = getJointDeviceFromAttr(tree, state);
        std::vector<double> maxForce;
        //std::vector<KinematicBody*> bodies;

        std::vector<std::pair<BodyInfo,Object::Ptr> > bodies;
        Body::Ptr base = NULL;
        int jIdx = 0;
        std::string framePrefix = device->getName() + ".";
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "KinematicJoint" ){
                //KinematicBody *body = readBodyInfo(p->second, "joint", framePrefix, state);
                //bodies.push_back( body );
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, state, device.get());
                bodies.push_back( part );
            } else if( p->first == "Link" ){
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, state, device.get());
                bodies.push_back( part );
			} else if( p->first == "FixedBase" ){
				base = readFixedBase(p->second, state, device.get());
			} else if( p->first == "KinematicBase" ){
				base = readKinematicBody(p->second, "frame", framePrefix, state);
            } else if( p->first == "RefBase" ){
                // base reference to a frame in a body in which it is attached
                base = readRefBody(p->second, state);
                //

			} else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
            jIdx++;
        }
        if( !base ){
        	RW_THROW("Parser error - KinematicDevice must define a base (FixedBase or KinematicBase)");
        }
        KinematicDevice::Ptr kdev = ownedPtr(new KinematicDevice(base, bodies, device));
        std::vector<Body::Ptr> links = kdev->getLinks();
        BOOST_FOREACH(Body::Ptr l, links){
        	state.dwc->addBody( l );
        }
        state.dwc->addDevice( kdev );
        return kdev;
    }

    RigidDevice::Ptr readRigidDevice(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadRigidDevice" << std::endl;
        JointDevice::Ptr device = getJointDeviceFromAttr(tree, state);
        //Q maxForce(device->getDOF());

        // first we get the force limits of all joints/constraints
        std::map<std::string, double> maxForceMap;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "ForceLimit") {
                string refjoint = device->getName() + "." + p->second.get_child("<xmlattr>").get<std::string>("joint");
                maxForceMap[refjoint] = p->second.get_value<double>();
            }
        }
        // we put them in an array so that they are sorted correctly
        std::vector<double> maxForce;
        BOOST_FOREACH(Joint* joint, device->getJoints()){
            if( maxForceMap.find( joint->getName() ) == maxForceMap.end() ){
                RW_THROW("A force limit for the joint \"" << joint->getName()
                         << "\" in device \"" << device->getName() << "\" has not been defined!" );
            }
            maxForce.push_back( maxForceMap[joint->getName()] );
        }

        // next we find the base and all links
        std::vector<std::pair<BodyInfo,Object::Ptr> > bodies;
        Body::Ptr base = NULL;
        int jIdx = 0;
        std::string framePrefix = device->getName() + ".";
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            //std::cout << p->first << std::endl;
        	if( p->first == "FixedBase" ){
            	base = readFixedBase(p->second, state, device.get());
            } else if( p->first == "KinematicBase" ){
				base = readKinematicBody(p->second, "frame", framePrefix, state);
            } else if( p->first == "RigidBase" ){
            	base = readRigidBody(p->second, framePrefix, state);
            } else if( p->first == "RefBase" ){
            	// base reference to a frame in a body in which it is attached
                base = readRefBody(p->second, state);
            } else if( p->first == "Link" ){
                std::pair<BodyInfo,Object::Ptr> part = readLink(p->second, state, device.get());
                bodies.push_back( part );
            } else if( p->first == "RigidJoint" ){
                // this can be both a link or a constraint or both. Here for backwards compatibility!!!!
                std::pair<BodyInfo,Object::Ptr> part = readRigidJoint(p->second, state, device.get());
                if(part.second!=NULL){
                    bodies.push_back( part );
                }
            } else if( p->first == "ForceLimit" ){
                // we allready processed this one
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element" << StringUtil::quote(p->first) );
            }
            jIdx++;
        }
        if( !base ){
        	RW_THROW("Parser error - RigidDevice must define a base (FixedBase or KinematicBase)");
        }
        RW_ASSERT(base!=NULL);

        RigidDevice::Ptr rigiddev = ownedPtr(new RigidDevice(base, bodies, device));
        rigiddev->setMotorForceLimits( Q(maxForce) );
        std::vector<Body::Ptr> links = rigiddev->getLinks();
        BOOST_FOREACH(Body::Ptr l, links){
            RW_DEBUGS("Adding body to dwc: " << l->getName());
        	state.dwc->addBody( l );
        }
        RW_DEBUGS("Adding device to dwc: " << rigiddev->getName());
        state.dwc->addDevice(rigiddev);
        return rigiddev;
    }

    void readTactileSensor(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadTactileData" << std::endl;
        Body::Ptr tactileFrame = getBodyFromAttr(tree, state, "frame");
        if(tactileFrame==NULL)
            RW_THROW("No Body is referenced by the tactile sensor.");
        Log::debugLog()<< "TactileFrameName: " << tactileFrame->getName()<< std::endl;
        string name = tree.get<string>("Name");
        Vector3D<> pos = readVector3D(tree.get_child("Pos"));
        Vector3D<> rpyTmp = readVector3D(tree.get_child("RPY"))*Deg2Rad;
        RPY<> rpy(rpyTmp[0],rpyTmp[1],rpyTmp[2]);
        Transform3D<> transform(pos,rpy);

        Q dimTmp = readQ(tree.get_child("TexelArray"));
        std::pair<int,int> dim((int)(dimTmp(0)+1),(int)(dimTmp(1)+1));
        Eigen::MatrixXf heightMap = readMatrix(tree.get_child("TexelHeightMap"), dim);
        Log::debugLog()<< "HeightMap" << heightMap << std::endl;
        Vector2D<> texelSize = readVector2D(tree.get_child("TexelSize"));
        double maxForce = tree.get<double>("MaxForce");
        double minForce = tree.get<double>("MinForce");
        //string bodyFrame = tree.get<string>("Body");

        TactileArraySensor::Ptr sensor =
            ownedPtr( new TactileArraySensor(name, tactileFrame, transform, heightMap, texelSize) );

        sensor->setPressureLimit(minForce, maxForce);
        state.dwc->addSensor(sensor);
    }

    void readBodySensor(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadBodyContactData" << std::endl;
        std::string bsname = tree.get_child("<xmlattr>").get<std::string>("name");

        Frame *bodyFrame = getFrameFromAttr(tree, state, "body");
        if(bodyFrame==NULL)
            RW_THROW("No frame is referenced by the body contact sensor.");

        BodyContactSensor::Ptr bsensor = ownedPtr( new BodyContactSensor(bsname, bodyFrame) );
        state.dwc->addSensor(bsensor);
    }
    void readTactileMultiAxisSensor(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadBodyContactData" << std::endl;
        std::string bsname = tree.get_child("<xmlattr>").get<std::string>("name");

        Frame *bodyFrame = getFrameFromAttr(tree, state, "body");
        Frame *body1Frame = getFrameFromAttr(tree, state, "body1");

        if(bodyFrame==NULL)
            RW_THROW("No frame is referenced by the body contact sensor.");

        if(body1Frame==NULL)
            RW_THROW("No frame is referenced by the body contact sensor.");

        Body::Ptr b1 = state.dwc->findBody( bodyFrame->getName() );
        Body::Ptr b2 = state.dwc->findBody( body1Frame->getName() );

        if(b1==NULL || b2==NULL)
            RW_THROW("The FTSensor must be specified between two constrained bodies."
            		 " One of the specified body names are not valid!");

        Frame* refframe = NULL;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if( p->first == "RefFrame" ){
                string refframeName = p->second.get_value<std::string>();
                refframe = state.wc->findFrame(refframeName);
                if( !refframe )
                    RW_THROW("RefFrame " << quote(refframeName) << " does not exist in workcell!");
            }
        }

        SimulatedFTSensor::Ptr bsensor = ownedPtr( new SimulatedFTSensor(bsname, b1, b2, refframe) );
        state.dwc->addSensor(bsensor);
    }

    void readFTSensor(PTree& tree, ParserState &state){
    	return readTactileMultiAxisSensor(tree,state);
    }

    JointController::ControlMode readControlMode(PTree& tree, const std::string& tname ){
        string controlType = tree.get<std::string>(tname);
        if(controlType=="Position") return JointController::POSITION;
        else if(controlType=="CntPosition") return JointController::CNT_POSITION;
        else if(controlType=="Velocity") return JointController::VELOCITY;
        else if(controlType=="Force") return JointController::FORCE;
        else if(controlType=="Current") return JointController::CURRENT;

        RW_THROW("Control type: \"" << controlType << "\" is not supported!");
        return JointController::POSITION;
    }

    void readPDDeviceController(PTree& tree, ParserState &state){
        Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
        std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");

        Device::Ptr dev = getDeviceFromAttr(tree, state);

        if(dev==NULL)
            RW_THROW("No valid is referenced by the PDDeviceController.");

        bool useSyncPD = readBool( tree.get_child("Sync") );
        JointController::ControlMode controlType = readControlMode( tree.get_child("<xmlattr>"), "type" );
        std::vector<double> params_tmp = readArray( tree.get_child("PDParams") );
        double dt = tree.get<double>("TimeStep");

        RW_ASSERT(params_tmp.size()>1);
        std::vector<PDParam> params;
        for(size_t i=0;i<params_tmp.size()/2;i++){
            params.push_back( PDParam(params_tmp[2*i],params_tmp[2*i+1]));
        }

        DynamicDevice::Ptr ddev = findDynamicDevice(state, dev.get());
        if(useSyncPD){

            RW_THROW("Not currently supported!");
            //SyncPDController *controller = new SyncPDController();
        } else {

            PDController::Ptr controller = ownedPtr( new PDController(controllername, ddev, controlType, params, dt) );
            state.dwc->addController( controller );
        }

    }

    void readPoseDeviceController(PTree& tree, ParserState &state){
		Log::debugLog()<< "ReadDeviceControllerData" << std::endl;
		std::string controllername = tree.get_child("<xmlattr>").get<std::string>("name");

		Device::Ptr dev = getDeviceFromAttr(tree, state);

		if(dev==NULL)
			RW_THROW("No valid is referenced by the PoseDeviceController.");

		//Frame* tcp = getFrameFromAttr(tree, state, "tcp");

		double dt = tree.get<double>("TimeStep");

		DynamicDevice::Ptr ddev = findDynamicDevice(state, dev.get());
		PoseController::Ptr controller = ownedPtr( new PoseController(controllername, ddev, state.wc->getDefaultState(), dt) );
		state.dwc->addController( controller );
		//state.controllers.push_back( controller );
	}

    void readFrictionDatas(PTree& tree, string first, string second, ParserState &state){
        Log::debugLog()<< "ReadFrictionDatas1" << std::endl;
        FrictionDataTmp dataTmp;
        dataTmp.matA = first;
        dataTmp.matB = second;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "FrictionData") {
                Log::debugLog()<< "FrictionData" << std::endl;
                std::string typestr = p->second.get_child("<xmlattr>").get<std::string>("type");
                dataTmp.data.type = Coulomb; //matMap.getDataID(typestr);
                // all elements of FrictionData must be param arrays
                for (CI d = p->second.begin(); d != p->second.end(); ++d) {
                    if( d->first != "<xmlattr>" && d->first != "<xmlcomment>"){
                        Q q = readQ(d->second);
                        dataTmp.data.parameters.push_back(std::make_pair(d->first,q));
                    }
                }
                state.fdatas.push_back( dataTmp );
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
        }

    }

    void readFrictionMap(PTree& tree, ParserState &state){
        Log::debugLog()<< "FrictionMap" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Pair") {
                string first = p->second.get_child("<xmlattr>").get<std::string>("first");
                string second = p->second.get_child("<xmlattr>").get<std::string>("second");
                readFrictionDatas(p->second, first, second, state);
            } else if( p->first!="<xmlcomment>" ){
                RW_THROW("Unknown element: \"" << p->first << "\"" );
            }
        }
    }

    void readMaterialDataList(PTree& tree, ParserState &state ){
        Log::debugLog()<< "ReadMaterialList" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Material") {
                string id = p->second.get_child("<xmlattr>").get<std::string>("id");
                string desc = p->second.get("Description","");
                state.materialData.add(id, desc);
            } else if(p->first == "Default"){
                string defMaterial = p->second.get_value<string>();
                state.defaultMaterial = defMaterial;
            } else if(p->first!="<xmlcomment>"){
                RW_THROW("Unknown element");
            }
        }
    }

    void readContactDatas(PTree& tree, string first, string second, ParserState &state){
        Log::debugLog()<< "ReadFrictionDatas" << std::endl;
        ContactDataTmp dataTmp;
        dataTmp.objA = first;
        dataTmp.objB = second;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "ContactData") {
                std::string typestr = p->second.get_child("<xmlattr>").get<std::string>("type");
                // todo: switch on type. Now we assume newton
                double cr = p->second.get<double>("cr");
                dataTmp.data.cr = cr;
                state.cdatas.push_back( dataTmp );
            } else if(p->first !="<xmlcomment>" && p->first != "<xmlattr>"){
                RW_THROW("Unknown element");
            }
        }
    }

    void readContactMap(PTree& tree, ParserState &state ){
        Log::debugLog()<< "ReadMaterialList" << std::endl;
        Log::debugLog()<< "FrictionMap" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "Pair") {
                string first = p->second.get_child("<xmlattr>").get<std::string>("first");
                string second = p->second.get_child("<xmlattr>").get<std::string>("second");
                readContactDatas(p->second, first, second, state);
            } else if(p->first!="<xmlcomment>"){
                RW_THROW("Unknown element");
            }
        }
    }

    void readContactDataList(PTree& tree, ParserState &state ){
        Log::debugLog() << "ReadMaterialList" << std::endl;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
            if (p->first == "ObjectType") {
                string id = p->second.get_child("<xmlattr>").get<std::string>("id");
                string desc = p->second.get("Description","");
                state.contactData.add(id, desc);
            } else if(p->first == "Default"){
                string defObjectType = p->second.get_value<string>();
                state.defaultObjectType = defObjectType;
            } else if(p->first!="<xmlcomment>"){
                RW_THROW("Unknown element");
            }
        }
    }

    void readSpring(PTree& tree, ParserState &state, Constraint::Ptr constraint = NULL ){
        Log::debugLog() << "ReadSpring" << std::endl;
        if (constraint == NULL) {
        	const string name = tree.get_child("<xmlattr>").get<std::string>("constraint");
        	Constraint::Ptr constraint = state.dwc->findConstraint(name);
        	if (constraint == NULL)
        		RW_THROW("Spring refers to constraint " << quote(name) << " that could not be found!");
        }
    	Constraint::SpringParams params;
    	params.enabled = true;
        for (CI p = tree.begin(); p != tree.end(); ++p) {
        	const int freeDOF = constraint->getDOF();
        	if (p->first == "Compliance") {
        		params.compliance = readMatrixD(p->second, std::make_pair<int,int>(freeDOF,freeDOF));
        	} else if (p->first == "Damping") {
        		params.damping = readMatrixD(p->second, std::make_pair<int,int>(freeDOF,freeDOF));
        	} else if(p->first!="<xmlcomment>" && p->first != "<xmlattr>"){
        		RW_THROW("Unknown element");
        	}
        }
        constraint->setSpringParams(params);
    }

    void readConstraint(PTree& tree, ParserState &state ){
        Log::debugLog() << "ReadConstraint" << std::endl;
        const string name = tree.get_child("<xmlattr>").get<std::string>("name");
        const string type = tree.get_child("<xmlattr>").get<std::string>("type");
        const string parent = tree.get_child("<xmlattr>").get<std::string>("parent");
        const string child = tree.get_child("<xmlattr>").get<std::string>("child");
        Constraint::ConstraintType cType;
        if (!Constraint::toConstraintType(type,cType)) {
        	RW_THROW("Constraint type " << type << " not recognized!");
        }
        const Body::Ptr body1 = state.dwc->findBody(parent);
        const Body::Ptr body2 = state.dwc->findBody(child);
        if (body1 == NULL)
        	RW_THROW("Parent body " << quote(parent) << " for constraint " << quote(name) << " not found!");
        if (body2 == NULL)
        	RW_THROW("Child body " << quote(child) << " for constraint " << quote(name) << " not found!");
        Constraint::Ptr constraint = ownedPtr(new Constraint(name,cType,body1.get(),body2.get()));
        state.dwc->addConstraint(constraint);
        for (CI p = tree.begin(); p != tree.end(); ++p) {
        	if (p->first == "Transform3D") {
        		constraint->setTransform(readTransform(p->second));
            } else if (p->first == "Spring") {
            	readSpring(p->second,state,constraint);
        	} else if(p->first!="<xmlcomment>" && p->first != "<xmlattr>"){
        		RW_THROW("Unknown element");
        	}
        }
    }



    void readInclude(PTree& tree, PTree &parent, CI &iter, ParserState& state){
    	CI lastIter = iter;
    	--lastIter;
    	std::string filename = tree.get_child("<xmlattr>").get<std::string>("file");
    	//std::string filename = tree.get<std::string>("file");
    	//std::cout << "Filename: " << filename << std::endl;

        if(!StringUtil::isAbsoluteFileName(filename)){
        	std::string fullname = state.dir;
        	fullname.append(filename);
        	filename = fullname;
        }

        PTree ntree;
        read_xml(filename, ntree);
        PTree &data = ntree.get_child("IncludeData");

        CI nextiter = iter;
        ++nextiter;
    	parent.insert(nextiter, data.begin(), data.end() );

    }

    void readPhysicsEngine(PTree& tree, ParserState& state){
        state.defaultcolmargin =  tree.get<double>("CollisionMargin", 0.01);
        state.defaultRestModel = tree.get<string>("RestitutionModel","Newton");
        state.defaultContactModel = tree.get<string>("ContactModel","Newton");
        std::string tmp = tree.get<string>("AutoDisable","true");
        if( tmp == "false" ){
        	state.autoDisable = false;
        } else if (tmp=="true"){
        	state.autoDisable = true;
        } else {
        	RW_THROW("Wrong parameter in AutoDisable element!");
        }

        // get all physics engine properties
        for (CI p = tree.begin(); p != tree.end(); ++p) {
        	if(p->first == "Property") {
        		std::string name = p->second.get_child("<xmlattr>").get<std::string>("name");
        		//std::string desc = p->second.get_child_optional("Description").get<std::string>("desc","");
        		std::string type = p->second.get_child("<xmlattr>").get<std::string>("type","string");
        		if( type=="string" ){
        			std::string value = p->second.get_value<std::string>();
        			state.engineProps.add<std::string>(name, "", value);
        		} else if( type=="int" ){
        			int value = p->second.get_value<int>();
        			state.engineProps.add<int>(name, "", value);
        		} else if( type=="float" ){
        			double value = p->second.get_value<double>();
        			state.engineProps.add<double>(name, "", value);
        		} else if( type=="Q" ){
        			rw::math::Q value = getValueAsQ(p->second.get_value<std::string>());

        			state.engineProps.add<rw::math::Q>(name, "", value);
        		} else {
        			RW_THROW("DynamicWorkCellLoader: Unknown engine property type: " << StringUtil::quote(type) );
        		}

        	}
        }

    }


    void getWorkCellOptionally(
        PTree& tree, ParserState& state)
    {
        if (state.wc){
            Log::debugLog()<< "workcell allready loadet" << std::endl;
            return;
        }
        const PTree &child = tree.get_child("<xmlattr>");
        const string workcell_name =  child.get<string>("workcell");

        if(StringUtil::isAbsoluteFileName(workcell_name)){
            state.wc = WorkCellFactory::load(workcell_name);
        } else {
            std::string directory = StringUtil::getDirectoryName(state.dwcfile);
            state.wc = WorkCellFactory::load(directory+workcell_name);
        }
    }


    DynamicWorkCell::Ptr readDynamicWC(PTree& tree, ParserState& state)
    {
        getWorkCellOptionally(tree, state);
        state.rwstate = state.wc->getDefaultState();
        BOOST_FOREACH(Device::Ptr dev, state.wc->getDevices()){
            state.deviceBases.push_back(dev->getBase());
        }

        // create the DynamicWorkCell so that the following can add stuff to it
        state.dwc = ownedPtr( new DynamicWorkCell(state.wc) );


        for (CI p = tree.begin(); p != tree.end(); ++p) {
        	Log::debugLog()<< p->first << std::endl;
        	if (p->first == "PhysicsEngine") {

                readPhysicsEngine(p->second, state);
            } else if (p->first == "DefaultRestitutionModel") {
                Log::debugLog()<< p->first << std::endl;
            } else if (p->first == "MaterialData") {
                Log::debugLog()<< p->first << std::endl;
                readMaterialDataList(p->second, state);
            } else if (p->first == "FrictionMap") {
                Log::debugLog()<< p->first << std::endl;
                readFrictionMap(p->second, state);
            } else if (p->first == "RigidBody") {
                Log::debugLog()<< p->first << std::endl;
                readRigidBody(p->second, state);
            } else if (p->first == "FixedBody") {
                Log::debugLog()<< p->first << std::endl;
                readFixedBody(p->second, state);
            } else if (p->first == "KinematicBody") {
                Log::debugLog()<< p->first << std::endl;
                KinematicBody::Ptr body = readKinematicBody(p->second, "frame", state);
            } else if (p->first == "Gravity") {
                state.gravity = readVector3D( p->second );
            } else if (p->first == "SuctionCup") {
                SuctionCup::Ptr sdev = readSuctionCup(p->second, state);
            } else if (p->first == "KinematicDevice") {
                KinematicDevice::Ptr kdev = readKinematicDevice(p->second, state);
            } else if (p->first == "RigidDevice") {
                RigidDevice::Ptr rdev = readRigidDevice(p->second, state);
            } else if (p->first == "ObjectTypeData") {
                readContactDataList(p->second,state);
            } else if (p->first == "ContactMap") {
                readContactMap(p->second,state);
            } else if (p->first == "ContactModel") {

            } else if (p->first == "Constraint") {
            	readConstraint(p->second,state);
            } else if (p->first == "Spring") {
            	readSpring(p->second,state);
            } else
            ///// THE DIFFERENT SENSOR MODELS
            if (p->first == "TactileArraySensor") {
                readTactileSensor(p->second, state);
            } else if (p->first == "BodyContactSensor") {
                readBodySensor(p->second, state);
            } else if (p->first == "TactileMultiAxisSensor") {
                readTactileMultiAxisSensor(p->second, state);
            } else if (p->first == "FTSensor") {
                readFTSensor(p->second, state); // same as TactileMultiAxisSensor
            } else
            ///// THE DIFFERENT CONTROLLER MODELS
            if (p->first == "PDDeviceController") {
                readPDDeviceController(p->second, state);
            } else if (p->first == "PoseDeviceController") {
                readPoseDeviceController(p->second, state);
            } else if (p->first == "SerialDeviceController") {
            	readSerialDeviceController(p->second, state);
            } else if (p->first == "SpringJointController") {
                readSpringJointController(p->second, state);
            } else if (p->first == "Include") {
            	readInclude(p->second, tree, p, state);
            } else if (p->first == "<xmlattr>") {
            } else if (p->first == "<xmlcomment>") {
            } else {
                RW_THROW(
                    "Unsupported element "
                    << quote(p->first) );
            }
        }
        
        // add all friction data too the materialdatamap
        BOOST_FOREACH(FrictionDataTmp &fdata, state.fdatas){
            try{
                state.materialData.addFrictionData(fdata.matA, fdata.matB, fdata.data);
            } catch (...){

            }
        }
        
        BOOST_FOREACH(ContactDataTmp &cdata, state.cdatas){
            state.contactData.addNewtonData(cdata.objA, cdata.objB, cdata.data);
        }
        
        // TODO: now check if all bodies has a correct material and objectId. If they has not then
        // we use the default

        BOOST_FOREACH(Body::Ptr body, state.dwc->getBodies() ){
            if( body->getInfo().material == ""){
                if(state.defaultMaterial=="")
                    RW_THROW("No default material defined! Either define one "
                             "or specify material for all objects..");
                body->getInfo().material = state.defaultMaterial;
            }

            if( body->getInfo().objectType == ""){
                if(state.defaultObjectType=="")
                    RW_THROW("No default object type defined! Either define one "
                             "or specify object types for all objects..");
                body->getInfo().objectType = state.defaultObjectType;
            }
        }
        
        //std::cout << "------------ nr controllers:  " <<  state.controllers.size() << std::endl;
        //  create the dynamic workcell
        //DynamicWorkCell *dynWorkcell =
            //new DynamicWorkCell(state.wc, state.bodies, state.devices, state.controllers);
		//	new DynamicWorkCell(state.wc, state.bodies, state.allbodies, state.devices, state.controllers);

        
        state.dwc->setGravity(state.gravity);
        state.dwc->getMaterialData() = state.materialData;
        state.dwc->getContactData() = state.contactData;
        state.dwc->getEngineSettings() = state.engineProps;

        return state.dwc;
    }
}

rw::common::Ptr<DynamicWorkCell> DynamicWorkCellLoader::load(const string& filename)
{
	std::string file = IOUtil::getAbsoluteFileName(filename);

    DynamicWorkCell::Ptr dwc;
    try {
        ParserState state(file);

        state.dir = StringUtil::getDirectoryName(file);

        PTree tree;
        read_xml(file, tree);

        // XML::printTree(tree);
        dwc = readDynamicWC(tree.get_child("DynamicWorkcell"), state);

    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
    
    return dwc;
}
