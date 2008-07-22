#include "XMLRWLoader.hpp"

#include "XMLRWParser.hpp"
#include "XMLParserUtil.hpp"

#include <rw/kinematics/StateStructure.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/FrameType.hpp>
#include <rw/kinematics/FixedFrame.hpp>
#include <rw/kinematics/MovableFrame.hpp>

#include <rw/models/Joint.hpp>
#include <rw/models/RevoluteJoint.hpp>
#include <rw/models/PrismaticJoint.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/ParallelDevice.hpp>
#include <rw/models/MobileDevice.hpp>
#include <rw/models/TreeDevice.hpp>
#include <rw/models/ParallelLeg.hpp>


#include <rw/math/Constants.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/RPY.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/IOUtil.hpp>
#include <rw/common/macros.hpp>

#include <rw/models/Accessor.hpp>

#include <rw/loaders/colsetup/CollisionSetupLoader.hpp>
#include <rw/proximity/CollisionSetup.hpp>

#include <rw/models/PassivePrismaticFrame.hpp>
#include <rw/models/PassiveRevoluteFrame.hpp>

#include <stack>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw::proximity;
using namespace rw;

namespace {



	struct InitialAction {
	public:
		virtual void setInitialState(rw::kinematics::State& state) = 0;
        virtual ~InitialAction() {}
	};

	struct DeviceInitState : public InitialAction {
	private:
	    rw::math::Q _q;
	    rw::models::Device *_dev;
	public:

	    DeviceInitState(rw::math::Q q, rw::models::Device* dev):
	        _q(q),_dev(dev){}

        void setInitialState(rw::kinematics::State& state) {
            _dev->setQ(_q, state);
        }

	};

	struct MovableInitState : public InitialAction {
	private:
		MovableFrame *_frame;
		Transform3D<> _t3d;
	public:
		MovableInitState(MovableFrame *frame, Transform3D<> t3d):
			_frame(frame),_t3d(t3d){}

		virtual ~MovableInitState(){}

    	void setInitialState(rw::kinematics::State& state) {
    		//std::cout << "Set initial state on " << _frame->getName() << std::endl;
    		_frame->setTransform(_t3d, state);
    	}
    };

    // container for collision setups, filename and scoped name
     typedef std::vector<DummyCollisionSetup> ColSetupList;

	struct DummySetup {
	public:
	    Frame *world;
	    StateStructure *tree;

	    std::map<std::string, Frame*> frameMap;
	    std::map<std::string, DummyFrame*> dummyFrameMap;
	    std::map<std::string, std::vector<Frame*> > toChildMap;
	    std::vector<InitialAction*> actions;
	    boost::shared_ptr<DummyWorkcell> dwc;

	    ColSetupList colsetups;
	    std::map<std::string,Device*> devMap;
	};

    // the parent frame must exist in tree allready
    void addToStateStructure(Frame *parent, DummySetup &setup){
        std::vector<Frame*> children = setup.toChildMap[parent->getName()];
        BOOST_FOREACH(Frame* child, children){
            DummyFrame *dframe = setup.dummyFrameMap[child->getName()];
            RW_DEBUG("Adding: " << parent->getName() << "<--" << dframe->getName());
            if( dframe->_isDaf ) setup.tree->addDAF(child, parent);
            else setup.tree->addFrame(child, parent);
            addToStateStructure(child, setup);
        }
    }

    // the parent to name must exist in tree
    void addToStateStructure(const std::string& name, DummySetup &setup){
        DummyFrame *dframe = setup.dummyFrameMap[name];
        RW_DEBUG("RefFrame : " << dframe->getRefFrame());
        Frame *parent = setup.frameMap[dframe->getRefFrame()];
        if(parent == NULL) RW_THROW("PARENT IS NULL");

        Frame *child = setup.frameMap[dframe->getName()];
        if( dframe->_isDaf ) setup.tree->addDAF(child, parent);
        else setup.tree->addFrame(child, parent);
        addToStateStructure(child, setup);
    }

    std::string createScopedName(std::string str, std::vector<std::string> scope){
        std::string tmpstr;
        for(size_t i = 0; i<scope.size(); i++){
            tmpstr += scope[i]+".";
        }
        tmpstr += str;
        return tmpstr;
    }

    typedef std::map<std::string,Frame*> FrameMap;

    boost::shared_ptr<Property<std::string> > createStringProperty(DummyProperty &dprop){
        boost::shared_ptr< Property<std::string> > prop(
            new common::Property<std::string>(dprop._name,dprop._desc,dprop._val) );
        return prop;
    }

    bool isIdentity(rw::math::Transform3D<>& t3d)
    {
        const double eps = 0.000001;
        if( ( fabs( t3d(0,0) - 1)< eps) &&
            ( fabs( t3d(1,1) - 1)< eps) &&
            ( fabs( t3d(2,2) - 1)< eps) &&
            ( fabs( t3d(0,1) ) < eps )  &&
            ( fabs( t3d(0,2) ) < eps )  &&
            ( fabs( t3d(0,3) ) < eps )  &&
            ( fabs( t3d(1,0) ) < eps )  &&
            ( fabs( t3d(1,2) ) < eps )  &&
            ( fabs( t3d(1,3) ) < eps )  &&
            ( fabs( t3d(2,0) ) < eps )  &&
            ( fabs( t3d(2,1) ) < eps )  &&
            ( fabs( t3d(2,3) ) < eps )  &&
            ( fabs( t3d(3,0) ) < eps )  &&
            ( fabs( t3d(3,1) ) < eps )  &&
            ( fabs( t3d(3,2) ) < eps ) )
            return true;
        return false;
    }



    Frame* addModelToFrame( DummyModel& model, Frame *parent, StateStructure *tree){
        // test if identity
        Frame *modelframe = parent;
        std::vector<std::string> scope =  model._scope;

        for( size_t i=0; i<model._geo.size(); i++) {
            std::ostringstream val;

            switch(model._geo[i]._type){
                case PolyType:
                    if( ! StringUtil::isAbsoluteFileName(
                            model._geo[i]._filename + ".tmp" ) ) {
                        val << StringUtil::getDirectoryName(
                            model._geo[i]._pos.file );
                    }
                    val << model._geo[i]._filename;
                    break;
                case CubeType:
                    val << "#Box " << model._geo[i]._x << " "
                        << model._geo[i]._y << " " << model._geo[i]._z;
                    break;
                case SphereType:
                    val << "#Sphere " << model._geo[i]._radius;
                    break;
                case ConeType:
                    val << "#Cone " << model._geo[i]._radius << " "
                        << model._geo[i]._z;
                    break;
                default:
                    val << "";
            }

            if( model._isDrawable ){
            	std::vector<DrawableModelInfo> info;
            	if( Accessor::drawableModelInfo().has(*modelframe) )
            		info = Accessor::drawableModelInfo().get(*modelframe);
            	info.push_back(DrawableModelInfo(val.str(),model._transform));
                Accessor::drawableModelInfo().set( *modelframe, info );
            }
            if( !model._isDrawable || model._colmodel ){
/*
                std::string name = createScopedName( model._name, scope );
                modelframe = new FixedFrame(name, model._transform);
                tree->addFrame(modelframe, parent);
                Accessor::frameType().set(
                    *modelframe, rw::kinematics::FrameType::FixedFrame );
*/
            	std::vector<CollisionModelInfo> info;
            	if( Accessor::collisionModelInfo().has(*modelframe) )
            		info = Accessor::collisionModelInfo().get(*modelframe);
            	info.push_back(CollisionModelInfo(val.str(),model._transform));
            	Accessor::collisionModelInfo().set( *modelframe, info );
            }
        }
        return modelframe;
    }

    void addLimit( const DummyLimit &limit, Joint* j){
        double convFactor = 1.0;
        if( dynamic_cast<RevoluteJoint*>(j) != NULL ){
            convFactor = Deg2Rad;
        }
        switch( limit._type ){
            case( PosLimitType ):
                j->setBounds( std::pair<double,double>(limit._min*convFactor, limit._max*convFactor) );
                break;
            case( VelLimitType ):
                j->setMaxVelocity( limit._max*convFactor );
                break;
            case( AccLimitType ):
                j->setMaxAcceleration( limit._max*convFactor );
                break;
            default:
                assert(0);
        }
    }

    void addLimits( std::vector<DummyLimit> &limits, Joint *j){
        for( size_t i = 0; i<limits.size(); i++)
            addLimit( limits[i], j );
    }

    void addLimitsToFrame( std::vector<DummyLimit> &limits, Frame *f){
        Joint *j = dynamic_cast<Joint*>(f);
        if( j==NULL )
            return;
        addLimits(limits,j);
    }

    Frame *createFrame(DummyFrame& dframe, DummySetup &setup) {
        Frame *frame = NULL;
        //std::cout << "Depend: " << dframe._isDepend << std::endl;

        if( dframe._isDepend ){
            std::map<std::string, Frame*>::iterator res =
                setup.frameMap.find(dframe.getDependsOn());
            if( res == setup.frameMap.end() ){
                RW_WARN("The frame: " << dframe.getName()
                          << " Depends on an unknown frame: "
                          << dframe.getDependsOn() );
                return NULL;
            }
            // then the frame depends on another joint
            Joint* owner = dynamic_cast<Joint*>( (*res).second );
            if( owner==NULL ){
                RW_THROW("The frame: " << dframe.getName()
                          << " Depends on an frame: "
                          << dframe._dependsOn << " which is not a Joint");
            }

            if( dframe._type == "Revolute" ){
                frame = new PassiveRevoluteFrame(dframe.getName(),
                                                 dframe._transform,
                                                 owner, dframe._gain,
                                                 dframe._offset);

                RW_DEBUG("Passive Revolute joint: " << dframe._gain << " "
                         << dframe._offset);
            } else if( dframe._type == "Prismatic" ){
                frame = new PassivePrismaticFrame(dframe.getName(),
                                                 dframe._transform,
                                                 owner, dframe._gain,
                                                 dframe._offset);
                //std::cout << "Passive prismatic joint: " << dframe._gain << " "
                //          << dframe._offset << std::endl;
            }  else {
                RW_THROW("Error: The type of frame: " << dframe.getName()
                          << " cannot depend on another joint!!");
            }

        } else if( dframe._type == "Fixed" ){
            frame = new FixedFrame(dframe.getName(), dframe._transform );
            Accessor::frameType().set(*frame, rw::kinematics::FrameType::FixedFrame);
        } else if( dframe._type == "Movable") {
            MovableFrame *mframe = new MovableFrame( dframe.getName() );
            frame = mframe;
            Accessor::frameType().set(*mframe, rw::kinematics::FrameType::MovableFrame);
            MovableInitState *init = new MovableInitState(mframe,dframe._transform);
            setup.actions.push_back(init);
        } else if( dframe._type == "Prismatic") {
            PrismaticJoint *j = new PrismaticJoint( dframe.getName(), dframe._transform );
            addLimits( dframe._limits, j );
            frame = j;
            Accessor::frameType().set(*frame, rw::kinematics::FrameType::PrismaticJoint);
            if( dframe._state == ActiveState)
                Accessor::activeJoint().set(*frame, true);
            //std::cout << "Prismatic joint!! " << j->getName() << std::endl;
        } else if( dframe._type == "Revolute") {
            RevoluteJoint *j = new RevoluteJoint( dframe.getName(), dframe._transform );
            addLimits( dframe._limits, j );
            frame = j;
            Accessor::frameType().set(*frame, rw::kinematics::FrameType::RevoluteJoint);
            if( dframe._state == ActiveState)
                Accessor::activeJoint().set(*frame, true);
        } else if( dframe._type == "EndEffector" ){
            frame = new FixedFrame(dframe.getName(), dframe._transform );
            Accessor::frameType().set(*frame, rw::kinematics::FrameType::FixedFrame);
        } else {
            RW_THROW("FRAME is of illegal type!! " << dframe._type);
        }

        // remember to add the frame to the frame map
        setup.dummyFrameMap[dframe.getName()] = &dframe;
        setup.frameMap[frame->getName()] = frame;
        setup.toChildMap[dframe.getRefFrame()].push_back(frame);
        RW_DEBUG("Frame created: " << frame->getName() << " --> " << dframe.getRefFrame());
        return frame;
    }

    void addFrameProps(DummyFrame &dframe, DummySetup &setup){
        Frame *frame = setup.frameMap[dframe.getName()];
        //std::cout << "Nr of properties in frame: " << dframe._properties.size() << std::endl;
        for(size_t i=0; i<dframe._properties.size(); i++){
            const DummyProperty& dprop = dframe._properties[i];
            frame->getPropertyMap().add(
                dprop._name, dprop._desc, dprop._val);
        }
        for(size_t i=0; i<dframe._models.size(); i++){
            addModelToFrame( dframe._models[i], frame, setup.tree);
        }
    }

/*
    void addStateStructure( Frame *frame, Frame *parent, boost::shared_ptr<StateStructure> tree){
        tree->addFrame( frame );
        Frame::iterator iter = frame->getChildren().first;
        for(; iter != frame->getChildren().second ; ++iter) {
            addStateStructure( &(*iter) , tree );
        }
    }
*/
    /**
     * @brief adds all device context defined properties to the frame
     */
    void addDevicePropsToFrame(DummyDevice &dev, const std::string& name, DummySetup &setup){
        Frame *frame = setup.frameMap[name];
        // add properties specified in device context
        //std::cout << "Search props: " << name << std::endl;
        std::vector<boost::shared_ptr<rw::common::Property<std::string> > > proplist =
            dev._propMap[frame->getName()];

        //std::cout << "Nr Props in list: " << proplist.size() << std::endl;
        for( size_t j=0; j<proplist.size(); j++ ){
            frame->getPropertyMap().add(
                proplist[j]->getIdentifier(),
                proplist[j]->getDescription(),
                proplist[j]->getValue());
        }

        // add models specified in device context
        //std::cout << "Search models: " << name << std::endl;
        std::vector<DummyModel> modellist = dev._modelMap[frame->getName()];
        //std::cout << "Nr of Models in list: " << modellist.size() << std::endl;
        for( size_t j=0; j<modellist.size(); j++ ){
            addModelToFrame( modellist[j], frame , setup.tree);
        }

        // add limits to frame
        std::vector<DummyLimit> limits = dev._limitMap[frame->getName()];
        //std::cout << " nr of limits: " << limits.size() << std::endl;
        addLimitsToFrame( limits , frame);
    }

    Device* createDevice(DummyDevice &dev, DummySetup &setup) {
        Device* model = NULL;
        if( dev._type==SerialType){
            std::vector< Frame* > chain;
            // add the rest of the chain
            BOOST_FOREACH(DummyFrame& dframe, dev._frames){
                chain.push_back(createFrame( dframe , setup));
            }
            // next add the device and model properties to the frames
            addToStateStructure(chain[0]->getName(),setup);
            // lastly add any props
            BOOST_FOREACH(DummyFrame& dframe, dev._frames){
                addFrameProps(dframe, setup);
                addDevicePropsToFrame(dev, dframe.getName(), setup);
            }
            //State state( tree );
            State state = setup.tree->getDefaultState();
            model = new SerialDevice( chain.front(), chain.back(), dev.getName(), state);
            //std::cout << "serial device created!!" << std::endl;
        } else if( dev._type==ParallelType){
            // a parallel device is composed of a number of serial chains
            std::vector< std::vector<Frame*> > chains;
            std::vector<Frame*> chain;
            std::map<std::string, Frame*> addFramesMap;
            Frame *parent = createFrame( dev._frames[0] , setup ); //base
            RW_ASSERT(parent);
            chain.push_back(parent);
            addFramesMap[parent->getName()] = parent;
            for( size_t i=1; i<dev._frames.size(); i++){
                Frame *frame = NULL;
                if( dev._frames[i].getRefFrame() == dev._frames[i-1].getName() ){
                    // the last frame was parent frame
                    frame = createFrame( dev._frames[i] , setup);
                } else {
                    // a new serial leg has started, find the parent frame
                    chains.push_back( chain );
                    parent = NULL;
                    std::map<std::string,Frame*>::iterator
                        res = addFramesMap.find( dev._frames[i].getRefFrame() );

                    if ( res == addFramesMap.end() ) {
                        RW_WARN("Error: the frame \"" << dev._frames[i].getName()
                                  << "\" references to a non existing or illegal frame!!"
                                  << dev._frames[i].getRefFrame());
                        RW_ASSERT(false);
                    } else {
                        parent = (*res).second;
                        frame = createFrame( dev._frames[i] , setup);
                        chain.clear();
                        chain.push_back( parent );
                    }
                }
                chain.push_back(frame);
                //tree->addFrame(frame, parent);
                addFramesMap[frame->getName()] = frame;
                parent = frame;
            }
            //std::cout << "Remember to push back the last chain!!" << std::endl;
            chains.push_back( chain );
            // Add frames to tree
            addToStateStructure( chains[0][0]->getName(), setup);
            BOOST_FOREACH(DummyFrame& dframe, dev._frames){
                // Add properties defined in device context
                addFrameProps(dframe, setup);
                addDevicePropsToFrame(dev, dframe.getName(), setup);
            }
            // Create the parallel legs
            std::vector< ParallelLeg* > legs;
            //tree->addFrame(*(chains[0])[0]);
            for( size_t i=0;i<chains.size();i++){
                //std::cout << "chain nr: " << i << std::endl;
                legs.push_back( new ParallelLeg(chains[i]) );
            }
            // And last create ParallelDevice
            //State state( tree );
            State state = setup.tree->getDefaultState();
            model = new ParallelDevice( legs, dev.getName(), state );
            //std::cout << "parallel device created!!" << std::endl;
        } else if( dev._type==TreeType){
            RW_ASSERT( dev._frames.size()!=0 );
            //std::cout << "TreeDevice not supported yet" << std::endl;
            FrameMap frameMap;
            std::vector<Frame*> endEffectors;
            Frame *base = createFrame( dev._frames[0] , setup); //base
            frameMap[base->getName()] = base;
            Frame *child = base;
            for(size_t i=1; i<dev._frames.size(); i++ ){
                DummyFrame frame = dev._frames[i];
                FrameMap::iterator res = frameMap.find( frame.getRefFrame() );
                if( res == frameMap.end() ){
                    RW_THROW("Error: the frame \"" << dev._frames[i].getName()
                             << "\" references to a non existing or illegal frame!!"
                             << dev._frames[i].getRefFrame());
                }

                child = createFrame( dev._frames[i] , setup);
                frameMap[ child->getName() ] = child;
                //tree->addFrame(child, res->second);

                if( dev._frames[i]._type == "EndEffector" ){
                    endEffectors.push_back(child);
                }
            }
            if(endEffectors.size()==0)
                endEffectors.push_back(child);

            addToStateStructure( base->getName(), setup);
            BOOST_FOREACH(DummyFrame& dframe, dev._frames){
                // Add properties defined in device context
                RW_DEBUG("Add props to : " << dframe.getName());
                addFrameProps(dframe, setup);
                addDevicePropsToFrame(dev, dframe.getName(), setup);
            }
            // And last create TreeDevice
            State state = setup.tree->getDefaultState();
            model = new TreeDevice( base, endEffectors, dev.getName(), state );
            //std::cout << "TreeDevice created!!" << std::endl;
        } else if( dev._type==MobileType){
            std::string tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._basename;
            MovableFrame *base = new MovableFrame(tmpstr);
            setup.tree->addDAF(base, setup.tree->getRoot() );
            MovableFrame *mframe = base;
            Accessor::frameType().set(*mframe, rw::kinematics::FrameType::MovableFrame);
            MovableInitState *init = new MovableInitState(mframe,Transform3D<>::identity());
            setup.actions.push_back(init);
            setup.frameMap[tmpstr] = base;

            Transform3D<> t3d = Transform3D<>::identity();
            t3d.R() = RPY<>(0,0,Pi/2).toRotation3D();
            t3d.P()[1] = dev._axelwidth/2;
            tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._leftname;
            RevoluteJoint *left = new RevoluteJoint(tmpstr,t3d);
            setup.tree->addFrame(left,base);
            setup.frameMap[tmpstr] = left;

            t3d.P()[1] = -dev._axelwidth/2;
            tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._rightname;
            RevoluteJoint *right = new RevoluteJoint(tmpstr,t3d);
            setup.tree->addFrame(right,base);
            setup.frameMap[tmpstr] = right;

            // add properties, col models, drawables to frames
            addDevicePropsToFrame(dev, base->getName(), setup);
            addDevicePropsToFrame(dev, left->getName(), setup);
            addDevicePropsToFrame(dev, right->getName() , setup);

            //std::cout << "Nr of frames in device: " << dev._frames.size() << std::endl;
            BOOST_FOREACH(DummyFrame& dframe,dev._frames){
                std::string pname = dframe.getRefFrame();

                std::map<std::string, Frame*>::iterator res = setup.frameMap.find(pname);
                if( res == setup.frameMap.end() ){
                    RW_THROW("Error: parent " << pname << " not found");
                }
                //std::cout << "Parent is: " << res->second->getName() << std::endl;
                createFrame(dframe , setup);
            }
            addToStateStructure(base, setup);
            addToStateStructure(left, setup);
            addToStateStructure(right, setup);

            BOOST_FOREACH(DummyFrame& dframe,dev._frames){
                // Add properties defined in device context
                addFrameProps(dframe, setup);
                addDevicePropsToFrame(dev, dframe.getName(), setup);
            }

            State state = setup.tree->getDefaultState();
            model = new MobileDevice( base, left, right, state, dev.getName() );
        } else if( dev._type==CompositeType ){
            RW_THROW("CompositeDevice not supported yet");
        } else {
            RW_THROW("Error: Unknown device type!!");
        }

        setup.devMap[dev.getName()] = model;
        // copy all collision setups from device to global collision setup container
        ColSetupList::iterator colsetup = dev._colsetups.begin();
        for(; colsetup != dev._colsetups.end(); ++colsetup){
            //std::cout << "Colsetup: " << (*colsetup)._filename << std::endl;
            setup.colsetups.push_back( *colsetup );
        }

        // add all configurations, add home configs to default state
        BOOST_FOREACH(QConfig& config, dev._qconfig){
            if(config.name == "Home") {
                rw::math::Q q(config.q.size());
                for(size_t i = 0; i < q.size(); i++)
                    q[i] = config.q[i];

                DeviceInitState *initDevState = new DeviceInitState(q,model);
                setup.actions.push_back(initDevState);
            }
        }

        return model;
    }

    CollisionSetup defaultCollisionSetup(const WorkCell& workcell)
    {
        // We build a list of frames
        std::list<Frame*> frameList;
        std::stack<Frame*> frameStack;
        frameStack.push(workcell.getWorldFrame());
        while(0 != frameStack.size()){
            Frame* frame = frameStack.top();
            frameStack.pop();

            for (Frame::iterator it = frame->getChildren().first;
                 it != frame->getChildren().second;
                 ++it)
            {
                frameStack.push(&*it);
                frameList.push_back(&*it);
            }
        }

        // Add frames to exclude list
        ProximityPairList excludeList;
        std::list<Frame*>::reverse_iterator rit;
        std::list<Frame*>::iterator it;
        for(rit=frameList.rbegin(); rit!=frameList.rend();rit++ ){

            for(it = frameList.begin(); (*it) != (*rit); it++){

                // Do not check a child against a parent geometry
                Frame* parent1 = (*it)->getParent(); // Link N
                Frame* parent2 = (*rit)->getParent(); // Link N+1

                if(parent1 && parent2 && parent2->getParent()!=NULL){
                    if(parent2->getParent() == parent1){
                        excludeList.push_back(
                            ProximityPair((*rit)->getName(), (*it)->getName()));
                    }
                }

                // Do not check a child agains its parent
                if((*it)->getParent() == (*rit) || (*rit)->getParent() == (*it) ){
                    excludeList.push_back(
                        ProximityPair((*rit)->getName(), (*it)->getName()));
                }
            }
        }
        return CollisionSetup(excludeList);
    }
}

std::auto_ptr<rw::models::WorkCell> XMLRWLoader::loadWorkCell(
    const std::string& filename)
{
    RW_DEBUG(" ******* Loading workcell from \"" << filename << "\" ");

    // container for actions to execute when all frames and devices has been loaded
    DummySetup setup;

    // Start parsing workcell
    //boost::shared_ptr<DummyWorkcell> workcell = XMLRWParser::parseWorkcell(filename);
    setup.dwc =  XMLRWParser::parseWorkcell(filename);

    // Now build a workcell from the parsed results
    setup.tree = new StateStructure();
    setup.world = setup.tree->getRoot();
    setup.frameMap[setup.world->getName()] = setup.world;

    // first create all frames defined in the workcell
    for(size_t i=0; i< setup.dwc->_framelist.size(); i++){
        createFrame( setup.dwc->_framelist[i], setup);
    }

    // next add the frames to the StateStructure, starting with world
    addToStateStructure(setup.world,setup);

    // and lastly all properties can be added
    for(size_t i=0; i< setup.dwc->_framelist.size(); i++){
        addFrameProps( setup.dwc->_framelist[i], setup );
    }


    // Now create all devices
    for(size_t i=0; i<setup.dwc->_devlist.size(); i++){
        createDevice( setup.dwc->_devlist[i] , setup);
    }

    // remember to add all models defined in the workcell to the frames
    for(size_t i=0; i<setup.dwc->_models.size(); i++){
        std::map<std::string, Frame*>::iterator parent =
            setup.frameMap.find( setup.dwc->_models[i]._refframe );
        if( parent == setup.frameMap.end() ){
            RW_THROW("Model \"" << setup.dwc->_models[i]._name << "\" "
                     "will not be loaded since it refers to an non existing frame!!");
        }
        addModelToFrame( setup.dwc->_models[i], (*parent).second, setup.tree);
    }
    State defaultState = setup.tree->getDefaultState();

    // now add any daf frames to their respectfull parent frames
    for(size_t i=0; i<setup.dwc->_framelist.size(); i++){
        DummyFrame &dframe = setup.dwc->_framelist[i];
        if( !dframe._isDaf )
            continue;
        std::map<std::string, Frame*>::iterator parent =
            setup.frameMap.find(dframe.getRefFrame());
        if( parent == setup.frameMap.end() ){
            RW_THROW("Frame \"" << dframe.getName() << "\" "
                      << "will not be loaded since it refers to an non existing frame!!"
                      << " refframe: \"" << dframe.getRefFrame()<< "\"");
        }
        Frame *frame = setup.frameMap[dframe.getName()];
        frame->attachTo((*parent).second, defaultState);
    }

    // and then add devices to their respectfull parent frames
/*    for(size_t i=0; i<setup.dwc->_devlist.size(); i++){
        std::map<std::string, Frame*>::iterator parent =
            frameMap.find( workcell->_devlist[i].getRefFrame() );
        if( parent == frameMap.end() ){
            std::cout << "Warning: Device \"" << workcell->_devlist[i].getName() << "\" "
                         "will not be loaded since it refers to an non existing frame!!" << std::endl;
            continue;
        }
        std::map<std::string, Device*>::iterator dev =
            devMap.find( workcell->_devlist[i].getName() );
        //tree->setDafParent( *((*dev).second->getBase()), *(*parent).second );
        (*dev).second->getBase()->attachTo((*parent).second, defaultState);
    }
*/

    // add collision models from workcell
    for(size_t i=0; i<setup.dwc->_colmodels.size(); i++){
        setup.colsetups.push_back( setup.dwc->_colmodels[i] );
    }

    // now initialize state with init actions and remember to add all devices
    //State state( tree );
    //State state = tree->getDefaultState();

    // now initialize state with initial actions
    std::vector<InitialAction*>::iterator action = setup.actions.begin();
    for(;action!=setup.actions.end();++action){
    	(*action)->setInitialState(defaultState);
    	delete (*action);
    }

    setup.tree->setDefaultState(defaultState);

    // Create WorkCell
    std::auto_ptr<WorkCell> wc(new WorkCell(setup.tree, filename));

    // add devices to workcell
    std::map<std::string, Device*>::iterator first = setup.devMap.begin();
    for(;first!=setup.devMap.end();++first){
        wc->addDevice( (*first).second );
    }

    // add collision setup from files
    CollisionSetup collisionSetup;
    ColSetupList::iterator colsetupIter = setup.colsetups.begin();
    for(; colsetupIter!=setup.colsetups.end(); ++colsetupIter ){
        std::string prefix = createScopedName("", (*colsetupIter )._scope);
        std::string filename = StringUtil::getDirectoryName( (*colsetupIter )._pos.file );
        filename += "/" + (*colsetupIter)._filename;
        //std::cout << "Colsetup prefix: " << prefix << std::endl;
        //std::cout << "Colsetup file  : " << filename << std::endl;
        CollisionSetup s = CollisionSetupLoader::load(prefix, filename );
        collisionSetup.merge(s);
    }
    // in case no collisionsetup info is supplied
    if( setup.colsetups.size()==0 ){
        collisionSetup = defaultCollisionSetup(*wc);
    }

    Accessor::collisionSetup().set( *setup.world, collisionSetup );

    return wc;
}
