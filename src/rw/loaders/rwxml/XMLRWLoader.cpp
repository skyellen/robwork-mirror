#include "XMLRWLoader.hpp"

#include "XMLRWParser.hpp"
//#include "XMLRWPreParser.hpp"
#include "XMLParserUtil.hpp"


#include <rw/kinematics/Tree.hpp>
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
    
    bool isIdentity(rw::math::Transform3D<>& t3d){
        if( ( fabs( t3d(0,0) - 1)< 0.000001) && 
            ( fabs( t3d(1,1) - 1)< 0.000001) &&
            ( fabs( t3d(2,2) - 1)< 0.000001) &&
            ( fabs( t3d(0,1) ) < 0.000001 )  &&
            ( fabs( t3d(0,2) ) < 0.000001 )  &&
            ( fabs( t3d(0,3) ) < 0.000001 )  &&
            ( fabs( t3d(1,0) ) < 0.000001 )  &&
            ( fabs( t3d(1,2) ) < 0.000001 )  &&
            ( fabs( t3d(1,3) ) < 0.000001 )  &&
            ( fabs( t3d(2,0) ) < 0.000001 )  &&
            ( fabs( t3d(2,1) ) < 0.000001 )  &&
            ( fabs( t3d(2,3) ) < 0.000001 )  &&
            ( fabs( t3d(3,0) ) < 0.000001 )  &&
            ( fabs( t3d(3,1) ) < 0.000001 )  &&
            ( fabs( t3d(3,2) ) < 0.000001 ) )
                return true;
        return false;
    }

    
    
    Frame* addModelToFrame( DummyModel& model, Frame *parent ){        
        // test if identity
        Frame *modelframe;
        std::string name = createScopedName( model._name, model._scope );
        modelframe = new FixedFrame(parent, name, model._transform);

        for( size_t i=0; i<model._geo.size(); i++) {
            std::ostringstream val;
            
            switch(model._geo[i]._type){
                case PolyType:
                    if( ! StringUtil::IsAbsoluteFileName( model._geo[i]._filename + ".tmp" ) ) {
                        val << StringUtil::GetDirectoryName( model._geo[i]._pos.file );
                    }
                    val << model._geo[i]._filename;
                    break;
                case CubeType: 
                    val << "#Cube " << model._geo[i]._x << " "
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
            
            Accessor::FrameType().set( *modelframe, rw::kinematics::FrameType::FixedFrame );
            if( model._isDrawable ){
                Accessor::DrawableID().set( *modelframe, val.str() );
                if(model._colmodel)
                    Accessor::CollisionModelID().set( *modelframe, val.str() );
            } else {
            	Accessor::CollisionModelID().set( *modelframe, val.str() );
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
        
    Frame *createFrame(DummyFrame& dframe, 
                       Frame *parent, 
                       std::map<std::string, Frame*> &frameMap,
                       std::vector<InitialAction*> &actions){
        Frame *frame = NULL;
        //std::cout << "Depend: " << dframe._isDepend << std::endl; 
        if( dframe._isDepend ){
            std::map<std::string, Frame*>::iterator res = frameMap.find(dframe.getDependsOn());
            if( res == frameMap.end() ){
                std::cout << "Error: The frame: " << dframe.getName() 
                          << " Depends on an unknown frame: " 
                          << dframe.getDependsOn() << std::endl;
                return NULL;
            }
            // then the frame depends on another joint
            Joint* owner = dynamic_cast<Joint*>( (*res).second );
            if( owner==NULL ){
                std::cout << "Error: The frame: " << dframe.getName() 
                          << " Depends on an frame: " 
                          << dframe._dependsOn << " which is not a Joint" 
                          << std::endl;
                return NULL;
            }
            
            if( dframe._type == "Revolute" ){
                frame = new PassiveRevoluteFrame(parent, dframe.getName(),
                                                 dframe._transform,
                                                 owner, dframe._gain,
                                                 dframe._offset);
                
                //std::cout << "Passive Revolute joint: " << dframe._gain << " " 
                //          << dframe._offset << std::endl;
            } else if( dframe._type == "Prismatic" ){
                frame = new PassivePrismaticFrame(parent, dframe.getName(),
                                                 dframe._transform,
                                                 owner, dframe._gain,
                                                 dframe._offset);
                //std::cout << "Passive prismatic joint: " << dframe._gain << " " 
                //          << dframe._offset << std::endl;
            }  else {
                std::cout << "Error: The type of frame: " << dframe.getName()
                          << " cannot depend on another joint!!" << std::endl;
                return NULL;
            }
            
        } else if( dframe._type == "Fixed" ){
            frame = new FixedFrame(parent, dframe.getName(), dframe._transform );
            Accessor::FrameType().set(*frame, rw::kinematics::FrameType::FixedFrame);
        } else if( dframe._type == "Movable") {
            MovableFrame *mframe = new MovableFrame( parent, dframe.getName() );
            frame = mframe;
            Accessor::FrameType().set(*mframe, rw::kinematics::FrameType::MovableFrame);
            MovableInitState *init = new MovableInitState(mframe,dframe._transform);
            actions.push_back(init);
        } else if( dframe._type == "Prismatic") {
            PrismaticJoint *j = new PrismaticJoint( parent, dframe.getName(), dframe._transform );
            addLimits( dframe._limits, j );
            frame = j;
            Accessor::FrameType().set(*frame, rw::kinematics::FrameType::PrismaticJoint);
            if( dframe._state == ActiveState)  
                Accessor::ActiveJoint().set(*frame, true);
            //std::cout << "Prismatic joint!! " << j->getName() << std::endl;
        } else if( dframe._type == "Revolute") {
            RevoluteJoint *j = new RevoluteJoint( parent, dframe.getName(), dframe._transform );
            addLimits( dframe._limits, j );
            frame = j;
            Accessor::FrameType().set(*frame, rw::kinematics::FrameType::RevoluteJoint);
            if( dframe._state == ActiveState)  
                Accessor::ActiveJoint().set(*frame, true);
        } else if( dframe._type == "EndEffector" ){
            frame = new FixedFrame(parent, dframe.getName(), dframe._transform );
            Accessor::FrameType().set(*frame, rw::kinematics::FrameType::FixedFrame);
        } else {
            std::cout << "FRAME is of illegal type!! " << dframe._type << std::endl;
            assert(true);
        }
        //std::cout << "Nr of properties in frame: " << dframe._properties.size() << std::endl; 
        for(size_t i=0; i<dframe._properties.size(); i++){
            frame->getPropertyMap().addProperty(
                createStringProperty( dframe._properties[i] ) );
        }
        //std::cout << "Nr of models in frame: " << dframe._models.size() << std::endl;
        for(size_t i=0; i<dframe._models.size(); i++){
            addModelToFrame( dframe._models[i], frame);
        }        
        // remember to add the frame to the frame map
        frameMap[frame->getName()] = frame;
        //std::cout << "Frame created!! " << frame->getName() << std::endl;
        return frame;
    }

    void addFrameTree( Frame *frame, boost::shared_ptr<Tree> tree){
        tree->addFrame( frame );
        ConcatVectorIterator<Frame> iter = frame->getChildren().first;
        for(; iter != frame->getChildren().second ; ++iter) {
            addFrameTree( &(*iter) , tree );
        }
    }
    
    /**
     * @brief adds all device context defined properties to the frame 
     */
    void addDevicePropsToFrame(DummyDevice &dev, const std::string& name, Frame &frame){
        // add properties specified in device context
        //std::cout << "Search props: " << name << std::endl;
        std::vector<boost::shared_ptr<rw::common::Property<std::string> > > proplist = 
            dev._propMap[name];
        //std::cout << "Nr Props in list: " << proplist.size() << std::endl;
        for( size_t j=0; j<proplist.size(); j++ ){
            frame.getPropertyMap().addProperty( proplist[j] );
        }
        
        // add models specified in device context
        //std::cout << "Search models: " << name << std::endl;
        std::vector<DummyModel> modellist = dev._modelMap[name];
        //std::cout << "Nr of Models in list: " << modellist.size() << std::endl;
        for( size_t j=0; j<modellist.size(); j++ ){
            addModelToFrame( modellist[j], &frame  );
        }
        
        // add limits to frame
        std::vector<DummyLimit> limits = dev._limitMap[name];
        //std::cout << " nr of limits: " << limits.size() << std::endl;
        addLimitsToFrame( limits , &frame);        
    }

    
    DeviceModel* createDevice(DummyDevice &dev,
                              boost::shared_ptr<Tree> tree,
                              std::map<std::string, Frame*> &frameMapGlobal,
                              std::vector<InitialAction*> &actions){
        DeviceModel* model = NULL;
        if( dev._type==SerialType){
            Frame *parent = NULL;
            std::vector< Frame* > chain;
            for( size_t i=0; i<dev._frames.size(); i++){
                parent = createFrame( dev._frames[i] , parent , frameMapGlobal, actions);
                chain.push_back( parent );
                // Add properties defined in device context
                addDevicePropsToFrame(dev, dev._frames[i].getName(), *parent);
            }
            addFrameTree(chain[0], tree );
            State state( tree );
            model = new SerialDevice( chain.front(), chain.back(), dev.getName(), state);
            //std::cout << "serial device created!!" << std::endl;
        } else if( dev._type==ParallelType){
            Frame *parent = NULL;
            //std::cout << "Creating parallel device!!" << std::endl;
            // a parallel device is composed of a number of serial chains
            std::vector< std::vector<Frame*> > chains;
            std::vector<Frame*> chain;
            std::map<std::string, Frame*> addFramesMap;
            parent = createFrame( dev._frames[0] , NULL , frameMapGlobal, actions ); //base
            if( parent == NULL ){
                std::cout << "Error: Parent is NULL" << std::endl;
                assert(false);
            }
            addDevicePropsToFrame(dev, dev._frames[0].getName(), *parent);
            //tree->addFrame(*parent);
            chain.push_back(parent);
            addFramesMap[parent->getName()] = parent;
            for( size_t i=1; i<dev._frames.size(); i++){
                //std::cout << dev._frames[i].getRefFrame() << "  ==  " << dev._frames[i-1].getName() << std::endl;
                if( dev._frames[i].getRefFrame() == dev._frames[i-1].getName() ){
                    parent = createFrame( dev._frames[i] , parent , frameMapGlobal, actions);
                } else {
                    chains.push_back( chain );
                    parent = NULL;
                    std::map<std::string,Frame*>::iterator
                        res = addFramesMap.find( dev._frames[i].getRefFrame() );
                    
                    if ( res == addFramesMap.end() ) {
                        std::cout << "Error: the frame \"" << dev._frames[i].getName()
                                  << "\" references to a non existing or illegal frame!!"
                                  << dev._frames[i].getRefFrame() << std::endl;
                        continue;
                    } else {
                        Frame* par = (*res).second;
                        parent = createFrame( dev._frames[i] , par , frameMapGlobal, actions);
                        chain.clear();
                        chain.push_back( par );
                    }
                }
                chain.push_back(parent);
                //tree->addFrame(*parent);
                addFramesMap[parent->getName()] = parent;
                // Add properties defined in device context
                addDevicePropsToFrame(dev, dev._frames[i].getName(), *parent);                
            }
            //std::cout << "Remember to push back the last chain!!" << std::endl;
            chains.push_back( chain );
            // Add frames to tree
            addFrameTree( chains[0][0], tree );
            
            // Create the parallel legs
            std::vector< ParallelLeg* > legs;
            //tree->addFrame(*(chains[0])[0]);
            for( size_t i=0;i<chains.size();i++){
                //std::cout << "chain nr: " << i << std::endl;
                legs.push_back( new ParallelLeg(chains[i]) );
            }
            // And last create ParallelDevice
            State state( tree );
            model = new ParallelDevice( legs, dev.getName(), state );
            //std::cout << "parallel device created!!" << std::endl;
        } else if( dev._type==TreeType){
            RW_ASSERT( dev._frames.size()!=0 );
            //std::cout << "TreeDevice not supported yet" << std::endl;
            FrameMap frameMap;
            std::vector<Frame*> endEffectors;
            Frame *base = createFrame( dev._frames[0] , NULL , frameMapGlobal, actions ); //base
            addDevicePropsToFrame(dev, base->getName(), *base);
            frameMap[base->getName()] = base;
            Frame *child = base;
            for(size_t i=1; i<dev._frames.size(); i++ ){
                DummyFrame frame = dev._frames[i];

                FrameMap::iterator res = frameMap.find( frame.getRefFrame() );
                if( res == frameMap.end() ){
                    std::cout << "Error: the frame \"" << dev._frames[i].getName()
                              << "\" references to a non existing or illegal frame!!"
                              << dev._frames[i].getRefFrame() << std::endl;
                    continue;
                }                
                child = createFrame( dev._frames[i] , res->second , frameMapGlobal, actions );
                frameMap[ child->getName() ] = child;
                
                if( dev._frames[i]._type == "EndEffector" ){
                    endEffectors.push_back(child);
                }

                // Add properties defined in device context
                addDevicePropsToFrame(dev, dev._frames[i].getName(), *child);                
            }
            if(endEffectors.size()==0)
                endEffectors.push_back(child);
            
            // remember to add all frames to the tree
            addFrameTree(base, tree);
            // And last create TreeDevice
            State state( tree );
            model = new TreeDevice( base, endEffectors, dev.getName(), state );
            //std::cout << "TreeDevice created!!" << std::endl;
        } else if( dev._type==MobileType){
            std::string tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._basename;
            MovableFrame *base = new MovableFrame(NULL, tmpstr);
            MovableFrame *mframe = base;
            Accessor::FrameType().set(*mframe, rw::kinematics::FrameType::MovableFrame);
            MovableInitState *init = new MovableInitState(mframe,Transform3D<>::Identity());
            actions.push_back(init);
            
            frameMapGlobal[tmpstr] = base;
            
            Transform3D<> t3d = Transform3D<>::Identity();
            t3d.R() = RPY<>(0,0,Pi/2).toRotation3D();
            t3d.P()[1] = dev._axelwidth/2;
            tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._leftname;
            RevoluteJoint *left = new RevoluteJoint(base,tmpstr,t3d);
            frameMapGlobal[tmpstr] = left;
            
            t3d.P()[1] = -dev._axelwidth/2;
            tmpstr = createScopedName(dev._name, dev._scope)+"."+dev._rightname;
            RevoluteJoint *right = new RevoluteJoint(base,tmpstr,t3d);
            frameMapGlobal[tmpstr] = right;
            
            // add properties, col models, drawables to frames
            addDevicePropsToFrame(dev, base->getName(), *base);            
            addDevicePropsToFrame(dev, left->getName(), *left);
            addDevicePropsToFrame(dev, right->getName() , *right);
            
            //std::cout << "Nr of frames in device: " << dev._frames.size() << std::endl;
            for( size_t i=0; i<dev._frames.size(); i++){
                std::string pname = dev._frames[i].getRefFrame();
                
                std::map<std::string, Frame*>::iterator res = frameMapGlobal.find(pname);
                if( res == frameMapGlobal.end() ){
                    std::cout << "Error: parent " << pname << " not found" << std::endl;
                    continue; 
                }
                //std::cout << "Parent is: " << res->second->getName() << std::endl;
                Frame *frame = createFrame( dev._frames[i] , res->second , frameMapGlobal, actions);
                // Add properties defined in device context
                addDevicePropsToFrame(dev, dev._frames[i].getName(), *frame);
            }
            
            // remember to add all frames to the tree
            addFrameTree(base, tree);
            
            State state(tree);
            model = new MobileDevice( base, left, right, state, dev.getName() );
        } else if( dev._type==CompositeType ){
            std::cout << "CompositeDevice not supported yet" << std::endl;
        } else {
            std::cout << "Error: Unknown device type!!" << std::endl;
            assert(false);
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


std::auto_ptr<rw::models::WorkCell> XMLRWLoader::LoadWorkCell(
    const std::string& filename)
{
    //std::cout << " ******* Loading workcell from \"" << filename << "\" " << std::endl;
    
    // container for collision setups, filename and scoped name
    typedef std::vector<DummyCollisionSetup> ColSetupList;
    ColSetupList colsetups;
    // container for actions to execute when all frames and devices has been loaded
    std::vector<InitialAction*> actions;

    // Start parsing workcell 
    boost::shared_ptr<DummyWorkcell> workcell = XMLRWParser::parseWorkcell(filename);

    // Now build a workcell from the parsed results
    boost::shared_ptr<Tree> tree = boost::shared_ptr<Tree>(new Tree());

    // put all frames in a frameMap
    std::map<std::string, Frame*> frameMap;
    FixedFrame *world = new FixedFrame(NULL, "World", Transform3D<>::Identity() );
    tree->addFrame(world);
    frameMap[ "World" ] = world;
    std::vector< Frame* > framelist;
    for(size_t i=0; i< workcell->_framelist.size(); i++){
        // all frames are considered DAF's
        Frame *frame = createFrame( workcell->_framelist[i], NULL, frameMap, actions);
        framelist.push_back(frame);
        addFrameTree( frame, tree);
        frameMap[ workcell->_framelist[i].getName() ] = frame;
    }

    // Create all devices
    std::map<std::string,DeviceModel*> devMap;
    for(size_t i=0; i<workcell->_devlist.size(); i++){
        DeviceModel *dev = createDevice( workcell->_devlist[i] , tree, frameMap, actions);
        if( dev == NULL )
            continue;
        devMap[workcell->_devlist[i].getName()] = dev;
        // copy all collision setups from device to global collision setup container
        DummyDevice &ddev = workcell->_devlist[i];
        ColSetupList::iterator colsetup = ddev._colsetups.begin();
        for(; colsetup != ddev._colsetups.end(); ++colsetup){
            //std::cout << "Colsetup: " << (*colsetup)._filename << std::endl;
            colsetups.push_back( *colsetup );
        }
    }
    
    // remember to add all models defined in the workcell to the frames
    for(size_t i=0; i<workcell->_models.size(); i++){
        std::map<std::string, Frame*>::iterator parent =
            frameMap.find( workcell->_models[i]._refframe );
        if( parent == frameMap.end() ){
            std::cout << "Warning: model \"" << workcell->_models[i]._name << "\" "
                         "will not be loaded since it refers to an non existing frame!!" << std::endl;
            continue;
        }
        Frame *model = addModelToFrame( workcell->_models[i], (*parent).second );
        if( model!=NULL )
            tree->addFrame(model);
    }
        
    // now add frames to their respectfull parent frames
    for(size_t i=0; i<workcell->_framelist.size(); i++){
        std::map<std::string, Frame*>::iterator parent =
            frameMap.find(workcell->_framelist[i].getRefFrame());
        if( parent == frameMap.end() ){
            std::cout << "Warning: Frame \"" << workcell->_framelist[i].getName() << "\" "
                      << "will not be loaded since it refers to an non existing frame!!" 
                      << " refframe: \"" << workcell->_framelist[i].getRefFrame()<< "\"" << std::endl;
            continue;
        }
        std::map<std::string, Frame*>::iterator frame =
            frameMap.find(workcell->_framelist[i].getName());
        tree->setDafParent( *(*frame).second , *(*parent).second );
    }
        
    // and then add devices to their respectfull parent frames
    for(size_t i=0; i<workcell->_devlist.size(); i++){
        std::map<std::string, Frame*>::iterator parent =
            frameMap.find( workcell->_devlist[i].getRefFrame() );
        if( parent == frameMap.end() ){
            std::cout << "Warning: Device \"" << workcell->_devlist[i].getName() << "\" "
                         "will not be loaded since it refers to an non existing frame!!" << std::endl;
            continue;
        }
        std::map<std::string, DeviceModel*>::iterator dev =
            devMap.find( workcell->_devlist[i].getName() );
        tree->setDafParent( *((*dev).second->getBase()), *(*parent).second );
    }
    
    // add collision models from workcell
    for(size_t i=0; i<workcell->_colmodels.size(); i++){
        colsetups.push_back( workcell->_colmodels[i] );
    }
    
    // now initialize state with init actions and remember to add all devices
    State state( tree );
    
    // now initialize state with initial actions
    std::vector<InitialAction*>::iterator action = actions.begin();
    for(;action!=actions.end();++action){
    	(*action)->setInitialState(state);
    	delete (*action);
    }
    
    // Create WorkCell
    rw::models::WorkCell *wc = new WorkCell( world , state );
    
    // add devices to workcell
    std::map<std::string, DeviceModel*>::iterator first = devMap.begin();
    for(;first!=devMap.end();++first){
        wc->addDevice( (*first).second );
    }
        
    // add collision setup from files
    CollisionSetup collisionSetup;
    ColSetupList::iterator setup = colsetups.begin();
    for(; setup!=colsetups.end(); ++setup){
        std::string prefix = createScopedName("", (*setup)._scope);
        std::string filename = StringUtil::GetDirectoryName( (*setup)._pos.file );
        filename += "/" + (*setup)._filename;
        //std::cout << "Colsetup prefix: " << prefix << std::endl;
        //std::cout << "Colsetup file  : " << filename << std::endl;
        CollisionSetup s = CollisionSetupLoader::Load(prefix, filename );
        collisionSetup.merge(s);
    }
    // in case no collisionsetup info is supplied
    if( colsetups.size()==0 ){
        collisionSetup = defaultCollisionSetup(*wc);
    }
        
    Accessor::CollisionSetup().set( *world, collisionSetup );
    
    std::auto_ptr<WorkCell> res( wc );
    
    return res;
}
