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

#include "WorkCellScene.hpp"

#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <rw/graphics/DrawableNodeClone.hpp>

#include <boost/foreach.hpp>

#include <vector>
#include <stack>

using namespace rw::math;
using namespace rw::graphics;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::sensor;

//----------------------------------------------------------------------------
namespace
{
    bool has(const std::string& name, GroupNode::Ptr& node){
        BOOST_FOREACH(SceneNode::Ptr dnode, node->_childNodes){
            if( name==dnode->getName() )
                return true;
        }
        return false;
    }


}

//----------------------------------------------------------------------------


WorkCellScene::WorkCellScene(SceneGraph::Ptr scene):
        _scene(scene),
        _fk(NULL),
        _worldNode( scene->makeGroupNode("World"))
{
    _scene->addChild(_worldNode, _scene->getRoot());
    _frameAxis = _scene->makeDrawableFrameAxis("FrameAxis", 0.25, DrawableNode::Virtual);
    setWorkCell(NULL);
}

WorkCellScene::~WorkCellScene()
{
    clearCache();
}

void WorkCellScene::clearCache()
{
    _frameNodeMap.clear();
}

void WorkCellScene::draw(SceneGraph::RenderInfo& info){
    _scene->draw(info);
}

void WorkCellScene::workCellChangedListener(int){
    State state = _wc->getDefaultState();
    updateSceneGraph( state );
}


void WorkCellScene::setWorkCell(rw::models::WorkCell::Ptr wc){
    if(_wc==wc)
        return;

    // TODO: if the workcell name matches the old one then see if we can transfer states of frames to the next workcell
    std::map<std::string, FrameVisualState> fnameToStateMap;
    if(wc!=NULL && _wc!=NULL && wc->getName()==_wc->getName()){
        typedef std::pair<Frame* const,FrameVisualState> StateMapData;
        BOOST_FOREACH(StateMapData& data, _frameStateMap){
            fnameToStateMap[ data.first->getName() ] = data.second;
        }
    }

    _frameStateMap.clear();
    _frameDrawableMap.clear();
    _nodeFrameMap.clear();
    _frameNodeMap.clear();
    _wc = wc;
    _scene->getRoot()->removeChild(_worldNode);

    if( wc != NULL ){
        _wc->workCellChangedEvent().add(boost::bind(&WorkCellScene::workCellChangedListener, this, _1), this);
        State state = _wc->getDefaultState();
        updateSceneGraph( state );
    } else {
        //std::cout << "************************* CLEAR *****************************" << std::endl;
        _worldNode = _scene->makeGroupNode("World");
        _scene->addChild(_worldNode, _scene->getRoot());
    }

    if(wc!=NULL){
        typedef std::pair<std::string ,FrameVisualState> StateStringMapData;
        BOOST_FOREACH(StateStringMapData data, fnameToStateMap){
            Frame *frame = _wc->findFrame(data.first);
            if(frame!=NULL){
                setVisible(data.second.visible, frame);
                setFrameAxisVisible(data.second.frameAxisVisible, frame);
                setHighlighted(data.second.highlighted, frame);
                setTransparency( data.second.alpha, frame );
                setDrawMask( data.second.dmask, frame );
                setDrawType( data.second.dtype, frame );
            }
        }
    }

}

rw::models::WorkCell::Ptr WorkCellScene::getWorkCell(){
    return _wc;
}

void WorkCellScene::setState(const rw::kinematics::State& state){
    _fk.reset(state);

    // iterate through all frame-node pairs and set the node transformations accordingly
    BOOST_FOREACH(FrameNodeMap::value_type data, _frameNodeMap){
        if( (data.first!=NULL) && (data.second!=NULL)){
            // 1. we update the transform of each GroupNode
            data.second->setTransform( data.first->getTransform(state) );

            // 2. also make sure that all parent relationships are updated
            if( data.first!=_wc->getWorldFrame() && Kinematics::isDAF(data.first) ){
                Frame *parent = data.first->getParent(state);
                if( !data.second->hasParent( _frameNodeMap[parent] ) ){
                    // 1. we remove the child from its parent
                    std::list<SceneNode::Ptr> pnodes = data.second->_parentNodes;
                    BOOST_FOREACH(SceneNode::Ptr parentNode, pnodes){
                        if(parentNode->asGroupNode()){
                            parentNode->asGroupNode()->removeChild( data.second );
                        }
                    }
                    // 2. and add the daf to its new parent
                    _scene->addChild( data.second , _frameNodeMap[parent]);

                }
            }
        }
    }

    // also iterate through all deformable objects and update their geometries
    typedef std::map< rw::models::DeformableObject::Ptr, std::vector<Model3D::Ptr>  > DeformObjectMap;
    BOOST_FOREACH( DeformObjectMap::value_type data, _deformableObjectsMap ){
    	BOOST_FOREACH( Model3D::Ptr model, data.second){
    		data.first->update( model, state );
    	}
    }

}

GroupNode::Ptr WorkCellScene::getWorldNode(){
    return _worldNode;
}

void WorkCellScene::updateSceneGraph(rw::kinematics::State& state){
    // here we find all drawables that belong to frames and order them according to translucency
    _fk.reset(state);
    // first check that the WORLD frame is in the scene, if its not add it
    _frameNodeMap[ NULL ] = _scene->getRoot(); // for world frame parent
    _nodeFrameMap[_scene->getRoot()] = NULL;
    RW_ASSERT(_scene->getRoot()!=NULL);
    std::stack<Frame*> frames;
    frames.push( _wc->getWorldFrame() );

    while(!frames.empty()){
        Frame *frame = frames.top();
        frames.pop();
        //std::cout << "Frame: " << frame->getName() << std::endl;
        // make sure that the frame is in the scene
        if( _frameNodeMap.find(frame)==_frameNodeMap.end() ){
            GroupNode::Ptr parentNode = _frameNodeMap[frame->getParent(state)];
            // frame is not in the scene, add it.
            GroupNode::Ptr node = _scene->makeGroupNode(frame->getName());
            _scene->addChild( node, parentNode );
            _frameNodeMap[ frame ] = node;
            _nodeFrameMap[node] = frame;
        }

        // the frame is there, check that the parent relationship is correct
        GroupNode::Ptr node = _frameNodeMap[frame];
        _nodeFrameMap[node] = frame;
        GroupNode::Ptr parentNode = _frameNodeMap[frame->getParent(state)];

        if(node==NULL){
            RW_WARN("Node is null!");
            continue;
        }
        if(parentNode==NULL){
            RW_WARN("ParentNode is null!");
            continue;
        }
        /*
        if( !node->hasParent( parentNode ) ){
            // find any parent node that is a frame and remove it

            std::vector<SceneNode::Ptr> nodesToDelete;
            BOOST_FOREACH(SceneNode::Ptr np, node->_parentNodes){
                GroupNode *gnode = np->asGroupNode();
                if(gnode ){
                    if(_nodeFrameMap.find(np.cast<GroupNode>())!=_nodeFrameMap.end())
                        nodesToDelete.push_back(np);
                }
            }
            BOOST_FOREACH(SceneNode::Ptr np, nodesToDelete){
                if( GroupNode* gnode = np->asGroupNode()){
                    gnode->removeChild(node);
                    node->removeParent(np);
                }
            }
            GroupNode::addChild(node, parentNode);
        }

        if( !parentNode->hasChild(node) ){
            parentNode->addChild(node);
        }
        */

        // now for each DrawableInfo on frame check that they are on the frame

        // The information of drawables is located in SceneDescriptor and "model::Object"s
        // first check if there is any object with a frame "frame" in it
        std::vector<Object::Ptr> objs = _wc->getObjects();
        State state = _wc->getDefaultState();

        BOOST_FOREACH(Object::Ptr obj, objs){
        	// just test if this object has anything to do with the frame
        	if(obj->getBase()!=frame)
				continue;


	        // check if obj is deformable object
	    	if( DeformableObject::Ptr dobj = obj.cast<DeformableObject>() ){
	    		// make deep copy of the models.
	    		std::vector<Model3D::Ptr> models = dobj->getModels(state);
	    		// update the models with current state
	    		_deformableObjectsMap[dobj] =  models;
				BOOST_FOREACH(Model3D::Ptr model, models){
					DrawableNode::Ptr dnode = _scene->makeDrawable(model->getName(), model, model->getMask() );
					_scene->addChild(dnode, node);
					_frameDrawableMap[frame].push_back( dnode );
				}

	    	} else {

				// the collision info is the geometry and visualization is Model3D
				// in case no models are available the collision geometry will be used
				std::vector<Geometry::Ptr> geoms = obj->getGeometry(state);
				BOOST_FOREACH(Geometry::Ptr geom, geoms){
					if(geom->getFrame()!=frame)
						continue;

					DrawableGeometryNode::Ptr dnode = _scene->makeDrawable(geom->getName(), geom, geom->getMask() );
					_scene->addChild(dnode, node);
					_frameDrawableMap[frame].push_back( dnode );
				}

				BOOST_FOREACH(Model3D::Ptr model, obj->getModels()){
					DrawableNode::Ptr dnode = _scene->makeDrawable(model->getName(), model, model->getMask() );
					_scene->addChild(dnode, node);
					_frameDrawableMap[frame].push_back( dnode );
				}
	    	}
		}


        Frame::iterator_pair iter = frame->getChildren(state);
        for(;iter.first!=iter.second; ++iter.first ){
            Frame* child = &(*iter.first);
            frames.push(child);
        }
    }

    //std::cout << "_opaqueDrawables: " << _opaqueDrawables.size()  << std::endl;
    //std::cout << "_translucentDrawables: " << _translucentDrawables.size()  << std::endl;
    _worldNode = _frameNodeMap[_wc->getWorldFrame()];
}


void WorkCellScene::setVisible( bool visible, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return;
    _frameStateMap[f].visible = visible;
    BOOST_FOREACH(SceneNode::Ptr& d, _frameNodeMap[f]->_childNodes){
        if(DrawableNode *dnode = d->asDrawableNode() )
            dnode->setVisible(visible);
    }
}

bool WorkCellScene::isVisible(rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return false;
    return _frameStateMap[f].visible;
}

void WorkCellScene::setHighlighted( bool highlighted, rw::kinematics::Frame* f){
    if(_frameDrawableMap.find(f)==_frameDrawableMap.end()){
        //std::cout << "frame not found!" << std::endl;
        return;
    }

    _frameStateMap[f].highlighted = highlighted;

    BOOST_FOREACH(DrawableNode::Ptr& d, _frameDrawableMap[f]){
        d->setHighlighted(highlighted);
    }
}

bool WorkCellScene::isHighlighted( rw::kinematics::Frame* f) {
    return _frameStateMap[f].highlighted;
}

void WorkCellScene::setFrameAxisVisible( bool visible, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end()){
        return;
    }
    _frameStateMap[f].frameAxisVisible = visible;
    GroupNode::Ptr node = _frameNodeMap[f];
    if( visible ){
        if( node->hasChild( "FrameAxis" ) ){
            // the frame axis is allready there
            return;
        }
        // if the frame axis is allready on a node, then do nothing, else add it

        DrawableNodeClone::Ptr dnode = ownedPtr( new DrawableNodeClone("FrameAxis",_frameAxis) );
        addDrawable(dnode, f);
        //_frameDrawableMap[f].push_back(dnode);
        //node->addChild( dnode );
        _scene->update();
    } else if( !visible && node->hasChild( "FrameAxis" ) ){
        // remove leaf
        //node->removeChild( "FrameAxis" );
        removeDrawable("FrameAxis", f);
        _scene->update();

    }

}

bool WorkCellScene::isFrameAxisVisible( rw::kinematics::Frame* f) {
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].frameAxisVisible;
}

void WorkCellScene::setDrawType( DrawableNode::DrawType type, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return;
    _frameStateMap[f].dtype = type;
    BOOST_FOREACH(DrawableNode::Ptr& d, _frameDrawableMap[f]){
        d->setDrawType(type);
    }
}

DrawableNode::DrawType WorkCellScene::getDrawType( rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].dtype;
}

void WorkCellScene::setDrawMask( unsigned int mask, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        RW_THROW("Frame is not in the scene!");
    _frameStateMap[f].dmask = mask;
    BOOST_FOREACH(DrawableNode::Ptr& d, _frameDrawableMap[f]){
        d->setMask(mask);
    }
}

unsigned int WorkCellScene::getDrawMask(rw::kinematics::Frame* f ){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        RW_THROW("Frame is not in the scene!");
    return _frameStateMap[f].dmask;
}


void WorkCellScene::setTransparency( double alpha, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return;

    _frameStateMap[f].alpha = alpha;
    BOOST_FOREACH(DrawableNode::Ptr& d, _frameDrawableMap[f]){
        d->setTransparency( (float) alpha );
    }
}
namespace {
    void addFrameNode(Frame *frame, SceneGraph::Ptr scene){

    }
}
void WorkCellScene::addDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame* frame){
    if(_wc==NULL)
        RW_THROW("Scene is not initialized with WorkCell yet! Drawable cannot be attached to Frame.");
    // add frame to frame map
    _frameDrawableMap[frame].push_back(drawable);

    // get or create the
    if( _frameNodeMap.find(frame) == _frameNodeMap.end() ){

        // the frame is not in the scene, add it
        State state = _wc->getDefaultState();

        Frame *p = frame->getParent(state);

        std::stack<Frame*> frames;
        frames.push(frame);
        while(_frameNodeMap.find(p) == _frameNodeMap.end()){
            frames.push(p);
            p = p->getParent(state);
            RW_ASSERT(p!=NULL);
        }

        while(!frames.empty()){

            Frame *f = frames.top();
            frames.pop();

            // now f's parent is supposed to have a node allready
            RW_ASSERT(_frameNodeMap.find( f->getParent(state) ) != _frameNodeMap.end());

            GroupNode::Ptr pnode = _frameNodeMap[f->getParent(state)];
            GroupNode::Ptr child = _scene->makeGroupNode(f->getName());
            GroupNode::addChild(child,pnode);
            _frameNodeMap[f] = child;
            _nodeFrameMap[child] = f;
        }
    }

    _frameNodeMap[frame]->addChild(drawable);

}

DrawableNode::Ptr WorkCellScene::addDrawable(const std::string& filename, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawable(filename, dmask);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableNode::Ptr WorkCellScene::addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawableFrameAxis(name,size,dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableGeometryNode::Ptr WorkCellScene::addGeometry(const std::string& name,rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask){
    DrawableGeometryNode::Ptr drawable = _scene->makeDrawable(name, geom);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableNode::Ptr WorkCellScene::addModel3D(const std::string& name, Model3D::Ptr model, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawable(name, model);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableNode::Ptr WorkCellScene::addImage(const std::string& name,const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawable(name, img);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableNode::Ptr WorkCellScene::addScan(const std::string& name,const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawable(name, scan);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableNode::Ptr WorkCellScene::addScan(const std::string& name,const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask){
    DrawableNode::Ptr drawable = _scene->makeDrawable(name, scan);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

DrawableGeometryNode::Ptr WorkCellScene::addLines(const std::string& name,const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask){
    DrawableGeometryNode::Ptr drawable = _scene->makeDrawable(name, lines);
    if(drawable==NULL)
        return drawable;
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}


DrawableNode::Ptr WorkCellScene::addRender(const std::string& name,rw::graphics::Render::Ptr render, rw::kinematics::Frame* frame, int dmask) {
    DrawableNode::Ptr drawable = _scene->makeDrawable(name, render);
    drawable->setMask(dmask);
    addDrawable(drawable, frame);
    return drawable;
}

std::vector<DrawableNode::Ptr> WorkCellScene::getDrawables(){
    return _scene->getDrawables();
}

std::vector<DrawableNode::Ptr> WorkCellScene::getDrawables(rw::kinematics::Frame* f){
    if(_frameDrawableMap.find(f)==_frameDrawableMap.end())
        return std::vector<DrawableNode::Ptr>();
    return _frameDrawableMap[f];
}

std::vector<DrawableNode::Ptr> WorkCellScene::getDrawablesRec(rw::kinematics::Frame* f, rw::kinematics::State&){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return std::vector<DrawableNode::Ptr>();
    return _scene->getDrawablesRec(_frameNodeMap[f]);
}


DrawableNode::Ptr WorkCellScene::findDrawable(const std::string& name){
    return _scene->findDrawable(name);
}
DrawableNode::Ptr WorkCellScene::findDrawable(const std::string& name, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return NULL;
    return _scene->findDrawable(name, _frameNodeMap[f]);
}

std::vector<DrawableNode::Ptr> WorkCellScene::findDrawables(const std::string& name){
    return _scene->findDrawables(name);
}

bool WorkCellScene::removeDrawable(DrawableNode::Ptr drawable){
    return _scene->removeDrawable(drawable);
}

bool WorkCellScene::removeDrawables(rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return false;
    return _scene->removeDrawables(_frameNodeMap[f]);
}

bool WorkCellScene::removeDrawable(const std::string& name){
    return _scene->removeDrawable(name);
}

bool WorkCellScene::removeDrawables(const std::string& name){
    return _scene->removeDrawables(name);
}

bool WorkCellScene::removeDrawable(DrawableNode::Ptr drawable, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return false;
    return _scene->removeDrawable(drawable, _frameNodeMap[f]);
}

bool WorkCellScene::removeDrawable(const std::string& name, rw::kinematics::Frame* f){
    if(_frameNodeMap.find(f)==_frameNodeMap.end())
        return false;
    return _scene->removeChild(name, _frameNodeMap[f]);
}

rw::kinematics::Frame* WorkCellScene::getFrame(DrawableNode::Ptr d){
    //std::cout << d->_parentNodes.size() << std::endl;
    GroupNode::Ptr gn = d->_parentNodes.front().cast<GroupNode>();
    if(gn==NULL) {
		RW_WARN("Group Node is NULL");
        return NULL;
	}
    return _nodeFrameMap[gn];
}
