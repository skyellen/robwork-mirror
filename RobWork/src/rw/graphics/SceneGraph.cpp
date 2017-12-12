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

#include "SceneGraph.hpp"
#include "SceneCamera.hpp"

#include <boost/foreach.hpp>

#include <stack>
#include <map>

using namespace rw::graphics;
using namespace rw::common;

void SceneGraph::setRoot(GroupNode::Ptr node){
    _root = node;
}

GroupNode::Ptr SceneGraph::getRoot(){
    return _root;
}

void SceneGraph::addChild(SceneNode::Ptr child, GroupNode::Ptr parent){
    if(!parent->hasChild(child))
        GroupNode::addChild(child, parent);
}

GroupNode::Ptr SceneGraph::makeGroupNode(const std::string& name){
    return ownedPtr( new GroupNode(name) );
}

namespace {

    struct DrawableVectorVisitor{
        SceneGraph::NodeVisitor functor;
        DrawableVectorVisitor(){
            functor =  boost::ref(*this);
        }
        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( child->asDrawableNode() ){
                DrawableNode::Ptr d = child.cast<DrawableNode>();
                if(dmap.find(d.get())==dmap.end()){
                    drawables.push_back(d);
                    dmap[d.get()] = 1;
                }
            }
            return false;
        }

        std::vector<DrawableNode::Ptr> drawables;
        std::map<DrawableNode*,int> dmap; // to check if the drawable has allready been added
    };

    struct FindDrawableVisitor {
        SceneGraph::NodeVisitor functor;

        FindDrawableVisitor(std::string name, bool findall):_name(name),_findall(findall){
            functor =  boost::ref(*this);
        }

        FindDrawableVisitor(DrawableNode::Ptr d, bool findall):_findall(findall),_drawable(d){
            functor =  boost::ref(*this);
        }

        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( child->asDrawableNode() ){
                bool found = false;
                if(_drawable){
                    // we search for this specific drawable
                    if(_drawable==child)
                        found = true;
                } else if(child->getName()==_name){
                    found = true;
                }
                DrawableNode::Ptr d = child.cast<DrawableNode>();
                if( d!=NULL && found){
                    if(_findall){
                        _dnodes.push_back(d);
                        _pnodes.push_back(parent);
                    } else {
                        _dnode = d;
                        _pnode = parent;
                        return true;
                    }
                }
            }
            return false;
        }

        DrawableNode::Ptr _dnode;
        SceneNode::Ptr _pnode;
        std::vector<DrawableNode::Ptr> _dnodes;
        std::vector<SceneNode::Ptr> _pnodes;
        std::string _name;
        bool _findall;
        DrawableNode::Ptr _drawable; // the one to match
    };

    struct NodeTypeExcludeFilter {
        SceneGraph::NodeFilter functor;
        NodeTypeExcludeFilter(int type):_type(type){
            functor = boost::ref(*this);
        };
        bool operator()(const SceneNode::Ptr& child) const { return child->getType()==_type; }
        int _type;
    };

    struct StaticFilter {
        SceneGraph::NodeFilter functor;
        StaticFilter(bool retValue):_retvalue(retValue){
            functor =  boost::ref(*this);
        }
        bool operator()(const SceneNode::Ptr& child) const { return _retvalue; }
        bool _retvalue;
    };

    struct NodeP{
        NodeP():_first(NULL), _second(NULL), visited(false){}
        NodeP(SceneNode::Ptr a, SceneNode::Ptr b, bool v):_first(a),_second(b),visited(v){}

        SceneNode::Ptr _first, _second;
        bool visited;
    };
}

void SceneGraph::traverse(SceneNode::Ptr& root, NodeVisitor& visitor, const NodeFilter& filter){
    // traverse from root node.

    //visitor(root, SceneNode::Ptr(NULL) );
    //if(filter(root))
    //    return;

    std::stack<NodeP> nodeStack;
    nodeStack.push( NodeP(root,NULL, false) );

    while(!nodeStack.empty()){
        NodeP npair = nodeStack.top();
        nodeStack.pop();
        //std::cout << "visit: " << npair._first->getName();
        //if(npair._second!=NULL)
        //    std::cout << "--->: " << npair._second->getName() << std::endl;
        visitor(npair._first, npair._second);
        if(GroupNode* gnode = npair._first->asGroupNode()){
            BOOST_FOREACH(SceneNode::Ptr& n, gnode->_childNodes){
                if(filter(n))
                    continue;
                nodeStack.push( NodeP(n, npair._first, false) );
            }
        }
    }
}

void SceneGraph::traverse(SceneNode::Ptr& node, SceneGraph::NodeVisitor& visitor){
    traverse(node,visitor, StaticFilter(false).functor);
}

void SceneGraph::traverse(SceneNode::Ptr& root, NodeVisitor& visitor, NodeVisitor& postvisitor, const NodeFilter& filter){
    // traverse from root node.
    //visitor(root, SceneNode::Ptr(NULL) );
    //if(filter(root))
    //    return;
    std::stack<NodeP> nodeStack;
    nodeStack.push( NodeP(root,NULL, false) );
    while(!nodeStack.empty()){
        NodeP &npair = nodeStack.top();
        if(npair.visited) {
            nodeStack.pop();
            postvisitor(npair._first, npair._second);
            continue;
        }
        npair.visited = true;
        visitor(npair._first, npair._second);
        if(GroupNode* gnode = npair._first->asGroupNode()){
            BOOST_FOREACH(SceneNode::Ptr& n, gnode->_childNodes){
                if( filter(n) )
                    continue;
                nodeStack.push( NodeP(n, npair._first, false) );
            }
        }
    }
}

void SceneGraph::traverse(SceneNode::Ptr& node, NodeVisitor& visitor, NodeVisitor& postvisitor){
    traverse(node,visitor, postvisitor, StaticFilter(false).functor);
}

std::vector<DrawableNode::Ptr> SceneGraph::getDrawables(){
    DrawableVectorVisitor visitor;
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    return visitor.drawables;
}

std::vector<DrawableNode::Ptr> SceneGraph::getDrawables(SceneNode::Ptr node){
    DrawableVectorVisitor visitor;
    traverse(node, visitor.functor, StaticFilter(true).functor);
    return visitor.drawables;
}

std::vector<DrawableNode::Ptr> SceneGraph::getDrawablesRec(SceneNode::Ptr node){
    DrawableVectorVisitor visitor;
    traverse(node, visitor.functor, NodeTypeExcludeFilter(SceneNode::CameraType).functor);
    return visitor.drawables;
}

DrawableNode::Ptr SceneGraph::findDrawable(const std::string& name){
    FindDrawableVisitor visitor(name, false);
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    return visitor._dnode;
}

DrawableNode::Ptr SceneGraph::findDrawable(const std::string& name, SceneNode::Ptr node){
    FindDrawableVisitor visitor(name, false);
    traverse(node, visitor.functor, StaticFilter(true).functor);
    return visitor._dnode;
}

std::vector<DrawableNode::Ptr> SceneGraph::findDrawables(const std::string& name){
    FindDrawableVisitor visitor(name, true);
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    return visitor._dnodes;
}


bool SceneGraph::removeDrawables(GroupNode::Ptr node){
	//Does not work with BOOST_FOREACH as we modify the list which we are iterating through
/*	for (std::list<SceneNode::Ptr>::iterator it = node->_childNodes.begin(); it != node->_childNodes.end();) {
		SceneNode::Ptr child = *it;		
		if( child->asDrawableNode() ) {
			child->removeParent( node );
			it = node->_childNodes.erase(it);
		} else {
			++it;
		}
	}
*/
	//The remove child method on GroupNode changed the iterator used by BOOST_FOREACH. 
	//It therefore may result in errors.
    BOOST_FOREACH(SceneNode::Ptr child, node->_childNodes){
        if( child->asDrawableNode() ) {
			child->removeParent( node );            
        }
    }
	node->_childNodes.clear();

/*
    BOOST_FOREACH(SceneNode::Ptr child, node->_childNodes){
        if( child->asDrawableNode() ){
            node->removeChild(child);
        }
    }*/
    return true;
}

bool SceneGraph::removeDrawables(const std::string& name){
    FindDrawableVisitor visitor(name, true);
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    if(visitor._pnodes.size()==0)
        return true;
    for(size_t i=0;i<visitor._pnodes.size();i++){
        if( GroupNode* gn = visitor._pnodes[i]->asGroupNode() ){
            gn->removeChild(visitor._dnodes[i]);
        }
    }
    return true;

}

bool SceneGraph::removeDrawable(DrawableNode::Ptr drawable){
    FindDrawableVisitor visitor(drawable, true);
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    if(visitor._pnodes.size()>0){
        BOOST_FOREACH(SceneNode::Ptr parent, visitor._pnodes){
            if( GroupNode* gn = parent->asGroupNode() ){
                gn->removeChild(drawable);
            }
        }
    }
    if(drawable->_parentNodes.size()>0){
        SceneNode::NodeList listTmp = drawable->_parentNodes;
        BOOST_FOREACH(SceneNode::Ptr parent, listTmp){
            if( GroupNode* gn = parent->asGroupNode() ){
                gn->removeChild(drawable);
            }
        }
    }
    return true;
}

bool SceneGraph::removeDrawable(DrawableNode::Ptr drawable, SceneNode::Ptr node){
    if( GroupNode* gn = node->asGroupNode() ){
        gn->removeChild(drawable);
    }
    return true;
}

bool SceneGraph::removeDrawable(const std::string& name){
    FindDrawableVisitor visitor(name, false);
    SceneNode::Ptr root = _root.cast<SceneNode>();
    traverse(root, visitor.functor, StaticFilter(false).functor);
    if(visitor._dnode==NULL)
        return true;
    if( GroupNode* gn = visitor._pnode->asGroupNode() ){
        gn->removeChild(visitor._dnode);
    }
    return true;
}

bool SceneGraph::removeChild(const std::string& name, GroupNode::Ptr node){
    BOOST_FOREACH(SceneNode::Ptr child, node->_childNodes){
        if(child->getName()==name){
            node->removeChild(child);
			return true;
        }
    }
    return false;
}

namespace {
    class SimpleCameraGroup: public CameraGroup {
    public:
        SimpleCameraGroup(const std::string& name):
            _enabled(false),
            _name(name),
            _offscreenRender(false),
            _offWidth(0),
            _offHeight(0)
            {}
        virtual ~SimpleCameraGroup(){}
        std::string getName(){ return _name; }
        bool isEnabled(){ return _enabled;}
        void setEnabled(bool enabled){ _enabled = true;}
        void insertCamera(SceneCamera::Ptr cam, int index){
            std::list<SceneCamera::Ptr>::iterator i = _cameras.begin();
            std::advance(i, index);
            _cameras.insert(i, cam);
        }
        void removeCamera(int index){
            std::list<SceneCamera::Ptr>::iterator i = _cameras.begin();
            std::advance(i, index);
            _cameras.erase(i);
        }

        std::list<SceneCamera::Ptr> getCameras(){
            return _cameras;
        }

        bool setOffscreenRenderEnabled( bool enable ){
            _offscreenRender = enable;
            return true;
        }

        bool isOffscreenRenderEnabled(){
            return _offscreenRender;
        }

        void setOffscreenRenderSize(int width, int height){ _offWidth=width; _offHeight=height;};

        void setOffscreenRenderColor(rw::sensor::Image::ColorCode color){
            _color = color;
        }
        void setCopyToImage(rw::sensor::Image::Ptr img){}
        void setCopyToScan25D(rw::common::Ptr<class rw::geometry::PointCloud> img){}
        void setMultiSample(int samples){};
        void setMainCamera(SceneCamera::Ptr cam){
            _maincam = cam;
        }
        SceneCamera::Ptr getMainCamera(){
            if(_maincam==NULL)
                if(_cameras.size()>0)
                    return _cameras.front();
            return _maincam;
        }

        std::list<SceneCamera::Ptr> _cameras;
        SceneCamera::Ptr _maincam;
        bool _enabled;
        std::string _name;

        bool _offscreenRender;
        int _offWidth, _offHeight;
        rw::sensor::Image::ColorCode _color;

    };
}



CameraGroup::Ptr SceneGraph::makeCameraGroup(const std::string& name){
    return ownedPtr( new SimpleCameraGroup(name) );
}

/*
CameraGroup::Ptr SceneGraph::getCameraGroup(int groupidx){
    std::list<CameraGroup::Ptr>::iterator i = _cameraGroups.begin();
    std::advance(i, groupidx);
    return *i;
}
*/

CameraGroup::Ptr SceneGraph::findCameraGroup(const std::string& name){
    BOOST_FOREACH(CameraGroup::Ptr& cam, _cameraGroups){
        if(cam->getName()==name)
            return cam;
    }
    RW_THROW("No CameraGroup with name: " << name);
    return NULL;
}

void SceneGraph::addCameraGroup(CameraGroup::Ptr cgroup){
    //std::list<CameraGroup::Ptr>::iterator i = _cameraGroups.begin();
    //std::advance(i, groupidx);
    _cameraGroups.push_back(cgroup);
}

void SceneGraph::removeCameraGroup(const std::string& name){
    CameraGroup::Ptr camg = findCameraGroup(name);
    if(camg!=NULL)
        removeCameraGroup(camg);
}

void SceneGraph::removeCameraGroup(CameraGroup::Ptr cgroup){
    std::list<CameraGroup::Ptr>::iterator i = std::find(_cameraGroups.begin(), _cameraGroups.end(), cgroup);
    if(i!=_cameraGroups.end())
        _cameraGroups.erase(i);
}

std::list<CameraGroup::Ptr> SceneGraph::getCameraGroups(){
    return _cameraGroups;
}
