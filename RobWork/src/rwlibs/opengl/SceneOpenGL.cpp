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


#include "SceneOpenGL.hpp"

#include <rw/graphics/SceneCamera.hpp>

#include <rwlibs/os/rwgl.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Accessor.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <rwlibs/opengl/DrawableGeometry.hpp>

#include <boost/foreach.hpp>
#include <rw/geometry/GeometryFactory.hpp>

#include <rw/graphics/Render.hpp>
#include <rwlibs/opengl/RenderGeometry.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableFactory.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <vector>
#include <stack>

using namespace rw::graphics;
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::common;
using namespace rw::sensor;

using namespace rwlibs::opengl;

SceneOpenGL::SceneOpenGL(): SceneGraph()
{

}

SceneOpenGL::~SceneOpenGL()
{
    clearCache();
}

void SceneOpenGL::clearCache()
{
    //BOOST_FOREACH(FrameMap::const_reference entry, _frameMap) {
    //    BOOST_FOREACH(rwlibs::opengl::Drawable::Ptr da, entry.second) {
    //        delete da;
    //    }
    //}
    //std::cout << "************************* FRAMEMAP CLEAR *****************************" << std::endl;
    //_frameMap.clear();
}


rw::math::Vector3D<> SceneOpenGL::unproject(SceneCamera::Ptr camera, int x, int y){
    GLfloat depth;
    glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLdouble objx, objy, objz;
    gluUnProject(x, y, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);
    return Vector3D<>(objx,objy,objz);
}


namespace {

    struct RenderPreVisitor {
        SceneGraph::NodeVisitor functor;
        RenderPreVisitor(SceneGraph::RenderInfo& info, bool transparent, bool pushNames=false):_info(info),_drawAlpha(transparent),_pushNames(pushNames){
            functor =  boost::ref(*this);
        }

        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( GroupNode *gnode=child->asGroupNode() ){
                // perform opengl transforms and stuff
                //GLfloat gltrans[16];
                glPushMatrix();

                //if (_scale != 1.0)
                //    glScalef(_scale, _scale, _scale);
                //std::cout << gnode->getName() << " --> " << gnode->getTransform() << "\n";

                //DrawableUtil::transform3DToGLTransform(gnode->getTransform(), gltrans);
                DrawableUtil::multGLTransform( gnode->getTransform() );
                //glMultMatrixf(gltrans);

            } else if( DrawableNode* dnode = child->asDrawableNode()){
                if(dnode->isTransparent()==_drawAlpha){
                    if(_pushNames){
                        glPushName( *( (GLuint*)dnode ) );
                        dnode->draw(_info);
                        glPopName();
                    } else {
                        dnode->draw(_info);
                    }
                }
            }
            return false;
        }
        SceneGraph::RenderInfo _info;
        bool _drawAlpha;
        bool _pushNames;
    };

    struct RenderPostVisitor {
        SceneGraph::NodeVisitor functor;
        RenderPostVisitor(){
            functor =  boost::ref(*this);
        }

        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( child->asGroupNode()!=NULL ){
                glPopMatrix();
            }
            return false;
        }
    };

    struct StaticFilter {
        SceneGraph::NodeFilter functor;
        StaticFilter(bool retValue):_retvalue(retValue){
            functor =  boost::ref(*this);
        }
        bool operator()(const SceneNode::Ptr& child) const { return _retvalue; }
        bool _retvalue;
    };

    void drawScene(SceneGraph* graph, CameraGroup::Ptr camGroup, SceneGraph::RenderInfo& info, SceneNode::Ptr node, RenderPreVisitor &previsitor, RenderPostVisitor &postvisitor, bool usePickMatrix = false, int pickx=0, int picky =0){
        if(node.get() == NULL)
            return;

        GLfloat matrix[16];
        union {
            struct{
                int x,y,w,h;
            };
            int viewport[4];
        } vp;
        // for each camera draw the scene starting from the node specified by the camera



        if(!camGroup->isEnabled())
            return;

        BOOST_FOREACH(SceneCamera::Ptr cam, camGroup->getCameras() ){
            if(!cam->isEnabled())
                continue;
            SceneNode::Ptr subRootNode = cam->getRefNode();
            if(subRootNode==NULL)
                continue;

            cam->getViewport(vp.x,vp.y,vp.w,vp.h);
            switch( cam->getAspectRatioControl() ){
            case(SceneCamera::Auto):{
                glViewport( vp.x, vp.y, vp.w, vp.h);
                break;
            }
            case(SceneCamera::ScaleX):
            case(SceneCamera::ScaleY):
            case(SceneCamera::Scale):{
                // choose scale axis
                //std::cout <<  vp.w/(double)vp.h << "<" << cam->getAspectRatio() << std::endl;
                if( vp.w/(double)vp.h<cam->getAspectRatio() ){
                    double h = vp.w/cam->getAspectRatio();
                    glViewport( vp.x, vp.y + (vp.h-h)/2.0, vp.w, h);
                } else {
                    double w = vp.h*cam->getAspectRatio();
                    glViewport( vp.x+ (vp.w-w)/2.0, vp.y, w, vp.h);
                }
                break;
            }
            case(SceneCamera::Fixed):{
                glViewport( vp.x, vp.y, vp.w, vp.h);

                break;
            }
            default:
                glViewport( vp.x, vp.y, vp.w, vp.h);
                break;
            }


            //std::cout << x << " " << y << " " << w << " " << h << " " << "\n";

            // optionally clear buffers
            if(cam->isClearBufferEnabled()){
                //std::cout << "clear buffer\n";
                glClear( cam->getClearBufferMask() );
            }

            if(cam->isDepthTestEnabled()) glEnable(GL_DEPTH_TEST);
            else glDisable(GL_DEPTH_TEST);

            if(cam->isLightningEnabled()) glEnable(GL_LIGHTING);
            else glDisable(GL_LIGHTING);


            // setup projection
            ProjectionMatrix projectionMatrix = cam->getProjectionMatrix();
            projectionMatrix.toOpenGLMatrix(matrix);
            glMatrixMode(GL_PROJECTION);

            if(usePickMatrix){
                glLoadIdentity();
                gluPickMatrix(pickx,picky,3,3,vp.viewport);
                glMultMatrixf( matrix );
            } else {
                glLoadMatrixf( matrix );
            }

            // setup model view
            glMatrixMode(GL_MODELVIEW);
            if(usePickMatrix)
                glInitNames();

            Transform3D<> camtransform = cam->getTransform();
            if( cam->getAttachedNode().get() != NULL ){
                // calculate kinematics from attached node to
                SceneNode::Ptr parent = cam->getAttachedNode();
                std::cout << parent->getName() << std::endl;
                do{
                    if(parent->asGroupNode()!=NULL)
                        camtransform = parent->asGroupNode()->getTransform()*camtransform;
                    parent = parent->_parentNodes.front();
                } while( parent!=cam->getRefNode() );
            }

            Transform3D<> viewMatrix = inverse( camtransform );
            DrawableUtil::transform3DToGLTransform(viewMatrix, matrix);
            glLoadMatrixf( matrix );

            // iterate scenegraph from node specified by camera.
            previsitor._drawAlpha = false;
            previsitor._info._mask = cam->getDrawMask();
            graph->traverse(subRootNode, previsitor.functor, postvisitor.functor, StaticFilter(false).functor);
            // now render transparent stuff
            previsitor._drawAlpha = true;
            graph->traverse(subRootNode, previsitor.functor, postvisitor.functor, StaticFilter(false).functor);
        }
    }

}

void SceneOpenGL::draw(SceneGraph::RenderInfo& info){
    // render the scene
    draw(info, _root);
}

void SceneOpenGL::draw(SceneGraph::RenderInfo& info, SceneNode::Ptr node){
    RenderPreVisitor preVisitor(info,false);
    RenderPostVisitor postVisitor;

    //drawScene(this, getCameraGroup(info.cameraGroup), info, node, preVisitor, postVisitor, false, 0,0);
    drawScene(this, info.cams, info, node, preVisitor, postVisitor, false, 0,0);
}

DrawableNode::Ptr SceneOpenGL::pickDrawable(SceneGraph::RenderInfo& info, int x, int y){
    #define GL_SELECT_BUFSIZE 512
    GLuint _selectBuf[GL_SELECT_BUFSIZE];

    // Start picking
    glSelectBuffer(GL_SELECT_BUFSIZE, _selectBuf);
    glRenderMode(GL_SELECT);

    RenderPreVisitor preVisitor(info, false, true);
    RenderPostVisitor postVisitor;
    //drawScene(this, getCameraGroup(info.cameraGroup), info, _root, preVisitor, postVisitor, true, x, y);
    drawScene(this, info.cams, info, _root, preVisitor, postVisitor, true, x, y);

    // returning to normal rendering mode
    int hits = glRenderMode(GL_RENDER);

    // if there are hits process them
    if (hits == 0)
        return NULL;

    // process the hits
    GLuint names, *ptrNames=NULL, numberOfNames=0;
    GLuint *ptr = (GLuint *) _selectBuf;
    GLuint minZ = 0xffffffff;
    for (int i = 0; i < hits; i++) {
       names = *ptr;
       ptr++;
       GLuint depthZ = *ptr;
       GLuint *ptrN = ptr+2;

       //std::cout << "Z depth: " << depthZ << " names: " << *ptrN << std::endl;
       //const Frame *frame = _cell.state->getFrame(*ptrN);
       //if( frame!=NULL )
       //    std::cout << " " << frame->getName() << std::endl;

       if (depthZ < minZ) {
           numberOfNames = names;
           minZ = depthZ;
           ptrNames = ptrN;
       }

       ptr += names+2;
     }
   //std::cout << "The closest hit names are "  << numberOfNames << std::endl;
   ptr = ptrNames;

   std::vector<DrawableNode::Ptr> drawables = getDrawables();
   BOOST_FOREACH(DrawableNode::Ptr d, drawables){
       if((*((GLuint*)d.get()))==*ptr){
           std::cout << "name: " << d->getName() << std::endl;
           return d;
       }
   }
   return NULL;
}

void SceneOpenGL::clear(){
    //_root = ownedPtr(new GroupNode("Root"));
}

void SceneOpenGL::update(){
    // calculate the transforms to the cameras
}


rw::graphics::DrawableNode::Ptr SceneOpenGL::makeDrawable(const rw::models::DrawableModelInfo& info){
    // forst check if the drawable is allready in the currentDrawables list
    rwlibs::opengl::Drawable::Ptr drawable = NULL;
	try {
         drawable = DrawableFactory::getDrawable(info.getId(), info.getName());
	} catch (const rw::common::Exception& exp){
		RW_WARN(exp.getMessage());
    }

    if (drawable) {
        // Set various properties for the drawable:
        drawable->setTransform(info.getTransform());
        drawable->setScale((float)info.getGeoScale());
        drawable->setMask( Drawable::DrawableObject | Drawable::Physical );

        if (info.isHighlighted())
            drawable->setHighlighted(true);

        if (info.isWireMode())
            drawable->setDrawType(DrawableNode::WIRE);

        return drawable;
    } 
    RW_THROW(
             "NULL drawable returned by loadDrawableFile() for GeoID "
             << info.getId());
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const rw::models::CollisionModelInfo& info){
    // forst check if the drawable is allready in the currentDrawables list
     rwlibs::opengl::Drawable::Ptr drawable = NULL;
     try {
         drawable = DrawableFactory::getDrawable(info.getGeoString(), info.getName());
     } catch (const rw::common::Exception& exp){
         RW_WARN(exp.getMessage());
     }

     if (drawable) {
         // Set various properties for the drawable:
         drawable->setTransform(info.getTransform());
         drawable->setScale((float)info.getGeoScale());
         drawable->setMask( Drawable::CollisionObject );

         return drawable;
     } else {
         RW_WARN(
             "NULL drawable returned by loadDrawableFile() for GeoString "
             << info.getGeoString());
     }
     return NULL;
}



SceneCamera::Ptr SceneOpenGL::makeCamera(const std::string& name){
    return ownedPtr(new SceneCamera(name, getRoot() ));
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& filename, int dmask){
    rwlibs::opengl::Drawable::Ptr drawable = NULL;
     try {
         drawable = DrawableFactory::loadDrawableFile(filename, filename);
         drawable->setMask(dmask);
     } catch (const rw::common::Exception& exp){
         RW_WARN(exp.getMessage());
     }
     return drawable;
}


DrawableGeometryNode::Ptr SceneOpenGL::makeDrawableFrameAxis(const std::string& name, double size, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addFrameAxis(size);
    return drawable;
}

DrawableGeometryNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,rw::geometry::Geometry::Ptr geom, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addGeometry(geom);
    return drawable;
}

DrawableGeometryNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const std::vector<rw::geometry::Line >& lines, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addLines(lines);
    return drawable;
}



DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name, Model3D::Ptr model, int dmask){
    RenderModel3D::Ptr render = ownedPtr(new RenderModel3D(model));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const rw::sensor::Image& img, int dmask){
    RenderImage::Ptr render = ownedPtr(new RenderImage(img));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const rw::sensor::Scan2D& scan, int dmask){
    RenderScan::Ptr render = ownedPtr(new RenderScan(scan));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const rw::sensor::Image25D& scan, int dmask){
    RenderScan::Ptr render = ownedPtr(new RenderScan(scan));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}


DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name, Render::Ptr render, int dmask) {
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

