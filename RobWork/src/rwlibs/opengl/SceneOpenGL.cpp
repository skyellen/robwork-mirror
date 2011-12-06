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

#include <rwlibs/os/rwgl.hpp>

#include <rw/geometry/GeometryFactory.hpp>
#include <rw/graphics/Render.hpp>

#include <rwlibs/opengl/RenderGeometry.hpp>
#include <rwlibs/opengl/DrawableGeometry.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableFactory.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rwlibs/opengl/RWGLFrameBuffer.hpp>

#include <rw/graphics/SceneCamera.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/models/WorkCell.hpp>
#include <rw/models/Accessor.hpp>

#include <rw/common/StringUtil.hpp>
#include <rw/common/Property.hpp>
#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>
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

    class SimpleCameraGroup: public CameraGroup {
    public:
        SimpleCameraGroup(const std::string& name):
            _enabled(false),
            _name(name),
            _offscreenRender(false),
            _offWidth(640),
            _offHeight(480),
            _initialized(false),
            _renderToImage(false),
            _renderToDepth(false),
            _fbId(-1),_renderId(-1),_renderDepthId(-1),textureId(-1)
            {}

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

        void init(){
            if( (_offscreenRender==false && _fbId>=0) ){
                // offsreenrendering has been disabled so relase all allocated storage
                // deallocate the framebuffer
                if(_fbId!=-1)
                    RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
                if(_renderId!=-1)
                    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
                if(_renderDepthId!=-1)
                    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
                _fbId = -1;
                _renderId = -1;
                _renderDepthId = -1;
            } else if(_offscreenRender==true){
                RWGLFrameBuffer::initialize();
                if(_fbId>=0){
                    // the parameters of the frame buffer should be changed so we create a new
                    if(_fbId!=-1)
                        RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
                    if(_renderId!=-1)
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
                    if(_renderDepthId!=-1)
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
                }
                _fbId = 0;
                _renderId = 0;
                RWGLFrameBuffer::glGenFramebuffersEXT(1, &_fbId);
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);
                RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderId);
                // select render
                RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);
                // create render storage
                RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB8, _offWidth, _offHeight);
                //Attach color buffer to FBO
                RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, _renderId);


                // now if we need depth of image we also attach depth render buffer
                //if(_renderToDepth==true){
                    RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderDepthId);
                    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderDepthId);
                    RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT24, _offWidth, _offHeight);
                     //Attach depth buffer to FBO
                    RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, _renderDepthId);
                //}


                //Does the GPU support current FBO configuration?
                GLenum status;
                status = RWGLFrameBuffer::glCheckFramebufferStatusEXT(GL_FRAMEBUFFER_EXT);
                switch(status){
                case GL_FRAMEBUFFER_COMPLETE_EXT:
                    break;
                default:
                    Log::errorLog() << "FrameBuffer not supported, offscreen rendering not possible! Status=" << status << std::endl;;
                    break;
                }

                RWGLFrameBuffer::test( Log::infoLog() );
                RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
            } else {

            }
            _initialized = true;
        }

        void setOffscreenRenderEnabled( bool enable ){
            // if both are same then do nothing
            if( enable == _offscreenRender )
                return;
            _offscreenRender = enable;
            _initialized=false;
        }

        bool isOffscreenRenderEnabled(){
            return _offscreenRender;
        }

        void setOffscreenRenderSize(int width, int height){
            _offWidth=width; _offHeight=height;
            _initialized=false;
        };

        void setOffscreenRenderColor(rw::sensor::Image::ColorCode color){
            _color = color;
            _initialized=false;
        }

        void bind(){ RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId); }
        void unbind(){ RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0); }

        bool isInitialized(){ return _initialized; }

        void setCopyToImage(rw::sensor::Image::Ptr img){
            _img = img;
            if(img!=NULL){
                _renderToImage = true;
            } else {
                _renderToImage = false;
            }
        };


        void setCopyToScan25D( rw::sensor::Image25D::Ptr scan){
            _scan25 = scan;
            if(scan!=NULL){
                _renderToDepth = true;
            } else {
                _renderToDepth = false;
            }
            _initialized = false;
        };


        std::list<SceneCamera::Ptr> _cameras;
        bool _enabled;
        std::string _name;

        bool _offscreenRender;
        int _offWidth, _offHeight;

        bool _initialized, _renderToImage, _renderToDepth;
        rw::sensor::Image::ColorCode _color;
        GLuint _fbId,_renderId,_renderDepthId,textureId;
        rw::sensor::Image::Ptr _img;
        rw::sensor::Image25D::Ptr _scan25;
        std::vector<float> _depthData;
    };

}




namespace {

    struct RenderPreVisitor {
        SceneGraph::NodeVisitor functor;
        RenderPreVisitor(SceneGraph::RenderInfo& info, std::stack<Transform3D<> >& stack,
                         bool transparent,
                         bool pushNames=false):
                             _info(info),
                             _drawAlpha(transparent),
                             _pushNames(pushNames),
                             _stack(stack)
        {
            functor =  boost::ref(*this);
        }

        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( GroupNode *gnode=child->asGroupNode() ){
                // perform opengl transforms and stuff
                //GLfloat gltrans[16];
                //glPushMatrix();

                //if (_scale != 1.0)
                //    glScalef(_scale, _scale, _scale);
                //std::cout << gnode->getName() << " --> " << gnode->getTransform() << "\n";

                //DrawableUtil::transform3DToGLTransform(gnode->getTransform(), gltrans);
                _stack.push( _stack.top() * gnode->getTransform() );

                //DrawableUtil::multGLTransform( gnode->getTransform() );
                //glMultMatrixf(gltrans);

            } else if( DrawableNode* dnode = child->asDrawableNode()){
                if(dnode->isTransparent()==_drawAlpha){
                    glPushMatrix();
                    DrawableUtil::multGLTransform( _stack.top() );
                    if(_pushNames){
                        GLuint uid = *((GLuint*)&dnode);
                        //glPushName( uid );
                        glLoadName(uid);
                        dnode->draw(_info);
                        //glPopName();
                    } else {
                        dnode->draw(_info);
                    }
                    glPopMatrix();

                }
            }
            return false;
        }
        SceneGraph::RenderInfo _info;
        std::stack<Transform3D<> >& _stack;
        bool _drawAlpha;
        bool _pushNames;
    };

    struct RenderPostVisitor {
        SceneGraph::NodeVisitor functor;
        RenderPostVisitor(std::stack<Transform3D<> >& stack):_stack(stack){
            functor =  boost::ref(*this);
        }

        bool operator()(SceneNode::Ptr& child, SceneNode::Ptr& parent){
            if( child->asGroupNode()!=NULL ){
                _stack.pop();
                //glPopMatrix();
            }
            return false;
        }

        std::stack<Transform3D<> >& _stack;
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

        if(camGroup==NULL)
            return;

        if(!camGroup->isEnabled())
            return;

        rw::common::Ptr<SimpleCameraGroup> scam = camGroup.cast<SimpleCameraGroup>();
        bool offscreenEnabled = false;
        GLint oldDim[4]; // viewport dimensions [ x,y,width,height ]
        RW_ASSERT(scam);
        if(scam!=NULL){
            if( !scam->isInitialized() )
                scam->init();
            offscreenEnabled = scam->isOffscreenRenderEnabled();

            if( offscreenEnabled ){
                glGetIntegerv(GL_VIEWPORT,oldDim); // get viewport dimensions
                scam->bind();
            }
        }

        if(usePickMatrix){
            // for picking we need an enabled z-buffer
            glInitNames();
        }

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
                    glViewport( vp.x, (GLint) (vp.y + (vp.h-h)/2.0), vp.w, (GLsizei)h);
                } else {
                    double w = vp.h*cam->getAspectRatio();
                    glViewport( (GLint)(vp.x+ (vp.w-w)/2.0), vp.y, (GLsizei)w, vp.h);
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
            if(usePickMatrix){
                //glInitNames();
                glPushName(0);
            }

            Transform3D<> camtransform = cam->getTransform();
            if( cam->getAttachedNode().get() != NULL ){
                // calculate kinematics from attached node to
                SceneNode::Ptr parent = cam->getAttachedNode();
                //std::cout << parent->getName() << std::endl;
                do{
                    if(parent->asGroupNode()!=NULL)
                        camtransform = parent->asGroupNode()->getTransform()*camtransform;
                    parent = parent->_parentNodes.front();
                } while( parent!=cam->getRefNode() );
            }

            Transform3D<> viewMatrix = inverse( camtransform );
            DrawableUtil::transform3DToGLTransform(viewMatrix, matrix);
            glLoadMatrixf( matrix );

            //GLfloat lpos[] = {0.0f, 10.0f, 10.0f, 0.0f};
            //glLightfv(GL_LIGHT2, GL_POSITION, lpos);


            // iterate scenegraph from node specified by camera.
            previsitor._drawAlpha = false;
            previsitor._info._mask = cam->getDrawMask();
            previsitor._info._renderTransparent = false;
            previsitor._info._renderSolid = true;
            graph->traverse(subRootNode, previsitor.functor, postvisitor.functor, StaticFilter(false).functor);
            // now render transparent stuff
            previsitor._drawAlpha = true;
            previsitor._info._renderTransparent = true;
            previsitor._info._renderSolid = false;
            graph->traverse(subRootNode, previsitor.functor, postvisitor.functor, StaticFilter(false).functor);

            if(usePickMatrix){
                if(!cam->isDepthTestEnabled())
                    glPopName();
                //glLoadName(0);
                //glPopName();
            }
        }
        if(scam!=NULL){
            if( (scam->_renderToImage) && scam->_img!=NULL){
                // copy rendered scene to image
                char *imgData = scam->_img->getImageData();
                glReadPixels(
                    0, 0,
                    scam->_img->getWidth(), scam->_img->getHeight(),
                    GL_RGB, GL_UNSIGNED_BYTE, imgData);
            }
            if( (scam->_renderToDepth) && scam->_scan25!=NULL){
                std::cout << "render to depth" << std::endl;
                if(scam->_depthData.size() != scam->_scan25->getWidth()*scam->_scan25->getHeight() )
                    scam->_depthData.resize(scam->_scan25->getWidth()*scam->_scan25->getHeight());
                // copy rendered depth scene to image
                glReadPixels(
                     0, 0,
                     scam->_scan25->getWidth(), scam->_scan25->getHeight(),
                     GL_DEPTH_COMPONENT, GL_FLOAT, &scam->_depthData[0]);

                GLdouble modelview[16];
                GLdouble projection[16];
                GLint viewport[4];

                glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
                glGetDoublev( GL_PROJECTION_MATRIX, projection );
                glGetIntegerv( GL_VIEWPORT, viewport );

                std::vector<rw::math::Vector3D<float> >* result = &scam->_scan25->getData();
                // now unproject all pixel values
                if (result != NULL && result->size() != scam->_scan25->getWidth()*scam->_scan25->getHeight())
                    result->resize(scam->_scan25->getWidth()*scam->_scan25->getHeight());

                for(size_t y=0;y<scam->_scan25->getHeight();y++){
                    for(size_t x=0;x<scam->_scan25->getWidth();x++){
                        double winX=x,winY=y,winZ=scam->_depthData[x+y*scam->_scan25->getWidth()];
                        double posX, posY, posZ;
                        gluUnProject( winX, winY, winZ,
                                modelview, projection, viewport,
                                &posX, &posY, &posZ);
                        if (result != NULL) {
                            Vector3D<float>& q = (*result)[x+y*scam->_scan25->getWidth()];
                            q(0) = (float)posX;
                            q(1) = (float)posY;
                            q(2) = (float)posZ;
                            std::cout << q << "\n";
                        }
                    }
                }

            }

        }
        if(offscreenEnabled){
            // check if we need to grab the image
            glViewport(oldDim[0],oldDim[1],oldDim[2],oldDim[3]); // set camera view port
            scam->unbind();
        }

    }

}

void SceneOpenGL::draw(SceneGraph::RenderInfo& info){
    // render the scene
    draw(info, _root);
}

void SceneOpenGL::draw(SceneGraph::RenderInfo& info, SceneNode::Ptr node){
    std::stack<rw::math::Transform3D<> > stack;
    stack.push( Transform3D<>::identity() );
    RenderPreVisitor preVisitor(info,stack,false);
    RenderPostVisitor postVisitor(stack);

    //drawScene(this, getCameraGroup(info.cameraGroup), info, node, preVisitor, postVisitor, false, 0,0);
    drawScene(this, info.cams, info, node, preVisitor, postVisitor, false, 0,0);
}
#include <stdio.h>
DrawableNode::Ptr SceneOpenGL::pickDrawable(SceneGraph::RenderInfo& info, int x, int y){
    #define GL_SELECT_BUFSIZE 512
    GLuint _selectBuf[GL_SELECT_BUFSIZE];

    // Start picking
    glSelectBuffer(GL_SELECT_BUFSIZE, _selectBuf);
    glRenderMode(GL_SELECT);

    std::stack<rw::math::Transform3D<> > stack;
    stack.push( Transform3D<>::identity() );
    RenderPreVisitor preVisitor(info, stack, false, true);
    RenderPostVisitor postVisitor(stack);
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
       /*
       printf( "Number: %d\n"
                  "Min Z: %d\n"
                  "Max Z: %d\n"
                  "Name on stack: %d\n",
                  (GLubyte)ptr[i * 4],
                  (GLubyte)ptr[i * 4 + 1],
                  (GLubyte)ptr[i * 4 + 2],
                  (GLubyte)ptr[i * 4 + 3]
                  );
        */

       ptr++;
       GLuint depthZ = *ptr;
       GLuint *ptrN = ptr+2;

       std::cout << "Z depth: " << depthZ << " names: " << *ptrN << std::endl;
       //const Frame *frame = _cell.state->getFrame(*ptrN);
       //if( frame!=NULL )
       //    std::cout << " " << frame->getName() << std::endl;
       if(names!=0){
           if (depthZ < minZ) {
               numberOfNames = names;
               minZ = depthZ;
               ptrNames = ptrN;
           }
       }
       ptr += names+2;
   }
   std::cout << "The closest hit names are "  << numberOfNames << std::endl;
   ptr = ptrNames;

   std::vector<DrawableNode::Ptr> drawables = getDrawables();
   //std::cout << "nr drawables: " << drawables.size() << std::endl;
   BOOST_FOREACH(DrawableNode::Ptr d, drawables){
       DrawableNode *dnode = d.get();
       std::cout << *((GLuint*)&dnode) << "==" << *ptr << std::endl;
       if(*((GLuint*)&dnode)==*ptr){
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
    return NULL;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const rw::models::CollisionModelInfo& info){

    // forst check if the drawable is allready in the currentDrawables list
     rwlibs::opengl::Drawable::Ptr drawable = NULL;
     try {
         drawable = DrawableFactory::constructFromGeometry(info.getGeoString(), info.getName());
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

rw::common::Ptr<CameraGroup> SceneOpenGL::makeCameraGroup(const std::string& name){
    return ownedPtr( new SimpleCameraGroup(name) );
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

