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

#include <rw/loaders/GeometryFactory.hpp>
#include <rw/graphics/Render.hpp>

#include <rwlibs/opengl/DrawableGeometry.hpp>
#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableFactory.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rwlibs/opengl/RenderModel3D.hpp>
#include <rwlibs/opengl/RenderScan.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/opengl/RWGLFrameBuffer.hpp>

#include <rw/graphics/SceneCamera.hpp>

#include <rw/common/macros.hpp>

#include <boost/foreach.hpp>
#include <vector>
#include <stack>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::math;

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
            _useMultiSample(false),
            _initialized(false),
            _renderToImage(false),
            _renderToDepth(false),
            _samples(0),
            _fbId(-1),_fbTmpId(-1),_renderColorTmpId(-1),_renderId(-1),_renderDepthId(-1),textureId(-1),_aMultisampleTexture(-1)
            {}

        virtual ~SimpleCameraGroup(){};
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

        void setMultiSample(int samples){
            _samples = samples;
            if(_samples<1){
                _samples=0;
                _useMultiSample = false;
            } else {
                _useMultiSample = true;
            }
        }

        void init(){
            GLuint maxGLuintSize = (GLuint) -1;
            if( (_offscreenRender==false && _fbId>=0) ){
                //RW_WARN("_offscreenRender==false && _fbId>=0");
                // offsreenrendering has been disabled so release all allocated storage
                // deallocate the framebuffer
                if(_fbId!=maxGLuintSize)
                    RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
                if(_renderId!=maxGLuintSize)
                    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
                if(_renderDepthId!=maxGLuintSize)
                    RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
                _fbId = maxGLuintSize;
                _renderId = maxGLuintSize;
                _renderDepthId = maxGLuintSize;

            } else if(_offscreenRender==true){
                //RW_WARN("_offscreenRender==true");
                bool useMultisample = _useMultiSample && !_renderToDepth;

                int samples;
                glGetIntegerv(GL_MAX_SAMPLES_EXT, &samples);
                if(samples<_samples){
                    RW_WARN("Current hardware only supports multisampling with "<< samples << " nr of samples! "
                            "The multi sampling will therefore be fixed to " << samples);
                    _samples = samples;
                }
                if(_samples==1){
                    _samples=0;
                    _useMultiSample = false;
                }

                //RW_WARN("useMultisample: " << useMultisample);
                //RW_WARN("_renderToDepth: " << _renderToDepth);
                RWGLFrameBuffer::initialize();
                if(_fbId>=0){
                    // the parameters of the frame buffer should be changed so we create a new
                    if(_fbId!=maxGLuintSize)
                        RWGLFrameBuffer::glDeleteFramebuffersEXT(1, &_fbId);
                    if(_renderId!=maxGLuintSize)
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderId);
                    if(_renderDepthId!=maxGLuintSize)
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderDepthId);
                    if( useMultisample ){
                        // TODO: we might need to do something here
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_fbTmpId);
                        RWGLFrameBuffer::glDeleteRenderbuffersEXT(1, &_renderColorTmpId);
                    }
                }
                _fbId = 0;
                _renderId = 0;

                if( useMultisample ){
                    //glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, _aMultisampleTexture);
                    //glRenderbufferStorageMultisample(GL_TEXTURE_2D_MULTISAMPLE, , GL_RGBA, , GL_TRUE);
                    //RWGLFrameBuffer::glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, _samples, GL_RGBA8, _offWidth, _offHeight);
                    //RWGLFrameBuffer::glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, _samples, GL_RGBA, _offWidth, _offHeight, GL_TRUE);
                }

                // if multisampling is enabled then create a temporary FrameBuffer to copy to in order to use
                // glReadPixels later
                if( useMultisample ){
                    RWGLFrameBuffer::glGenFramebuffersEXT(1, &_fbTmpId);
                    RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbTmpId);
                    RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderColorTmpId);
                    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderColorTmpId);
                    RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB8, _offWidth, _offHeight);
                    RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, _renderColorTmpId);
                    //RW_WARN("multisample: " << _samples);
                    RWGLFrameBuffer::testFrameBufferCompleteness();
                }


                RWGLFrameBuffer::glGenFramebuffersEXT(1, &_fbId);
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);
                RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderId);

                // select render
                RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderId);
                //RW_WARN("error: " << glGetError());
                // create render storage
                if( useMultisample ){
                    RWGLFrameBuffer::glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, _samples, GL_RGBA8,_offWidth, _offHeight);
                } else {
                    RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_RGB8, _offWidth, _offHeight);
                }
                //Attach color buffer to FBO
                RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_RENDERBUFFER_EXT, _renderId);
                //RW_WARN("error: " << glGetError());

                //RW_WARN("fb: " << _offWidth << " : " <<  _offHeight);
                RWGLFrameBuffer::testFrameBufferCompleteness();

                // now if we need depth of image (and we need it even just for correct rendering) we also attach depth render buffer
                //if(_renderToDepth==true){
                    RWGLFrameBuffer::glGenRenderbuffersEXT(1, &_renderDepthId);
                    RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, _renderDepthId);
                    if( useMultisample ){
                        RWGLFrameBuffer::glRenderbufferStorageMultisampleEXT(GL_RENDERBUFFER_EXT, _samples, GL_DEPTH24_STENCIL8, _offWidth, _offHeight);
                    } else {
                        RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH24_STENCIL8, _offWidth, _offHeight);
                        //RWGLFrameBuffer::glRenderbufferStorageEXT(GL_RENDERBUFFER_EXT, GL_DEPTH_COMPONENT32, _offWidth, _offHeight);
                    }

                     //Attach depth buffer to FBO
                    RWGLFrameBuffer::glFramebufferRenderbufferEXT(GL_FRAMEBUFFER_EXT, GL_DEPTH_ATTACHMENT_EXT, GL_RENDERBUFFER_EXT, _renderDepthId);
                //}

                // Enable multisampling
                if( useMultisample ){
                    glEnable(GL_MULTISAMPLE);
                }

                //Does the GPU support current FBO configuration?

                RWGLFrameBuffer::testFrameBufferCompleteness();

                RWGLFrameBuffer::test( Log::infoLog() );
                RWGLFrameBuffer::glBindRenderbufferEXT(GL_RENDERBUFFER_EXT, 0);
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
            } else {

            }

            _initialized = true;
        }

        void copyToImage( ){
            if(_useMultiSample){
                RWGLFrameBuffer::glBindFramebufferEXT(GL_READ_FRAMEBUFFER_EXT, _fbId); // the multisampled frame buffer
                RWGLFrameBuffer::glBindFramebufferEXT(GL_DRAW_FRAMEBUFFER_EXT, _fbTmpId); // our temporary FBO to copy multisampled image into
                RWGLFrameBuffer::glBlitFrameBufferEXT(0, 0, _offWidth, _offHeight, 0, 0, _offWidth, _offHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST);
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbTmpId); // now bind it so we can read from it
                char *imgData = _img->getImageData();
                glReadPixels(
                    0, 0, _img->getWidth(), _img->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, imgData);
            } else {
                RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId); // the multisampled frame buffer
                char *imgData = _img->getImageData();
                glReadPixels(
                    0, 0, _img->getWidth(), _img->getHeight(), GL_RGB, GL_UNSIGNED_BYTE, imgData);
            }
        }



        void setMainCamera(SceneCamera::Ptr cam){
            _mainCam = cam;
        }

        SceneCamera::Ptr getMainCamera() {
            if(_mainCam==NULL)
                if(_cameras.size()>0)
                    return _cameras.front();
            return _mainCam;
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
            _initialized=false;
        }

        void bind(){
            RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, _fbId);
        }

        void unbind(){
            RWGLFrameBuffer::glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, 0);
        }

        bool isInitialized(){ return _initialized; }

        void setCopyToImage(rw::sensor::Image::Ptr img){
            _img = img;
            if(img!=NULL){
                _renderToImage = true;
            } else {
                _renderToImage = false;
            }
        };


        void setCopyToScan25D( rw::geometry::PointCloud::Ptr scan){
            _scan25 = scan;
            if(scan!=NULL){
                _renderToDepth = true;
            } else {
                _renderToDepth = false;
            }
            _initialized = false;
        };


        std::list<SceneCamera::Ptr> _cameras;
        SceneCamera::Ptr _mainCam;
        bool _enabled;
        std::string _name;

        bool _offscreenRender;
        int _offWidth, _offHeight;

        bool _useMultiSample;

        bool _initialized, _renderToImage, _renderToDepth;
        int _samples;
        GLuint _fbId,_fbTmpId,_renderColorTmpId,_renderId,_renderDepthId,textureId,_aMultisampleTexture;
        rw::sensor::Image::Ptr _img;
        rw::geometry::PointCloud::Ptr _scan25;
        std::vector<GLfloat> _depthData;
    };

}




namespace {

    struct RenderPreVisitor {
        SceneGraph::NodeVisitor functor;
        RenderPreVisitor(SceneGraph::RenderInfo& info, std::stack<Transform3D<> >& stack,
                         bool transparent,
                         bool pushNames=false):
                             _info(info),
                             _stack(stack),
                             _drawAlpha(transparent),
                             _pushNames(pushNames)

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

    namespace {


        typedef
          union {
            struct{
                int x,y,w,h;
            };
            int viewport[4];
        } ViewPort;



        ViewPort getViewPort(SceneCamera::Ptr cam){
            ViewPort vp;
            cam->getViewport(vp.x,vp.y,vp.w,vp.h);
            switch( cam->getAspectRatioControl() ){
            case(SceneCamera::Auto):{
                //return vp;
                //glViewport( vp.x, vp.y, vp.w, vp.h);
                break;
            }
            case(SceneCamera::ScaleX):
            case(SceneCamera::ScaleY):
            case(SceneCamera::Scale):{
                // choose scale axis
                //std::cout <<  vp.w/(double)vp.h << "<" << cam->getAspectRatio() << std::endl;
                if( vp.w/(double)vp.h<cam->getAspectRatio() ){
                    double h = vp.w/cam->getAspectRatio();
                    vp.y = (int)(vp.y + (vp.h-h)/2.0);
                    vp.h = (int)h;
                    //glViewport( vp.x, (GLint) (vp.y + (vp.h-h)/2.0), vp.w, (GLsizei)h);
                } else {
                    double w = vp.h*cam->getAspectRatio();
                    vp.x = (int)(vp.x+ (vp.w-w)/2.0);
                    vp.w = (int)w;
                    //glViewport( (GLint)(vp.x+ (vp.w-w)/2.0), vp.y, (GLsizei)w, vp.h);
                }
                break;
            }
            case(SceneCamera::Fixed):{
                //glViewport( vp.x, vp.y, vp.w, vp.h);
                break;
            }
            default:
                //glViewport( vp.x, vp.y, vp.w, vp.h);
                break;
            }
            return vp;
        }

    }


    void drawScene(SceneGraph* graph, CameraGroup::Ptr camGroup, SceneGraph::RenderInfo& info, SceneNode::Ptr node, RenderPreVisitor &previsitor, RenderPostVisitor &postvisitor, bool usePickMatrix = false, int pickx=0, int picky =0){
        if(node.get() == NULL)
            return;

        GLfloat matrix[16];


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
                //RW_WARN("offscreenEnabled: " << offscreenEnabled);
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

            ViewPort vp = getViewPort(cam);
            glViewport( vp.x, vp.y, vp.w, vp.h);


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
                scam->copyToImage();
            }
            if( (scam->_renderToDepth) && scam->_scan25!=NULL){
                scam->bind();
                if(glGetError()>1)
                    RW_WARN("error: " << glGetError());

                //std::cout << "render to depth" << std::endl;
                if(scam->_depthData.size() != (unsigned int)(scam->_scan25->getWidth()*scam->_scan25->getHeight()) )
                    scam->_depthData.resize(scam->_scan25->getWidth()*scam->_scan25->getHeight());

                SceneCamera::Ptr maincam = scam->getMainCamera();

                // copy rendered depth scene to image

                glReadPixels(
                     0, 0,
                     scam->_scan25->getWidth(), scam->_scan25->getHeight(),
                     GL_DEPTH_COMPONENT, GL_FLOAT, &scam->_depthData[0]);

                //glReadPixels(
                //     0, 0,
                //     scam->_scan25->getWidth(), scam->_scan25->getHeight(),
                //     GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, &scam->_depthData[0]);

                {
                    int error = glGetError();
                    if(error>1)
                        RW_WARN("error: " << error);
                }


                GLdouble modelview[16];
                GLdouble projection[16];
                //GLint viewport[4];


                ProjectionMatrix projectionMatrix = maincam->getProjectionMatrix();
                projectionMatrix.toOpenGLMatrix(projection);

                Transform3D<> camtransform = maincam->getTransform();
                if( maincam->getAttachedNode().get() != NULL ){
                    // calculate kinematics from attached node to
                    SceneNode::Ptr parent = maincam->getAttachedNode();
                    //std::cout << parent->getName() << std::endl;
                    do{
                        if(parent->asGroupNode()!=NULL)
                            camtransform = parent->asGroupNode()->getTransform()*camtransform;
                        parent = parent->_parentNodes.front();
                    } while( parent!=maincam->getRefNode() );
                }

                //Transform3D<> viewMatrix = inverse( camtransform );
                Transform3D<> viewMatrix = Transform3D<>::identity(); //inverse( camtransform );
                DrawableUtil::transform3DToGLTransform(viewMatrix, modelview);

                ViewPort vp = getViewPort(maincam);
                //glViewport( vp.x, vp.y, vp.w, vp.h);

                //glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
                //glGetDoublev( GL_PROJECTION_MATRIX, projection );
                //glGetIntegerv( GL_VIEWPORT, viewport );

                std::vector<rw::math::Vector3D<float> >* result = &scam->_scan25->getData();
                // now unproject all pixel values
                if (result != NULL && result->size() != (unsigned int)(scam->_scan25->getWidth()*scam->_scan25->getHeight()))
                    result->resize(scam->_scan25->getWidth()*scam->_scan25->getHeight());

                for(size_t y=0;y<(unsigned int)scam->_scan25->getHeight();y++){
                    for(size_t x=0;x<(unsigned int)scam->_scan25->getWidth();x++){
                        GLfloat depth24_8 = scam->_depthData[x+y*scam->_scan25->getWidth()];
                        double winZ=  depth24_8; //(depth24_8>>8)&0x00FFFFFF;
                        //double winZ= scam->_depthData[x+y*scam->_scan25->getWidth()];

                        double winX=(double)x,winY=(double)y;//

                        double posX, posY, posZ;
                        gluUnProject( winX, winY, winZ,
                                modelview, projection, vp.viewport,
                                &posX, &posY, &posZ);
                        if (result != NULL) {
                            Vector3D<float>& q = (*result)[x+y*scam->_scan25->getWidth()];
                            q(0) = (float)posX;
                            q(1) = (float)posY;
                            q(2) = (float)posZ;
                            //std::cout << q << " " << winZ << "\n";
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

/*
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
*/


SceneCamera::Ptr SceneOpenGL::makeCamera(const std::string& name){

    return ownedPtr(new SceneCamera(name, getRoot() ));
}

rw::common::Ptr<CameraGroup> SceneOpenGL::makeCameraGroup(const std::string& name){
    return ownedPtr( new SimpleCameraGroup(name) );
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& filename, int dmask){
    rwlibs::opengl::Drawable::Ptr drawable = NULL;
     //try {
         drawable = DrawableFactory::getDrawable(filename, filename);
         drawable->setMask(dmask);
     //} catch (const rw::common::Exception& exp){
     //    RW_WARN(exp.getMessage());
     //}
     return drawable;
}


DrawableGeometryNode::Ptr SceneOpenGL::makeDrawableFrameAxis(const std::string& name, double size, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addFrameAxis(size);
    return drawable;
}

DrawableGeometryNode::Ptr SceneOpenGL::makeDrawable(const std::string& name, rw::common::Ptr<rw::geometry::Geometry> geom, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addGeometry(geom);
    return drawable;
}

DrawableGeometryNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const std::vector<rw::geometry::Line >& lines, int dmask){
    DrawableGeometry::Ptr drawable = ownedPtr( new DrawableGeometry(name, dmask) );
    drawable->addLines(lines);
    return drawable;
}



DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name, rw::common::Ptr<Model3D> model, int dmask){
    RenderModel3D::Ptr render = ownedPtr(new RenderModel3D(model));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const rw::sensor::Image& img, int dmask){
    RenderImage::Ptr render = ownedPtr(new RenderImage(img));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name,const rw::geometry::PointCloud& scan, int dmask){
    RenderScan::Ptr render = ownedPtr(new RenderScan(scan));
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}


DrawableNode::Ptr SceneOpenGL::makeDrawable(const std::string& name, rw::common::Ptr<Render> render, int dmask) {
    Drawable::Ptr drawable = ownedPtr( new Drawable(render, name, dmask) );
    return drawable;
}

