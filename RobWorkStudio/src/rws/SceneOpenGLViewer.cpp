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


#include "SceneOpenGLViewer.hpp"

#include <cmath> 

#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>

#include "ArcBallController.hpp"

#include <QThread>
#include <boost/foreach.hpp>

using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::graphics;
using namespace rw::math;
using namespace rw::common;
using namespace rw::geometry;
using namespace rwlibs::opengl;

using namespace rws;

namespace
{

    class RenderQuad : public Render
    {
    private:
        float _size;
        GLUquadricObj *_quadratic;
        GLuint _displayListId;
        GLfloat _colorTop[4],_colorBottom[4];
        int _x,_y,_width,_height;
    public:
        //! @brief smart pointer type to this class
        typedef rw::common::Ptr<RenderQuad> Ptr;

        /* Functions inherited from Render */
        /**
         * @copydoc Render::draw
         */
        void draw(const DrawableNode::RenderInfo& info, DrawType type, double alpha) const {
            glPushMatrix();
            glLoadIdentity();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glBegin(GL_QUADS);
            glColor3fv(_colorBottom);
            glVertex2f(_x, _y);
            glVertex2f(_width, _y);
            glColor3fv(_colorTop);
            glVertex2f(_width, _height);
            glVertex2f(_x, _height);
            glEnd();

            glPopMatrix();
        }

        void setTopColor(const Vector3D<>& color){
            _colorTop[0] = color[0];
            _colorTop[1] = color[1];
            _colorTop[2] = color[2];
        }

        void setBottomColor(const Vector3D<>& color){
            _colorBottom[0] = color[0];
            _colorBottom[1] = color[1];
            _colorBottom[2] = color[2];
        }

        void setViewPort(int x,int y,int width,int height){
            _x = x; _y = y; _width = width; _height = height;
        }

    };

    /**
     * @brief clamp val to either min or max
     *
     * @param val [in] the value that is to be clamped
     * @param min [in] the minimum allowed value
     * @param max [in] the maximum allowed value
     * @return the clamped value of val
     */
    template<class T>
    T clampI(T val, T min, T max)
    {
        if (val < min)
            return min;
        if(val > max)
            return max;
        return val;
    }

    QGLFormat makeQGLFormat(PropertyMap *map){
        QGLFormat glf = QGLFormat::defaultFormat();

        if(map!=NULL){
            int nrSamples = map->add<int>("GL_NR_SAMPLES","",4)->getValue();
            if( map->add<bool>("GL_MULTISAMPLE","",false)->getValue() ){
                nrSamples = clampI(nrSamples, 0, 8);
                glf.setSampleBuffers(true);
                glf.setSamples(nrSamples);
            }
        }
        return glf;
    }

}

void SceneOpenGLViewer::init(){
    // extract the propertymap from
    //_pmap = _rwStudio->getPropertyMap().add<PropertyMap>("SceneViewer","",PropertyMap());

    _pmap->getValue().add<bool>("ReInitializeGL","Reinitializes the opengl configuration.",false)->changedEvent().add(
            boost::bind(&SceneOpenGLViewer::propertyChangedListener,this,_1), this );

    //_pmap->add<Vector3D<> >("BackGroundColorBottom","desc",Vector3D<>(0,0,0) )->changedEvent().add(
    //        boost::bind(&SceneOpenGLViewer::propertyChangedListener,this,_1), this );

    _viewBackground = _pmap->getValue().add<bool>("DrawBackGround", "Draw Back Ground", true);
    _viewBackground->changedEvent().add(boost::bind(&SceneOpenGLViewer::propertyChangedListener,this, _1));

    //this->setCheckForCollision(check);

    // add the default/main cameraview group
    _mainCamGroup = _scene->makeCameraGroup("MainView");
    _scene->addCameraGroup(_mainCamGroup);
    _mainCamGroup->setEnabled(true);

    // add a node to render background
    rw::common::Ptr<RenderQuad> backgroundRender = ownedPtr(new RenderQuad());
    backgroundRender->setTopColor(Vector3D<>(1.0,1.0,1.0));
    backgroundRender->setBottomColor(Vector3D<>(0.2,0.2,1.0));
    backgroundRender->setViewPort(0,0,640,480);
    _backgroundRender = backgroundRender;
    DrawableNode::Ptr node = _scene->makeDrawable("BackgroundRender", _backgroundRender, DrawableNode::ALL);
    _scene->addChild(node, _scene->getRoot());

    _worldNode = _scene->makeGroupNode("World");
    _scene->addChild(_worldNode, _scene->getRoot());

    _mainView = ownedPtr( new SceneViewer::View("MainView") );
    _currentView = _mainView;
    // add background camera
    _backCam = _scene->makeCamera("BackgroundCam");
    _backCam->setEnabled(true);
    _backCam->setRefNode(node);
    _backCam->setProjectionMatrix( ProjectionMatrix::makeOrtho(0,640,0,480, -1, 1) );
    _backCam->setClearBufferEnabled(true);
    _backCam->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    _backCam->setDepthTestEnabled( false );
    _backCam->setLightningEnabled( false );
    _mainCamGroup->insertCamera(_backCam, 0);

    // main camera
    _mainCam = _scene->makeCamera("MainCam");
    _mainCam->setDrawMask( DrawableNode::ALL );
    _mainCam->setEnabled(false);
    _mainCam->setPerspective(45, 640, 480, 0.1, 30);
    _mainCam->setClearBufferEnabled(false);
    _mainCam->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    _mainCam->setDepthTestEnabled( true );
    _mainCam->setLightningEnabled( true );
    _mainCam->setRefNode(_scene->getRoot());
    _mainCamGroup->insertCamera(_mainCam, 1);
    // TODO: foreground camera
    _mainView->_viewCamera = _mainCam;
    _mainView->_camGroup = _mainCamGroup;
    _pivotDrawable = NULL; //= _scene->makeDrawable("Pivot", Geometry::makeBox(1.0,1.0,1.0), DrawableNode::Virtual);
    //_pivotDrawable = _scene->makeDrawableFrameAxis("Pivot", 1.0, DrawableNode::Virtual );
    //_scene->addChild(_pivotDrawable, _scene->getRoot());
    //_pivotDrawable->setColor( Vector3D<>(1.0f, 0.0f, 0.0f) );

/*
    _currCam = ownedPtr( new SceneCamera() );
    _currCam->setPerspective(45, 640, 480, 0.1, 30);
    _currCam->setClearBufferEnabled(true);
    _currCam->setClearBufferMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    _cameraViews.push_back( _currCam );
*/
    this->setFocusPolicy(Qt::StrongFocus);
}

SceneViewer::View::Ptr SceneOpenGLViewer::createView(const std::string& name){
    SceneViewer::View::Ptr nview = ownedPtr( new SceneViewer::View(name) );
    nview->_viewCamera = _scene->makeCamera("ViewCamera");
    nview->_camGroup = _scene->makeCameraGroup("ViewCamera");

    nview->_viewCamera->setEnabled(true);
    nview->_viewCamera->setPerspective(45, 640, 480, 0.1, 30);
    nview->_viewCamera->setClearBufferEnabled(true);
    nview->_viewCamera->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    nview->_viewCamera->setDepthTestEnabled( true );
    nview->_viewCamera->setLightningEnabled( true );
    nview->_viewCamera->setRefNode(_scene->getRoot());
    nview->_viewCamera->setViewport(0,0,_width,_height);
    nview->_camGroup->insertCamera(nview->_viewCamera, 0);
    nview->_camGroup->setEnabled(true);
    nview->_viewCamera->setAspectRatioControl(SceneCamera::Scale);
    _views.push_back(nview);
    return nview;
}


SceneOpenGLViewer::SceneOpenGLViewer(QWidget* parent):
            //QGLWidget( QGLFormat(QGL::DepthBuffer), parent),
            QGLWidget( makeQGLFormat(NULL), parent),
            _scene( ownedPtr(new SceneOpenGL()) ),
            _logoFont("Helvetica [Cronyx]", 24, QFont::DemiBold , true),
            _viewLogo("RobWork"),
            _cameraCtrl( ownedPtr( new ArcBallController(640,480) ) )
{
    // start by initializing propertymap
    _pmap = ownedPtr( new Property<PropertyMap>("SceneViewer","",PropertyMap()) );
    _pmap->getValue().add<int>("GL_NR_SAMPLES","",4);
    _pmap->getValue().add<bool>("GL_MULTISAMPLE","",false);
    init();
}

SceneOpenGLViewer::SceneOpenGLViewer(PropertyMap& pmap, QWidget* parent) :
    //QGLWidget( QGLFormat(QGL::DepthBuffer) , parent),
    QGLWidget( makeQGLFormat( pmap.getPtr<PropertyMap>("SceneViewer") ) , parent),
    _scene( ownedPtr(new SceneOpenGL()) ),
    _logoFont("Helvetica [Cronyx]", 24, QFont::DemiBold , true),
    _viewLogo("RobWork"),
    _cameraCtrl( ownedPtr( new ArcBallController(640,480) ) )
{
    // start by initializing propertymap
    _pmap = pmap.add<PropertyMap>("SceneViewer","",PropertyMap());

    init();

}

SceneOpenGLViewer::~SceneOpenGLViewer()
{
}


void SceneOpenGLViewer::setWorldNode(rw::graphics::GroupNode::Ptr wnode){
    RW_ASSERT( wnode!=NULL );

    if(_pivotDrawable==NULL){
        /// TODO: this should be simplified to orthographic camera view. And only drawn in 2D
        _pivotDrawable = _scene->makeDrawable("Pivot", Geometry::makeSphere(0.01), DrawableNode::Virtual);
        //_pivotDrawable = _scene->makeDrawableFrameAxis("Pivot", 1.0, DrawableNode::Virtual );
        _scene->addChild(_pivotDrawable, _scene->getRoot());
        _pivotDrawable->setColor( Vector3D<>(1.0f, 0.0f, 0.0f) );

    }

    if(wnode == NULL){
        _mainCam->setEnabled(false);
    } else {
        _mainCam->setEnabled(true);
    }

    // reattach the pivot drawable
    _scene->removeDrawable(_pivotDrawable);
    _scene->getRoot()->removeChild(_worldNode);
    _worldNode = wnode;
    if(!_scene->getRoot()->hasChild(_worldNode)){
        _scene->addChild(_worldNode, _scene->getRoot());
    }
    _scene->addChild(_pivotDrawable, _worldNode);

    _mainCam->setRefNode(_worldNode);
    RW_ASSERT(_worldNode->hasChild(_pivotDrawable)==true);
    RW_ASSERT(_pivotDrawable->hasParent(_worldNode)==true);
}

void SceneOpenGLViewer::keyPressEvent(QKeyEvent *e)
{
    e->ignore();
    QGLWidget::keyPressEvent(e);
}

void SceneOpenGLViewer::clear()
{
    // reset everything
}

void SceneOpenGLViewer::initializeGL()
{
    /****************************************/
    /* Set up OpenGL lights etc.            */
    /****************************************/
    if( _pmap->getValue().add<bool>("GL_DEPTH_TEST","Enable depth testing",true) ){
        glEnable(GL_DEPTH_TEST);
        std::string desc("Possible values: GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, GL_GREATER, GL_NOTEQUAL, GL_GEQUAL, GL_ALWAYS");
        int val = _pmap->getValue().add<int>("glDepthFunc",desc,GL_LESS)->getValue();
        glDepthFunc(val); // GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL, GL_GREATER, GL_NOTEQUAL, GL_GEQUAL, GL_ALWAYS
    } else {
        glDisable(GL_DEPTH_TEST);
    }
    int val = _pmap->getValue().add<int>("glShadeModel","",GL_SMOOTH)->getValue();
    glShadeModel(val);   // GL_FLAT, GL_SMOOTH

    //glDisable( GL_COLOR_MATERIAL );

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    if( _pmap->getValue().add<bool>("GL_LIGHTING","",true)->getValue() )
        glEnable( GL_LIGHTING );
    if( _pmap->getValue().add<bool>("GL_NORMALIZE","",true)->getValue() )
        glEnable( GL_NORMALIZE );
    if( _pmap->getValue().add<bool>("GL_DEPTH_TEST","",true)->getValue() )
        glEnable( GL_DEPTH_TEST );

    GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.1f, 1.0f};
    GLfloat light0_diffuse[] =  {.8f, .8f, 0.8f, 1.0f};
    GLfloat light0_specular[] = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat light0_position[] = {0.0f, 0.0f, 1.0f, 0.0f}; // point light, from above

    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

    glEnable(GL_LIGHT0);
    DrawableUtil::setupHighlightLight();
    glEnable(GL_COLOR_MATERIAL);
    //GLenum matRendering = GL_FRONT_AND_BACK;
    GLenum matRendering = GL_FRONT;
    glColorMaterial(matRendering, GL_AMBIENT_AND_DIFFUSE);
    GLfloat specularReflection[]={1.0,1.0,1.0,1.0};
    glMaterialfv(matRendering, GL_SPECULAR, specularReflection);
    glMateriali(matRendering, GL_SHININESS, 128);

    //glEnable(GL_COLOR_MATERIAL);
    //glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);


    if( _pmap->getValue().add<bool>("GL_LINE_SMOOTH","",true)->getValue() ){
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST); // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    } else {
        glDisable(GL_LINE_SMOOTH);
    }

    if( _pmap->getValue().add<bool>("GL_POINT_SMOOTH","",false)->getValue() ){
        glEnable(GL_POINT_SMOOTH);
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    } else {
        glDisable(GL_POINT_SMOOTH);
    }

    if( _pmap->getValue().add<bool>("GL_POLYGON_SMOOTH","",false)->getValue() ){
        glEnable(GL_POLYGON_SMOOTH);
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST); // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    } else {
        glDisable(GL_POLYGON_SMOOTH);
    }

    //if( _pmap->getValue().add<bool>("GL_PERSPECTIVE_CORRECTION","",false)->getValue() ){
    //    glEnable(GL_PERSPECTIVE_CORRECTION);
    //    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // GL_FASTEST, GL_NICEST, GL_DONT_CARE
    //} else {
    //    glDisable(GL_PERSPECTIVE_CORRECTION);
    //}

    if( _pmap->getValue().add<bool>("GL_BLEND","",true)->getValue() ){
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //glBlendFunc(GL_ONE_MINUS_DST_ALPHA,GL_DST_ALPHA);
        //glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
    } else {
        glDisable(GL_BLEND);
    }

    if( _pmap->getValue().add<bool>("GL_ALHPA_TEST","",false)->getValue() ){
        glEnable(GL_ALPHA_TEST); // allows alpha channels or transperancy
        glAlphaFunc(GL_GREATER, 0.1f); // sets aplha function
    } else {
        glDisable(GL_ALPHA_TEST);
    }
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

void SceneOpenGLViewer::paintGL()
{
    if( _cameraCtrl ){
        // for now we scale the pivot such that its not unrealistically large/small
        getViewCamera()->setTransform(_cameraCtrl->getTransform());

        //double dist = -(inverse(_cameraCtrl->getTransform())*_pivotDrawable->getTransform().P())[2];
        //_pivotDrawable->setScale( Math::clamp(dist/5.0,0.00001,1) ); // 5/5
    }

    // update the position of the light
/*    Transform3D<> camTw = inverse(getViewCamera()->getTransform());
    Transform3D<> wTlight = Transform3D<>(Vector3D<>(0,0,20));
    Vector3D<> lightPos = (camTw*wTlight).P();// Vector3D<>(0,0,1);
    //Vector3D<> lightPos = (camTw.R() * Vector3D<>(0,0,1) );

    GLfloat lpos[] = {0.0f, 0.0f, 1.0f, 1.0f};
    lpos[0] = lightPos[0];
    lpos[1] = lightPos[1];
    lpos[2] = lightPos[2];
    glLightfv(GL_LIGHT0, GL_POSITION, lpos);
*/
    //std::cout << _currentView->_name << std::endl;
    _renderInfo._mask = DrawableNode::ALL;
    _renderInfo.cams = _currentView->_camGroup;
    _scene->draw( _renderInfo );
}

void SceneOpenGLViewer::resizeGL(int width, int height)
{
    _width = width;
    _height = height;

    BOOST_FOREACH(SceneCamera::Ptr cam, _currentView->_camGroup->getCameras()){
        cam->setViewport(0,0,_width,_height);
    }

    ((RenderQuad*)_backgroundRender.get())->setViewPort(0,0,_width,_height);
    _cameraCtrl->setBounds(width, height);
}

void SceneOpenGLViewer::selectView(View::Ptr view){
    _currentView = view;
    BOOST_FOREACH(SceneCamera::Ptr cam, _currentView->_camGroup->getCameras()){
        cam->setViewport(0,0,_width,_height);
    }
}

void SceneOpenGLViewer::destroyView(View::Ptr view){
    //TODO: remove camera from scene
    if( _mainView==view ){
        RW_THROW("The View \"" << view->_name << "\" is the MainView, which cannot be removed!");
    }

    if(_currentView == view || _mainView==view){
        RW_THROW("The View \"" << view->_name << "\" is Active. Please select another view before removing it!");
    }

    std::vector<View::Ptr> nviews;
    BOOST_FOREACH(View::Ptr v, _views){
        if(v!=view)
            nviews.push_back(v);
    }
    _views = nviews;

    _scene->removeCameraGroup( view->_camGroup );
}



/*

#define GL_SELECT_BUFSIZE 512
GLuint _selectBuf[GL_SELECT_BUFSIZE];

rw::kinematics::Frame* SceneOpenGLViewer::pickFrame(int cursorX, int cursorY){

    // Start picking
    GLint viewport[4];

    glSelectBuffer(GL_SELECT_BUFSIZE, _selectBuf);
    glRenderMode(GL_SELECT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glGetIntegerv(GL_VIEWPORT, viewport);

    gluPickMatrix(cursorX,cursorY,3,3,viewport);
    const GLCameraView& v = _cameraViews[0];

    //gluPerspective(45, ratio, 0.1, 1000);
    //setupCameraView(_cameraNr,false);
    GLdouble aspect = (GLdouble)v.width / v.height;
    //gluPerspective(fieldOfView, aspectRatio, near, far);
    gluPerspective((GLdouble)v.fovy, aspect, (GLdouble)v.vnear, (GLdouble)v.vfar);
    //gluOrtho2D(0, v.width, 0, v.height);

    // draw pickable objects
    // Setup projection to draw background
    glMatrixMode(GL_MODELVIEW);
    glInitNames();
    // Setup the correct camera projection
    glLoadIdentity();

    if( _cameraNr==0 ){
        drawGLStuff(false);
    } else {
        glScalef(_zoomScale,_zoomScale,_zoomScale);
    }

    if (_cell) {
        // draw the workcell.
        _workcellGLDrawer->drawAndSelect(*_cell.state, _cell.workcell);
    }

    // stop picking

    // restoring the original projection matrix
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glFlush();

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

       //std::cout << "Z depth: " << depthZ << " names: " << names << std::endl;
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

   Frame *frame = _cell.state->getFrame(*ptr);
   //if( frame!=NULL )
   //    std::cout << "-- " << frame->getName() << std::endl;

   return frame;
    return NULL;
}
*/

DrawableNode::Ptr SceneOpenGLViewer::pickDrawable(int x, int y){
    SceneGraph::RenderInfo info;
    info._mask = DrawableNode::ALL;
    return _scene->pickDrawable(info, x, y);
}

void SceneOpenGLViewer::mouseDoubleClickEvent(QMouseEvent* event)
{
    // 6/5/2010
    if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ControlModifier) {
/*
        int winx = event->x();
        int winy = height()-event->y();
        // we pick the scene before
        Frame *frame = pickFrame(winx,winy);
        if( frame ){
            _rwStudio->frameSelectedEvent().fire( frame );
        }
*/
    } else if (event->button() == Qt::LeftButton) {
        int winx = event->x();
        int winy = height()-event->y();

        Vector3D<> pos = _scene->unproject(_mainCam, winx, winy);
        if (pos[2] != 1) {
            if (event->modifiers() == Qt::ShiftModifier) {
                positionSelectedEvent().fire( pos );
            } else {
                _cameraCtrl->setCenter(pos, Vector2D<>(event->x(), event->y()));
                _pivotDrawable->setTransform( Transform3D<>(pos, Rotation3D<>::identity()) );
                updateGL();
            }
        }

    } else {
        event->ignore();
    }
    std::cout << "forward double click" << std::endl;
    QGLWidget::mouseDoubleClickEvent(event);
}

void SceneOpenGLViewer::mousePressEvent(QMouseEvent* event)
{
	// 6/5/2010
    _cameraCtrl->handleEvent( event );

    /*
    if (event->buttons() == Qt::RightButton &&
        event->modifiers() == Qt::ControlModifier)
    {
        //saveBufferToFileQuery();
    }
    */
    //_rwStudio->mousePressedEvent().fire(event);

    //event->ignore();
    QGLWidget::mousePressEvent(event);
}

void SceneOpenGLViewer::mouseMoveEvent(QMouseEvent* event)
{
    _cameraCtrl->handleEvent(event);

    //std::cout<<"Event Time"<<eventTimer.getTime()<<std::endl;
    updateGL();

    //event->ignore();
    QGLWidget::mouseMoveEvent(event);
}

void SceneOpenGLViewer::wheelEvent(QWheelEvent* event)
{
    _cameraCtrl->handleEvent( event );
    updateGL();
    QGLWidget::wheelEvent(event);
}

void SceneOpenGLViewer::saveBufferToFile(const std::string& stdfilename)
{
    QString filename(stdfilename.c_str());
    /*
    const char* type = "PNG";
    if (filename.endsWith(".BMP", Qt::CaseInsensitive))
        type = "BMP";
    else if (filename.endsWith(".JPG", Qt::CaseInsensitive))
        type = "JPG";
    else if (filename.endsWith(".PNG", Qt::CaseInsensitive))
        type = "PNG";
    else {
        throw std::string(
            "SceneOpenGLViewer::saveBufferToFile: The selected file format is not supported");
    }
    */
    QImage img = grabFrameBuffer();
    if(_currentView->_viewCamera->getAspectRatioControl()==SceneCamera::Fixed){

        int x,y,w,h;
        _currentView->_viewCamera->getViewport(x,y,w,h);

        // Get height of grabbed image
        const int width = img.size().rwidth(),
                  height = img.size().rheight();

        // Instantiate result
        QImage dstimg;
        if(height >= h) { // If the image is taller than the camera view port
            // Move down to where the image starts and copy
            dstimg = img.copy(0, height-h, w, h);
        } else { // Else
            // Fill
            dstimg = QImage(w, h, img.format());
            dstimg.fill(qRgb(0, 0, 0));
            // Insert at bottom of destination
            const int yOffset = h - height;
            for(int x = 0; x < std::min(width, w); ++x) {
                for(int y = yOffset; y < h; ++y) {
                    dstimg.setPixel(x, y, img.pixel(x, y-yOffset));
                }
            }
        }

        img = dstimg;
    }

    if (!img.save(filename))
        throw std::string(
            "SceneOpenGLViewer::saveBufferToFile: Could not save file") +
            filename.toStdString();
}



void SceneOpenGLViewer::propertyChangedListener(PropertyBase* base){
    std::string id = base->getIdentifier();
    std::cout << "Property Changed Listerner SceneOpenGLViewer: " << id << std::endl;

    if(id=="ShowCollisionModels"){
        Property<bool> *p = toProperty<bool>(base);
        if(p==NULL)
            return;
        if(p->getValue()) getViewCamera()->setDrawMask( DrawableNode::CollisionObject | DrawableNode::Virtual );
        else getViewCamera()->setDrawMask( DrawableNode::DrawableObject | DrawableNode::Physical | DrawableNode::Virtual );
    } else if(id=="BackGroundColorBottom"){

    } else if(id=="DrawWorldGrid" ){

    }

}
