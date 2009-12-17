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


#include "ViewGL.hpp"

#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>

#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/drawable/DrawableUtil.hpp>

#include "ArcBall.hpp"

#include "RobWorkStudio.hpp"

#include <QThread>
#include <boost/foreach.hpp>

const int WHEEL_DELTA_ZOOM = 120;

// Background Color definitions
const GLfloat TOP_BG_COLOR[] = {1.0f,1.0f,1.0f};
const GLfloat BOTTOM_BG_COLOR[] = {0.2f,0.2f,1.0f};


using namespace robwork;
using namespace rw::proximity;
using namespace rwlibs::drawable;
using namespace rw::kinematics;

namespace
{
    Timer eventTimer;

    void setDrawablesHighlighted(
        const std::vector<Drawable*>& drawables,
        bool value)
    {
        BOOST_FOREACH(Drawable* da, drawables) { da->setHighlighted(value); }
    }

    void setPairHighlighted(
        WorkCellGLDrawer& drawer,
        const FramePair& pair,
        bool value)
    {
        setDrawablesHighlighted(drawer.getDrawablesForFrame(pair.first), value);
        setDrawablesHighlighted(drawer.getDrawablesForFrame(pair.second), value);
    }

    void setCollisionPairsHighlighted(
        WorkCellGLDrawer& drawer,
        const FramePairSet& pairs,
        bool value)
    {
        BOOST_FOREACH(const FramePair& pair, pairs) {
        	setPairHighlighted(drawer, pair, value);
        }
    }

    FramePairSet highLightCollisionPairsStep(
        const State& state,
        CollisionDetector& detector,
        const FramePairSet& previous,
        WorkCellGLDrawer& drawer)
    {
        setCollisionPairsHighlighted(drawer, previous, false);

        FramePairSet current;
        detector.inCollision(state, &current);

        setCollisionPairsHighlighted(drawer, current, true);
        return current;
    }

    void setOrthographicProjection(int width, int height)
    {
        // switch to projection mode
        glMatrixMode(GL_PROJECTION);
        // save previous matrix which contains the
        //settings for the perspective projection
        glPushMatrix();
        // reset matrix
        glLoadIdentity();
        // set a 2D orthographic projection
        //glOrtho (0, _width, _height, 0, 0, 1);
        gluOrtho2D(0, width, 0, height);
        //gluOrtho2D(0, w, 0, h);
        // invert the y axis, down is positive
        //glScalef(1, -1, 1);
        // mover the origin from the bottom left corner
        // to the upper left corner
        //glTranslatef(0, -768, 0);
        glMatrixMode(GL_MODELVIEW);
    }

    void resetPerspectiveProjection()
    {
        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
    }

    /* Draws grid in xy-plane */
    void drawWorldGrid(float size, float resolution){
        glDisable(GL_LIGHTING);
        //glEnable(GL_LIGHT7);

        glPushAttrib(GL_CURRENT_BIT);
        glColor4f(0.8f, 0.8f, 0.8f, 1.0f);
        float halfsize = size/2;
        glBegin(GL_LINES);

        for (size_t i=0; i*resolution <= halfsize; i++){
            glVertex4f(  halfsize,  resolution*i , 0 , 0.5);
            glVertex4f( -halfsize,  resolution*i , 0 , 0.5);
            glVertex4f(  halfsize, -resolution*i , 0 , 0.5);
            glVertex4f( -halfsize, -resolution*i , 0 , 0.5);
            glVertex4f(  resolution*i,  halfsize , 0 , 0.5);
            glVertex4f(  resolution*i, -halfsize , 0 , 0.5);
            glVertex4f( -resolution*i,  halfsize , 0 , 0.5);
            glVertex4f( -resolution*i, -halfsize , 0 , 0.5);
        }
        glEnd();
        glPopAttrib();
        glEnable(GL_LIGHTING);
    }

    void drawCamera(const ViewGL::GLCameraView& cam)
    {
    	// calculate  aspect and fovy
    	double aspect = cam.width/(double)cam.height;
    	double fovy = cam.fovy * Deg2Rad * 0.5;
    	double z = -2.0;
    	double y = tan(fovy)*z;
    	double x = y*aspect;
    	double ynear = tan(fovy)*cam.vnear;
    	double xnear = ynear*aspect;
    	//std::cout << x << " " << y << std::endl;
    	// and now draw it
        glDisable(GL_LIGHTING);
        glPushAttrib(GL_CURRENT_BIT);
        glColor4f(0.5,0.5,0.5,1.0);
        glBegin(GL_LINES);
        {
        	glVertex4f(  0 ,  0 ,  0 , 0.5);
            glVertex4f(  x ,  y ,  z , 0.5);
            glVertex4f(  0 ,  0 ,  0 , 0.5);
            glVertex4f(  x , -y ,  z , 0.5);
        	glVertex4f(  0 ,  0 ,  0 , 0.5);
            glVertex4f( -x ,  y ,  z , 0.5);
            glVertex4f(  0 ,  0 ,  0 , 0.5);
            glVertex4f( -x , -y ,  z , 0.5);

            glVertex4f(  x ,  y ,  z , 0.5);
            glVertex4f( -x ,  y ,  z , 0.5);
            glVertex4f(  x , -y ,  z , 0.5);
            glVertex4f(  x ,  y ,  z , 0.5);

            glVertex4f(  x ,  -y ,  z , 0.5);
            glVertex4f( -x ,  -y ,  z , 0.5);
            glVertex4f( -x ,  -y ,  z , 0.5);
            glVertex4f( -x ,   y ,  z , 0.5);

            // draw near clip
            glVertex4f(  xnear ,  ynear ,  -cam.vnear , 0.5);
            glVertex4f( -xnear ,  ynear ,  -cam.vnear , 0.5);
            glVertex4f(  xnear , -ynear ,  -cam.vnear , 0.5);
            glVertex4f(  xnear ,  ynear ,  -cam.vnear , 0.5);

            glVertex4f(  xnear ,  -ynear ,  -cam.vnear , 0.5);
            glVertex4f( -xnear ,  -ynear ,  -cam.vnear , 0.5);
            glVertex4f( -xnear ,  -ynear ,  -cam.vnear , 0.5);
            glVertex4f( -xnear ,   ynear ,  -cam.vnear , 0.5);
        }
        glEnd();
        glPopAttrib();
        glEnable(GL_LIGHTING);
    }

    void drawPivot(const Vector3D<float>& pos, float radius, float scale, GLUquadricObj* sphere){
        // Save and restore the color so that everything doesn't turn red.
        glPushAttrib(GL_CURRENT_BIT);
        {
            glPushMatrix();
            glColor3f(1,0,0);
            glTranslatef(pos(0)*scale, pos(1)*scale, pos(2)*scale);
            gluSphere(sphere, radius, 8, 8);
            glPopMatrix();
        }
        glPopAttrib();
    }

}

ViewGL::ViewGL(RobWorkStudio* rwStudio, QWidget* parent) :
    QGLWidget(QGLFormat(QGL::DepthBuffer), parent),
    _viewRotation(RPY<float>( 0, 0, -45*Deg2Rad ).toRotation3D()),
    _viewPos(0,0,-5),
    _drawType(Render::SOLID),
    _alpha(1),
    _width(640),
    _height(480),
    _cameraCtrl( new ArcBall(_width,_height) ),
    _zoomFactor(0.0),
    _zoomScale(1.0),
    _rwStudio(rwStudio),
    _workcellGLDrawer(rwStudio->getWorkCellGLDrawer()),
    _cameraNr(0),
    _logoFont("Helvetica [Cronyx]", 24, QFont::DemiBold , true),
    _viewLogo("RobWork")
{
    // add the default cameraview
    _cameraViews.push_back( GLCameraView(60, _width, _height, NULL) );

    _sphereObj = gluNewQuadric();
    _showSolidAction = new QAction(QIcon(":/images/solid.png"), tr("&Solid"), this); // owned
    _showSolidAction->setCheckable(true);
    _showSolidAction->setChecked(true);

    connect(_showSolidAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot()));

    _showWireAction = new QAction(QIcon(":/images/wireframe.png"), tr("&Wire"), this); // owned
    _showWireAction->setCheckable(true);
    connect(_showWireAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot()));

    _showOutlineAction = new QAction(
        QIcon(":/images/outline.png"), tr("&Outline"), this); // owned

    _showOutlineAction->setCheckable(true);
    connect(_showOutlineAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot()));

    QActionGroup* viewtypes = new QActionGroup(this); // owned
    viewtypes->addAction(_showSolidAction);
    viewtypes->addAction(_showWireAction);
    viewtypes->addAction(_showOutlineAction);

    _showTransparentAction =
        new QAction(QIcon(":/images/transparent.png"), tr("&Transparent"), this); // owned
    _showTransparentAction->setCheckable(true);
    connect(
        _showTransparentAction,
        SIGNAL(triggered()),
        this,
        SLOT(setTransparentSlot()));

    _showPivotPointAction =
        new QAction(QIcon(":/images/pivotpoint.png"), tr("Pivot Point"), this); // owned
    _showPivotPointAction->setCheckable(true);
    _showPivotPointAction->setChecked(true);

    // We set the GUI element to checked, but we happen to also need to set this
    // to true manually.
    _showPivotPoint = true;

    connect(_showPivotPointAction, SIGNAL(triggered()), this, SLOT(showPivotPointSlot()));

    _checkForCollision =
        new QAction(QIcon(":/images/collision.png"), tr("Collision Checking"), this); // owned
    _checkForCollision->setCheckable(true);
    _checkForCollision->setChecked(true);
    connect(
        _checkForCollision,
        SIGNAL(triggered(bool)),
        this,
        SLOT(setCheckForCollision(bool)));

    _saveBufferToFileAction = new QAction(tr("Save view..."), this); // owned
    connect(
        _saveBufferToFileAction,
        SIGNAL(triggered()),
        this,
        SLOT(saveBufferToFileQuery()));

    this->setFocusPolicy(Qt::StrongFocus);
}

ViewGL::~ViewGL()
{
    gluDeleteQuadric(_sphereObj);
}

void ViewGL::keyPressEvent(QKeyEvent *e)
{
    size_t camNr=0;
    switch(e->key()){
    case(Qt::Key_1): camNr = 0; break;
    case(Qt::Key_2): camNr = 1; break;
    case(Qt::Key_3): camNr = 2; break;
    case(Qt::Key_4): camNr = 3; break;
    case(Qt::Key_5): camNr = 4; break;
    default:
        _rwStudio->keyEvent().fire(e->key(), e->modifiers());
        return;
    }
    e->accept();

    if (camNr>_cameraViews.size()){
        _rwStudio->keyEvent().fire(e->key(), e->modifiers());
        return;
    } else {
        _cameraNr = camNr;
        updateGL();
    }

    _rwStudio->keyEvent().fire(e->key(), e->modifiers());
}

void ViewGL::setupToolBar(QToolBar* toolbar)
{
    toolbar->addAction(_showSolidAction);
    toolbar->addAction(_showWireAction);
    toolbar->addAction(_showOutlineAction);

    toolbar->addSeparator();

    toolbar->addAction(_showTransparentAction);
    toolbar->addAction(_showPivotPointAction);
    toolbar->addAction(_checkForCollision);
}

void ViewGL::setupMenu(QMenu* menu)
{
    menu->addAction(_showSolidAction);
    menu->addAction(_showWireAction);
    menu->addAction(_showOutlineAction);

    menu->addSeparator();

    menu->addAction(_showTransparentAction);
    menu->addAction(_showPivotPointAction);
    menu->addAction(_checkForCollision);

    menu->addSeparator();

    menu->addAction(_saveBufferToFileAction);
}

void ViewGL::addDrawable(Drawable* drawable)
{
    _drawables.push_back(drawable);
}

void ViewGL::addWorkCell(
    WorkCell* workcell,
    State* state,
    CollisionDetector* collisionDetector)
{
    assert(workcell && state);

    _cell = Cell(workcell, state, collisionDetector);

    // add camera views (frames with camera property)

    BOOST_FOREACH(
        Frame* frame,
        Kinematics::findAllFrames(workcell->getWorldFrame(), *state)) {

        if (frame->getPropertyMap().has("Light")) {
            std::cout << "Parsing light source!" << std::endl;
            GLLightSource ls;
            int source;
            std::string camParam = frame->getPropertyMap().get<std::string>("Light");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> source >>
                ls.pos[0] >> ls.pos[1] >> ls.pos[2] >> ls.pos[3] >>
                ls.ambient[0] >> ls.ambient[1] >> ls.ambient[2] >> ls.ambient[3] >>
                ls.diffuse[0] >> ls.diffuse[1] >> ls.diffuse[2] >> ls.diffuse[3];
            GLenum light;
            switch(source){
            case(1):ls.light = GL_LIGHT1;break;
            case(2):ls.light = GL_LIGHT2;break;
            case(3):ls.light = GL_LIGHT3;break;
            case(4):ls.light = GL_LIGHT4;break;
            case(5):ls.light = GL_LIGHT5;break;
            case(6):ls.light = GL_LIGHT6;break;
            }

            _lights.push_back(ls);
            ls.init();
            ls.enable();
        }

    	if (frame->getPropertyMap().has("Camera")) {
    		double fovy;
    		int width,height;
    		std::string camId("Camera");
    		std::string camParam = frame->getPropertyMap().get<std::string>(camId);
    		std::istringstream iss (camParam, std::istringstream::in);
    		iss >> fovy >> width >> height;
    	    GLCameraView view(fovy, width, height, frame);
    	    addCameraView(view);
    	}
    }
    getAllDrawables(_cell);
}

void ViewGL::clear()
{
    BOOST_FOREACH(GLLightSource &ls, _lights){ls.disable();}
    _lights.clear();
    _cameraNr = 0;
    _drawables.clear();
    _cameraViews.clear();
	_cameraViews.push_back( GLCameraView(60, _width, _height, NULL) );
    _cell = Cell();
}

std::vector<Drawable*> ViewGL::getAllDrawables(const Cell& cell)
{
    return _workcellGLDrawer->getAllDrawables(*cell.state, cell.workcell);
}

void ViewGL::setDrawType(Render::DrawType drawType)
{
    // set DrawType for all Drawable in the view
    BOOST_FOREACH(Drawable* da, _drawables) { da->setDrawType(drawType); }

    // set DrawType for all drawables in the workcell.
    if (_cell) {
        BOOST_FOREACH(Drawable* da, getAllDrawables(_cell)) {
            da->setDrawType(drawType);
        }
    }
}

void ViewGL::setDrawTypeSlot()
{
    if (_showSolidAction->isChecked())
        setDrawType(Render::SOLID);
    else if (_showWireAction->isChecked())
        setDrawType(Render::WIRE);
    else if (_showOutlineAction->isChecked())
        setDrawType(Render::OUTLINE);

    updateGL();
}

void ViewGL::setCheckForCollision(bool check)
{
    _checkForCollision->setChecked(check);
    if (_cell) {
        if (!check) {
            setCollisionPairsHighlighted(
                *_workcellGLDrawer,
                _cell.collisionPairs,
                false);
        }
    }

    updateGL();
}

void ViewGL::setTransparentSlot()
{
    double alpha;
    if (_showTransparentAction->isChecked())
        alpha = 0.5;
    else
        alpha = 1.0;

    // set alpha for all Drawable in the view
    BOOST_FOREACH(Drawable* da, _drawables) { da->setAlpha(alpha); }

    // set alpha for the workcell.
    if (_cell) {
        BOOST_FOREACH(Drawable* da, getAllDrawables(_cell)) {
            da->setAlpha(alpha);
        }
    }

    updateGL();
}

void ViewGL::showPivotPointSlot()
{
    showPivotPoint(_showPivotPointAction->isChecked());
}

void ViewGL::showPivotPoint(bool visible)
{
    _showPivotPoint = visible;
    updateGL();
}

void ViewGL::initializeGL()
{
    /****************************************/
    /* Set up OpenGL lights etc.            */
    /****************************************/
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH);   // Enable smooth shading.

    //glDisable( GL_COLOR_MATERIAL );

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glEnable( GL_LIGHTING );
    glEnable( GL_NORMALIZE );
    glEnable( GL_DEPTH_TEST );

    GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.1f, 1.0f};
    GLfloat light0_diffuse[] =  {.8f, .8f, 0.8f, 1.0f};
    GLfloat light0_specular[] = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat light0_position[] = {0.0f, 0.0f, 60.0f, 0.0f}; // point light, from above

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

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glEnable(GL_POINT_SMOOTH);
    //glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    //glEnable(GL_POLYGON_SMOOTH);
    //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    //glEnable(GL_PERSPECTIVE_CORRECTION);
    //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);

    glAlphaFunc(GL_GREATER, 0.1f); // sets aplha function
    glEnable(GL_ALPHA_TEST); // allows alpha channels or transperancy

}

void ViewGL::setupCameraView(int camNr, bool setupViewport){
    const GLCameraView& v = _cameraViews[camNr];

    // set camera view port and scale it to width or height
    if( setupViewport ){
        if( camNr == 0){
            glViewport( 0 , 0, (GLsizei)_width, _height);
        } else if( _width/(double)_height < v.width/(double)v.height ){
            // use width
            double height = v.height*_width/(double)v.width;
            glViewport( 0 , (GLint)((_height-height)/2.0), (GLsizei)_width, (GLsizei)height);
        } else {
            double width = v.width*_height/(double)v.height;
            glViewport((GLint)((_width-width)/2.0), 0, (GLsizei)width, (GLsizei)_height);
        }
    }
    // switch to projection mode
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLdouble aspect = (GLdouble)v.width / v.height;
    gluPerspective((GLdouble)v.fovy, aspect, (GLdouble)v.vnear, (GLdouble)v.vfar);

    //gluPerspective(fieldOfView, aspectRatio, near, far);


    // switch back to ModelView
    glMatrixMode(GL_MODELVIEW);
}

void ViewGL::paintGL()
{
    if( _cameraViewChanged ){
        _cameraViewChanged = false;
        setupCameraView(_cameraNr);
    }

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Setup projection to draw background
    if( _cameraNr==0 ){
        setOrthographicProjection(_width,_height);
        drawGLBackground();
        resetPerspectiveProjection();
    }

    // Setup the correct camera projection
    glLoadIdentity();

    if( _cameraNr==0 ){
        drawGLStuff(_showPivotPoint);
    } else {
        glScalef(_zoomScale,_zoomScale,_zoomScale);
    }

    if (_cell) {

		if( _cameraNr==0 ){
			// draw the workcell.
			_workcellGLDrawer->draw(*_cell.state, _cell.workcell);
		} else {
			GLCameraView &v = _cameraViews[_cameraNr];
			_workcellGLDrawer->drawCameraView(*_cell.state , v.frame );
		}
    }
    
    drawRWLogo();
}


void ViewGL::drawRWLogo() {
    glRasterPos2f(20,_height - 20);
    glColor4d(1,1,1,0.5);
    renderText(10, _height-10, _viewLogo, _logoFont);
}

// must be in projection mode
void ViewGL::drawGLStuff(bool showPivot){
    // Rotate and place camera/scene
    DrawableUtil::multGLTransform( Transform3D<float>(_viewPos, _viewRotation) );

    // scale from zoomfactom
    glTranslated(-_pivotPoint(0) * _zoomScale,
                 -_pivotPoint(1) * _zoomScale,
                 -_pivotPoint(2) * _zoomScale);

    // TODO: draw arcball, TODO: not really nice looking _arcBall.draw()

    // draw center point
    if (showPivot){
        drawPivot(_pivotPoint, _viewPos.norm2()*0.01, _zoomScale, _sphereObj);
    }

    glScalef(_zoomScale,_zoomScale,_zoomScale);

    drawWorldGrid(10,0.5);

    // draw all drawables
    BOOST_FOREACH(Drawable* da, _drawables) { da->draw(); }

    if (_cell) {
        // Collision check the workcell.
        if (_checkForCollision->isChecked()) {
            _cell.collisionPairs =
                highLightCollisionPairsStep(
                    *_cell.state,
                    *_cell.detector,
                    _cell.collisionPairs,
                    *_workcellGLDrawer);
        }

        // draw all cameras
        //BOOST_FOREACH(const GLCameraView& cv, _cameraViews) {
		for (size_t i = 1; i<_cameraViews.size(); i++) {
            glPushMatrix();
			const GLCameraView& cv = _cameraViews[i];
            DrawableUtil::multGLTransform(Kinematics::worldTframe(cv.frame, *_cell.state));
            drawCamera(cv);
            glPopMatrix();
        }
    }

}

void ViewGL::resizeGL(int width, int height)
{
    _width = width;
    _height = height;


    _cameraViews[0].width = width;
    _cameraViews[0].height = height;

    _cameraViewChanged = true;

    _cameraCtrl->setBounds(width,height);
}

#define GL_SELECT_BUFSIZE 512
GLuint _selectBuf[GL_SELECT_BUFSIZE];

rw::kinematics::Frame* ViewGL::pickFrame(int cursorX, int cursorY){
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
/*
   for (unsigned int j = 0; j < numberOfNames; j++, ptr++) {

       printf ("%d ", *ptr);
       if(_cell){
           const Frame *frame = _cell.state->getFrame(*ptr);
           if( frame!=NULL )
               std::cout << " " << frame->getName() << std::endl;

       }
   }
   printf ("\n");


    return NULL;
    */
}

void ViewGL::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ControlModifier) {

        int winx = event->x();
        int winy = height()-event->y();
        // we pick the scene before
        Frame *frame = pickFrame(winx,winy);
        if( frame ){
            _rwStudio->frameSelectedEvent().fire( frame );
        }

    } else if (event->button() == Qt::LeftButton) {
        GLfloat depth;
        int winx = event->x();
        int winy = height()-event->y();

        glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
        GLdouble modelMatrix[16];
        GLdouble projMatrix[16];
        GLint viewport[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
        glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
        glGetIntegerv(GL_VIEWPORT, viewport);
        GLdouble objx, objy, objz;
        gluUnProject(winx, winy, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);

        // TODO: fire an event that sends the 3d position
        if (depth != 1) {
            if (event->modifiers() == Qt::ShiftModifier) {
                _rwStudio->positionSelectedEvent().fire(Vector3D<>(objx, objy, objz));
            } else {
                _viewPos -= _viewRotation*_pivotPoint;
                _pivotPoint(0) = objx;
                _pivotPoint(1) = objy;
                _pivotPoint(2) = objz;
                _viewPos += _viewRotation*_pivotPoint;

                // update arcball center
                _cameraCtrl->setCenter((float)objx, (float)objy);
                updateGL();
            }
        }

    }
}

void ViewGL::mousePressEvent(QMouseEvent* event)
{
    _lastPos(0) = event->x();
    _lastPos(1) = event->y();
    _cameraCtrl->click( _lastPos(0), _lastPos(1));

    if (event->buttons() == Qt::RightButton &&
        event->modifiers() == Qt::ControlModifier)
    {
        saveBufferToFileQuery();
    }

    _rwStudio->mousePressedEvent().fire(event);
}

void ViewGL::mouseMoveEvent(QMouseEvent* event)
{
    eventTimer.pause();
    //std::cout<<"Event Time"<<eventTimer.getTime()<<std::endl;
    if (event->buttons() == Qt::LeftButton) {
        if (event->modifiers() == Qt::ControlModifier) {
            _viewPos(2) -= (event->y()-_lastPos(1))/_height*10;
        } else { // The mouse is being dragged
            /*
              float ry = (event->x()-_lastPos(0))*2*M_PI/_width;
              float rx = (event->y()-_lastPos(1))*2*M_PI/_height;
              RPY<float> rpyrot(0.0f, ry, rx);
              Rotation3D<float> rot;
              rpyrot.toRotation3D(rot);
              _viewRotation = rot*_viewRotation;
            */
            float rx = (event->x());
            float ry = (event->y());

            // Update End Vector And Get Rotation As Quaternion
            Quaternion<float> quat = _cameraCtrl->drag(rx, ry);

            // Convert Quaternion Into Rotation3D
            Rotation3D<float> thisRot = quat.toRotation3D();

            // Accumulate Last Rotation Into This One
            _viewRotation = thisRot*_viewRotation;
            _cameraCtrl->click(rx,ry);
        }
    }
    if (event->buttons() == Qt::RightButton) {
        _viewPos(0) += (event->x()-_lastPos(0))/_width*10;
        _viewPos(1) -= (event->y()-_lastPos(1))/_height*10;
    }
    _lastPos(0) = event->x();
    _lastPos(1) = event->y();
    updateGL();
    eventTimer.reset();
    eventTimer.resume();
}

void ViewGL::wheelEvent(QWheelEvent* event)
{
    //float distToPivot = norm_2( _pivotPoint - _viewPos );
    // somehow compensate when we are very close to objects
    _zoomFactor += event->delta()/(WHEEL_DELTA_ZOOM);

    if(_zoomFactor>=0){
        _zoomScale = 1.0+0.5*_zoomFactor;
    } else {
        _zoomScale = -1.0/(0.5*_zoomFactor-1);
    }
    // _viewPos(2) += (event->delta())/(WHEEL_DELTA_ZOOM*_zoomFactor);
    updateGL();
}

void ViewGL::saveBufferToFile(const QString& filename)
{
    const char* type = "PNG";
    if (filename.endsWith(".BMP", Qt::CaseInsensitive))
        type = "BMP";
    else if (filename.endsWith(".JPG", Qt::CaseInsensitive))
        type = "JPG";
    else if (filename.endsWith(".PNG", Qt::CaseInsensitive))
        type = "PNG";
    else {
        throw std::string(
            "ViewGL::saveBufferToFile: The selected file format is not supported");
    }
    if (!grabFrameBuffer().save(filename, type))
        throw std::string(
            "ViewGL::saveBufferToFile: Could not save file") +
            filename.toStdString();
}

void ViewGL::saveBufferToFileQuery()
{
    QString filename = QFileDialog::getSaveFileName(
        this, "Save Image", "./","Images (*.png *.bmp *.jpg)");

    if (!filename.isEmpty()) {
        try {
            saveBufferToFile(filename);
        } catch (const std::string& exp) {
            QMessageBox::information(
                this, "Failed to save file", exp.c_str(), QMessageBox::Ok);
        }
    }
}

void ViewGL::drawGLBackground(){
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBegin(GL_QUADS);
    glColor3fv(BOTTOM_BG_COLOR);
    glVertex2f(0, 0);
    glVertex2f(_width, 0);
    glColor3fv(TOP_BG_COLOR);
    glVertex2f(_width, _height);
    glVertex2f(0, _height);
    glEnd();

  //  glRasterPos2f(20,_height - 20);
//    glColor4d(1,1,1,0.5);
    //renderText(10, _height-10, _viewLogo, _logoFont);

    // setup projection to draw the rest of the scene
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
}

