/************************************************************************
 * RobWorkStudio Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * This Software is developed using the Qt Open Source Edition, and is
 * therefore only available under the GNU General Public License (GPL).
 *
 * RobWorkStudio can be used, modified and redistributed freely.
 * RobWorkStudio is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWorkStudio relies on RobWork, which has a different
 * license. For more information goto your RobWork directory and read license.txt.
 ************************************************************************/

#include "GLViewRW.hpp"

#include <rw/common/Timer.hpp>
#include <rw/graphics/WorkCellScene.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>

#include <rwlibs/opengl/Drawable.hpp>
#include <rwlibs/opengl/DrawableUtil.hpp>
#include <rwlibs/os/rwgl.hpp>

#include "RobWorkStudio.hpp"

#include <QFileDialog>
#include <QMessageBox>
#include <QString>
#include <QMouseEvent>
#include <QAction>
#include <QToolBar>
#include <QMenu>

#include <boost/foreach.hpp>

USE_ROBWORK_NAMESPACE

const int WHEEL_DELTA_ZOOM = 120;

// Background Color definitions
const GLfloat TOP_BG_COLOR[] = {1.0f,1.0f,1.0f};
const GLfloat BOTTOM_BG_COLOR[] = {0.2f,0.2f,1.0f};

using namespace robwork;
using namespace rw::proximity;
using namespace rwlibs::opengl;
using namespace rws;
namespace
{
    Timer eventTimer;

    void setDrawablesHighlighted(
        const std::vector<Drawable::Ptr>& drawables,
        bool value)
    {
        BOOST_FOREACH(Drawable::Ptr da, drawables) { da->setHighlighted(value); }
    }
/*
    void setPairHighlighted(
        WorkCellGLDrawer& drawer,
        const FramePair& pair,
        bool value)
    {
        setDrawablesHighlighted(drawer.getDrawablesForFrame(pair.first), value);
        setDrawablesHighlighted(drawer.getDrawablesForFrame(pair.second), value);
    }
*/
    void setCollisionPairsHighlighted(
        WorkCellScene& drawer,
        const FramePairSet& pairs,
        bool value)
    {
        BOOST_FOREACH(const FramePair& pair, pairs) {
        	drawer.setHighlighted(value, pair.first);
        	drawer.setHighlighted(value, pair.second);
        }
    }

    FramePairSet highLightCollisionPairsStep(
        const State& state,
        CollisionDetector& detector,
        const FramePairSet& previous,
        WorkCellScene& drawer)
    {
        setCollisionPairsHighlighted(drawer, previous, false);

        CollisionDetector::QueryResult current;
        detector.inCollision(state, &current);

        setCollisionPairsHighlighted(drawer, current.collidingFrames, true);
        return current.collidingFrames;
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

struct GLViewRW::GLData {
	GLData(): sphereObj(gluNewQuadric()) {}
	~GLData() {
	    gluDeleteQuadric(sphereObj);
	}
    GLUquadricObj* sphereObj;
};

GLViewRW::GLViewRW(QWidget* parent) :
    QGLWidget(QGLFormat(QGL::DepthBuffer), parent),
    _viewRotation(RPY<float>( 0, 0, static_cast<float>(-45*Deg2Rad) ).toRotation3D()),
    _viewPos(0,0,-5),
    _drawType(DrawableNode::SOLID),
    _alpha(1),
	_gl(new GLData()),
    _width(640),
    _height(480),
    _arcBall(_width,_height),
    _zoomFactor(0.0),
    _zoomScale(1.0),
    _logoFont("Helvetica [Cronyx]", 16, QFont::DemiBold , true),
    _viewLogo("RWSim")
{
    // add the default cameraview
    _showSolidAction = new QAction(QIcon(":/images/solid.png"), tr("&Solid"), this); // owned
    _showSolidAction->setCheckable(true);
    _showSolidAction->setChecked(true);

    if (!connect(_showSolidAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot())))
        std::cout << "Failed to connect.\n";

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

GLViewRW::~GLViewRW() {
	delete _gl;
}

void GLViewRW::keyPressEvent(QKeyEvent *e)
{
    e->accept();
}

void GLViewRW::setupToolBar(QToolBar* toolbar)
{
    toolbar->addAction(_showSolidAction);
    toolbar->addAction(_showWireAction);
    toolbar->addAction(_showOutlineAction);

    toolbar->addSeparator();

    toolbar->addAction(_showTransparentAction);
    toolbar->addAction(_showPivotPointAction);
    toolbar->addAction(_checkForCollision);
}

void GLViewRW::setupMenu(QMenu* menu)
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

void GLViewRW::addDrawable(Drawable* drawable)
{
    _drawables.push_back(drawable);
}

void GLViewRW::clear()
{
    _drawables.clear();
}

void GLViewRW::setDrawType(Render::DrawType drawType)
{
    // set DrawType for all Drawable in the view
    BOOST_FOREACH(Drawable* da, _drawables) { da->setDrawType(drawType); }
}

void GLViewRW::setDrawTypeSlot()
{
    if (_showSolidAction->isChecked())
        setDrawType(DrawableNode::SOLID);
    else if (_showWireAction->isChecked())
        setDrawType(DrawableNode::WIRE);
    else if (_showOutlineAction->isChecked())
        setDrawType(DrawableNode::OUTLINE);

    updateGL();
}

void GLViewRW::setTransparentSlot()
{
    double alpha;
    if (_showTransparentAction->isChecked())
        alpha = 0.5;
    else
        alpha = 1.0;

    // set alpha for all Drawable in the view
    BOOST_FOREACH(Drawable* da, _drawables) { da->setTransparency(alpha); }

    updateGL();
}

void GLViewRW::showPivotPointSlot()
{
    showPivotPoint(_showPivotPointAction->isChecked());
}

void GLViewRW::showPivotPoint(bool visible)
{
    _showPivotPoint = visible;
    updateGL();
}

void GLViewRW::initializeGL()
{
    /****************************************/
    /* Set up OpenGL lights etc.            */
    /****************************************/
    glShadeModel(GL_SMOOTH);   // Enable smooth shading.
    glEnable(GL_LIGHTING);
    glEnable( GL_NORMALIZE );
    glEnable(GL_DEPTH_TEST);
    GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
    GLfloat light0_diffuse[] =  {.6f, .6f, 0.6f, 1.0f};
    GLfloat light0_specular[] = { 0.5, 0.5, 0.5, 1.0 };
    GLfloat light0_position[] = {.5f, .5f, 1.0f, 0.0f};

    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

    glEnable(GL_LIGHT0);

    DrawableUtil::setupHighlightLight();

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT /*_AND_BACK*/, GL_AMBIENT_AND_DIFFUSE);
    glMaterialfv(GL_FRONT/*_AND_BACK*/, GL_SPECULAR, light0_specular);
    glMateriali(GL_FRONT/*_AND_BACK*/, GL_SHININESS, 128);

    glEnable(GL_LINE_SMOOTH);
    //glEnable(GL_POINT_SMOOTH);
    //glEnable(GL_POLYGON_SMOOTH);
    //glEnable(GL_PERSPECTIVE_CORRECTION);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glBlendFunc(GL_SRC_ALPHA_SATURATE, GL_ONE);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

}

void GLViewRW::setupCameraView(int camNr, bool setupViewport){
    // set camera view port and scale it to width or height
    glViewport( 0 , 0, (GLsizei)_width, _height);
    // switch to projection mode
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLdouble aspect = (GLdouble)_width / _height;
    gluPerspective((GLdouble)50, aspect, (GLdouble)0.01, (GLdouble)100);

    // switch back to ModelView
    glMatrixMode(GL_MODELVIEW);
}

void GLViewRW::paintGL()
{
    if( _cameraViewChanged ){
        _cameraViewChanged = false;
        setupCameraView(0);
    }

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    // Setup projection to draw background
	setOrthographicProjection(_width,_height);
	drawGLBackground();
	resetPerspectiveProjection();

    // Setup the correct camera projection
    glLoadIdentity();

    drawGLStuff(_showPivotPoint);

    // draw all drawables
    DrawableNode::RenderInfo info;
    info._mask = DrawableNode::ALL;
    BOOST_FOREACH(Drawable* da, _drawables) { da->draw(info); }

}

// must be in projection mode
void GLViewRW::drawGLStuff(bool showPivot){
    // Rotate and place camera/scene
    DrawableUtil::multGLTransform( Transform3D<float>(_viewPos, _viewRotation) );

    // scale from zoomfactom
    glTranslated(-_pivotPoint(0) * _zoomScale,
                 -_pivotPoint(1) * _zoomScale,
                 -_pivotPoint(2) * _zoomScale);

    // TODO: draw arcball, TODO: not really nice looking _arcBall.draw()

    // draw center point
    if (showPivot){
        drawPivot(_pivotPoint, _viewPos.norm2()*0.01, _zoomScale, _gl->sphereObj);
    }

    glScalef(_zoomScale,_zoomScale,_zoomScale);

    drawWorldGrid(10,0.5);
}

void GLViewRW::resizeGL(int width, int height)
{
    _width = width;
    _height = height;

    _cameraViewChanged = true;

    _arcBall.setBounds(width,height);
}

void GLViewRW::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton &&
            event->modifiers() == Qt::ControlModifier) {

        //int winx = event->x();
        //int winy = height()-event->y();
        // we pick the scene before
        //Frame *frame = pickFrame(winx,winy);
        //if( frame ){
            //_rwStudio->frameSelectedEvent().fire( frame );
        //}

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
            _viewPos -= _viewRotation*_pivotPoint;
            _pivotPoint(0) = objx;
            _pivotPoint(1) = objy;
            _pivotPoint(2) = objz;
            _viewPos += _viewRotation*_pivotPoint;

            // update arcball center
            _arcBall.setCenter( cast<double>(_pivotPoint), Vector2D<>(objx, objy) );
        }
        updateGL();
    }
}

void GLViewRW::mousePressEvent(QMouseEvent* event)
{
    _lastPos(0) = event->x();
    _lastPos(1) = event->y();
    _arcBall.click( _lastPos(0), _lastPos(1));

    if (event->buttons() == Qt::RightButton &&
        event->modifiers() == Qt::ControlModifier)
    {
        saveBufferToFileQuery();
    }
}

void GLViewRW::mouseMoveEvent(QMouseEvent* event)
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
            Quaternion<> quat = _arcBall.drag(rx, ry);

            // Convert Quaternion Into Rotation3D
            Rotation3D<float> thisRot = cast<float>( quat.toRotation3D() );

            // Accumulate Last Rotation Into This One
            _viewRotation = thisRot*_viewRotation;
            _arcBall.click(rx,ry);
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

void GLViewRW::wheelEvent(QWheelEvent* event)
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

void GLViewRW::saveBufferToFile(const QString& filename)
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
            "GLViewRW::saveBufferToFile: The selected file format is not supported");
    }
    if (!grabFrameBuffer().save(filename, type))
        throw std::string(
            "GLViewRW::saveBufferToFile: Could not save file") +
            filename.toStdString();
}

void GLViewRW::saveBufferToFileQuery()
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

void GLViewRW::drawGLBackground(){
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

    glRasterPos2f(20,_height - 20);
    glColor4d(1,1,1,0.5);
    renderText(10, _height-10, _viewLogo, _logoFont);

    // setup projection to draw the rest of the scene
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
}

