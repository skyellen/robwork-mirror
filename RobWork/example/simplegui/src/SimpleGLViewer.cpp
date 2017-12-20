#include "SimpleGLViewer.hpp"

#include "ArcBall.hpp"
#include "EventListener.hpp"
#include "Menu.hpp"
#include "MenuItem.hpp"

#include <iostream>
#include <string>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <time.h>
#include <cstdio>

//#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/graphics/WorkCellScene.hpp>

#include <rwlibs/opengl/SceneOpenGL.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
//#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <map>

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::math;
using rw::models::WorkCell;
using namespace rw::graphics;

using namespace rwlibs::opengl;
using rwlibs::simulation::SimulatedCamera;

struct SimpleGLViewer::InternalData {
	InternalData():
		_x(0), _y(0), _width(640), _height(480),
		_arcBall(new ArcBall(_width,_height)),
		_mainWindow(0),
		_curCameraView(0), _camView(false),
		_mouseState(NONE),
		_keyListener(NULL),
		_viewRotation(Rotation3D<float>::identity()),
		_viewPos(0,0,-10),
		_pivotPoint(0,0,0),
		_lastPos(0,0,0)
	{
		for (int i = 0; i < 10; i++)
			_myFps[i] = 0;
	}

	~InternalData() {
		delete _arcBall;
	}

    View::Ptr _currentView;
    rw::graphics::SceneGraph::RenderInfo _renderInfo;

    int _x, _y, _width, _height;

    ArcBall* _arcBall;

    int _mainWindow;

    std::vector<rwlibs::simulation::SimulatedCamera*> _cameras;
    int _curCameraView;
    bool _camView;

    char _myFps[10];

    enum MouseDragMode {NONE, ZOOM, TRANSLATE, ROTATE};
    MouseDragMode _mouseState;

    EventListener* _keyListener;

    /* constants for the right click menu */
    enum{QUIT=0};
    std::map<int,MenuItem*> _menuItemMap;

    Rotation3D<float> _viewRotation;
    Vector3D<float> _viewPos;
    Vector3D<float> _pivotPoint;
    Vector3D<float> _lastPos;

    // Initialize collision stuff variables
    //CollisionDetector *_collisionDetector;
    //bool _collisionCheckEnabled = true;
};

namespace {

std::map<int,SimpleGLViewer*> viewers;

class RenderQuad : public Render {
private:
	GLfloat _colorTop[4],_colorBottom[4];
	int _x,_y,_width,_height;
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<RenderQuad> Ptr;

	/* Functions inherited from Render */
	/**
	 * @copydoc Render::draw
	 */
	void draw(const DrawableNode::RenderInfo&, DrawType, double) const {

		//glDisable(GL_TEXTURE_2D);

		glPushMatrix();
		glLoadIdentity();

		//GLenum matRendering = GL_FRONT;
		//GLfloat specularReflection[]={1.0f,1.0f,1.0f,1.0f};
		//GLfloat matEmission[]={0.0f,0.0f,0.0f,1.0f};
		//glMaterialfv(matRendering, GL_SPECULAR, specularReflection);
		//glMaterialfv(matRendering, GL_EMISSION, matEmission);
		//glMateriali(matRendering, GL_SHININESS, 128);

		glPolygonMode(GL_FRONT, GL_FILL);
		glBegin(GL_QUADS);
		glColor4fv(_colorBottom);
		glVertex2f(_x, _y);
		glVertex2f(_width, _y);
		glColor4fv(_colorTop);
		glVertex2f(_width, _height);
		glVertex2f(_x, _height);
		glEnd();

		glPopMatrix();
		//glEnable(GL_TEXTURE_2D);
	}

	void setTopColor(const Vector3D<>& color){
		_colorTop[0] = color[0];
		_colorTop[1] = color[1];
		_colorTop[2] = color[2];
		_colorTop[3] = 1;
	}

	void setBottomColor(const Vector3D<>& color){
		_colorBottom[0] = color[0];
		_colorBottom[1] = color[1];
		_colorBottom[2] = color[2];
		_colorBottom[3] = 1;
	}

	void setViewPort(int x,int y,int width,int height){
		_x = x; _y = y; _width = width; _height = height;
	}

};

/**************************************** myGlutMenu() **********/
/* The right click mouse menu callback function */
void myGlutMenu(int value){
    std::cout << "" << std::endl;
    const int window = glutGetWindow();
	const SimpleGLViewer::InternalData* const data = viewers[window]->_data;
    switch(value){
    case(SimpleGLViewer::InternalData::QUIT):
        glutDestroyWindow(window);
    	viewers.erase(window);
    	if (viewers.size() == 0) {
    		exit(0);
    		break;
    	} else {
    		return;
    	}
    }

    std::map<int,MenuItem*>::const_iterator itemFind = data->_menuItemMap.find(value);
    if( itemFind != data->_menuItemMap.end() ){
        std::cout << "menu event" << std::endl;
        itemFind->second->event();
    }

    // you would want to redraw now
    glutPostRedisplay();
}

void placeCenter(float x, float y){
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;
    std::cout << x << "  "<< y<< std::endl;
    GLfloat depth;
    int winx = (int)x;
    int winy = (int)y;
    glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    GLdouble modelMatrix[16];
    GLdouble projMatrix[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
    GLdouble objx, objy, objz;
    gluUnProject(winx, winy, depth, modelMatrix, projMatrix, viewport, &objx, &objy, &objz);
    std::cout << data->_pivotPoint << std::endl;
    if (depth != 1) {
      data->_viewPos -= data->_viewRotation*data->_pivotPoint;
      data->_pivotPoint(0) = objx;
      data->_pivotPoint(1) = objy;
      data->_pivotPoint(2) = objz;
      data->_viewPos += data->_viewRotation*data->_pivotPoint;
    }
    std::cout << objx << "  "<< objy<< " "<< objz << std::endl;
    // update arcball center
	data->_arcBall->setCenter( std::pair<float,float>(objx,objy) );
}

/**************************************** myGlutKeyboard() **********/
/* This function is called when a key on the keyboard is pressed */

void myGlutKeyboard(unsigned char key, int x, int y) {
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;
    //float step = 0.1;
    switch (key){
        case 27:
        case 'q': exit(0); break;
        case 'c': // Toggle collision check
            //_collisionCheckEnabled = !_collisionCheckEnabled;
            //if(_collisionCheckEnabled)
            //    std::cout << " | Collision Check enabled!" << std::endl;
            //else
            //    std::cout << " | Collision Check disabled!" << std::endl;
            //break;
        case '1':
            if(data->_camView)
            	data->_curCameraView = 0;
            break;
        case '2':
            if(data->_camView)
            	data->_curCameraView = 1;
            break;
        case '3':
            if(data->_camView)
            	data->_curCameraView = 2;
            break;
        case 'a': // toggle application view versus tool cam view
        	data->_camView = !data->_camView;
            break;
        case 'z': // toggle application view versus tool cam view
        	viewers[window]->autoZoom();
            break;
        case 's': // take snapshot of toolCam
        {
            break;
        }
        case 'p': // perform pick
            placeCenter(data->_lastPos(0),data->_lastPos(1));
            break;
    }
    if(data->_keyListener != NULL){
        std::pair<int,int> pos(x,y);
        data->_keyListener->event(EventListener::e_KEY_EVENT,&key);
    }
    glutPostRedisplay();
}

bool idlelock_a = false;
bool idlelock_b = false;

/***************************************** myGlutIdle() ***********/
void myGlutIdle( void )
{
    // TODO: implement more robust mutex
    idlelock_a = false;
    while(idlelock_b);
    idlelock_a = true;

    /* According to the GLUT specification, the current window is
     undefined during an idle callback.  So we need to explicitly change
     it if necessary */
    //if ( glutGetWindow() != _mainWindow )
    //    glutSetWindow(_mainWindow);

    glutPostRedisplay();
}

/***************************************** myGlutMouse() **********/
void myGlutMouse(int button, int button_state, int x, int y )
{
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;

	data->_lastPos(0) = x;
	data->_lastPos(1) = y;

    // Select the mouse drag mode.
    int mods = glutGetModifiers();
    if(button == GLUT_LEFT_BUTTON){
        if(button_state == GLUT_DOWN){
        	data->_arcBall->click( data->_lastPos(0), data->_lastPos(1) );
            if(GLUT_ACTIVE_CTRL & mods){
            	data->_mouseState = SimpleGLViewer::InternalData::ZOOM;
            } else if(GLUT_ACTIVE_SHIFT & mods){
            	data->_mouseState = SimpleGLViewer::InternalData::TRANSLATE;
            } else {
            	data->_mouseState = SimpleGLViewer::InternalData::ROTATE;
            }
        } else {
        	data->_mouseState = SimpleGLViewer::InternalData::NONE;
        }
    } else if(button == GLUT_LEFT_BUTTON){
    	data->_mouseState = SimpleGLViewer::InternalData::TRANSLATE;
    } /*else if(button == GLUT_WHEEL_UP){

    } else if(button == GLUT_WHEEL_DOWN){

    }*/ else {
    	data->_mouseState = SimpleGLViewer::InternalData::NONE;
    }
    glutPostRedisplay();
}

/***************************************** myGlutMotion() **********/
void myGlutMotion(int x, int y)
{
    bool redraw = false;
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;
    switch(data->_mouseState){
        case(SimpleGLViewer::InternalData::ROTATE):
        {
            Quaternion<float> quat(0.0f,0.0f,0.0f,0.0f);
            data->_arcBall->drag( std::pair<float,float>(x,y), quat); // Update End Vector And Get Rotation As Quaternion

            Rotation3D<float> thisRot = quat.toRotation3D(); // Convert Quaternion Into Rotation3D
            data->_viewRotation = thisRot*data->_viewRotation; // Accumulate Last Rotation Into This One
            data->_arcBall->click(x,y);
            redraw = true;
            break;
        }
        case (SimpleGLViewer::InternalData::ZOOM):
        {
        	data->_viewPos(2) -= (y-data->_lastPos(1))/data->_height*10;
            redraw = true;
            break;
        }
        case(SimpleGLViewer::InternalData::TRANSLATE):
        {
        	data->_viewPos(0) += (x-data->_lastPos(0))/data->_width*10;
            data->_viewPos(1) -= (y-data->_lastPos(1))/data->_height*10;
            redraw = true;
            break;
        }
        case SimpleGLViewer::InternalData::NONE:
        {
            break;
        }
        default:
            break;
    }
    data->_lastPos(0) = x;
    data->_lastPos(1) = y;
    if(redraw)
        glutPostRedisplay();
}


void myGlutPassiveMotion(int, int)
{
    glutPostRedisplay();
}

/**************************************** myGlutReshape() *************/
void myGlutReshape( int width, int height )
{
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;
	data->_width = width;
	data->_height = height;

    glViewport(0, 0, width, height);
    // resize arcball window
    data->_arcBall->setBounds(width,height);

    glMatrixMode(GL_PROJECTION);
    {
        glLoadIdentity();
        GLdouble aspect = (GLdouble)width / height;
        gluPerspective(60, aspect, 0.10, 1000);
        //    glTranslated(0, 0, -10);
    }
    glMatrixMode(GL_MODELVIEW);
    glutPostRedisplay();
}

/**************************************** myGlutSpecial() *************/
/* Special keyboard input (function keys, arrow keys, and more). */
void myGlutSpecial(int key, int x, int y) {
    const int window = glutGetWindow();
	SimpleGLViewer::InternalData* const data = viewers[window]->_data;
    // Arrow keys are used for moving the camera.
    if (GLUT_ACTIVE_CTRL & glutGetModifiers()) {
        switch (key) {
            case GLUT_KEY_UP: data->_viewPos(2)   -= 0.15; break;
            case GLUT_KEY_DOWN: data->_viewPos(2) += 0.15; break;
        }
    } else if (GLUT_ACTIVE_SHIFT & glutGetModifiers()) {
        switch (key) {
            case GLUT_KEY_LEFT:  data->_viewPos(0) -= 0.05; break;
            case GLUT_KEY_RIGHT: data->_viewPos(0) += 0.05; break;
            case GLUT_KEY_UP:    data->_viewPos(1) += 0.05; break;
            case GLUT_KEY_DOWN:  data->_viewPos(1) -= 0.05; break;
        }
    }
    if(data->_keyListener != NULL){
        std::pair<int,int> pos(x,y);
        data->_keyListener->event(EventListener::e_KEY_EVENT,&key);
    }

    glutPostRedisplay();
}

/************************************** GetFPS ****************
* This function calculates and prints Frames Per Second. Should
* be called each time a frame is displayed.
***************************************************************/
void calcFPS(char myFps[10])
{
    static int FPS = 0;
    // This is the last second that occured.
    static long prevClock = 0;
    long currClock = clock();
    FPS++;
    if(currClock > prevClock+CLOCKS_PER_SEC){
        prevClock += CLOCKS_PER_SEC;
        sprintf(myFps, "%d FPS", FPS);
        FPS = 0;
    }
}

/***************************************** myGlutDisplay() *****************/
void myGlutDisplay( void )
{
	const int window = glutGetWindow();
	SimpleGLViewer* const viewer = viewers[window];
	SimpleGLViewer::InternalData* const data = viewer->_data;
    glClearColor( .9f, .9f, .9f, 1.0f );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if( data->_camView ){
        if(data->_curCameraView>=0 && data->_curCameraView < (int)data->_cameras.size()){
            std::cout << ".";
            data->_cameras[data->_curCameraView]->acquire();
        }
        //_toolCam.acquire();
    } else {
        Transform3D<> viewTransform(cast<double>(data->_viewPos),cast<double>(data->_viewRotation));
        viewer->getCurrentView()->_viewCamera->setTransform(viewTransform);
        //if( _cameraCtrl ){
            // for now we scale the pivot such that its not unrealistically large/small
            //getViewCamera()->setTransform(_cameraCtrl->getTransform());

            //double dist = -(inverse(_cameraCtrl->getTransform())*_pivotDrawable->getTransform().P())[2];
            //_pivotDrawable->setScale( Math::clamp(dist/5.0,0.00001,1) ); // 5/5
        //}

        //std::cout << _currentView->_name << std::endl;
        data->_renderInfo._drawType = data->_currentView->_drawType;
        data->_renderInfo._mask = data->_currentView->_drawMask;
        data->_renderInfo.cams = data->_currentView->_camGroup;
        viewer->getScene()->draw( data->_renderInfo );

        GLenum res = glGetError();
        if(res!=GL_NO_ERROR){
            std::cout << "AN OPENGL ERROR: " << res << "\n";
        }
    }
    /* Disable lighting and set up ortho projection to render text */
    glDisable( GL_LIGHTING );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluOrtho2D( 0.0, data->_width, data->_height , 0.0  );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    glColor3ub( 0, 0, 0 );

    // Display the FrameRate
    calcFPS(data->_myFps);
    glColor3f(0,0,0);
    glRasterPos2i( data->_width-50, 20 );
    for(int i=0; i<(int)strlen( data->_myFps ); i++ )
        glutBitmapCharacter( GLUT_BITMAP_HELVETICA_10, data->_myFps[i] );
    glEnable( GL_LIGHTING );

    glutSwapBuffers();
}

} // end anonymous namespace

void SimpleGLViewer::setKeyListener(EventListener *listener){
    _data->_keyListener = listener;
}

SimpleGLViewer::SimpleGLViewer():
	_data(new InternalData()),
	_scene(ownedPtr(new SceneOpenGL()))
{
}

SimpleGLViewer::~SimpleGLViewer() {
	delete _data;
}

SceneGraph::Ptr SimpleGLViewer::getScene() {
	return _scene;
}

const std::string& SimpleGLViewer::getLogo() const {
	static const std::string logo;
	return logo;
}

void SimpleGLViewer::setLogo(const std::string&) {
}

PropertyMap& SimpleGLViewer::getPropertyMap() {
	static PropertyMap map;
	return map;
}

void SimpleGLViewer::updateView() {
}

void SimpleGLViewer::updateState(const State& state) {
	if(_state==NULL)
		_state = ownedPtr(new State());
	*_state = state;
	_data->_renderInfo._state = _state.get();
}

void SimpleGLViewer::setWorldNode(GroupNode::Ptr wnode) {
    if(wnode == NULL){
    	_data->_currentView->_viewCamera->setEnabled(false);
    } else {
    	_data->_currentView->_viewCamera->setEnabled(true);
    }
    _scene->getRoot()->removeChild(_worldNode);
    _worldNode = wnode;
    _scene->addChild(_worldNode, _scene->getRoot());
    _data->_currentView->_viewCamera->setRefNode(_worldNode);
}

GroupNode::Ptr SimpleGLViewer::getWorldNode() {
	return _worldNode;
}

void SimpleGLViewer::saveBufferToFile(const std::string&, int, int, int) {
}

SceneCamera::Ptr SimpleGLViewer::getViewCamera() {
	return _data->_currentView->_viewCamera;
}

Vector3D<> SimpleGLViewer::getViewCenter() {
	return cast<double>(_data->_viewPos);
}

DrawableNode::Ptr SimpleGLViewer::pickDrawable(int x, int y) {
    return _scene->pickDrawable(_data->_renderInfo, x, y);
}

DrawableNode::Ptr SimpleGLViewer::pickDrawable(SceneGraph::RenderInfo& info, int x, int y) {
    return _scene->pickDrawable(info, x, y);
}

SceneViewer::View::Ptr SimpleGLViewer::createView(const std::string&, bool) {
	return NULL;
}

SceneViewer::View::Ptr SimpleGLViewer::getMainView() {
	return _data->_currentView;
}

void SimpleGLViewer::destroyView(View::Ptr) {
}

void SimpleGLViewer::selectView(View::Ptr) {
}

SceneViewer::View::Ptr SimpleGLViewer::getCurrentView() {
	return _data->_currentView;
}

std::vector<SceneViewer::View::Ptr> SimpleGLViewer::getViews() {
	std::vector<SceneViewer::View::Ptr> views(1,_data->_currentView);
	return views;
}

void SimpleGLViewer::renderView(View::Ptr) {
}

void SimpleGLViewer::zoom(double amount) {
	_data->_viewPos -= _data->_viewRotation.getCol(2)*amount;
}

void SimpleGLViewer::autoZoom() {
    if (_wc.isNull()) {
        RW_WARN("Can't autozoom when no workcell is loaded");
        return;
    }
    static const double fovy = 45.*Deg2Rad;
    const double aspectRatio = static_cast<double>(_data->_width)/static_cast<double>(_data->_height);

    // The intention is to get a list off "interest points in the workcell"
    // In the first implementation, interest points is the origo of the frames in the workcell.
    const std::vector<Frame*> frames = _wc->getFrames();
    std::vector<Vector3D<double> > points;
    const State zoomState = (_state.isNull())? _wc->getDefaultState() : *_state;
    Vector3D<double> currentPoint;

    for (const Frame* it : frames) {
        // Transform points to camera frame and add them to the list.
        currentPoint = inverse(Transform3D<>(cast<double>(_data->_viewPos),cast<double>(_data->_viewRotation)))*(it->wTf(zoomState).P());
        points.push_back(currentPoint);
    }

    double max_x = 0;
    double max_y = 0;
    double max_relation_x = 0;
    double max_relation_y = 0;
    double max_xz = 0;
    double max_yz = 0;

    for (const Vector3D<double>& it : points) {
        const double x = it[0];
        const double y = it[1];
        const double z = it[2];

        // Get highest ratio of max(x,y)/z
        const double current_x = std::abs(x)/std::abs(z);
        const double current_y = std::abs(y)/std::abs(z);
        if (current_x > max_relation_x) {
        	max_relation_x = current_x;
            max_xz = -z;
            max_x = std::abs(x);
        }
        if (current_y > max_relation_y) {
        	max_relation_y = current_y;
            max_yz = -z;
            max_y = std::abs(y);
        }
    }
    // Vector max now holds the points furthest from the center of the rendering.
    // Now we can zoom camera, to obtain the target ratio between x and z or y and z.
    static const double extraZoom = 0.02; // add a bit of zoom out, as we do not yet consider the geometry.
    const double z_optimal_y = max_yz-(max_y+extraZoom)/std::tan(fovy/2);
    const double z_optimal_x = max_xz-(max_x+extraZoom)/std::tan(fovy/2)/aspectRatio;

    // Now zoom the camera with z_optimal
    zoom(std::min(z_optimal_x,z_optimal_y)-extraZoom);
}

void SimpleGLViewer::setWorkcell(WorkCell::Ptr workcell){
    // TODO: wait to obtain lock in a better way ;(
    std::cout << "Setting Workcell model" << std::endl;
    idlelock_b = true;
    while(idlelock_a);


    _wc = workcell;
    _state = new State( _wc->getDefaultState() );
    std::vector<Frame*> cameraViews;// = _wc->getCameraViews();
    //for(unsigned int i=0; i<cameraViews.size();i++){
    //    GLFrameGrabber *grapper = new GLFrameGrabber(640,480,(50.0/180.0)*3.14,&_wcscene,*_state);
    //    VirtualCamera *cam = new VirtualCamera("VCam",*grapper,cameraViews[i]);
    //    cam->start();
    //    if( cam->isInitialized() && cam->isStarted() )
    //        std::cout << " | Cam initialized..." << std::endl;
    //    else
    //        std::cout << " | Cam not initialized..." << std::endl;
    //    _cameras.push_back(cam);
    //}

    std::cout << "Creating collision detector!!" << std::endl;
    // create a collision detector
    //_collisionDetector = new collision::CollisionDetector(_wc, &_pqpStrategy );
    //_pqpStrategy.setFirstContact(false);

    // TODO: release lock
    idlelock_b = false;
}

bool SimpleGLViewer::start(){
    // TODO: start
    std::cout << "Init Glut" << std::endl;
    initGlut(_data->_x,_data->_y,_data->_width,_data->_height);
     std::cout << "Init lights" << std::endl;
    initLights();
    std::cout << "Init Menu" << std::endl;
    initMenu();
    std::cout << "Initializing WorkCellScene" << std::endl;
    _wcscene = new WorkCellScene( _scene );

    // initialize cameras
    // add the default/main cameraview group
    const rw::common::Ptr<rw::graphics::CameraGroup> mainCamGroup = _scene->makeCameraGroup("MainView");
    _scene->addCameraGroup(mainCamGroup);
    mainCamGroup->setEnabled(true);

    // add a node to render background
    rw::common::Ptr<RenderQuad> backgroundRender = ownedPtr(new RenderQuad());
    backgroundRender->setTopColor( Vector3D<>(1.0,1.0,1.0) );
    backgroundRender->setBottomColor( Vector3D<>(0.2,0.2,1.0) );
    backgroundRender->setViewPort(0,0,640,480);

    const rw::common::Ptr<DrawableNode> backgroundnode = _scene->makeDrawable("BackgroundRender", backgroundRender, DrawableNode::ALL);
    _scene->addChild(backgroundnode, _scene->getRoot());
    backgroundnode->setVisible(true);

    if (!_wc.isNull()) {
    	_wcscene->setWorkCell(_wc);
    	_wcscene->setState(_wc->getDefaultState());
    	_worldNode = _wcscene->getWorldNode();
    } else {
    	_worldNode = _scene->makeGroupNode("World");
    }

    _scene->addChild(_worldNode, _scene->getRoot());

    int dmask = 0;
    dmask |= DrawableNode::Virtual;
    dmask |= DrawableNode::Physical;
    dmask |= DrawableNode::DrawableObject;
    dmask |= DrawableNode::ALL;

    const SceneViewer::View::Ptr mainView = ownedPtr( new SceneViewer::View("MainView") );
    mainView->_drawMask = dmask;
    _data->_currentView = mainView;
    // add background camera
    rw::graphics::SceneCamera::Ptr backCam;
    backCam = _scene->makeCamera("BackgroundCam");
    backCam->setEnabled(true);
    backCam->setRefNode(backgroundnode);
    backCam->setProjectionMatrix( ProjectionMatrix::makeOrtho(0,640,0,480, -1, 1) );
    backCam->setClearBufferEnabled(true);
    backCam->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    backCam->setDepthTestEnabled( false );
    backCam->setLightningEnabled( false );
    mainCamGroup->insertCamera(backCam, 0);

    // main camera
    rw::graphics::SceneCamera::Ptr mainCam;
    mainCam = _scene->makeCamera("MainCam");
    mainCam->setDrawMask( dmask );
    mainCam->setEnabled(true);
    mainCam->setPerspective(45, 640, 480, 0.1, 30);
    mainCam->setClearBufferEnabled(false);
    mainCam->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    mainCam->setDepthTestEnabled( true );
    mainCam->setLightningEnabled( true );
    mainCam->setRefNode(_scene->getRoot());
    mainCamGroup->insertCamera(mainCam, 1);
    // TODO: foreground camera
    mainView->_viewCamera = mainCam;
    mainView->_camGroup = mainCamGroup;

    const Transform3D<float> viewTransform = Transform3D<float>::makeLookAt(Vector3D<float>(5,5,5),Vector3D<float>::zero(),Vector3D<float>::z());
    _data->_viewPos = viewTransform.P();
    _data->_viewRotation = viewTransform.R();

    glutMainLoop();

    return true;
}

bool SimpleGLViewer::stop(){
    return true;
}

void SimpleGLViewer::initMenu(){
    glutCreateMenu(myGlutMenu);

    std::map<int,Menu*>::iterator menuIter = _menuMap.begin();
    for(;menuIter!=_menuMap.end();++menuIter){
        Menu *menu = menuIter->second;
        int submenuid = menuIter->first;
        std::cout << "AddSubMenu" << std::endl;
        glutAddSubMenu( menu->getName().c_str(), submenuid );
    }


    glutAddMenuEntry("Quit", SimpleGLViewer::InternalData::QUIT);
    // Let the menu respond on the right mouse button
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void SimpleGLViewer::addSubMenus(Menu *menu){
    std::vector<Menu*> menus = menu->getMenus();
    for(unsigned int i=0 ; i<menus.size(); i++){
        //addMenu( menus[i] );
        int submenuid = glutCreateMenu(myGlutMenu);

        std::vector<MenuItem*> items = menu->getMenuItems();
        for(unsigned int j=0 ; j<items.size(); j++){
            // add all items
            _data->_menuItemMap[items[i]->getId()] = items[i];
            glutAddMenuEntry( items[i]->getName().c_str(),items[i]->getId() );
        }

        glutAddSubMenu( menu->getName().c_str(), submenuid );

        addSubMenus(menus[i]);
    }
}

void SimpleGLViewer::addMenu(Menu *menu){
    int submenuid = glutCreateMenu(myGlutMenu);
    _menuMap[submenuid] = menu;

    std::vector<MenuItem*> items = menu->getMenuItems();
    for(unsigned int i=0 ; i<items.size(); i++){
        // add all items
    	_data->_menuItemMap[items[i]->getId()] = items[i];
        glutAddMenuEntry(items[i]->getName().c_str(),items[i]->getId() );
    }
    addSubMenus(menu);
}

const State& SimpleGLViewer::getState(){
    return *_state;
}

void SimpleGLViewer::init(int argc, char** argv){
    glutInit(&argc, argv);
}

void SimpleGLViewer::initGlut(int x, int y, int width, int height){
    /****************************************/
    /*   Initialize GLUT and create window  */
    /****************************************/
    std::cout << " | Initializing Glut display functions..." << std::endl;

    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition (x, y);
    glutInitWindowSize (width, height);

    _data->_mainWindow = glutCreateWindow ("Simple GL viewer for RobWork");
    viewers[_data->_mainWindow] = this;
    // Register callback functions.
    glutDisplayFunc( myGlutDisplay );
    glutIdleFunc( myGlutIdle );
    glutReshapeFunc( myGlutReshape );
    glutKeyboardFunc( myGlutKeyboard );
    glutSpecialFunc( myGlutSpecial );
    glutMouseFunc( myGlutMouse );
    glutMotionFunc( myGlutMotion );
    glutPassiveMotionFunc( myGlutPassiveMotion );
}

void SimpleGLViewer::initLights(){
    /****************************************/
    /* Set up OpenGL lights etc.            */
    /****************************************/
    std::cout << " | Initializing OpenGL Lights..." << std::endl;
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

    GLfloat light1_ambient[] =  {1.0f, 0.0f, 0.0f, 1.0f};
    GLfloat light1_diffuse[] =  {.6f, .3f, 0.3f, 1.0f};
    GLfloat light1_specular[] = { 0.5, 0.2, 0.2, 1.0f};
    GLfloat light1_position[] = {.5f, .5f, 1.0f, 0.0f};

    glLightfv(GL_LIGHT1, GL_AMBIENT, light1_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);


    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, light0_specular);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 128);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}
