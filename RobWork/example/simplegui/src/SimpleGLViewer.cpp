#include "SimpleGLViewer.h"

#include <iostream>
#include <string>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <time.h>
#include <cstdio>

#include <rwlibs/opengl/SceneOpenGL.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Quaternion.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/sensor/Camera.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/graphics/WorkCellScene.hpp>

#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <map>

#include "ArcBall.hpp"

using namespace rwlibs::proximitystrategies;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::sensor;
using namespace rw::graphics;
using namespace rw;

// private prototypes
void myGlutReshape( int x, int y );

/* constants for the right click menu */
enum{QUIT=0};

//void GetFPS();

/* The window upper left corner and height and width parameters */
int _x=0, _y=0, _width=640,_height=480;

Rotation3D<float> _viewRotation(Rotation3D<float>::identity());
Vector3D<float> _viewPos(0,0,-10);
Vector3D<float> _lastViewPos(0,0,0);
Vector3D<float> _pivotPoint(0,0,0);
Vector3D<float> _lastPos(0,0,0);

ArcBall _arcBall(_width,_height);

bool _showPivotPoint = false;

// Initialize collision stuff variables
CollisionDetector *_collisionDetector;
CollisionStrategy *_cdStrategy;
bool _collisionCheckEnabled = true;

// the workcellModel
WorkCell::Ptr _workcellModel = NULL;
State *_state;
WorkCellScene *_workcellGLDrawer;
std::vector<SimulatedCamera*> _cameras;

// variables for the camera view
int _curCameraView=0;
bool _camView = false;

/* The main glut display window */
int main_window;

/* This char array is updated with the FPS count */
char  myFps[10] = {0};

/* Variables for keeping track of dragging of the mouse. */
enum MouseDragMode {NONE, ZOOM, TRANSLATE, ROTATE};
MouseDragMode mouseState = NONE;

GLdouble startX;
GLdouble startY;

std::map<int,Menu*> _menuMap;
std::map<int,MenuItem*> _menuItemMap;

EventListener *_keyListener = NULL;

WorkCellScene* SimpleGLViewer::getWorkCellGLDrawer(){ return &_workcellGLDrawer; };

void SimpleGLViewer::setKeyListener(EventListener *listener){
    _keyListener = listener;
}

/**************************************** myGlutMenu() **********/
/* The right click mouse menu callback function */
void myGlutMenu(int value){
    std::cout << "" << std::endl;
    switch(value){
    case(QUIT):
        glutDestroyWindow(main_window);
        exit(0);
        break;
    }

    std::map<int,MenuItem*>::iterator itemFind = _menuItemMap.find(value);
    if( itemFind != _menuItemMap.end() ){
        std::cout << "menu event" << std::endl;
        itemFind->second->event();
    }

    // you would want to redraw now
    glutPostRedisplay();
}

void placeCenter(float x, float y){
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
    std::cout << _pivotPoint << std::endl;
    if (depth != 1) {
      _viewPos -= _viewRotation*_pivotPoint;
      _pivotPoint(0) = objx;
      _pivotPoint(1) = objy;
      _pivotPoint(2) = objz;
      _viewPos += _viewRotation*_pivotPoint;
    }
    std::cout << objx << "  "<< objy<< " "<< objz << std::endl;
    // update arcball center
    _arcBall.setCenter( std::pair<float,float>(objx,objy) );
}

/**************************************** myGlutKeyboard() **********/
/* This function is called when a key on the keyboard is pressed */

std::map<int,EventListener*> _keyList;

void myGlutKeyboard(unsigned char key, int x, int y)
{
    //float step = 0.1;
    switch (key){
        case 27:
        case 'q': exit(0); break;
        case 'c': // Toggle collision check
            _collisionCheckEnabled = !_collisionCheckEnabled;
            if(_collisionCheckEnabled)
                std::cout << " | Collision Check enabled!" << std::endl;
            else
                std::cout << " | Collision Check disabled!" << std::endl;
            break;
        case '1':
            if(_camView)
                _curCameraView = 0;
            break;
        case '2':
            if(_camView)
                _curCameraView = 1;
            break;
        case '3':
            if(_camView)
                _curCameraView = 2;
            break;
        case 'a': // toggle application view versus tool cam view
            _camView = !_camView;
            break;
        case 's': // take snapshot of toolCam
        {
            break;
        }
        case 'p': // perform pick
            placeCenter(_lastPos(0),_lastPos(1));
            break;
    }
    if(_keyListener != NULL){
        std::pair<int,int> pos(x,y);
        _keyListener->event(EventListener::e_KEY_EVENT,&key);
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
    if ( glutGetWindow() != main_window )
        glutSetWindow(main_window);

    glutPostRedisplay();
}

/***************************************** myGlutMouse() **********/
void myGlutMouse(int button, int button_state, int x, int y )
{
    _lastPos(0) = x;
    _lastPos(1) = y;

    // Select the mouse drag mode.
    int mods = glutGetModifiers();
    if(button == GLUT_LEFT_BUTTON){
        if(button_state == GLUT_DOWN){
            _arcBall.click( _lastPos(0), _lastPos(1) );
            if(GLUT_ACTIVE_CTRL & mods){
                mouseState = ZOOM;
            } else if(GLUT_ACTIVE_SHIFT & mods){
                mouseState = TRANSLATE;
            } else {
                mouseState = ROTATE;
            }
        } else {
            mouseState = NONE;
        }
    } else if(button == GLUT_LEFT_BUTTON){
        mouseState = TRANSLATE;
    } /*else if(button == GLUT_WHEEL_UP){

    } else if(button == GLUT_WHEEL_DOWN){

    }*/ else {
        mouseState = NONE;
    }
    glutPostRedisplay();
}

/***************************************** myGlutMotion() **********/
void myGlutMotion(int x, int y)
{
    bool redraw = false;
    switch(mouseState){
        case(ROTATE):
        {
            math::Quaternion<float> quat(0.0f,0.0f,0.0f,0.0f);
            _arcBall.drag( std::pair<float,float>(x,y), quat); // Update End Vector And Get Rotation As Quaternion

            math::Rotation3D<float> thisRot = quat.toRotation3D(); // Convert Quaternion Into Rotation3D
            _viewRotation = thisRot*_viewRotation; // Accumulate Last Rotation Into This One
            _arcBall.click(x,y);
            redraw = true;
            break;
        }
        case (ZOOM):
        {
            _viewPos(2) -= (y-_lastPos(1))/_height*10;
            redraw = true;
            break;
        }
        case(TRANSLATE):
        {
            _viewPos(0) += (x-_lastPos(0))/_width*10;
            _viewPos(1) -= (y-_lastPos(1))/_height*10;
            redraw = true;
            break;
        }
        case NONE:
        {
            break;
        }
        default:
            break;
    }
    _lastPos(0) = x;
    _lastPos(1) = y;
    if(redraw)
        glutPostRedisplay();
}


void myGlutPassiveMotion(int x, int y)
{
    glutPostRedisplay();
}

/**************************************** myGlutReshape() *************/
void myGlutReshape( int width, int height )
{
    _width = width;
    _height = height;

    glViewport(0, 0, width, height);
    // resize arcball window
    _arcBall.setBounds(width,height);

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

std::map<int,EventListener*> _specialKeyList;

/**************************************** myGlutSpecial() *************/
/* Special keyboard input (function keys, arrow keys, and more). */
void myGlutSpecial(int key, int x, int y)
{
    // Arrow keys are used for moving the camera.
    if (GLUT_ACTIVE_CTRL & glutGetModifiers()) {
        switch (key) {
            case GLUT_KEY_UP: _viewPos(2)   -= 0.15; break;
            case GLUT_KEY_DOWN: _viewPos(2) += 0.15; break;
        }
    } else if (GLUT_ACTIVE_SHIFT & glutGetModifiers()) {
        switch (key) {
            case GLUT_KEY_LEFT:  _viewPos(0) -= 0.05; break;
            case GLUT_KEY_RIGHT: _viewPos(0) += 0.05; break;
            case GLUT_KEY_UP:    _viewPos(1) += 0.05; break;
            case GLUT_KEY_DOWN:  _viewPos(1) -= 0.05; break;
        }
    }
    if(_keyListener != NULL){
        std::pair<int,int> pos(x,y);
        _keyListener->event(EventListener::e_KEY_EVENT,&key);
    }

    glutPostRedisplay();
}

/************************************** GetFPS ****************
* This function calculates and prints Frames Per Second. Should
* be called each time a frame is displayed.
***************************************************************/
void calcFPS()
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

void convertRotAndPosToArray(const math::Rotation3D<float>& orien,
                             const math::Vector3D<float>& pos, GLfloat* data) {
    for (int j = 0; j<3; j++) {
        for (int k = 0; k<3; k++)
              data[j+4*k] = orien(j,k);
    }
    data[3] = 0;
    data[7] = 0;
    data[11] = 0;
    data[12] = pos(0);
    data[13] = pos(1);
    data[14] = pos(2);
    data[15] = 1;
}

/***************************************** myGlutDisplay() *****************/
void myGlutDisplay( void )
{
    glClearColor( .9f, .9f, .9f, 1.0f );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if( _camView ){
        if(_curCameraView>=0 && _curCameraView < (int)_cameras.size()){
            std::cout << ".";
            _cameras[_curCameraView]->acquire();
        }
        //_toolCam.acquire();
    } else {

        glMatrixMode(GL_PROJECTION);
        {
            glLoadIdentity();
            GLdouble aspect = (GLdouble)_width / _height;
            gluPerspective(60, aspect, 0.10, 1000);
        }

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Rotate and place camera/scene
        GLfloat data[16];
        convertRotAndPosToArray(, data);
        glMultMatrixf(data);
        glTranslated(-_pivotPoint(0), -_pivotPoint(1), -_pivotPoint(2));

        Transform3D<> vt3d(_viewPos,_viewRotation);

        getViewCamera()->setTransform(_cameraCtrl->getTransform());

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
        _renderInfo._drawType = _currentView->_drawType;
        _renderInfo._mask = _currentView->_drawMask;
        _renderInfo.cams = _currentView->_camGroup;
        _scene->draw( _renderInfo );

        // TODO: draw the ArcBall
        if(_workcellModel!=NULL){

            _workcellGLDrawer->draw() draw(*_state, _workcellModel.get());
        }
    }
    /* Disable lighting and set up ortho projection to render text */
    glDisable( GL_LIGHTING );

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluOrtho2D( 0.0, _width, _height , 0.0  );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();

    glColor3ub( 0, 0, 0 );

    // Display the FrameRate
    calcFPS();
    glColor3f(0,0,0);
    glRasterPos2i( _width-50, 20 );
    for(int i=0; i<(int)strlen( myFps ); i++ )
        glutBitmapCharacter( GLUT_BITMAP_HELVETICA_10, myFps[i] );
    glEnable( GL_LIGHTING );

    glutSwapBuffers();
}

void SimpleGLViewer::setWorkcellModel(WorkCell::Ptr workcellModel){
    // TODO: wait to obtain lock in a better way ;(
    std::cout << "Setting Workcell model" << std::endl;
    idlelock_b = true;
    while(idlelock_a);


    _workcellModel = workcellModel;
    _state = new State( _workcellModel->getDefaultState() );
    std::vector<kinematics::Frame*> cameraViews;// = _workcellModel->getCameraViews();
    //for(unsigned int i=0; i<cameraViews.size();i++){
    //    GLFrameGrabber *grapper = new GLFrameGrabber(640,480,(50.0/180.0)*3.14,&_workcellGLDrawer,*_state);
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
    //_collisionDetector = new collision::CollisionDetector(_workcellModel, &_pqpStrategy );
    //_pqpStrategy.setFirstContact(false);

    // TODO: release lock
    idlelock_b = false;
}

void initGlut(int,int,int,int);
void initLights();
void initMenu();


bool SimpleGLViewer::start(){
    // TODO: start
    std::cout << "Init Glut" << std::endl;
    initGlut(_x,_y,_width,_height);
     std::cout << "Init lights" << std::endl;
    initLights();
    std::cout << "Init Menu" << std::endl;
    initMenu();
    std::cout << "Glut main loop" << std::endl;
    glutMainLoop();
    std::cout << "Initializing WorkCellScene" << std::endl;
    _workcellGLDrawer = new WorkCellScene( new SceneOpenGL() );

    // initialize cameras
    // add the default/main cameraview group
    _mainCamGroup = _scene->makeCameraGroup("MainView");
    _scene->addCameraGroup(_mainCamGroup);
    _mainCamGroup->setEnabled(true);

    // add a node to render background
    rw::common::Ptr<RenderQuad> backgroundRender = ownedPtr(new RenderQuad());

    _backgroundRender = backgroundRender;
    _backgroundnode = _scene->makeDrawable("BackgroundRender", _backgroundRender, DrawableNode::ALL);
    _scene->addChild(_backgroundnode, _scene->getRoot());

    _worldNode = _scene->makeGroupNode("World");
    _scene->addChild(_worldNode, _scene->getRoot());



    _mainView = ownedPtr( new SceneViewer::View("MainView") );
    _currentView = _mainView;
    // add background camera
    _backCam = scene->makeCamera("BackgroundCam");
    _backCam->setEnabled(true);
    _backCam->setRefNode(_backgroundnode);
    _backCam->setProjectionMatrix( ProjectionMatrix::makeOrtho(0,640,0,480, -1, 1) );
    _backCam->setClearBufferEnabled(true);
    _backCam->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    _backCam->setDepthTestEnabled( false );
    _backCam->setLightningEnabled( false );
    _mainCamGroup->insertCamera(_backCam, 0);

    // main camera
    _mainCam = _scene->makeCamera("MainCam");
    _mainCam->setDrawMask( dmask );
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


    return true;
}

bool SimpleGLViewer::stop(){
    return true;
}

void initMenu(){
    glutCreateMenu(myGlutMenu);

    std::map<int,Menu*>::iterator menuIter = _menuMap.begin();
    for(;menuIter!=_menuMap.end();++menuIter){
        Menu *menu = menuIter->second;
        int submenuid = menuIter->first;
        std::cout << "AddSubMenu" << std::endl;
        glutAddSubMenu( menu->getName().c_str(), submenuid );
    }


    glutAddMenuEntry("Quit", QUIT);
    // Let the menu respond on the right mouse button
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void addSubMenus(Menu *menu){
    std::vector<Menu*> menus = menu->getMenus();
    for(unsigned int i=0 ; i<menus.size(); i++){
        //addMenu( menus[i] );
        int submenuid = glutCreateMenu(myGlutMenu);

        std::vector<MenuItem*> items = menu->getMenuItems();
        for(unsigned int j=0 ; j<items.size(); j++){
            // add all items
            _menuItemMap[items[i]->getId()] = items[i];
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
        _menuItemMap[items[i]->getId()] = items[i];
        glutAddMenuEntry(items[i]->getName().c_str(),items[i]->getId() );
    }
    addSubMenus(menu);
}

void SimpleGLViewer::setState(const rw::kinematics::State& state){
    *_state = state;
}

const rw::kinematics::State& SimpleGLViewer::getState(){
    return *_state;
}

void SimpleGLViewer::init(int argc, char** argv){
    glutInit(&argc, argv);
}

void initGlut(int x, int y, int width, int height){
    /****************************************/
    /*   Initialize GLUT and create window  */
    /****************************************/
    std::cout << " | Initializing Glut display functions..." << std::endl;
    /*
    std::string test("SimpleGLViewer");
    int argc=1; char *argv = (char*)test.c_str();
    glutInit(&argc, &argv); // &
    */
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition (x, y);
    glutInitWindowSize (width, height);

    main_window = glutCreateWindow ("Simple GL viewer for RobWork");
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

void initLights(){
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
