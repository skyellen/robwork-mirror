
#include <rw/pathplanning/StraightLinePathPlanner.hpp>
#include <rw/pathplanning/PathPlanner.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/collision/CollisionSetup.hpp>
#include <rw/collision/CollisionDetector.hpp>

#include <rwlibs/pathplanners/rrt/RRTPathPlanner.hpp>
#include <rwlibs/pathplanners/lazyprm/LazyPRMPathPlanner.hpp>

#include <rwlibs/collisionstrategies/CDStrategyOpcode.hpp>

#include <rw/loaders/WorkCellLoader.hpp>

#include <boost/shared_ptr.hpp>

#include <rw/drawable/WorkCellGLDrawer.hpp>
#include <rw/drawable/Drawable.hpp>

#include <time.h>
#include <ios>
#include <iostream>

#include <GL/gl.h>
#include <GL/glut.h>

#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

using namespace rwlibs::pathplanners;
using namespace rwlibs::collisionstrategies;

State *state;
std::auto_ptr<WorkCell> workcell;
CDStrategyOpcode *cdStrategy;
WorkCellGLDrawer drawer;
CollisionSetup *colSetup;

// Current configuration
Q q;

// Start configuration
Q qInit;

// Goal configuration
Q qGoal;

const double RESOLUTION = 0.01;

RRTPathPlanner *rrtPathPlanner;
StraightLinePathPlanner *straightLinePathPlanner;
LazyPRMPathPlanner *lazyPRMPathPlanner;

Device* device = NULL;
PathPlanner* planner;

/* Animation */
Path path;
Path::iterator it = path.end();

namespace
{
void idlefunc();

bool fill = true;


/*
 * Rotation and translation of the camera.
 */
GLdouble rotationX = 0;
GLdouble rotationY = 0;

GLdouble translationX = 0;
GLdouble translationY = 0;
GLdouble translationZ = 0;

/*
 * Variables for keeping track of dragging of the mouse.
 */
enum MouseDragMode {None, Zoom, Translate, Rotate};

MouseDragMode mouseDrag = None;
GLdouble startX;
GLdouble startY;

/*
 * Width and height of the window.
 */
int height;
int width;

/*
 * Clock
 */
double start;
double finish;

void help()
{
    const std::string msg =
	" ...................................................................\n"
	" Help - key specification\n"
	" 'h'      this screen\n"
	" [0 - 9]  choose a joint axis of the robot\n"
	" '-'      decrements the choosen joint axis\n"
	" '+'      decrements the choosen joint axis\n"
	" <enter>  starts path planning from home to end\n"
	" 'f'      toggles between SOLID and WIRE rendering\n"
	" 's'      use straight line planner\n"
	" 'l' 	    use lazy prm planner\n"
	" 'r' 	    use RRT planner\n"
	" 'd'      animate the planned path\n"
	" 'z'      set the robot in zero position\n"
	" <home>   sets current position to home position\n"
	" <end>    sets current position to end position\n"
	" ...................................................................\n";
    std::cout << msg;
}

/*
 * Standard keyboard input (e.g. letters and digits).
 */
void keyboard(unsigned char key, int x, int y)
{
    const static double DELTA = 0.01;
    static unsigned int axisNr = 0;
    if('0' <= key && key <= '9'){
        axisNr = key - '0';
    }else if(key == '-' && axisNr<q.size()){
        double val = q[axisNr];
        val -= DELTA;
        if(val >= device->getBounds().first[axisNr])
            q[axisNr] = val;
        device->setQ(q,*state);
        glutPostRedisplay();
    }else if(key == '+' && axisNr<q.size()){
        double val = q[axisNr];
        val += DELTA;
        if(val <= device->getBounds().second[axisNr])
            q[axisNr] = val;
        device->setQ(q,*state);
        glutPostRedisplay();
    }else if(key == 'f'){
        fill = !fill;
        std::vector<Drawable*> drawables = drawer.getAllDrawables(*state,workcell.get());
        for(size_t i=0; i<drawables.size(); i++){
        	if(!fill)
        		drawables[i]->setDrawType(Drawable::WIRE);
        	else
        		drawables[i]->setDrawType(Drawable::SOLID);
        }
        glutPostRedisplay();
    }else if(key == '\r'){
        std::cout << "Planning, from: " <<
            qInit <<
            " to: " <<
            qGoal << "";
        path.clear();
        for(unsigned int num = 0; num < 1; num++){
            start = clock();
            if(planner->query(qInit, qGoal, path, 400.0 /* seconds */)) break;
        }

        if(path.size()==0){
            std::cout << "No path found!" << std::endl;
        }else{
            finish = clock();
            std::cout << "Path found! Length: "<< path.size() << " Time: " << (double)(finish-start)/CLOCKS_PER_SEC << " secs" << std::endl;
        }

        it = path.begin();
        glutIdleFunc(idlefunc);

    }else if(key == 's'){
        planner = straightLinePathPlanner;
    }else if(key == 'r'){
        planner = rrtPathPlanner;
    }else if(key == 'l'){
        planner = lazyPRMPathPlanner;
    }else if(key == 'd'){
        it = path.begin();
        glutIdleFunc(idlefunc);
    }else if(key == 'z'){
        for(unsigned int i=0; i<q.size(); i++){
            q[i] = 0.0;
        }
        device->setQ(q,*state);
        glutPostRedisplay();
    } else if(key == 'h'){
    	help();
    }
}

/*
 * Special keyboard input (function keys, arrow keys, and more).
 */
void special(int key, int x, int y)
{
    if(GLUT_KEY_F1 <= key &&
       key <= GLUT_KEY_F10 &&
       ((unsigned int)(key-GLUT_KEY_F1))<q.size())
    {
        if(GLUT_ACTIVE_SHIFT & glutGetModifiers()){
            //q[key-GLUT_KEY_F1] -= 0.1;
            //scene.robot().validateJointConfiguration(q);
        }else{
            //q[key-GLUT_KEY_F1] += 0.1;
            //scene.robot().validateJointConfiguration(q);
        }
    }else if(key == GLUT_KEY_HOME){
        qInit = q;
        std::cout << "Home set" << std::endl;
    }else if(key == GLUT_KEY_END){
        qGoal = q;
        std::cout << "End set" << std::endl;
    }

    // Arrow keys are used for moving the camera.
    if (GLUT_ACTIVE_CTRL & glutGetModifiers()) {
        switch (key) {
        case GLUT_KEY_UP: translationZ -= 0.15; break;
        case GLUT_KEY_DOWN: translationZ += 0.15; break;
        }
    } else if (GLUT_ACTIVE_SHIFT & glutGetModifiers()) {
        switch (key) {
        case GLUT_KEY_LEFT: translationX -= 0.05; break;
        case GLUT_KEY_RIGHT: translationX += 0.05; break;
        case GLUT_KEY_UP: translationY += 0.05; break;
        case GLUT_KEY_DOWN: translationY -= 0.05; break;
        }
    } else {
        switch (key) {
        case GLUT_KEY_LEFT: rotationY -= 3; break;
        case GLUT_KEY_RIGHT: rotationY += 3; break;
        case GLUT_KEY_UP: rotationX -= 3; break;
        case GLUT_KEY_DOWN: rotationX += 3; break;
        }
    }
    glutPostRedisplay();
}

/*
 * Mouse button actions.
 */
void mouse(int button, int state, int x, int y)
{
    startX = x;
    startY = y;

    // Select the mouse drag mode.
    int mods = glutGetModifiers();
    mouseDrag =
        button != GLUT_LEFT_BUTTON || state == GLUT_UP ? None :
        GLUT_ACTIVE_CTRL & mods ? Zoom :
        GLUT_ACTIVE_SHIFT & mods ? Translate :
        Rotate;
}

/*
 * Mouse movement.
 */
void motion(int x, int y)
{
    GLdouble dx = x - startX;
    GLdouble dy = y - startY;

    // Update the camera transformation.
    switch (mouseDrag) {
    case Rotate:
        rotationY += 360 * dx / width;
        rotationX += 360 * dy / height;
        break;
    case Zoom:
        translationZ += 10 * dy / height;
        break;
    case Translate:
        translationX += 10 * dx / width;
        translationY += - 10 * dy / height;
        break;
    case None:
        break;
    }

    startX = x;
    startY = y;

    glutPostRedisplay();
}

void idlefunc(){
    static clock_t nextupdate = 0;
    static clock_t interval = 0;

    if(it==path.begin() && it!=path.end()){
        interval = (clock_t)((double)(CLOCKS_PER_SEC * 5 / path.size()) * RESOLUTION);
        q = *it;
        device->setQ(q,*state);
        nextupdate = clock() + interval;
        ++it;
    }else if(it!=path.end())
    {
        if(clock() > nextupdate){
            if(norm_2(*it - q) < RESOLUTION)
                ++it;
            if(it!=path.end()){
                q = q + ((*it - q)/norm_2(*it - q)) * RESOLUTION;
                device->setQ(q,*state);
            }else
                glutIdleFunc(NULL);

            nextupdate += interval;

        }
    }
    glutPostRedisplay();
}

void displayText(std::string text){
    //glColor3fv(white);
    glColor3d(1.0, 0.0, 0.0);
    glRasterPos2d(-3.6, 2.6);
    for(unsigned int i=0;i<text.length();i++){
        glutBitmapCharacter( GLUT_BITMAP_TIMES_ROMAN_24, text.at(i));
    }
}


/*
 * Updating of the display.
 */
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    {
        // Position the camera.
        glTranslated(translationX, translationY, translationZ);
        glRotated(rotationX, 1, 0, 0);
        glRotated(rotationY, 0, 1, 0);
        glRotated(-90, 1, 0, 0);

        if(fill)
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        drawer.draw(*state,workcell.get());
    }
    glPopMatrix();
    glutSwapBuffers();
    glFlush();
}

/*
 * Reshaping of the window.
 */
void reshape(int newWidth, int newHeight)
{
    width = newWidth;
    height = newHeight;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    {
        glLoadIdentity();

        GLdouble aspect = (GLdouble)width / height;
        gluPerspective(60, aspect, 0.10, 1000);
        glTranslated(0, 0, -5);
    }
    glMatrixMode(GL_MODELVIEW);
}
}

void text(int t){

}

int main(int argc, char** argv)
{
    if (argc != 2){
        std::cerr << "Usage: " << argv[0] << " <scene filename>" << std::endl;
        return -1;
    }

    workcell = rw::loaders::WorkCellLoader::load(argv[1]);

    if(workcell->getDevices().size() == 0){
        std::cerr << "Error: workcell has no devices!" << std::endl;
        return -1;
    }

    help();

    state = new State( workcell->getDefaultState() );
    device = workcell->getDevices()[0];

    CollisionSetup colSetup( CollisionSetup::PairList() );
    cdStrategy = new CDStrategyOpcode();

    CollisionDetector* collisionDetector =
        new CollisionDetector(workcell.get(), cdStrategy);

    rrtPathPlanner =
        new RRTPathPlanner(workcell.get(), device, collisionDetector, RESOLUTION);
    straightLinePathPlanner =
        new StraightLinePathPlanner(
            device, workcell->getDefaultState(), collisionDetector, RESOLUTION);
    lazyPRMPathPlanner =
        new LazyPRMPathPlanner(
            workcell.get(), device, collisionDetector, RESOLUTION);

    lazyPRMPathPlanner->initialize(device);

    planner = rrtPathPlanner;

    q = device->getQ(*state);
    qInit = q;
    qGoal = q;

    // Initialize Glut.
    glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize (800, 600);
    glutInitWindowPosition (0, 0);
    glutCreateWindow ("Viewer");

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

    // Register callback functions.
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    //glutIdleFunc(display);

    // Start the program.
    glutMainLoop();

    return 0;
}
