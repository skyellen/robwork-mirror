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

#include "RWStudioView3D.hpp"

#include <rw/common/Timer.hpp>
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include "RobWorkStudio.hpp"
#include "SceneOpenGLViewer.hpp"
#include <QThread>
#include <boost/foreach.hpp>

using namespace rw::graphics;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rws;

void RWStudioView3D::setupActions(){
    // solid action
    _showSolidAction = new QAction(QIcon(":/images/solid.png"), tr("&Solid"), this); // owned
    _showSolidAction->setCheckable(true);
    _showSolidAction->setChecked(true);
    connect(_showSolidAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot()));

    // wire action
    _showWireAction = new QAction(QIcon(":/images/wireframe.png"), tr("&Wire"), this); // owned
    _showWireAction->setCheckable(true);
    connect(_showWireAction, SIGNAL(triggered()), this, SLOT(setDrawTypeSlot()));

    // outline action
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
    connect(_showTransparentAction, SIGNAL(triggered()), this, SLOT(setTransparentSlot()));

    _showPivotPointAction =
        new QAction(QIcon(":/images/pivotpoint.png"), tr("Pivot Point"), this); // owned
    _showPivotPointAction->setCheckable(true);
    _showPivotPointAction->setChecked(true);
    connect(_showPivotPointAction, SIGNAL(triggered()), this, SLOT(showPivotPointSlot()));

    _checkForCollision =
        new QAction(QIcon(":/images/collision.png"), tr("Collision Checking"), this); // owned
    _checkForCollision->setCheckable(true);
    _checkForCollision->setChecked(true);
    connect(_checkForCollision, SIGNAL(triggered(bool)), this, SLOT(setCheckForCollision(bool)));

    _saveBufferToFileAction = new QAction(tr("Save view..."), this); // owned
    connect(_saveBufferToFileAction, SIGNAL(triggered()), this, SLOT(saveBufferToFileQuery()));

    // view perspective
    _setPerspectiveViewAction = new QAction(QIcon(":/images/default_view_100.png"), tr("&Perspective view"), this); // owned
    connect(_setPerspectiveViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _setOrthographicViewAction = new QAction(QIcon(":/images/default_view_100.png"), tr("&Orthographic view"), this); // owned
    connect(_setOrthographicViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    // view transforms
    _axometricViewAction =
        new QAction(QIcon(":/images/default_view_100.png"), tr("&Axiometric"), this); // owned
    connect(_axometricViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _frontViewAction =
        new QAction(QIcon(":/images/front_view_100.png"), tr("Front"), this); // owned
    connect(_frontViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _rightViewAction =
        new QAction(QIcon(":/images/right_view_100.png"), tr("Right"), this); // owned
    connect(_rightViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _topViewAction = new QAction(QIcon(":/images/top_view_100.png"), tr("Top"), this); // owned
    connect(_topViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _rearViewAction = new QAction(QIcon(":/images/rear_view_100.png"), tr("Rear"), this); // owned
    connect(_rearViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _leftViewAction = new QAction(QIcon(":/images/left_view_100.png"), tr("Left"), this); // owned
    connect(_leftViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _bottomViewAction = new QAction(QIcon(":/images/bottom_view_100.png"), tr("Bottom"), this); // owned
    connect(_bottomViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _addViewAction = new QAction(tr("Add view..."), this); // owned
    connect(_addViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _clearViewAction = new QAction(tr("Clear views"), this); // owned
    connect(_clearViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

}




RWStudioView3D::RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent) :
    QWidget(parent),
    _view(NULL),
    _wcscene( NULL ),
    _wc(NULL),
    _rws(rwStudio),
    _viewLogo("RobWork")
{
    std::cout << "RWStudioView3D construct" << std::endl;


    _pmap = _rws->getPropertyMap().add<PropertyMap>("StudioView3D","",PropertyMap());

    setupActions();

    SceneOpenGLViewer *sceneview = new SceneOpenGLViewer(_rws->getPropertyMap(), this);
    //setSceneViewerWidget(sceneview);

    _view = sceneview;
    _wcscene = ownedPtr( new WorkCellScene(_view->getScene()) );
    _view->setWorldNode( _wcscene->getWorldNode() );

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget( sceneview->getWidget() );

    setLayout(layout);

    // We set the GUI element to checked, but we happen to also need to set this
    // to true manually.
    //_showPivotPoint = true;

    //_view->setWorldNode( _wcscene->getWorldNode() );

    //QVBoxLayout *layout = new QVBoxLayout(this);
    //layout->addWidget(viewer->getWidget());

    //setLayout(layout);

    //this->setFocusPolicy(Qt::StrongFocus);

    std::cout << "RWStudioView3D construct" << std::endl;
}

void RWStudioView3D::setupGUI(QMainWindow* mainwindow){
    // setup toolbar
    setupToolBarAndMenu(mainwindow);
}

RWStudioView3D::~RWStudioView3D()
{

}

void RWStudioView3D::setSceneViewerWidget(SceneViewerWidget* viewer){
    _view = viewer;
    _wcscene = ownedPtr( new WorkCellScene(_view->getScene()) );
    _view->setWorldNode( _wcscene->getWorldNode() );

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget( viewer->getWidget() );

    setLayout(layout);
}

void RWStudioView3D::setWorkCell(rw::models::WorkCell::Ptr workcell){

    _wc = workcell;
    _wcscene->setWorkCell(_wc);
    _view->setWorldNode( _wcscene->getWorldNode() );
    // add a floor grid drawable to the scene



    std::vector<Line> lines;
    lines.push_back(Line(Vector3D<>(5,0,0),Vector3D<>(-5,0,0)));
    lines.push_back(Line(Vector3D<>(0,5,0),Vector3D<>(0,-5,0)));
    _floorDrawable = _wcscene->addLines("FloorGrid", Line::makeGrid(10,10,0.5,0.5), workcell->getWorldFrame(), DrawableNode::Virtual);
    _floorDrawable->setColor( Vector3D<>(0.8f, 0.8f, 0.8f) );

    BOOST_FOREACH(SceneNode::Ptr n, _wcscene->getWorldNode()->_childNodes ){
        std::cout << "SADAS:  " << n->getName() << std::endl;
    }

    // look for all cameras in the scene
    /*
    std::vector<Frame*> frames = Kinematics::findAllFrames(workcell->getWorldFrame(), *state);
    BOOST_FOREACH(Frame* frame, frames) {
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
    */


    /* this belong in the loading of the scene graph
     *
     *         if (frame->getPropertyMap().has("Light")) {
                //std::cout << "Parsing light source!" << std::endl;
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
     */


};

rw::kinematics::Frame* RWStudioView3D::pickFrame(int x, int y){
    DrawableNode::Ptr d = _view->pickDrawable(x, y);
    if(d==NULL)
        return NULL;
    return _wcscene->getFrame(d);
}

void RWStudioView3D::mouseDoubleClickEvent(QMouseEvent* event){
    std::cout << "double mouse click" << std::endl;
    if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ControlModifier) {
        int winx = event->x();
        int winy = height()-event->y();
        // we pick the scene before
        Frame *frame = pickFrame(winx,winy);
        if( frame ){
            _rws->frameSelectedEvent().fire( frame );
        }
    }
}


void RWStudioView3D::keyPressEvent(QKeyEvent *e)
{
    size_t camNr=-1;
    switch(e->key()){
    case(Qt::Key_1): camNr = 0; break;
    case(Qt::Key_2): camNr = 1; break;
    case(Qt::Key_3): camNr = 2; break;
    case(Qt::Key_4): camNr = 3; break;
    case(Qt::Key_5): camNr = 4; break;
    case(Qt::Key_6): camNr = 5; break;
    case(Qt::Key_7): camNr = 6; break;
    case(Qt::Key_8): camNr = 7; break;
    case(Qt::Key_9): camNr = 8; break;
    default:
        return;
    }
    // select the camera number
    if(camNr>=0){
        e->accept();
    } else {
        e->ignore();
    }
}

void RWStudioView3D::setState(const rw::kinematics::State& state){
    _wcscene->setState(state);
}

void RWStudioView3D::setupToolBarAndMenu(QMainWindow* mwindow)
{
    QToolBar* toolbar = mwindow->addToolBar(tr("View3D"));
    toolbar->addAction(_showSolidAction);
    toolbar->addAction(_showWireAction);
    toolbar->addAction(_showOutlineAction);

    toolbar->addSeparator();

    toolbar->addAction(_showTransparentAction);
    toolbar->addAction(_showPivotPointAction);
    toolbar->addAction(_checkForCollision);

    QToolBar* stdviewtoolbar = mwindow->addToolBar(tr("Standard views"));
    stdviewtoolbar->addAction(_axometricViewAction);
    stdviewtoolbar->addSeparator();
    stdviewtoolbar->addAction(_frontViewAction);
    stdviewtoolbar->addAction(_rightViewAction);
    stdviewtoolbar->addAction(_topViewAction);

    stdviewtoolbar->addSeparator();
    stdviewtoolbar->addAction(_rearViewAction);
    stdviewtoolbar->addAction(_leftViewAction);
    stdviewtoolbar->addAction(_bottomViewAction);

    QMenu* menu = mwindow->menuBar()->addMenu(tr("&View3D"));

    menu->addSeparator();
    menu->addAction(_setPerspectiveViewAction);
    menu->addAction(_setOrthographicViewAction);
    menu->addSeparator();

    menu->addAction(_showSolidAction);
    menu->addAction(_showWireAction);
    menu->addAction(_showOutlineAction);
    menu->addSeparator();

    menu->addAction(_showTransparentAction);
    menu->addAction(_showPivotPointAction);
    menu->addAction(_checkForCollision);
    menu->addSeparator();

    // standard views
    QMenu* standardViewMenu = menu->addMenu(tr("Standard views"));
    standardViewMenu->addAction(_axometricViewAction);
    standardViewMenu->addSeparator();
    standardViewMenu->addAction(_frontViewAction);
    standardViewMenu->addAction(_rightViewAction);
    standardViewMenu->addAction(_topViewAction);

    standardViewMenu->addSeparator();
    standardViewMenu->addAction(_rearViewAction);
    standardViewMenu->addAction(_leftViewAction);
    standardViewMenu->addAction(_bottomViewAction);

    _customViewMenu = menu->addMenu(tr("Custom views"));
    _customViewMenu->addAction(_addViewAction);
    _customViewMenu->addAction(_clearViewAction);
    _customViewMenu->addSeparator();



    menu->addSeparator();
    menu->addAction(_saveBufferToFileAction);
    RW_WARN("done setup");
}

void RWStudioView3D::setDrawType(Render::DrawType drawType)
{
    // set DrawType for all Drawable in the view

}

void RWStudioView3D::setCheckAction(){
    QObject *obj = sender();

    Vector3D<> center = _view->getViewCenter();
    Transform3D<> view = _view->getTransform();
    Vector3D<> v2c = view.P()-center;

    if(obj==_axometricViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>( v2c.norm2()/sqrt(3),v2c.norm2()/sqrt(3),v2c.norm2()/sqrt(3)), center, Vector3D<>::z() ) );
    } else if(obj==_frontViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>( v2c.norm2(),0,0), center, Vector3D<>::z() ) );
    } else if(obj==_rearViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>(-v2c.norm2(),0, 0), center, Vector3D<>::z() )  );
    } else if(obj==_topViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>(0,0,v2c.norm2()), center, -Vector3D<>::x() )  );
    } else if(obj==_bottomViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>(0,0,-v2c.norm2()), center, Vector3D<>::x() )  );
    } else if(obj==_rightViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>(0,v2c.norm2(),0), center, Vector3D<>::z() ) );
    } else if(obj==_leftViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>(0,-v2c.norm2(),0), center, Vector3D<>::z() ) );
    }

    else if(obj == _addViewAction){
        // add the current view transform to the view list
        size_t nrView = _customViews.size();
        if(nrView<10){
            std::stringstream sstr;
            sstr << "view" << nrView;
            QAction* nAction = _customViewMenu->addAction(sstr.str().c_str());
            _customViews.push_back(std::make_pair(nAction, _view->getTransform()));
            connect(nAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));
            _customViewMenu->addAction(nAction);
        }
    } else if(obj == _clearViewAction){
        for(size_t i=0;i<_customViews.size();i++){
            _customViewMenu->removeAction(_customViews[i].first);
        }
        _customViews.clear();
    }


    else if(obj == _setOrthographicViewAction){
        int x,y,w,h;
        _view->getViewCamera()->getViewport(x,y,w,h);
        _view->getViewCamera()->setProjectionMatrix( ProjectionMatrix::makeOrtho(-1.0,1.0,-1.0,1.0, 0.1, 30) );
    } else if(obj == _setPerspectiveViewAction){
        int x,y,w,h;
        _view->getViewCamera()->getViewport(x,y,w,h);
        _view->getViewCamera()->setProjectionMatrix( ProjectionMatrix::makePerspective(45, w/((double)h), 0.1, 30) );
    } else {
        for(size_t i=0;i<_customViews.size();i++){
            if(obj == _customViews[i].first){
                _view->setTransform( _customViews[i].second );
                break;
            }
        }
    }
    //std::cout << _view->getTransform() << std::endl;
}

void RWStudioView3D::setCheckForCollision(bool){
    // set check for collision
}

void RWStudioView3D::setDrawTypeSlot()
{
    /*
    if (_showSolidAction->isChecked())
        setDrawType(Render::SOLID);
    else if (_showWireAction->isChecked())
        setDrawType(Render::WIRE);
    else if (_showOutlineAction->isChecked())
        setDrawType(Render::OUTLINE);
    */
    _view->updateView();
}

void RWStudioView3D::setTransparentSlot()
{
    double alpha;
    if (_showTransparentAction->isChecked())
        alpha = 0.5;
    else
        alpha = 1.0;

    // set alpha for all Drawable in the view
    //BOOST_FOREACH(Drawable::Ptr da, _drawables) { da->setAlpha(alpha); }


    _view->updateView();
}

void RWStudioView3D::showPivotPointSlot()
{
    showPivotPoint(_showPivotPointAction->isChecked());
}

void RWStudioView3D::showPivotPoint(bool visible)
{
    //_showPivotPoint = visible;
    //updateGL();
    _view->updateView();
}

