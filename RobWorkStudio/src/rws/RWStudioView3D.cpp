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

#include <rw/common/macros.hpp>
#include <rw/geometry/Line.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/opengl/RenderCameraFrustum.hpp>
#include <rws/propertyview/PropertyViewDialog.hpp>
#include "RobWorkStudio.hpp"
#include "SceneOpenGLViewer.hpp"

#include <QAction>
#include <QFileDialog>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMainWindow>
#include <QMouseEvent>
#include <QToolBar>
#include <QVBoxLayout>

#include <boost/foreach.hpp>

using namespace rw::graphics;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rwlibs::opengl;
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
    connect(_saveBufferToFileAction, SIGNAL(triggered()), this, SLOT(saveBufferToFileDialog()));

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


    _addCameraViewAction = new QAction(tr("Add Camera View..."), this); // owned
    connect(_addCameraViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _clearCameraViewsAction = new QAction(tr("Clear Camera Views..."), this); // owned
    connect(_clearCameraViewsAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));

    _selectMainViewAction = new QAction(tr("Main View..."), this); // owned
     connect(_selectMainViewAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));



     int mask = getDrawMask();

     _physicalMaskEnabled = new QAction(tr("Physical group"), this); // owned
     connect(_physicalMaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _physicalMaskEnabled->setCheckable(true);
     _physicalMaskEnabled->setChecked( DrawableNode::Physical & mask);

     _virtualMaskEnabled = new QAction(tr("Virtual group"), this); // owned
     connect(_virtualMaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _virtualMaskEnabled->setCheckable(true);
     _virtualMaskEnabled->setChecked( DrawableNode::Virtual & mask);

     _drawableMaskEnabled = new QAction(tr("Drawable group"), this); // owned
     connect(_drawableMaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _drawableMaskEnabled->setCheckable(true);
     _drawableMaskEnabled->setChecked( DrawableNode::DrawableObject & mask);

     _collisionMaskEnabled = new QAction(tr("Collision group"), this); // owned
     connect(_collisionMaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _collisionMaskEnabled->setCheckable(true);
     _collisionMaskEnabled->setChecked( DrawableNode::CollisionObject & mask);

     _user1MaskEnabled = new QAction(tr("User1 group"), this); // owned
     connect(_user1MaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _user1MaskEnabled->setCheckable(true);
     _user1MaskEnabled->setChecked( DrawableNode::User1 & mask);

     _user2MaskEnabled = new QAction(tr("User2 group"), this); // owned
     connect(_user2MaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _user2MaskEnabled->setCheckable(true);
     _user2MaskEnabled->setChecked( DrawableNode::User2 & mask);

     _user3MaskEnabled = new QAction(tr("User3 group"), this); // owned
     connect(_user3MaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _user3MaskEnabled->setCheckable(true);
     _user3MaskEnabled->setChecked( DrawableNode::User3 & mask);

     _user4MaskEnabled = new QAction(tr("User4 group"), this); // owned
     connect(_user4MaskEnabled, SIGNAL(triggered()), this, SLOT(setMaskCheckAction()));
     _user4MaskEnabled->setCheckable(true);
     _user4MaskEnabled->setChecked( DrawableNode::User4 & mask);

}


RWStudioView3D::RWStudioView3D(RobWorkStudio* rwStudio, QWidget* parent) :
    QWidget(parent),
    _view(NULL),
    _wcscene( NULL ),
    _wc(NULL),
    _rws(rwStudio),
    _viewLogo("RobWork")
{
    _pmap = _rws->getPropertyMap().add<PropertyMap>("StudioView3D","",PropertyMap());

    SceneOpenGLViewer *sceneview = new SceneOpenGLViewer(_rws->getPropertyMap(), this);

    _viewWidget = sceneview;
    _view = sceneview;
    _wcscene = ownedPtr( new WorkCellScene(_view->getScene()) );
    _view->setWorldNode( _wcscene->getWorldNode() );
    _view->setWorkCellScene(_wcscene);

    setupActions();

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

    // this is for the right click menu. The functionality is not complete yet...
    //sceneview->getWidget()->setContextMenuPolicy(Qt::CustomContextMenu);
    //connect(sceneview->getWidget(), SIGNAL(customContextMenuRequested(const QPoint&)),
    //    this, SLOT(ShowContextMenu(const QPoint&)));


    this->setFocusPolicy(Qt::StrongFocus);

    setAcceptDrops(true);
}

RWStudioView3D::~RWStudioView3D()
{

}


void RWStudioView3D::ShowContextMenu(const QPoint& pos){
    //QMenu *menu = _editor->createStandardContextMenu();

    // for most widgets
    QPoint globalPos = this->mapToGlobal(pos);
    // for QAbstractScrollArea and derived classes you would use:
    // QPoint globalPos = myWidget->viewport()->mapToGlobal(pos);

    QMenu myMenu;
    myMenu.addAction("Menu Item 1");
    // ...

    QAction* selectedItem = myMenu.exec(globalPos);
    if (selectedItem)
    {
        // something was chosen, do stuff
    }
    else
    {
        // nothing was chosen
    }
}

void RWStudioView3D::setupGUI(QMainWindow* mainwindow){
    // setup toolbar
    setupToolBarAndMenu(mainwindow);
}

int RWStudioView3D::getDrawMask(){
    return _view->getMainView()->_drawMask;
}

void RWStudioView3D::setSceneViewerWidget(SceneViewerWidget* viewer){
    _viewWidget = viewer;
    _view = viewer;
    _wcscene = ownedPtr( new WorkCellScene(_view->getScene()) );
    _view->setWorldNode( _wcscene->getWorldNode() );

    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->addWidget( viewer->getWidget() );

    setLayout(layout);
}


void RWStudioView3D::setWorkCell(rw::models::WorkCell::Ptr wc){
    _sensorCameraViews.clear();
    _wc = wc;
    if(_wc==NULL){
        RW_THROW("Workcell is null!");
    }
    BOOST_FOREACH(const FramePair& pair, _qryResult.collidingFrames) {
        _wcscene->setHighlighted(false, pair.first);
        _wcscene->setHighlighted(false, pair.second);
    }
    _qryResult.collidingFrames.clear();
	resetCameraViewMenu();

	_wcscene->setWorkCell( _wc );
    _view->setWorldNode( _wcscene->getWorldNode() );

    // if the grid is allready there then don't add it again
    if( !_wcscene->getWorldNode()->hasChild( "FloorGrid" ) ){
        // std::cout << "No floor grid" << std::endl;
        // add a floor grid drawable to the scene

        std::vector<Line> lines;
        lines.push_back(Line(Vector3D<>(5,0,0),Vector3D<>(-5,0,0)));
        lines.push_back(Line(Vector3D<>(0,5,0),Vector3D<>(0,-5,0)));
        _floorDrawable = _wcscene->addLines("FloorGrid", Line::makeGrid(10,10,0.5,0.5), _wc->getWorldFrame(), DrawableNode::Virtual);
        _floorDrawable->setColor( Vector3D<>(0.8f, 0.8f, 0.8f) );
    }
    // look for all cameras in the scene

    std::vector<Frame*> frames = Kinematics::findAllFrames(_wc->getWorldFrame(), _wc->getDefaultState());
    BOOST_FOREACH(Frame* frame, frames) {
        if (frame->getPropertyMap().has("Camera")) {
            double fovy;
            int width,height;
            std::string camId("Camera");
            std::string camParam = frame->getPropertyMap().get<std::string>(camId);
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            std::string camName = frame->getName() + "Cam";
            if(!_wcscene->findDrawable(camName, frame)){
                SensorCameraView view = makeCameraView(frame->getName() + "Cam", fovy, (double)width, (double)height, 0.001, 4.0, frame);

                RenderCameraFrustum::Ptr camFrustum = ownedPtr(new RenderCameraFrustum());
                camFrustum->setPerspective(width/(double)height, fovy, 0.001, 4.0);
                _wcscene->addRender("CamFrustrum", camFrustum , frame,  DrawableNode::Virtual);
                _sensorCameraViews.push_back( std::make_pair(view, camFrustum) );
            }
        }
    }



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
    DrawableNode::Ptr d = _view->pickDrawable( x, y);
    if(d==NULL) {
		RW_WARN("pickFrame(): drawable is NULL");
        return NULL;
	}
	Log::debugLog() << "Drawable name is " << d->getName() << std::endl;
    Frame *res = _wcscene->getFrame(d);
    return res;
}

rw::graphics::DrawableNode::Ptr RWStudioView3D::pick(int x, int y){
    DrawableNode::Ptr d = _view->pickDrawable( x, y);
    return d;
}

void RWStudioView3D::mouseDoubleClickEvent(QMouseEvent* event){
    if (event->button() == Qt::LeftButton && event->modifiers() == Qt::ControlModifier) {
		
		Log::debugLog() << "Mouse double click with control modifier..." << std::endl;
        int winx = event->x();
        int winy = height()-event->y();
        // we pick the scene before
        Frame *frame = pickFrame(winx,winy);
        if( frame != NULL){
            _rws->frameSelectedEvent().fire( frame );
            Log::debugLog() << "Frame: " << frame->getName() << std::endl;
        } else {
			Log::debugLog() << "Frame is NULL." << std::endl;
		}
    }
}

void RWStudioView3D::contextMenuEvent ( QContextMenuEvent * event ){
    //std::cout << "Menu event ;)" << std::endl;

}

void RWStudioView3D::keyPressEvent(QKeyEvent *e)
{
    // change camera view according to the keyboard inputs
    if(e->key() == Qt::Key_G && e->modifiers() == Qt::ControlModifier){
        saveBufferToFileDialog();
    } else if(e->modifiers() == Qt::ControlModifier){
        // get the currently selected view
        size_t currentView = 0;
        SceneViewer::View::Ptr currView = _view->getCurrentView();
        if(currView!=_view->getMainView() ){
            for(size_t i=0;i<_sensorCameraViews.size();i++){
                if(_sensorCameraViews[i].first._view==currView){
                    currentView = i+1;
                    break;
                }
            }
        }

        int camNr=-1;
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
        case(Qt::Key_Left):
                camNr = (int)currentView-1;
                if(camNr<0)
                    camNr=int(_sensorCameraViews.size());
                break;
        case(Qt::Key_Right):
                camNr = (int)currentView+1;
                if(camNr>int(_sensorCameraViews.size()))
                    camNr=0;
                break;
        default:
            return;
        }

        // select the camera number
        if(camNr==0){
            // select main view
            _view->selectView( _view->getMainView() );
            Log::infoLog() << "Camera view \""<< _view->getMainView()->_name << "\" selected!\n";
            _view->updateView();
        } else if(camNr>0){
            if( camNr-1<int(_sensorCameraViews.size()) ){
                _view->selectView( _sensorCameraViews[camNr-1].first._view );
                Log::infoLog() << "Camera view \""<< _sensorCameraViews[camNr-1].first._view->_name << "\" selected!\n";
                _view->updateView();
            } else {
                Log::warningLog() << "The selected camera view is not available!\n";
            }
            e->accept();
        } else {
            QWidget::keyPressEvent(e);
        }
    }
    // INSERT MORE HANDLERS HERE

    else {
        QWidget::keyPressEvent(e);
    }

}

void RWStudioView3D::setState(const rw::kinematics::State& state){
    // if collision detection is enabled then run it now, and highlight any frames that are overlapping
    if(_checkForCollision->isChecked()){
        BOOST_FOREACH(const FramePair& pair, _qryResult.collidingFrames) {
            _wcscene->setHighlighted(false, pair.first);
            _wcscene->setHighlighted(false, pair.second);
        }

        _qryResult.collidingFrames.clear();
        //std::cout << "incollision?" << std::endl;
        if( _rws->getCollisionDetector()->inCollision(state, &_qryResult) ){
            //std::cout << "\t true" << std::endl;
            BOOST_FOREACH(const FramePair& pair, _qryResult.collidingFrames) {
                _wcscene->setHighlighted(true, pair.first);
                _wcscene->setHighlighted(true, pair.second);
            }
        }
    } else if(_qryResult.collidingFrames.size()>0){
        BOOST_FOREACH(const FramePair& pair, _qryResult.collidingFrames) {
            _wcscene->setHighlighted(false, pair.first);
            _wcscene->setHighlighted(false, pair.second);
        }
        _qryResult.collidingFrames.clear();
    }

    _wcscene->setState(state);
    _view->updateState(state);
    _view->updateView();
}

void RWStudioView3D::setupToolBarAndMenu(QMainWindow* mwindow)
{
    /// --------------------------------------------------------------------------
    QToolBar* toolbar = mwindow->addToolBar(tr("View3D"));
    toolbar->setObjectName("View3D");
    toolbar->addAction(_showSolidAction);
    toolbar->addAction(_showWireAction);
    toolbar->addAction(_showOutlineAction);

    toolbar->addSeparator();

    toolbar->addAction(_showTransparentAction);
    toolbar->addAction(_showPivotPointAction);
    toolbar->addAction(_checkForCollision);

    /// --------------------------------------------------------------------------
    QToolBar* stdviewtoolbar = mwindow->addToolBar(tr("Standard views"));
    stdviewtoolbar->setObjectName("StandardViewsView3D");
    stdviewtoolbar->addAction(_axometricViewAction);
    stdviewtoolbar->addSeparator();
    stdviewtoolbar->addAction(_frontViewAction);
    stdviewtoolbar->addAction(_rightViewAction);
    stdviewtoolbar->addAction(_topViewAction);

    stdviewtoolbar->addSeparator();
    stdviewtoolbar->addAction(_rearViewAction);
    stdviewtoolbar->addAction(_leftViewAction);
    stdviewtoolbar->addAction(_bottomViewAction);

    /// --------------------------------------------------------------------------
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

    /// --------------------------------------------------------------------------
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

    /// --------------------------------------------------------------------------
	_cameraViewMenu = menu->addMenu(tr("Camera views"));
	resetCameraViewMenu();
	menu->addSeparator();

    _drawMaskMenu = menu->addMenu(tr("Render groups..."));
	_drawMaskMenu->addAction( _physicalMaskEnabled );
    _drawMaskMenu->addAction( _virtualMaskEnabled );
    _drawMaskMenu->addAction( _drawableMaskEnabled );
    _drawMaskMenu->addAction( _collisionMaskEnabled );
    _drawMaskMenu->addAction( _user1MaskEnabled );
    _drawMaskMenu->addAction( _user2MaskEnabled );
    _drawMaskMenu->addAction( _user3MaskEnabled );
    _drawMaskMenu->addAction( _user4MaskEnabled );


    /// --------------------------------------------------------------------------

    menu->addSeparator();
    menu->addAction(_saveBufferToFileAction);
}

void RWStudioView3D::resetCameraViewMenu() {
	//We need to delete all the old camera views.
	QList<QAction*> actions = _cameraViewMenu->actions();
	BOOST_FOREACH(QAction* action, actions) {
		_cameraViewMenu->removeAction(action);
	}

    _cameraViewMenu->addAction(_addCameraViewAction);
    _cameraViewMenu->addAction(_clearCameraViewsAction);
    _cameraViewMenu->addAction(_selectMainViewAction);
    _cameraViewMenu->addSeparator();
}

void RWStudioView3D::clear(){
	_view->selectView( _view->getMainView() );
    _sensorCameraViews.clear();
    _wc = NULL;
    _wcscene->setWorkCell(_wc);
    _qryResult.collidingFrames.clear();
    _view->setWorldNode( _wcscene->getWorldNode() );
	resetCameraViewMenu();
}

void RWStudioView3D::setDrawType(Render::DrawType drawType)
{
    // set DrawType for all Drawable in the view
    _view->getCurrentView()->_drawType = drawType;
}

void RWStudioView3D::setMaskCheckAction(){
	QObject *obj = sender();
	
	if (obj == _physicalMaskEnabled) {
		_rws->getPropertyMap().get<PropertyMap>("SceneViewer").set<bool>("ShowPhysicalModels", _physicalMaskEnabled->isChecked());
	}
	else if (obj == _virtualMaskEnabled) {
		_rws->getPropertyMap().get<PropertyMap>("SceneViewer").set<bool>("ShowVirtualModels", _virtualMaskEnabled->isChecked());
	}
	else if (obj == _drawableMaskEnabled) {
		_rws->getPropertyMap().get<PropertyMap>("SceneViewer").set<bool>("ShowDrawableModels", _drawableMaskEnabled->isChecked());
	}
	else if (obj == _collisionMaskEnabled) {
		_rws->getPropertyMap().get<PropertyMap>("SceneViewer").set<bool>("ShowCollisionModels", _collisionMaskEnabled->isChecked());
	}
	
	// create new mask
    int mask = 0;

    if( _physicalMaskEnabled->isChecked() )
        mask = mask | DrawableNode::Physical;
    if( _virtualMaskEnabled->isChecked() )
        mask = mask | DrawableNode::Virtual;
    if( _drawableMaskEnabled->isChecked() )
        mask = mask | DrawableNode::DrawableObject;
    if( _collisionMaskEnabled->isChecked() )
        mask = mask | DrawableNode::CollisionObject;
    if( _user1MaskEnabled->isChecked() )
        mask = mask | DrawableNode::User1;
    if( _user2MaskEnabled->isChecked() )
        mask = mask | DrawableNode::User2;
    if( _user3MaskEnabled->isChecked() )
        mask = mask | DrawableNode::User3;
    if( _user4MaskEnabled->isChecked() )
        mask = mask | DrawableNode::User4;

    // set draw mask for camera
    _view->getMainView()->_viewCamera->setDrawMask(mask);
    
    // refresh view
    _view->updateView();
}

void RWStudioView3D::setCheckAction(){
    QObject *obj = sender();

    Vector3D<> center = Vector3D<>::zero(); // = _view->getViewCenter();
    Transform3D<> view = _view->getTransform();
    Vector3D<> v2c = view.P()-center;

    if(obj==_axometricViewAction){
        _view->setTransform( Transform3D<>::makeLookAt(Vector3D<>( v2c.norm2()/sqrt(3.0),v2c.norm2()/sqrt(3.0),v2c.norm2()/sqrt(3.0)), center, Vector3D<>::z() ) );
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



    else if(obj == _addCameraViewAction){
        // add the current view transform to the view list
        size_t nrView = _sensorCameraViews.size();
        if(nrView<10){
            std::stringstream sstr;
            sstr << "Cam" << nrView;

            // TODO: create
            PropertyMap map;
            map.add<std::string>("Name", "Name of the camera", sstr.str());
            map.add<double>("Fovy", "Vertical Field Of View in Degree", 45);
            map.add<double>("Width", "Width", 640);
            map.add<double>("Height", "Height", 480);
            map.add<double>("Near", "Near Clipping Plane", 0.01);
            map.add<double>("Far", "Far Clipping Plane", 3.0);

            std::vector<Frame*> allFrames = _wc->findFrames<Frame>();
            std::vector<std::string> strlist;
            BOOST_FOREACH(Frame* f, allFrames){ strlist.push_back(f->getName()); }
            //map.add("Frame", "Frame name", strlist);
            map.add<std::string>("Frame", "Frame name", "WORLD");

            PropertyViewDialog *dialog = new PropertyViewDialog(&map, this);
            if( dialog->exec() == QDialog::Accepted ){
                std::string fname = map.get<std::string>("Frame");
                Frame *frame = _wc->findFrame(fname);
                if(frame!=NULL){
                    std::string name = map.get<std::string>("Name");
                    SensorCameraView view = makeCameraView(
                                            name,
                                            map.get<double>("Fovy"),
                                            map.get<double>("Width"),
                                            map.get<double>("Height"),
                                            map.get<double>("Near"),
                                            map.get<double>("Far"),
                                            frame);

                    // set params 
                    RenderCameraFrustum::Ptr camFrustum = ownedPtr(new RenderCameraFrustum());
                    camFrustum->setPerspective(view._width/view._height, view._fovy, view._near, view._far);
                    _wcscene->addRender(name, camFrustum , frame,  DrawableNode::Virtual);

                    _sensorCameraViews.push_back( std::make_pair(view, camFrustum) );
                }
            }
        }
    } else if(obj == _clearCameraViewsAction){
        for(size_t i=0;i<_sensorCameraViews.size();i++){
            _cameraViewMenu->removeAction(_sensorCameraViews[i].first._action);
        }
        _sensorCameraViews.clear();
    } else if( obj == _selectMainViewAction){
        _view->selectView( _view->getMainView() );
        _view->updateView();
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

        for(size_t i=0;i<_sensorCameraViews.size();i++){
            if(obj == _sensorCameraViews[i].first._action){
                // the sensor camera is selected to be rendered
                //std::cout << "SELECTING VIEW" << std::endl;
                _view->selectView( _sensorCameraViews[i].first._view );
                _view->updateView();
                break;
            }
        }

    }
    _view->updateView();
    //std::cout << _view->getTransform() << std::endl;
}

RWStudioView3D::SensorCameraView RWStudioView3D::makeCameraView(const std::string& name, double fovy, double w, double h, double n, double f, Frame* frame){

    QAction* nAction = _cameraViewMenu->addAction( name.c_str() );
    SensorCameraView view(fovy, w, h, n, f, frame);
    bool enableBackground=true;
    // add CameraGroup to scenegraph
    view._view = _view->createView(name, enableBackground);
    // setup the view camera
    GroupNode::Ptr fnode = _wcscene->getNode(frame);
    //if(fnode == NULL)
    //    std::cout << "FNODE is NULL" << std::endl;
    view._view->_viewCamera->setAspectRatioControl(SceneCamera::Scale);
    view._view->_viewCamera->setEnabled(true);

    view._view->_viewCamera->setRefNode( _view->getScene()->getRoot() );
    //std::cout << view._width <<  " " << view._height << std::endl;
    view._view->_viewCamera->setPerspective(view._fovy, view._width, view._height, view._near, view._far);
    view._view->_viewCamera->setViewport(0,0, view._width, view._height);
    view._view->_viewCamera->setAspectRatioControl(SceneCamera::Fixed);
    view._view->_viewCamera->attachTo(fnode);
    view._view->_viewCamera->setDrawMask(DrawableNode::Physical);
    view._action = nAction;

    connect(nAction, SIGNAL(triggered()), this, SLOT(setCheckAction()));
	
    _cameraViewMenu->addAction(nAction);

    return view;
}

void RWStudioView3D::setCheckForCollision(bool){
    // set check for collision
    setState( _rws->getState() );
}

void RWStudioView3D::setDrawTypeSlot()
{

    if (_showSolidAction->isChecked())
        setDrawType(DrawableNode::SOLID);
    else if (_showWireAction->isChecked())
        setDrawType(DrawableNode::WIRE);
    else if (_showOutlineAction->isChecked())
        setDrawType(DrawableNode::OUTLINE);
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
    BOOST_FOREACH(DrawableNode::Ptr da, _wcscene->getDrawables()) {
		da->setTransparency(alpha);
	}

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
    _viewWidget->getPivotDrawable()->setVisible(visible);
    _view->updateView();
}

void RWStudioView3D::saveBufferToFileDialog()
{
    // Get last save location - default to ./
    std::string lastDir = _pmap->getValue().get<std::string>("LastDir", "./");
    // Open a file dialog
    QString filename = QFileDialog::getSaveFileName(
        this, "Save Image", lastDir.c_str() ,"Images (*.png *.bmp *.jpg *.jpeg *.gif *.ppm *.tiff *.xpm *.xbm)");

    if (!filename.isEmpty()) {
        try {
            // Get the save location
            QFileInfo fi(filename);
            // test if we need to append a specific image format
            if(fi.suffix().isEmpty())
                fi = QFileInfo(filename + ".png");
            lastDir = fi.absolutePath().toStdString();
            // Store new save location
            _pmap->getValue().set<std::string>("LastDir", lastDir);
            
            // Save
            _view->saveBufferToFile(filename.toStdString(), 0, 0, 0);
        } catch (const std::string& exp) {
            QMessageBox::information(
                this, "Failed to save file ", exp.c_str(), QMessageBox::Ok);
        }
    }
}

void RWStudioView3D::saveSettings(){

}

void RWStudioView3D::restoreSettings(){

}


