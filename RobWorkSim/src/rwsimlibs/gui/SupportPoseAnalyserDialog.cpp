#include "SupportPoseAnalyserDialog.hpp"

#include "RestingPoseDialog.hpp"
#include "GLViewRW.hpp"

#include <sstream>
#include <fstream>

#include <boost/foreach.hpp>

#include "RWSimGuiConfig.hpp"

#ifdef USE_OPENCV
#include <cv.h>
#include <highgui.h>
#endif

#include <rw/math/RPY.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rwlibs/opengl/RenderFrame.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
//#include <rw/proximity/CollisionDetector.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/opengl/Drawable.hpp>

#include <rw/sensor/Image.hpp>
#include <rw/sensor/ImageUtil.hpp>

#include <rwsim/drawable/RenderPoints.hpp>
#include <rwsim/drawable/RenderPlanes.hpp>
#include <rwsim/drawable/RenderCircles.hpp>

#include <rwsim/util/CircleModel.hpp>

#include <rwsim/util/PlanarSupportPoseGenerator.hpp>

//#include <rw/geometry/Geometry.hpp>
#include <rwlibs/algorithms/kdtree/KDTree.hpp>
#include <rwlibs/algorithms/kdtree/KDTreeQ.hpp>

#include "ui_SupportPoseAnalyserDialog.h"

#include <QGraphicsPixmapItem>
#include <QFileDialog>

USE_ROBWORK_NAMESPACE
using namespace std;
using namespace robwork;

using namespace rwlibs::algorithms;
using namespace rwsim::dynamics;
using namespace rwsim::util;
using namespace rwsim::drawable;

using namespace rw::geometry;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rwlibs::opengl;
using namespace rw::sensor;

namespace {

	std::string openFile(std::string& previousOpenDirectory, QWidget *parent){
	    QString selectedFilter;

	    const QString dir(previousOpenDirectory.c_str());

	    QString filename = QFileDialog::getOpenFileName(
	    	parent,
	        "Open State path", // Title
	        dir, // Directory
	        " RW state path ( *.rwplay )"
	        " \n All ( *.* )",
	        &selectedFilter);

	    std::string file = filename.toStdString();
	    if (!file.empty())
	        previousOpenDirectory = rw::common::StringUtil::getDirectoryName(file);

	    return file;
	}
}

void SupportPoseAnalyserDialog::updateHoughThres(){
    // we set the thres hold such that [30,50] maps to [100,1000]
    std::size_t samples = _xaxis[0].size();
    double a = ((50.0-30.0)/(1000.0-100.0));
    double b = 30-100.0*a;
    double thres = a*samples+b;
    thres = rw::math::Math::clamp(thres,20.0,250.0);
    _ui->_thresholdSpin->setValue((int)thres);
}

rwsim::dynamics::RigidBody::Ptr SupportPoseAnalyserDialog::getSelectedBody(){
    int i = _ui->_selectObjBox->currentIndex();
    return _bodies[i];
}


SupportPoseAnalyserDialog::SupportPoseAnalyserDialog(const rw::kinematics::State& state,
                                    DynamicWorkCell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    rws::RobWorkStudio *rwstudio,
                                    QWidget *parent):
    QDialog(parent),
    _defaultState(state),
    _wc(dwc->getWorkcell().get() ),
    _dwc(dwc),
    _detector(detector),
    _restPoseDialog(NULL),
    _rwstudio(rwstudio)
{
    _ui = new Ui::SupportPoseAnalyserDialog();
    _ui->setupUi(this);
    _pitem = new QGraphicsPixmapItem();
    QGraphicsScene *scene = new QGraphicsScene();
    _ui->_distributionView->setScene(scene);
    _ui->_distributionView->scene()->addItem(_pitem);
    _bodies = DynamicUtil::getRigidBodies(*_dwc);

	_xaxis.resize( _bodies.size() );
	_yaxis.resize( _bodies.size() );
	_zaxis.resize( _bodies.size() );

    // load combobox
    BOOST_FOREACH(RigidBody::Ptr  body, _bodies){
        _ui->_selectObjBox->addItem( body->getBodyFrame()->getName().c_str() );
    }
    _frameRender = ownedPtr(new RenderFrame());
    _fDraw = new Drawable( _frameRender , "FrameRender0");
    _fDraw1 = new Drawable( _frameRender, "FrameRender1" );
    _fDraw2 = new Drawable( _frameRender, "FrameRender2" );
    _fDraw3 = new Drawable( _frameRender, "FrameRender3" );
    _fDraw4 = new Drawable( _frameRender, "FrameRender4" );
    _fDraw5 = new Drawable( _frameRender, "FrameRender5" );

    _xRender = ownedPtr(new RenderPoints());
    _yRender = ownedPtr(new RenderPoints());
    _zRender = ownedPtr(new RenderPoints());
    _xRender->setColor(1.0,0.0,0.0);
    _yRender->setColor(0.0,1.0,0.0);
    _zRender->setColor(0.0,0.0,1.0);
    _xDraw = new Drawable( _xRender, "PointsRenderx" );
    _yDraw = new Drawable( _yRender, "PointsRendery" );
    _zDraw = new Drawable( _zRender, "PointsRenderz" );

    _selPosePntRenderX = ownedPtr(new RenderPoints());
    _selPoseDrawX = new Drawable( _selPosePntRenderX , "PoseRenderx");
    _selPosePntRenderY = ownedPtr(new RenderPoints());
    _selPoseDrawY = new Drawable( _selPosePntRenderY, "PoseRendery" );
    _selPosePntRenderZ = ownedPtr(new RenderPoints());
    _selPoseDrawZ = new Drawable( _selPosePntRenderZ , "PoseRenderz");
    _selPosePntRenderX->setColor(1.0,0.0,0.0);
    _selPosePntRenderY->setColor(0.0,1.0,0.0);
    _selPosePntRenderZ->setColor(0.0,0.0,1.0);


    _xcRender = ownedPtr(new RenderCircles());
    _ycRender = ownedPtr(new RenderCircles());
    _zcRender = ownedPtr(new RenderCircles());
    _xcRender->setColor(1.0,0.0,0.0);
    _ycRender->setColor(0.0,1.0,0.0);
    _zcRender->setColor(0.0,0.0,1.0);
    _xcDraw = new Drawable( _xcRender , "CirclesRenderx");
    _ycDraw = new Drawable( _ycRender , "CirclesRendery");
    _zcDraw = new Drawable( _zcRender , "CirclesRenderz");

    connect(_ui->_processBtn       ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_listenForDataBtn ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_loadFromFileBtn  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_loadStartPosesBtn  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_analyzePlanarBtn ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_calcBtn ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_saveDistBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_ghostStartBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_ui->_ghostEndBtn,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_ui->_drawXBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_drawYBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_drawZBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    connect(_ui->_drawPointsBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_drawCirclesBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_drawStartPosesBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    connect(_ui->_selectObjBox   ,SIGNAL(currentIndexChanged(int)), this, SLOT(changedEvent()) );
    connect(_ui->_resultView   ,SIGNAL(itemSelectionChanged()), this, SLOT(changedEvent()) );

    _view = new GLViewRW( _ui->_glframe );
    QGridLayout* lay = new QGridLayout(_ui->_glframe);
    _ui->_glframe->setLayout(lay);
    lay->addWidget(_view);

    _view->addDrawable( _xDraw );
    _view->addDrawable( _yDraw );
    _view->addDrawable( _zDraw );
    _view->addDrawable( _fDraw );
    _view->addDrawable( _fDraw1 );
    _view->addDrawable( _fDraw2 );
    _view->addDrawable( _fDraw3 );
    _view->addDrawable( _fDraw4 );
    _view->addDrawable( _fDraw5 );

    _view->addDrawable( _selPoseDrawX );
    _view->addDrawable( _selPoseDrawY );
    _view->addDrawable( _selPoseDrawZ );

    _view->addDrawable( _xcDraw );
    _view->addDrawable( _ycDraw );
    _view->addDrawable( _zcDraw );

	_xcDraw->setVisible( false );
	_ycDraw->setVisible( false );
	_zcDraw->setVisible( false );

	_ui->_resetBtn->setEnabled(false);
	_ui->_processBtn->setEnabled(false);


    _bodies = DynamicUtil::getRigidBodies(*_dwc);
    // load combobox
    BOOST_FOREACH(RigidBody::Ptr body, _bodies){
        _ui->_planarObjectBox->addItem( body->getBodyFrame()->getName().c_str() );
    }

	//tabWidget->setTabEnabled(1,false);
}

void SupportPoseAnalyserDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _ui->_loadFromFileBtn ){
        _ui->_resetBtn->setEnabled(true);
        _ui->_processBtn->setEnabled(true);
    	//tabWidget->setTabEnabled(1,false);

    	std::string filename = openFile(_previousOpenDirectory, this);
    	if(filename.empty())
    		return;
    	try{
                _path = ownedPtr(new rw::trajectory::TimedStatePath);
    		*_path = PathLoader::loadTimedStatePath(*_wc,filename);
    	} catch(const Exception&) {
    		_path = NULL;
    		_ui->_dataLoadedLbl->setText("Load failed!");
    		return;
    	}
    	_ui->_dataLoadedLbl->setText("State data loaded!");
    	addStatePath(_path);
    } else if( obj == _ui->_loadStartPosesBtn ) {
    	std::string filename = openFile(_previousOpenDirectory, this);
    	if(filename.empty())
    		return;
    	try{
                _startPath = ownedPtr(new rw::trajectory::TimedStatePath);
                *_startPath = PathLoader::loadTimedStatePath(*_wc,filename);
    	} catch(const Exception&) {
    		_startPath = NULL;
    		_ui->_dataLoadedLbl->setText("Load start poses failed!");
    		return;
    	}
    	_ui->_dataLoadedLbl->setText("State start data loaded!");
    	addStateStartPath(_startPath);
    } else if( obj == _ui->_listenForDataBtn ) {
    	//tabWidget->setTabEnabled(1,false);
    	// create the resting pose dialog
        _ui->_resetBtn->setEnabled(true);
        _ui->_processBtn->setEnabled(true);
		std::cout << "Rest pose " << std::endl;
	    if (!_restPoseDialog ) {
	    	std::cout << "Rest pose " << std::endl;
	    	_restPoseDialog = new RestingPoseDialog(_defaultState, _dwc, _detector,  this);
	        connect(_restPoseDialog,SIGNAL(stateChanged(const rw::kinematics::State&)),this,SIGNAL(stateChanged(const rw::kinematics::State&)) );
	        connect(_restPoseDialog,SIGNAL(restingPoseEvent(const rw::kinematics::State&,const rw::kinematics::State&)),this,SLOT(addRestingPose(const rw::kinematics::State&,const rw::kinematics::State&)) );
	    }

	    _restPoseDialog->show();
	    _restPoseDialog->raise();
	    _restPoseDialog->activateWindow();

    } else if( obj == _ui->_processBtn ) {
   		process();
   		_ui->_resetBtn->setEnabled(true);
   		//tabWidget->setTabEnabled(1,true);
    } else if( obj == _ui->_resetBtn ) {
        _ui->_processBtn->setEnabled(false);
    	//tabWidget->setTabEnabled(1,false);
    	_xaxis.clear();
    	_yaxis.clear();
    	_zaxis.clear();

    	_xaxisS.clear();
    	_yaxisS.clear();
    	_zaxisS.clear();

    	_xRender->clear();
    	_yRender->clear();
    	_zRender->clear();

    	_xcRender->clear();
    	_ycRender->clear();
    	_zcRender->clear();
    } else if( obj == _ui->_analyzePlanarBtn ) {
    	std::cout << "analyzing planar stable poses of object" << std::endl;
    	// retrieve the object

    	RigidBody* selectedObj=NULL;
    	std::string selectedName = _ui->_planarObjectBox->currentText().toStdString();
    	BOOST_FOREACH(RigidBody::Ptr obj, _bodies){
    	    if( obj->getBodyFrame()->getName()==selectedName ){
    	        selectedObj = obj.get();
    	        break;
    	    }
    	}
    	if(selectedObj==NULL) return;

    	// get triangle mesh of object
    	std::vector<rw::common::Ptr<Geometry> > geoms = selectedObj->getGeometry();

    	// add it to the planar support pose analyzer
    	PlanarSupportPoseGenerator gen;
    	gen.analyze(geoms, selectedObj->getBodyFrame(), _defaultState);
    	_supportPoses[selectedObj] = gen.getSupportPoses();

        std::string locale = setlocale(LC_ALL, NULL);
//        setlocale( LC_ALL, "C" );

    	// now sample the initial object pose that will result in the individual support poses
    	std::vector<Transform3D<> > poses,misses;
        for(size_t i=0; i<_supportPoses[selectedObj].size();i++ ) {
            std::cout << i << ":" << _supportPoses[selectedObj].size() << std::endl;
            poses.clear();
            misses.clear();

            gen.calculateDistribution((int)i,poses,misses);
            _supportPoseDistributions[selectedObj].push_back(poses);
            _supportPoseDistributionsMisses[selectedObj].push_back(misses);

            /*
            // convert to format that is easy to read
            std::stringstream filename;
            filename << "c:/tmp/planar_distribution_" << i << ".txt";
            std::string filenameStr = filename.str();
            std::cout << "Openning file: " << filenameStr << std::endl;
            std::ofstream file( filenameStr.c_str() );
            TimerUtil::sleepMs(100);
            while(!file.is_open()){
                std::cout << "Openning file: " << filenameStr << std::endl;
                file.open(filenameStr.c_str());
                TimerUtil::sleepMs(100);
            }
            file << "p[0] \t p[1] \t p[2] \t rpy[0] \t rpy[1] \t rpy[2] \t eaa[0] \t eaa[1] \t eaa[2] \n";
            for(size_t j=0;j<poses.size();j++){
                Transform3D<> t = poses[j];
                EAA<> eaa(t.R());
                RPY<> rpy(t.R());
                Rotation3D<> rot = t.R();
                file << t.P()[0] << "\t" << t.P()[1] << "\t" << t.P()[2] << "\t";
                file << rpy(0) << "\t" << rpy(1) << "\t" << rpy(2) << "\t";
                file << eaa(0) << "\t" << eaa(1) << "\t" << eaa(2) << "\n";
            }
            file.close();
            */
        }
        updateResultView();
        // setlocale( LC_ALL, locale.c_str());

    } else if(obj == _ui->_calcBtn) {
        showPlanarDistribution();
    } else if(obj == _ui->_saveDistBtn){
        saveDistribution();
    } else if(obj == _ui->_ghostStartBtn){

        RigidBody* selectedObj=NULL;
        std::string selectedName = _ui->_planarObjectBox->currentText().toStdString();
        BOOST_FOREACH(RigidBody::Ptr obj, _bodies){
            if( obj->getBodyFrame()->getName()==selectedName ){
                selectedObj = obj.get();
                break;
            }
        }
        if(selectedObj==NULL) return;

        //RenderGhost *ghStart = new RenderGhost( selectedObj , _rwstudio->getWorkcellGLDrawer() );



    } else if(obj == _ui->_ghostEndBtn){

    } else  {

    }
}

namespace {
    void saveDist(std::string filename, std::vector<Transform3D<> >& poses, SupportPose &spose){
        std::cout << "Openning file: " << filename << std::endl;
        std::ofstream file( filename.c_str() );
        TimerUtil::sleepMs(100);

        file << "axis[0] \t axis[1] \t axis[2] \n";
        file << spose._rotAxes[0][0] << "\t" << spose._rotAxes[0][1] << "\t" << spose._rotAxes[0][2] << "\n";

        file << "p[0] \t p[1] \t p[2] \t rpy[0] \t rpy[1] \t rpy[2] \t eaa[0] \t eaa[1] \t eaa[2] \t";
        file << "rot(0,0) \t rot(1,0) \t rot(2,0) \t";
        file << "rot(0,1) \t rot(1,1) \t rot(2,1) \t";
        file << "rot(0,2) \t rot(1,2) \t rot(2,2) \n";
        for(size_t j=0;j<poses.size();j++){
            Transform3D<> t = poses[j];
            EAA<> eaa(t.R());
            RPY<> rpy(t.R());
            Rotation3D<> rot = t.R();
            file << t.P()[0] << "\t" << t.P()[1] << "\t" << t.P()[2] << "\t";
            file << rpy(0) << "\t" << rpy(1) << "\t" << rpy(2) << "\t";
            file << eaa(0) << "\t" << eaa(1) << "\t" << eaa(2) << "\t";
            file << rot(0,0) << "\t" << rot(1,0) << "\t" << rot(2,0) << "\t";
            file << rot(0,1) << "\t" << rot(1,1) << "\t" << rot(2,1) << "\t";
            file << rot(0,2) << "\t" << rot(1,2) << "\t" << rot(2,2) << "\n";

        }
        file.close();

    }

}

void SupportPoseAnalyserDialog::saveDistribution(){
    RigidBody *body = getSelectedBody().get();
    int poseIdx = _ui->_resultView->currentRow();

    std::vector<Transform3D<> > &poses = _supportPoseDistributions[body][poseIdx];
    std::vector<Transform3D<> > &misses = _supportPoseDistributionsMisses[body][poseIdx];

    SupportPose &spose = _supportPoses[body][poseIdx];

    std::stringstream filename, filename1;
    filename << "c:/tmp/planar_distribution_" << poseIdx << ".txt";
    saveDist(filename.str(), poses, spose);
    filename1 << "c:/tmp/planar_distribution_" << poseIdx << "_misses.txt";
    saveDist(filename1.str(), misses, spose);

}



void SupportPoseAnalyserDialog::showPlanarDistribution(){
    RigidBody* body=NULL;
    std::string selectedName = _ui->_planarObjectBox->currentText().toStdString();
    BOOST_FOREACH(RigidBody::Ptr obj, _bodies){
        if( obj->getBodyFrame()->getName()==selectedName ){
            body = obj.get();
            break;
        }
    }
    if(body==NULL) return;

    int poseIdx = _ui->_resultView->currentRow();
    std::cout << "pos1 " << poseIdx << " < " << _supportPoseDistributions[body].size() << std::endl;
    std::vector<Transform3D<> > &poses = _supportPoseDistributions[body][poseIdx];
    std::cout << "pos2";
    QImage img(640,640, QImage::Format_Mono);
    img.fill(0);

    int xIdx = Math::clamp(_ui->_xAxisBox->currentIndex(), 0, 8);
    int yIdx = Math::clamp(_ui->_yAxisBox->currentIndex(), 0, 8);
    Q q(9);

    _xaxis.clear();
    _yaxis.clear();
    _zaxis.clear();

    _xaxis.push_back( std::vector<Vector3D<> >(poses.size()) );
    _yaxis.push_back( std::vector<Vector3D<> >(poses.size()) );
    _zaxis.push_back( std::vector<Vector3D<> >(poses.size()) );

    int j=0;
    BOOST_FOREACH(const Transform3D<>& t, poses){
        // add a point to the graphics view
        EAA<> eaa(t.R());
        RPY<> rpy(t.R());
        for(int i=0;i<3;i++){
            q(i) = t.P()[i];
            q(i+3) = rpy(i);
            q(i+6) = eaa(i);
        }
        Rotation3D<> rot = t.R();
        int x = (int)Math::clamp(q(xIdx)/(50.0*Deg2Rad)*640+640/2.0, 0.0, 639.0);
        int y = (int)Math::clamp(q(yIdx)/(50.0*Deg2Rad)*640+640/2.0, 0.0, 639.0);
        img.setPixel( x, y, 1);
        RW_ASSERT( j<(int)_xaxis[0].size() );
        _xaxis[0][j] = Vector3D<>(rot(0,0),rot(1,0),rot(2,0));
        _yaxis[0][j] = Vector3D<>(rot(0,1),rot(1,1),rot(2,1));
        _zaxis[0][j] = Vector3D<>(rot(0,2),rot(1,2),rot(2,2));
        j++;
    }

    QPixmap map = QPixmap::fromImage(img);
    _pitem->setPixmap(map);
    std::cout << "pos5";
    updateRenderView();
    //_distributionView->re
}

void SupportPoseAnalyserDialog::addRestingPose(
		const rw::kinematics::State& startPose,
		const rw::kinematics::State& restPose)
{
	for(size_t j=0;j<_bodies.size();j++){
		RigidBody* body = _bodies[j].get();
		//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
		Rotation3D<> rot = body->getMovableFrame()->getTransform(restPose).R();
		_xaxis[j].push_back(Vector3D<>(rot(0,0),rot(1,0),rot(2,0)));
		_yaxis[j].push_back(Vector3D<>(rot(0,1),rot(1,1),rot(2,1)));
		_zaxis[j].push_back(Vector3D<>(rot(0,2),rot(1,2),rot(2,2)));

		rot = body->getMovableFrame()->getTransform(startPose).R();
		_xaxisS[body].push_back(Vector3D<>(rot(0,0),rot(1,0),rot(2,0)));
		_yaxisS[body].push_back(Vector3D<>(rot(0,1),rot(1,1),rot(2,1)));
		_zaxisS[body].push_back(Vector3D<>(rot(0,2),rot(1,2),rot(2,2)));
	}
	int objIdx = _ui->_selectObjBox->currentIndex();
	RigidBody *body = getSelectedBody().get();
	if(!body)
		return;
	_xRender->addPoint( _xaxis[objIdx].back() );
	_yRender->addPoint( _yaxis[objIdx].back() );
	_zRender->addPoint( _zaxis[objIdx].back() );
	updateHoughThres();
}

void SupportPoseAnalyserDialog::changedEvent(){
    QObject *obj = sender();
    if( obj == _ui->_drawXBox ){
    	_xDraw->setVisible( _ui->_drawXBox->isChecked() && _ui->_drawPointsBox->isChecked() );
    	_xcDraw->setVisible( _ui->_drawXBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    } else if( obj == _ui->_drawYBox ){
    	_yDraw->setVisible( _ui->_drawYBox->isChecked() && _ui->_drawPointsBox->isChecked());
    	_ycDraw->setVisible( _ui->_drawYBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    } else if( obj == _ui->_drawZBox ) {
    	_zDraw->setVisible( _ui->_drawZBox->isChecked() && _ui->_drawPointsBox->isChecked());
    	_zcDraw->setVisible( _ui->_drawZBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    } else if( obj == _ui->_drawPointsBox ){
    	_xDraw->setVisible( _ui->_drawXBox->isChecked() && _ui->_drawPointsBox->isChecked() );
    	_yDraw->setVisible( _ui->_drawYBox->isChecked() && _ui->_drawPointsBox->isChecked());
    	_zDraw->setVisible( _ui->_drawZBox->isChecked() && _ui->_drawPointsBox->isChecked());
    } else if( obj == _ui->_drawCirclesBox ){
    	_xcDraw->setVisible( _ui->_drawXBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    	_ycDraw->setVisible( _ui->_drawYBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    	_zcDraw->setVisible( _ui->_drawZBox->isChecked() && _ui->_drawCirclesBox->isChecked() );
    } else if( obj == _ui->_drawStartPosesBox){
    	bool drawStartPoses = _ui->_drawStartPosesBox->isChecked();
    	_selPoseDrawX->setVisible( drawStartPoses && _ui->_drawXBox->isChecked());
    	_selPoseDrawY->setVisible( drawStartPoses && _ui->_drawYBox->isChecked());
    	_selPoseDrawZ->setVisible( drawStartPoses && _ui->_drawZBox->isChecked());
    } else if( obj == _ui->_selectObjBox){
    	updateRenderView();
    	updateResultView();
    } else if( obj == _ui->_resultView ){
    	RigidBody *body = getSelectedBody().get();
    	int poseIdx = _ui->_resultView->currentRow();
    	std::cout << "Pose: " << poseIdx << std::endl;
    	// get the support pose and the default state
    	SupportPose &pose = _supportPoses[body][poseIdx];
    	State state = _wc->getDefaultState();
    	// calculate a transform that is part of the support pose
    	//Transform3D<> trans = body->getMovableFrame()->getTransform(state);
    	Transform3D<> wTb = Kinematics::worldTframe( body->getMovableFrame() , state );

    	Vector3D<> wTdir_current = wTb.R() * pose._rotAxes[0];
        Vector3D<> wTdir_target = pose._rotAxesTable[0];

    	// we want to rotate into wTdir_target
    	EAA<> eaa(wTdir_current, wTdir_target);

    	double ang = angle(wTdir_current, wTdir_target, normalize( cross(wTdir_current, wTdir_target)));//acos( dot(v,p) );
    	std::cout << "- RotAxisTable:  " <<  pose._rotAxesTable[0] << "\n"
    	          << "- RotAxisT    :  " <<  (wTb.R()*pose._rotAxes[0]) << "\n"
    	           <<"- RotAxis     :  " <<  pose._rotAxes[0] << std::endl;
    	std::cout << "- Angle       :  " << ang << wTdir_current << wTdir_target << std::endl;

    	//EAA<> eaa( normalize( cross(v,p) ) , -ang );

    	// now we add this rotation to the object, such that its rotation axis gets aligned with the supporting axis
    	wTb.R() = inverse( eaa.toRotation3D() )*wTb.R();
    	body->getMovableFrame()->setTransform(wTb,state);

    	Rotation3D<> rot10 = EAA<>(normalize(pose._rotAxes[0]),10*Deg2Rad).toRotation3D();
    	_fDraw1->setTransform(Transform3D<>(Vector3D<>(0,0,0),rot10));
        Rotation3D<> rot20 = EAA<>(normalize(pose._rotAxes[0]),20*Deg2Rad).toRotation3D();
    	_fDraw2->setTransform(Transform3D<>(Vector3D<>(0,0,0),rot20));

    	Rotation3D<> rot30 = EAA<>(normalize(pose._rotAxes[0]),0).toRotation3D();
        _fDraw3->setTransform(Transform3D<>(Vector3D<>(0,0,0),rot30));

        Rotation3D<> rot10i = EAA<>(normalize(pose._rotAxes[0]),-10*Deg2Rad).toRotation3D();
        _fDraw4->setTransform(Transform3D<>(Vector3D<>(0,0,0),rot10i));
        Rotation3D<> rot20i = EAA<>(normalize(pose._rotAxes[0]),-20*Deg2Rad).toRotation3D();
        _fDraw5->setTransform(Transform3D<>(Vector3D<>(0,0,0),rot20i));

        _fDraw1->setVisible( false );
        _fDraw2->setVisible( false );
        _fDraw3->setVisible( false );
        _fDraw4->setVisible( false );
        _fDraw5->setVisible( false );
    	// now we also need to show the starting points

    	int bodyIdx = _ui->_selectObjBox->currentIndex();
    	std::vector<int> poseIdxList = _supportToPose[std::make_pair(bodyIdx,poseIdx)];
    	std::sort(poseIdxList.begin(), poseIdxList.end());

    	std::cout << "PoseIdxList: " << poseIdxList.size() << std::endl;
    	_selPosePntRenderX->clear();
    	_selPosePntRenderY->clear();
    	_selPosePntRenderZ->clear();

    	_selPoseDrawX->setTransparency(0.3f);


    	std::vector<Vector3D<> > &xaxis = _xaxisS[body];
    	//std::vector<Vector3D<> > &yaxis = _yaxisS[body];
    	//std::vector<Vector3D<> > &zaxis = _zaxisS[body];
    	std::cout << "xaxis.size()==_xaxis.size() " << xaxis.size() << "==" << _xaxis[bodyIdx].size()<< std::endl;
    	if(xaxis.size()==_xaxis[bodyIdx].size()){
			BOOST_FOREACH(int idx, poseIdxList){
			    // visualize the EAA instead
			    if(_ui->_rpyBox->isChecked() ){
			        RPY<> eaa(_startTransforms[bodyIdx][idx].R() );
			        _selPosePntRenderZ->addPoint( Vector3D<>(eaa(0)/4,eaa(1)/4,eaa(2)/4 ) );
			    } else {
                    EAA<> eaa(_startTransforms[bodyIdx][idx].R() );
                    _selPosePntRenderZ->addPoint( Vector3D<>(eaa[0]/4,eaa[1]/4,eaa[2]/4 ) );
			    }
			    //_selPosePntRenderX->addPoint( xaxis[idx] );
				//_selPosePntRenderY->addPoint( yaxis[idx] );
				//_selPosePntRenderZ->addPoint( zaxis[idx] );
			}

			// now add everything else
			std::size_t pidx = 0;
			for(std::size_t i=0;i<_startTransforms[bodyIdx].size(); i++){
			    if( pidx<poseIdxList.size() && (int)i==poseIdxList[pidx] ){
			        pidx++;
			        continue;
			    }
			    if( _startTransforms[bodyIdx][i].P()[2]<0 )
			        continue;
			    if(_ui->_rpyBox->isChecked() ){
                    RPY<> eaa(_startTransforms[bodyIdx][i].R() );
                    _selPosePntRenderX->addPoint( Vector3D<>(eaa(0)/4,eaa(1)/4,eaa(2)/4 ) );
                } else {
                    EAA<> eaa(_startTransforms[bodyIdx][i].R() );
                    _selPosePntRenderX->addPoint( Vector3D<>(eaa[0]/4,eaa[1]/4,eaa[2]/4 ) );
                }
            }
    	}
    	// signal to update the transform
    	//showPlanarDistribution();
    	stateChanged(state);
    }
    _view->updateGL();
}


namespace {

	void mapToImage(std::vector<Vector3D<> >& points, Image& img){
		ImageUtil::reset(img,0);
		int width = img.getWidth();
		int height = img.getHeight();
		for(size_t j=0;j<points.size();j++){
			Vector3D<> &p = points[j];
			double phi = atan2(p(1),p(0)); // atan(y,x)
			double theta = atan2(sqrt( p(1)*p(1)+p(0)*p(0) ), p(2) );
			// draw the coordinate in the image
			// [-Pi,Pi] -> [0,width]
			// [0,Pi] -> [0,height]
			int x = (int)(((phi+Pi)*width )/(2*Pi));
			int y = (int)(((theta)*height )/(Pi));
			img.getImageData()[x+y*width] = (char)240;
		}
	}

	Vector3D<> fromSpherical(double phi, double theta, double r){
		double x = r * sin(theta) * cos(phi);
		double y = r * sin(theta) * sin(phi);
		double z = r * cos(theta);
		return Vector3D<>(x,y,z);
	}


#ifdef USE_OPENCV
	void test(){
		  cvNamedWindow( "My Window", 1 );
		  IplImage *img = cvCreateImage( cvSize( 640, 480 ), IPL_DEPTH_8U, 1 );
		  CvFont font;
		  double hScale = 1.0;
		  double vScale = 1.0;
		  int lineWidth = 1;
		  cvInitFont( &font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC,
		              hScale, vScale, 0, lineWidth );
		  cvPutText( img, "Hello World!", cvPoint( 200, 400 ), &font,
		             cvScalar( 255, 255, 0 ) );
		  cvShowImage( "My Window", img );
		  cvWaitKey();
	}

	void mapToIplImage(std::vector<Vector3D<> >& points, IplImage* img){
		uchar *data      = (uchar *)img->imageData;
		int width = img->width;
		int height = img->height;
		int widthStep = img->widthStep;
		for(size_t j=0;j<points.size();j++){
			Vector3D<> &p = points[j];
			double phi = atan2(p(1),p(0)); // atan(y,x)
			double theta = atan2(sqrt( p(1)*p(1)+p(0)*p(0) ), p(2) );
			// draw the coordinate in the image
			// [-Pi,Pi] -> [0,width]
			// [0,Pi] -> [0,height]
			int x = (int)(((phi+Pi)*width )/(2*Pi));
			int y = (int)(((theta)*height )/(Pi));
			data[y*widthStep+x] = 255;
			//data[x+y*width] = 240;
		}
	}

#endif

	Vector2D<> toVector2D(const Vector3D<>& v){
		return Vector2D<>(v(0),v(1));
	}



    struct Line {
    	Line(double r, double t):rho(r),theta(t){};
    	void print(){std::cout << "- Line: [" << rho << "," << theta << "]" << std::endl;};
    	double rho,theta;
    };

    typedef std::vector<Line> LineGroup;
	std::vector< CircleModel > refitCircles(
			std::vector< CircleModel >& circles,
			std::vector<Vector3D<> >& points,
			const double epsilon)
	{
		typedef std::vector<Vector3D<> > PointList;
		std::vector<CircleModel> circResult;
		std::vector<size_t> pointsInCircle;
		//std::vector<PointList> circPoints(circles.size());
		// first find all points that are close to each circle
		for(size_t i=0;i<circles.size();i++){
			const CircleModel& circle = circles[i];
			PointList circPoints;
			for(size_t j=0;j<points.size();j++){
				if(circle.isClose(points[j], epsilon)){
					circPoints.push_back(points[j]);
				}
			}
			// ignore the circle if the number of points is very low
			if(circPoints.size()<4+points.size()/50)
				continue;

			// refit the circle using the complete dataset
			CircleModel ncirc = CircleModel::fitTo( circPoints );

			// Last we test if any of the newly created circles are close enough to be equal
			bool merged = false;
			for(size_t j=0;j<circResult.size(); j++){
				if( ncirc.isClose(circResult[j], 0.1) ){
					merged = true;
					// overwrite existing circle if new cricle has more points
					if(circPoints.size()<pointsInCircle[j])
						break;

					circResult[j] = ncirc;
					pointsInCircle[j] = circPoints.size();
					break;
				}
			}
			if(merged)
				continue;

			// if the circle is unique then save it
			pointsInCircle.push_back(circPoints.size());
			circResult.push_back(ncirc);
			// test if
			//circle.print();
			//std::cout << "Nr of points in circle: " << circPoints.size() << std::endl;
			//ncirc.print();
		}
		return circResult;
	}


#ifdef USE_OPENCV
	std::vector< CircleModel > extractCircles(std::vector<Vector3D<> >& points,
				int houghThres){

		// ************************************************
		// first we convert the points to spherical coordinates and map them
		// onto a 2d image. In this mapped image lines map back to circles. So
		// we find all lines in order to find circles using Hough
		const int width = 1000, height=1000;
		IplImage* img = cvCreateImage( cvSize(width,height), IPL_DEPTH_8U, 1);
		mapToIplImage(points, img);

        IplImage* dst = cvCreateImage( cvSize(width,height), IPL_DEPTH_8U, 1);
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* lines = 0;

        cvCanny( img, dst, 50, 200, 3 );

        lines = cvHoughLines2( dst, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, houghThres, 0, 0 );
        //log().debug() << "- Number of unfiltered lines found: " << lines->total << std::endl;

        std::vector<LineGroup> lineGroups;

        // ************************************************
        // We collect lines that are close into groups
        for(int i = 0; i < lines->total; i++ ){
        	const double epsRho = sqrt( Math::sqr(width)+Math::sqr(height) )/80;
        	const double epsTheta = (CV_PI/180)*3;
        	float* cvline = (float*)cvGetSeqElem(lines,i);
        	Line line(cvline[0],cvline[1]);
        	// line.print();
        	bool inGroup=false;
            for(size_t j=0; j<lineGroups.size() && !inGroup; j++) {
            	LineGroup &group = lineGroups[j];
            	BOOST_FOREACH(Line& gline, group){
            		if(fabs(gline.rho-line.rho)<epsRho &&
            		   fabs(gline.theta-line.theta)<epsTheta){
						inGroup=true;
						break;
            		}
            	}
            	if(inGroup){
            		group.push_back(line);
            	}
            }
            if(!inGroup){
            	LineGroup nGroup;
            	nGroup.push_back(line);
            	lineGroups.push_back(nGroup);
            }
        }
        std::cout << "- Number of line groups: " << lineGroups.size() << std::endl;

        // ************************************************
        // then for each group we calculate the line that is the average of the group
        std::vector<Line> avgLines;
        BOOST_FOREACH(LineGroup& group, lineGroups){
        	Line avgline(0,0);
        	//std::cout << "Group: " << group.size() << std::endl;
        	BOOST_FOREACH(Line& gline, group){
        		//gline.print();
        		avgline.rho += gline.rho;
        		avgline.theta += gline.theta;
        	}
    		avgline.rho = avgline.rho/group.size();
    		avgline.theta = avgline.theta/group.size();
    		//std::cout << "Average: "; avgline.print();
    		avgLines.push_back(avgline);
        }
        //log().debug() << "- Number of lines after filtering: " << avgLines.size() << std::endl;

        // ************************************************
        // now for each average line we calculate the circle.
		std::vector<CircleModel> circles;
        for(size_t i = 0; i < avgLines.size(); i++ ) {
        	Line &line = avgLines[i];
            //line.print();
            const double epsilon = 0.001;
            double a = cos(line.theta), b = sin(line.theta);

            Vector2D<> sp1(0,0), sp2(0,0), sp3(0,0);
            if( fabs(b)<epsilon ){ // vertical line
            	sp1[0] = sp2[0] = line.rho;
            	sp1[1] = 0;
            	sp2[1] = height;
            } else if( fabs(a)<epsilon ){ // horizontal
            	sp1[1] = sp2[1] = line.rho;
            	sp1[0] = 0;
            	sp2[0] = width;
            } else {
            	sp1[0] = 0;
            	sp1[1] = line.rho/b;
            	sp2[0] = line.rho/a;
            	sp2[1] = 0;
            	if(sp2[0]<0){
            		double t = width/(sp1[0]-sp2[0]);
            		sp2 = sp1+(sp1-sp2)*t;
            	} else if(sp2[0]>width){
            		double t = width/(sp2[0]-sp1[0]);
            		sp2 = sp1+(sp2-sp1)*t;
            	}
            }
			// [0,width] -> [-Pi,Pi]
			// [0,height]-> [  0,Pi]
            double scalex=2.0*Pi/width, scaley=Pi/height;
            Vector2D<> offset(-Pi,0);
            /*
            line.print();
            std::cout << "non Scaled points: " << std::endl
				      << "-- " <<  sp1 << std::endl
				      << "-- " <<  sp2 << std::endl
				      << "-- " <<  sp3 << std::endl;
            std::cout << "scalex: " << scalex << std::endl;
            */
            sp1 = Vector2D<>(scalex*sp1[0],scaley*sp1[1]) + offset;
            sp3 = Vector2D<>(scalex*sp2[0],scaley*sp2[1]) + offset;

            Vector2D<> step = (sp3-sp1)/4.0;
            sp2 = sp1+step;
            sp3 = sp2+step;

            Vector3D<> p1 = fromSpherical( sp1[0], sp1[1], 1 );
            Vector3D<> p2 = fromSpherical( sp2[0], sp2[1], 1 );
            Vector3D<> p3 = fromSpherical( sp3[0], sp3[1], 1 );

            CircleModel circ( p1, p2, p3 );

            // make sure that circles that are too small are sorted away.
            // since these are actually groupings of points...
            if(circ._r<0.01)
            	continue;

            // now test if any of the circles are close
/*
            std::cout << "Scaled points: " << std::endl
				      << "-- " <<  sp1 << std::endl
				      << "-- " <<  sp2 << std::endl
				      << "-- " <<  sp3 << std::endl;

            std::cout << "points 3d: " << std::endl
				      << "-- " <<  p1 << std::endl
				      << "-- " <<  p2 << std::endl
				      << "-- " <<  p3 << std::endl;
*/
            circles.push_back( circ );
            circles.back().print();
        }
        return circles;
	}

	#endif

	struct CircleIdx {
		CircleIdx():
			xidx(-1),yidx(-1),zidx(-1)
		{
		};

		CircleIdx(int x,int y, int z):
			xidx(x),yidx(y),zidx(z)
		{
		};

		friend bool operator==(const CircleIdx& A, const CircleIdx& B){
			return (A.xidx == B.xidx) && (A.yidx == B.yidx) && (A.zidx == B.zidx);
		}
		int xidx,yidx,zidx;
		Rotation3D<> invRot;
	};


	class Vector3DSet{
	public:
		Vector3DSet(double epsilon):_eps(epsilon)
		{
		}

		void add(const Vector3D<>& v, const Vector3D<>& n, int idx){
			int i=0;
			BOOST_FOREACH(const Vector3D<>& sv, _set){
				if(MetricUtil::dist2(v,sv)<_eps){
					_setStats[i]++;
					_poseIdx[i].push_back(idx);
					return;
				}
				i++;
			}
			_set.push_back(v);
			_setWorld.push_back(n);
			_setStats.push_back(1);
			_poseIdx.push_back(std::vector<int>(1,idx));
		}

		void clear(){
			_set.clear();
			_setWorld.clear();
			_setStats.clear();
		};

		std::vector<std::vector<int > > getPoseIdx(){return _poseIdx;};
		std::vector<Vector3D<> >& getList(){return _set;};
		std::vector<Vector3D<> >& getWorldList(){return _setWorld;};

		std::vector<int>& getStatList(){return _setStats;};
	private:
		std::vector<Vector3D<> > _set;
		std::vector<Vector3D<> > _setWorld;
		std::vector<int > _setStats;
		std::vector<std::vector<int > > _poseIdx;
		double _eps;
	};
}

void SupportPoseAnalyserDialog::addStateStartPath(rw::trajectory::TimedStatePath::Ptr path){
	_xaxisS.clear();
	_yaxisS.clear();
	_zaxisS.clear();
	_startTransforms.clear();

    _startTransforms.resize(_bodies.size(),std::vector<Transform3D<> >(path->size()) );

	for(size_t j=0;j<_bodies.size();j++){
		std::vector<Vector3D<> > &xaxis = _xaxisS[_bodies[j].get() ];
		std::vector<Vector3D<> > &yaxis = _yaxisS[_bodies[j].get() ];
		std::vector<Vector3D<> > &zaxis = _zaxisS[_bodies[j].get() ];
		xaxis.resize(path->size());
		yaxis.resize(path->size());
		zaxis.resize(path->size());
		for(size_t i=0; i<path->size();i++){
			const State &state = (*path)[i].getValue();
			RigidBody *body = _bodies[j].get();

			//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
			Transform3D<> t3d = body->getMovableFrame()->getTransform(state);
			Rotation3D<> rot = t3d.R();
			xaxis[i] = Vector3D<>(rot(0,0),rot(1,0),rot(2,0));
			yaxis[i] = Vector3D<>(rot(0,1),rot(1,1),rot(2,1));
			zaxis[i] = Vector3D<>(rot(0,2),rot(1,2),rot(2,2));
			_startTransforms[j][i] = t3d;
		}
	}

}

void SupportPoseAnalyserDialog::addStatePath(rw::trajectory::TimedStatePath::Ptr path){
	if( _bodies.size()==0 )
		return;

	_xaxis.clear();
	_yaxis.clear();
	_zaxis.clear();
	_endTransforms.clear();

	_xaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );
	_yaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );
	_zaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );
	_endTransforms.resize(_bodies.size(),std::vector<Transform3D<> >(path->size()) );

	// copy all poses of all body/state sets into an array
	std::cout << "Size of path is: " << path->size() << std::endl;
	const State defState = (*path)[0].getValue();



	for(size_t i=0; i<path->size();i++){
		const State &state = (*path)[i].getValue();
		for(size_t j=0;j<_bodies.size();j++){
			RigidBody *body = _bodies[j].get();

			//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
			Transform3D<> t3d = body->getMovableFrame()->getTransform(state);
			Rotation3D<> rot = t3d.R();

			_xaxis[j][i] = Vector3D<>(rot(0,0),rot(1,0),rot(2,0));
			_yaxis[j][i] = Vector3D<>(rot(0,1),rot(1,1),rot(2,1));
			_zaxis[j][i] = Vector3D<>(rot(0,2),rot(1,2),rot(2,2));
			_endTransforms[j][i] = t3d;
		}
	}
	updateHoughThres();
	updateRenderView();
}


namespace {

    rw::common::Ptr< std::map<int,std::vector<int> > > calculateRegions(const std::vector<Transform3D<> >& data, double dist, double angle){
        std::cout << "Dist : " << dist << std::endl;
        std::cout << "Angle: " << angle*Rad2Deg << "deg" << std::endl;
        // now build a kdtree with all end configurations
        typedef boost::tuple<int,int> KDTreeValue; // (transform index, region index)
        std::vector<KDTreeQ<KDTreeValue>::KDNode> nodes;
        for(size_t i=0;i<data.size();i++){
            Transform3D<> k = data[i];
            if(k.P()[2]<-2)
                continue;
            EAA<> r( k.R());
            //Vector3D<> r = k.R()*Vector3D<>::z();
            Q key(6, k.P()[0],k.P()[1],k.P()[2], r[0],r[1],r[2]);
            nodes.push_back(KDTreeQ<KDTreeValue>::KDNode(key, KDTreeValue((int)i, -1)));
        }

        std::cout << "Nodes created, building tree.. " << std::endl;
        KDTreeQ<KDTreeValue>* nntree = KDTreeQ<KDTreeValue>::buildTree(nodes);
        // todo estimate the average distance between neighbors
        std::cout << "Tree build, finding regions" << std::endl;
        std::map<int, bool > regions;
        int freeRegion = 0;
        std::list<const KDTreeQ<KDTreeValue>::KDNode*> result;
        Q diff(6, dist, dist, dist, angle, angle, angle);
        // find neighbors and connect them
        BOOST_FOREACH(KDTreeQ<KDTreeValue>::KDNode &n, nodes){
            KDTreeValue &val = n.value;
            // check if the node is allready part of a region
            if(val.get<1>() >=0)
                continue;

            result.clear();
            nntree->nnSearchRect(n.key-diff, n.key+diff, result);
            int currentIndex = -1;
            // first see if any has an id
            BOOST_FOREACH(const KDTreeQ<KDTreeValue>::KDNode* nn, result){
                KDTreeValue nnval = nn->value;
                if(nnval.get<1>() >=0){
                    currentIndex = nnval.get<1>();
                    break;
                }
            }
            if( currentIndex<0 ){
                currentIndex=freeRegion;
                //std::cout << "Adding " << freeRegion << std::endl;
                regions[freeRegion] = true;
                freeRegion++;
            }
            val.get<1>() = currentIndex;
            BOOST_FOREACH(const KDTreeQ<KDTreeValue>::KDNode* nn, result){
                KDTreeValue nnval = nn->value;
                if(nnval.get<1>() >=0 && nnval.get<1>()!=currentIndex){

                    //std::cout << "Merging regions " << currentIndex << "<--" << nnval.get<1>() << std::endl;

                    // merge all previously defined nnval.get<1>() into freeRegion
                    regions[nnval.get<1>()] = false;
                    BOOST_FOREACH(KDTreeQ<KDTreeValue>::KDNode &npro, nodes){
                        KDTreeValue &npval = npro.value;
                        // check if the node is allready part of a region
                        if(npval.get<1>() == nnval.get<1>())
                            npval.get<1>()=currentIndex;
                    }
                }
            }

        }

        // now print region information
        std::vector<int> validRegions;
        typedef std::map<int,bool>::value_type mapType;
        BOOST_FOREACH(mapType val , regions){
            if(val.second==true){
                validRegions.push_back(val.first);
            }
        }


        std::cout << "Nr of detected regions: " << validRegions.size() << std::endl;
        std::map<int,std::vector<int> >* statMap = new std::map<int,std::vector<int> >();

        BOOST_FOREACH(KDTreeQ<KDTreeValue>::KDNode &n, nodes){
            KDTreeValue &val = n.value;
            // check if the node is allready part of a region
            (*statMap)[val.get<1>()].push_back( val.get<0>() );
        }
        std::cout << "found regions" << std::endl;
        return ownedPtr( statMap );
    }


}


void SupportPoseAnalyserDialog::process(){
	// first we need to figure out what frames that are of interest
	if( _bodies.size()==0 )
		return;

	_supportPoseDistributions.clear();

    //int houghThres = _thresholdSpin->value();
    //double epsilon = _epsilonSpin->value();

    double dist = _ui->_distSpin->value();
    double angle = _ui->_angleSpin->value() * Deg2Rad;

    // map the points on the sphere to a plane of spherical coordinates
    for(size_t j=0;j<_bodies.size();j++){
        //const Transform3D<> wTp = Kinematics::worldTframe(_bodies[j]->getMovableFrame()->getParent(), _defaultState);
        const Transform3D<> wTb = Kinematics::worldTframe(_bodies[j]->getMovableFrame(), _defaultState);

        // first we compute regions which should work as stable poses
        Ptr< std::map<int, std::vector<int> > > regions = calculateRegions(_endTransforms[j], dist, angle);

        std::vector<SupportPose> &sposes = _supportPoses[_bodies[j].get()];
        sposes.clear();

        typedef std::map<int,std::vector<int> >::value_type mapType2;
        BOOST_FOREACH(mapType2 val, *regions){
            if(val.second.size() > 4)
                Log::infoLog() << "Region stat: " << val.first << ":" << val.second.size() ;

            Vector3D<> dir(0,0,0);
            double angle = 0;
            int count = 0;
            std::vector<Transform3D<> > transformations;
            BOOST_FOREACH(int idx, val.second){
                count ++;
                std::cout << idx << std::endl;
                RW_ASSERT(_startTransforms.size()>j);
                RW_ASSERT(_endTransforms.size()>j);
                RW_ASSERT((int)_startTransforms[j].size()>idx);
                RW_ASSERT((int)_endTransforms[j].size()>idx);
                Transform3D<> s = _startTransforms[j][ idx ];
                EAA<> sr( s.R() );
                //Vector3D<> srz = s.R()*Vector3D<>::z();
                Transform3D<> e = _endTransforms[j][ idx ];
                EAA<> er( e.R() );
                Vector3D<> erz = e.R()*Vector3D<>::z();

                dir += erz;
                //dir += er.axis();
                angle += er.angle();

                Transform3D<> aTb = inverse( s )*e;
                //Vector3D<> abp = aTb.P();
                EAA<> abe(aTb.R() );
                RPY<> abr(aTb.R() );
                //Vector3D<> aby = aTb.R()*Vector3D<>::z();
                transformations.push_back( s );
            }

            //std::cout << "calculate stable poses " << std::endl;
            double probability = (count*100.0)/(double)_endTransforms[j].size();
            count = 0;
            if(val.second.size() > 4){
                // add the stable pose
                sposes.push_back( SupportPose(1, probability) );
                sposes.back()._rotAxesTable[0] = normalize(dir);
                sposes.back()._rotAxes[0] = inverse(wTb).R() * normalize(dir);

                _supportPoseDistributions[_bodies[j].get()].push_back( transformations );

                _supportToPose[std::make_pair((int)j,(int)(sposes.size()-1))] = val.second;
            }
        }




    }




#ifdef USE_OPENCV

	int houghThres = _thresholdSpin->value();
	double epsilon = _epsilonSpin->value();

	// map the points on the sphere to a plane of spherical coordinates
	for(size_t j=0;j<_bodies.size();j++){
		const Transform3D<> wTp = Kinematics::worldTframe(_bodies[j]->getMovableFrame()->getParent(), _defaultState);
		const Transform3D<> wTb = Kinematics::worldTframe(_bodies[j]->getMovableFrame(), _defaultState);

		std::vector<CircleModel> circlesXTmp = extractCircles(_xaxis[j],houghThres);
		std::vector<CircleModel> circlesYTmp = extractCircles(_yaxis[j],houghThres);
		std::vector<CircleModel> circlesZTmp = extractCircles(_zaxis[j],houghThres);

		// TODO: Before using the circles filter them using the point sets.
		// For each circle we locate the points that are close to it.
		// Then we refit a new circle using all the points and replace the old circle
		std::vector<CircleModel> circlesX = refitCircles(circlesXTmp, _xaxis[j], epsilon);
		std::vector<CircleModel> circlesY = refitCircles(circlesYTmp, _yaxis[j], epsilon);
		std::vector<CircleModel> circlesZ = refitCircles(circlesZTmp, _zaxis[j], epsilon);
		std::cout << "Fitted X: " << circlesXTmp.size() << " -> " << circlesX.size()<< std::endl;
		std::cout << "Fitted Y: " << circlesYTmp.size() << " -> " << circlesY.size()<< std::endl;
		std::cout << "Fitted Z: " << circlesZTmp.size() << " -> " << circlesZ.size()<< std::endl;

		_xcircBodyMap[_bodies[j]] = circlesX;
		_ycircBodyMap[_bodies[j]] = circlesY;
		_zcircBodyMap[_bodies[j]] = circlesZ;

		int c0=0,c1=0,c2=0,c3=0;

		std::vector<CircleIdx> circlePoses;
		Vector3DSet invAxesSet(0.1);
		// the circles that are returned should be analysed such that support poses
		// can be identified.
		// A the normal to a circle defines a rotation axis (possibly two) of a
		// support pose.
		//const double epsilon = 0.0001;
		for(size_t i=0;i<_xaxis[j].size();i++){
			int xcirc = -1, ycirc = -1, zcirc = -1;
			for(size_t f=0;f<circlesX.size();f++){
				if(circlesX[f].isClose((_xaxis[j])[i],epsilon) ){
					xcirc = f;
					break;
				}
			}
			for(size_t f=0;f<circlesY.size();f++){
				if(circlesY[f].isClose((_yaxis[j])[i],epsilon) ){
					ycirc = f;
					break;
				}
			}
			for(size_t f=0;f<circlesZ.size();f++){
				if(circlesZ[f].isClose((_zaxis[j])[i],epsilon) ){
					zcirc = f;

					break;
				}
			}
			Rotation3D<> rot(_xaxis[j][i],_yaxis[j][i],_zaxis[j][i]);
			Rotation3D<> invRot = inverse(rot);

			CircleIdx circlePose(xcirc,ycirc,xcirc);
			circlePose.invRot = invRot;
			// test if we have one, two or three circles for this point
			if(xcirc<0 && ycirc<0 && zcirc<0){
				// this special case is when the rotation does not belong to any support pose
				//std::cout << _xaxis[j][i] << _xaxis[j][i] << _xaxis[j][i] << std::endl;
				c0++;
			} else if( xcirc<0 && ycirc<0 ) {
				c1++;
			} else if( xcirc<0 && zcirc<0 ) {
				c1++;
			} else if( ycirc<0 && zcirc<0 ) {
				c1++;
			} else if( xcirc<0 ) {
				c2++;
				CircleModel circ = circlesY[ycirc];
				// transform the circle normal to frame
				Vector3D<> rotAxis = invRot*circ._n;
				//std::cout << "x: " << _xaxis[j][i] << std::endl;
 				//std::cout << "n: " << rotAxis << std::endl;
				invAxesSet.add( rotAxis, circ._n, i);
			} else if( zcirc<0 || ycirc<0 ) {
				c2++;
				CircleModel circ = circlesX[xcirc];
				//if( ycirc>=0 && )
				// transform the circle normal to frame
				Vector3D<> rotAxis = invRot*circ._n;
				invAxesSet.add( rotAxis,circ._n,i );
			} else {
				CircleModel circ = circlesX[xcirc];
				if(circ._r<circlesY[ycirc]._r)
					circ = circlesY[ycirc];
				if(circ._r<circlesZ[zcirc]._r)
					circ = circlesZ[zcirc];

				// all points lie on a circle
				c3++;
				// The support pose has a normal

				Vector3D<> rotAxis = invRot*circ._n;
				invAxesSet.add( rotAxis,circ._n,i );

				// transform the circle normal to frame
				//std::cout << "Circ: ntrnas: " << rotAxis << std::endl;

			}
			bool hasPose = false;
			BOOST_FOREACH(CircleIdx &circleIdx, circlePoses){
				if(circlePose==circleIdx){
					hasPose = true;
					break;
				}
			}
			if(!hasPose)
				circlePoses.push_back(circlePose);
		}

		//std::cout << "c0: " << c0 << std::endl
		//		  << "c1: " << c1 << std::endl
		//		  << "c2: " << c2 << std::endl
		//		  << "c3: " << c3 << std::endl;
		c0=0;c1=0;c2=0;c3=0;
		//std::cout << "Nr of axes in set        : " << invAxesSet.getList().size() << std::endl;

		// Now we create the support poses
		std::vector<Vector3D<> > &iaxes = invAxesSet.getList();
		std::vector<int > &istats = invAxesSet.getStatList();
		std::vector<SupportPose> &sposes = _supportPoses[_bodies[j]];
		sposes.clear();
		for(size_t i=0; i<iaxes.size();i++){
			const Vector3D<>& rotAxis = iaxes[i];
			const Vector3D<>& rotAxisWorld = invAxesSet.getWorldList()[i];
			double probability = istats[i]*100.0/(double)_xaxis[j].size();

			// TODO: generate a quality value
			sposes.push_back(SupportPose(1, probability));
			sposes.back()._rotAxesTable[0] = rotAxisWorld;
			sposes.back()._rotAxes[0] = rotAxis;

			std::cout << "invAxesSet.getPoseIdx()[i] size: " << invAxesSet.getPoseIdx()[i].size() << std::endl;
			_supportToPose[std::make_pair(j,sposes.size()-1)] = invAxesSet.getPoseIdx()[i];
			std::cout << rotAxis << " wTb: " << sposes.back()._rotAxesTable[0] << std::endl;
		}
		//std::cout << "Nr of unique circle poses: " << circlePoses.size() << std::endl;

	}
#endif

	updateRenderView();
	updateResultView();

	_view->updateGL();
}

void SupportPoseAnalyserDialog::updateResultView(){
	RigidBody *body = getSelectedBody().get();
	if(!body)
		return;
	_ui->_resultView->clear();

	int i=0;
	BOOST_FOREACH(const SupportPose &pose, _supportPoses[body]){
		std::stringstream str;
		str << "(" << i << ") [ " << pose._degree << " , " << pose._probability << " ] ";

		_ui->_resultView->addItem( str.str().c_str() );
		i++;
	}
}

void SupportPoseAnalyserDialog::updateRenderView(){
	int objIdx = _ui->_selectObjBox->currentIndex();
	RigidBody *body = getSelectedBody().get();
	if(!body)
		return;

	_xRender->clear();
	_yRender->clear();
	_zRender->clear();

	_xcRender->clear();
	_ycRender->clear();
	_zcRender->clear();


        _xRender->addPoints( _xaxis[objIdx] );
        _yRender->addPoints( _yaxis[objIdx] );
        _zRender->addPoints( _zaxis[objIdx] );

        _xcRender->addCircles( _xcircBodyMap[body] );
        _ycRender->addCircles( _ycircBodyMap[body] );
        _zcRender->addCircles( _zcircBodyMap[body] );
}
