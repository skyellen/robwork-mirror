#include "SupportPoseAnalyserDialog.hpp"

#include <iostream>

#include <boost/foreach.hpp>

#include "RWSimGuiConfig.hpp"

#ifdef USE_OPENCV
#include <cv.h>
#include <highgui.h>
#endif

#include <rw/math/RPY.hpp>
#include <rw/math/Math.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Kinematics.hpp>

#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/DynamicUtil.hpp>

#include <rwsim/simulator/PhysicsEngineFactory.hpp>

#include <rw/common/TimerUtil.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/proximity/Proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/drawable/Drawable.hpp>
#include <rwsim/util/PointRANSACFitting.hpp>
#include <rwsim/util/PlaneModel.hpp>
#include <rwsim/util/DistModel.hpp>

#include <rw/sensor/Image.hpp>
#include <rw/sensor/ImageUtil.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Line2D.hpp>

#include <rwsim/util/HughLineExtractor.hpp>
#include <rwsim/util/CircleModel.hpp>

using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim::util;
using namespace rwsim::drawable;

using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::common;
using namespace rw::proximity;
using namespace rw::loaders;
using namespace rw::trajectory;
using namespace rwlibs::drawable;
using namespace rw::sensor;

#define RW_DEBUGS( str ) std::cout << str  << std::endl;

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

SupportPoseAnalyserDialog::SupportPoseAnalyserDialog(const rw::kinematics::State& state,
                                    DynamicWorkcell *dwc,
                                    rw::proximity::CollisionDetector *detector,
                                    QWidget *parent):
    QDialog(parent),
    _defaultState(state),
    _wc(dwc->getWorkcell() ),
    _dwc(dwc),
    _detector(detector),
    _restPoseDialog(NULL)
{
    setupUi(this);

    _bodies = DynamicUtil::getRigidBodies(*_dwc);

	_xaxis.resize( _bodies.size() );
	_yaxis.resize( _bodies.size() );
	_zaxis.resize( _bodies.size() );

    // load combobox
    BOOST_FOREACH(RigidBody* body, _bodies){
    	_selectObjBox->addItem( body->getBodyFrame().getName().c_str() );
    }

    _xRender = ownedPtr(new RenderPoints());
    _yRender = ownedPtr(new RenderPoints());
    _zRender = ownedPtr(new RenderPoints());
    _xRender->setColor(1.0,0.0,0.0);
    _yRender->setColor(0.0,1.0,0.0);
    _zRender->setColor(0.0,0.0,1.0);
    _xDraw = new Drawable( _xRender );
    _yDraw = new Drawable( _yRender );
    _zDraw = new Drawable( _zRender );

    _selPosePntRenderX = ownedPtr(new RenderPoints());
    _selPoseDrawX = new Drawable( _selPosePntRenderX );
    _selPosePntRenderY = ownedPtr(new RenderPoints());
    _selPoseDrawY = new Drawable( _selPosePntRenderY );
    _selPosePntRenderZ = ownedPtr(new RenderPoints());
    _selPoseDrawZ = new Drawable( _selPosePntRenderZ );
    _selPosePntRenderX->setColor(1.0,0.0,0.0);
    _selPosePntRenderY->setColor(0.0,1.0,0.0);
    _selPosePntRenderZ->setColor(0.0,0.0,1.0);


    _xcRender = ownedPtr(new RenderCircles());
    _ycRender = ownedPtr(new RenderCircles());
    _zcRender = ownedPtr(new RenderCircles());
    _xcRender->setColor(1.0,0.0,0.0);
    _ycRender->setColor(0.0,1.0,0.0);
    _zcRender->setColor(0.0,0.0,1.0);
    _xcDraw = new Drawable( _xcRender );
    _ycDraw = new Drawable( _ycRender );
    _zcDraw = new Drawable( _zcRender );

    connect(_processBtn       ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_resetBtn    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_listenForDataBtn ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_loadFromFileBtn  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_loadStartPosesBtn  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(_drawXBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_drawYBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_drawZBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    connect(_drawPointsBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_drawCirclesBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );
    connect(_drawStartPosesBox  ,SIGNAL(stateChanged(int)), this, SLOT(changedEvent()) );

    connect(_selectObjBox   ,SIGNAL(currentIndexChanged(int)), this, SLOT(changedEvent()) );
    connect(_resultView   ,SIGNAL(itemSelectionChanged()), this, SLOT(changedEvent()) );

    _view = new GLViewRW( _glframe );
    QGridLayout* lay = new QGridLayout(_glframe);
    _glframe->setLayout(lay);
    lay->addWidget(_view);

    _view->addDrawable( _xDraw );
    _view->addDrawable( _yDraw );
    _view->addDrawable( _zDraw );

    _view->addDrawable( _selPoseDrawX );
    _view->addDrawable( _selPoseDrawY );
    _view->addDrawable( _selPoseDrawZ );

    _view->addDrawable( _xcDraw );
    _view->addDrawable( _ycDraw );
    _view->addDrawable( _zcDraw );

	_xcDraw->setEnabled( false );
	_ycDraw->setEnabled( false );
	_zcDraw->setEnabled( false );

	_resetBtn->setEnabled(false);
	_processBtn->setEnabled(false);
	//tabWidget->setTabEnabled(1,false);
}

void SupportPoseAnalyserDialog::btnPressed(){
    QObject *obj = sender();
    if( obj == _loadFromFileBtn ){
    	_resetBtn->setEnabled(true);
    	_processBtn->setEnabled(true);
    	//tabWidget->setTabEnabled(1,false);

    	std::string filename = openFile(_previousOpenDirectory, this);
    	if(filename.empty())
    		return;
    	try{
    		_path = PathLoader::loadTimedStatePath(*_wc,filename).release();
    	} catch(const Exception& exp) {
    		_path = NULL;
    		_dataLoadedLbl->setText("Load failed!");
    		return;
    	}
    	_dataLoadedLbl->setText("State data loaded!");
    	addStatePath(_path);
    } else if( obj == _loadStartPosesBtn ) {
    	std::string filename = openFile(_previousOpenDirectory, this);
    	if(filename.empty())
    		return;
    	try{
    		_startPath = PathLoader::loadTimedStatePath(*_wc,filename).release();
    	} catch(const Exception& exp) {
    		_startPath = NULL;
    		_dataLoadedLbl->setText("Load start poses failed!");
    		return;
    	}
    	_dataLoadedLbl->setText("State start data loaded!");
    	addStateStartPath(_startPath);
    } else if( obj == _listenForDataBtn ) {
    	//tabWidget->setTabEnabled(1,false);
    	// create the resting pose dialog
    	_resetBtn->setEnabled(true);
    	_processBtn->setEnabled(true);
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

    } else if( obj == _processBtn ) {
   		process();
   		_resetBtn->setEnabled(true);
   		//tabWidget->setTabEnabled(1,true);
    } else if( obj == _resetBtn ) {
    	_processBtn->setEnabled(false);
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

    } else  {

    }
}

void SupportPoseAnalyserDialog::addRestingPose(
		const rw::kinematics::State& startPose,
		const rw::kinematics::State& restPose)
{
	for(size_t j=0;j<_bodies.size();j++){
		RigidBody *body = _bodies[j];
		//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
		Rotation3D<> rot = body->getMovableFrame().getTransform(restPose).R();
		_xaxis[j].push_back(Vector3D<>(rot(0,0),rot(1,0),rot(2,0)));
		_yaxis[j].push_back(Vector3D<>(rot(0,1),rot(1,1),rot(2,1)));
		_zaxis[j].push_back(Vector3D<>(rot(0,2),rot(1,2),rot(2,2)));

		rot = body->getMovableFrame().getTransform(startPose).R();
		_xaxisS[body].push_back(Vector3D<>(rot(0,0),rot(1,0),rot(2,0)));
		_yaxisS[body].push_back(Vector3D<>(rot(0,1),rot(1,1),rot(2,1)));
		_zaxisS[body].push_back(Vector3D<>(rot(0,2),rot(1,2),rot(2,2)));
	}
	int objIdx = _selectObjBox->currentIndex();
	RigidBody *body = getSelectedBody();
	if(!body)
		return;
	_xRender->addPoint( _xaxis[objIdx].back() );
	_yRender->addPoint( _yaxis[objIdx].back() );
	_zRender->addPoint( _zaxis[objIdx].back() );
	updateHoughThres();
}

void SupportPoseAnalyserDialog::changedEvent(){
    QObject *obj = sender();
    if( obj == _drawXBox ){
    	_xDraw->setEnabled( _drawXBox->isChecked() && _drawPointsBox->isChecked() );
    	_xcDraw->setEnabled( _drawXBox->isChecked() && _drawCirclesBox->isChecked() );
    } else if( obj == _drawYBox ){
    	_yDraw->setEnabled( _drawYBox->isChecked() && _drawPointsBox->isChecked());
    	_ycDraw->setEnabled( _drawYBox->isChecked() && _drawCirclesBox->isChecked() );
    } else if( obj == _drawZBox ) {
    	_zDraw->setEnabled( _drawZBox->isChecked() && _drawPointsBox->isChecked());
    	_zcDraw->setEnabled( _drawZBox->isChecked() && _drawCirclesBox->isChecked() );
    } else if( obj == _drawPointsBox ){
    	_xDraw->setEnabled( _drawXBox->isChecked() && _drawPointsBox->isChecked() );
    	_yDraw->setEnabled( _drawYBox->isChecked() && _drawPointsBox->isChecked());
    	_zDraw->setEnabled( _drawZBox->isChecked() && _drawPointsBox->isChecked());
    } else if( obj == _drawCirclesBox ){
    	_xcDraw->setEnabled( _drawXBox->isChecked() && _drawCirclesBox->isChecked() );
    	_ycDraw->setEnabled( _drawYBox->isChecked() && _drawCirclesBox->isChecked() );
    	_zcDraw->setEnabled( _drawZBox->isChecked() && _drawCirclesBox->isChecked() );
    } else if( obj == _drawStartPosesBox){
    	bool drawStartPoses = _drawStartPosesBox->isChecked();
    	_selPoseDrawX->setEnabled( drawStartPoses && _drawXBox->isChecked());
    	_selPoseDrawY->setEnabled( drawStartPoses && _drawYBox->isChecked());
    	_selPoseDrawZ->setEnabled( drawStartPoses && _drawZBox->isChecked());
    } else if( obj == _selectObjBox){
    	updateRenderView();
    	updateResultView();
    } else if( obj == _resultView ){
    	RigidBody *body = getSelectedBody();
    	int poseIdx = _resultView->currentRow();
    	// get the support pose and the default state
    	SupportPose &pose = _supportPoses[body][poseIdx];
    	State state = _wc->getDefaultState();
    	// calculate a transform that is part of the support pose
    	Transform3D<> trans = body->getMovableFrame().getTransform(state);
    	Vector3D<> p = pose._rotAxesTable[0];
    	Vector3D<> v = trans.R()*pose._rotAxes[0];

    	double ang = angle(v, p, normalize( cross(v,p)));//acos( dot(v,p) );
    	std::cout << pose._rotAxesTable[0] << " " << pose._rotAxes[0] << std::endl;
    	std::cout << "Angle: " << ang << v << p << std::endl;
    	EAA<> eaa( normalize( cross(v,p) ) , ang );
    	trans.R() = eaa.toRotation3D();
    	body->getMovableFrame().setTransform(trans,state);


    	// now we also need to show the starting points

    	int bodyIdx = _selectObjBox->currentIndex();
    	std::vector<int> &poseIdxList = _supportToPose[std::make_pair(bodyIdx,poseIdx)];
    	std::cout << "PoseIdxList: " << poseIdxList.size() << std::endl;
    	_selPosePntRenderX->clear();
    	_selPosePntRenderY->clear();
    	_selPosePntRenderZ->clear();
    	std::vector<Vector3D<> > &xaxis = _xaxisS[body];
    	std::vector<Vector3D<> > &yaxis = _yaxisS[body];
    	std::vector<Vector3D<> > &zaxis = _zaxisS[body];
    	std::cout << "xaxis.size()==_xaxis.size() " << xaxis.size() << "==" << _xaxis[bodyIdx].size()<< std::endl;
    	if(xaxis.size()==_xaxis[bodyIdx].size()){
			BOOST_FOREACH(int idx, poseIdxList){
				_selPosePntRenderX->addPoint( xaxis[idx] );
				_selPosePntRenderY->addPoint( yaxis[idx] );
				_selPosePntRenderZ->addPoint( zaxis[idx] );
			}
    	}
    	// signal to update the transform
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
			img.getImageData()[x+y*width] = 240;
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

void SupportPoseAnalyserDialog::addStateStartPath(rw::trajectory::TimedStatePathPtr path){
	_xaxisS.clear();
	_yaxisS.clear();
	_zaxisS.clear();
	for(size_t j=0;j<_bodies.size();j++){
		std::vector<Vector3D<> > &xaxis = _xaxisS[_bodies[j]];
		std::vector<Vector3D<> > &yaxis = _yaxisS[_bodies[j]];
		std::vector<Vector3D<> > &zaxis = _zaxisS[_bodies[j]];
		xaxis.resize(path->size());
		yaxis.resize(path->size());
		zaxis.resize(path->size());
		for(size_t i=0; i<path->size();i++){
			const State &state = (*path)[i].getValue();
			RigidBody *body = _bodies[j];

			//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
			Rotation3D<> rot = body->getMovableFrame().getTransform(state).R();
			xaxis[i] = Vector3D<>(rot(0,0),rot(1,0),rot(2,0));
			yaxis[i] = Vector3D<>(rot(0,1),rot(1,1),rot(2,1));
			zaxis[i] = Vector3D<>(rot(0,2),rot(1,2),rot(2,2));
		}
	}

}

void SupportPoseAnalyserDialog::addStatePath(rw::trajectory::TimedStatePathPtr path){
	if( _bodies.size()==0 )
		return;

	_xaxis.clear();
	_yaxis.clear();
	_zaxis.clear();

	_xaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );
	_yaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );
	_zaxis.resize(_bodies.size(),std::vector<Vector3D<> >(path->size()) );


	// copy all poses of all body/state sets into an array
	std::cout << "Size of path is: " << path->size() << std::endl;
	const State defState = (*path)[0].getValue();

	for(size_t i=0; i<path->size();i++){
		const State &state = (*path)[i].getValue();
		for(size_t j=0;j<_bodies.size();j++){
			RigidBody *body = _bodies[j];

			//Rotation3D<> rot = Kinematics::worldTframe( _bodies[j]->getMovableFrame(), state ).R();
			Rotation3D<> rot = body->getMovableFrame().getTransform(state).R();
			_xaxis[j][i] = Vector3D<>(rot(0,0),rot(1,0),rot(2,0));
			_yaxis[j][i] = Vector3D<>(rot(0,1),rot(1,1),rot(2,1));
			_zaxis[j][i] = Vector3D<>(rot(0,2),rot(1,2),rot(2,2));
		}
	}
	updateHoughThres();
	updateRenderView();
}

void SupportPoseAnalyserDialog::process(){
	// first we need to figure out what frames that are of interest

	if( _bodies.size()==0 )
		return;

	int houghThres = _thresholdSpin->value();
	double epsilon = _epsilonSpin->value();

	// map the points on the sphere to a plane of spherical coordinates
	for(size_t j=0;j<_bodies.size();j++){
		const Transform3D<> wTp = Kinematics::worldTframe(_bodies[j]->getMovableFrame().getParent(), _defaultState);
		const Transform3D<> wTb = Kinematics::worldTframe(&(_bodies[j]->getMovableFrame()), _defaultState);

#ifdef USE_OPENCV
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

#endif
	}

	updateRenderView();
	updateResultView();

	_view->updateGL();
}

void SupportPoseAnalyserDialog::updateResultView(){
	RigidBody *body = getSelectedBody();
	if(!body)
		return;
	_resultView->clear();

	int i=0;
	BOOST_FOREACH(const SupportPose &pose, _supportPoses[body]){
		std::stringstream str;
		str << "(" << i << ") [ " << pose._degree << " , " << pose._probability << " ] ";

		_resultView->addItem( str.str().c_str() );
		i++;
	}
}

void SupportPoseAnalyserDialog::updateRenderView(){
	int objIdx = _selectObjBox->currentIndex();
	RigidBody *body = getSelectedBody();
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
