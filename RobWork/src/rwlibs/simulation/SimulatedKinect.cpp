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

#include "SimulatedKinect.hpp"

#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/Math.hpp>

using namespace rwlibs::simulation;
using namespace rw::sensor;
using namespace rw::math;
using namespace rw::graphics;
using namespace rw::common;

namespace {
/*
    class KinnectSensorWrapper: public rw::sensor::Scanner25D, public virtual
    {
    public:
        SimulatedScanner25D *_simscanner;

        KinnectSensorWrapper(SimulatedScanner25D *scanner,rw::kinematics::Frame *sframe, const std::string& name):
            Scanner25D(name),
            _simscanner(scanner)
        {
            attachTo(sframe);
        }

        //const Image25D& getImage() const{ return _simscanner->getScan(); };
        virtual const Image25D& getImage(){ return _simscanner->getImage();} ;

        void open(){ _simscanner->open(); }
        bool isOpen(){ return  _simscanner->isOpen(); }
        void close() {_simscanner->close(); }
        void acquire(){  _simscanner->acquire(); }
        bool isScanReady() { return _simscanner->isScanReady(); }
        std::pair<double,double> getRange() { return _simscanner->getRange(); }
        double getFrameRate(){ return  _simscanner->getFrameRate(); }

    };
*/

	class KinectSensorModel: public SensorModel {
	public:
		rw::sensor::CameraModel::Ptr _camModel;
		rw::sensor::Scanner25DModel::Ptr _scanModel;

		KinectSensorModel(rw::sensor::CameraModel::Ptr camModel, rw::sensor::Scanner25DModel::Ptr scanModel):
			SensorModel(camModel->getName(),camModel->getFrame()),_camModel(camModel),_scanModel(scanModel)
		{

		}
		static rw::common::Ptr<KinectSensorModel> make(rw::sensor::CameraModel::Ptr camModel, rw::sensor::Scanner25DModel::Ptr scanModel){
			return rw::common::ownedPtr( new KinectSensorModel(camModel,scanModel));
		}
	};

	rw::common::Ptr<CameraModel> makeDefaultCameraModel(const std::string& name,
			rw::kinematics::Frame *frame, double fov, int w, int h, double near, double far)
	{
		return rw::common::ownedPtr( new CameraModel(ProjectionMatrix::makePerspective(fov*Deg2Rad,w,h,near,far), name, frame ) );
	}

	rw::common::Ptr<Scanner25DModel> makeDefaultScannerModel(const std::string& name,
			rw::kinematics::Frame *frame, double fov, int w, int h, double near, double far)
	{
		return rw::common::ownedPtr( new Scanner25DModel(name, w, h, frame ) );
	}

}

SimulatedKinect::SimulatedKinect(const std::string& name, rw::kinematics::Frame *frame):
		SimulatedSensor( KinectSensorModel::make( makeDefaultCameraModel(name,frame,43,640,480,0.01,6), makeDefaultScannerModel(name,frame,43,640,480,0.01,6) )),
		_frameRate(1),
        _dtsum(0),
		_noiseEnabled(true),
        _near(0.01),
        _far(6),
        _fieldOfView(43),
        _grabSingleFrame(false),
        _width(640),
        _height(480),
        _img(new rw::sensor::Image( _width, _height, rw::sensor::Image::RGB, rw::sensor::Image::Depth8U )),
        _scan(new rw::geometry::PointCloud(_width,_height))
{
    //_rsensor = rw::common::ownedPtr( new Scanner25DWrapper(this, frame,  name) );
}

SimulatedKinect::SimulatedKinect(const std::string& name,
		const std::string& desc,
		rw::kinematics::Frame *frame):
		SimulatedSensor( KinectSensorModel::make( makeDefaultCameraModel(name,frame,43,640,480,0.01,6), makeDefaultScannerModel(name,frame,43,640,480,0.01,6) )),
		_frameRate(30),
		_dtsum(0),
        _noiseEnabled(true),
        _near(0.01),
        _far(6),
		_fieldOfView(43),
		_grabSingleFrame(false),
        _width(640),
        _height(480),
        _img(new rw::sensor::Image( _width, _height, rw::sensor::Image::RGB, rw::sensor::Image::Depth8U )),
        _scan(new rw::geometry::PointCloud(_width,_height))
{

    //_rsensor = rw::common::ownedPtr( new Scanner25DWrapper(this, frame,  name) );
}

SimulatedKinect::SimulatedKinect(rw::sensor::CameraModel::Ptr camModel, rw::sensor::Scanner25DModel::Ptr scannerModel):
	SimulatedSensor( KinectSensorModel::make(camModel,scannerModel)),
	_frameRate(1),
    _dtsum(0),
	_noiseEnabled(true),
    _near(0.01),
    _far(6),
    _fieldOfView(43),
    _grabSingleFrame(false),
    _width(640),
    _height(480),
    _img(new rw::sensor::Image( _width, _height, rw::sensor::Image::RGB, rw::sensor::Image::Depth8U )),
    _scan(new rw::geometry::PointCloud(_width,_height))
{

}

SimulatedKinect::~SimulatedKinect(){}

void SimulatedKinect::init(rw::graphics::SceneViewer::Ptr drawer){
    _drawer = drawer;

    SceneViewer::View::Ptr view = _drawer->createView("CameraSensorView");

    view->_viewCamera->setAspectRatioControl(SceneCamera::Scale);
    view->_viewCamera->setEnabled(true);
    view->_viewCamera->setClearBufferEnabled(true);
    view->_viewCamera->setClearBufferMask( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    view->_viewCamera->setDepthTestEnabled( true );
    view->_viewCamera->setLightningEnabled( true );
    view->_viewCamera->setRefNode( drawer->getScene()->getRoot() );
    //std::cout << width <<  " " << height << std::endl;
    view->_viewCamera->setPerspective(_fieldOfView, _width, _height, _near, _far);
    view->_viewCamera->setViewport(0,0, _width, _height);
    view->_viewCamera->setAspectRatioControl(SceneCamera::Fixed);
    view->_viewCamera->attachTo( drawer->getMainView()->_viewCamera->getRefNode() );
    view->_viewCamera->setDrawMask(DrawableNode::Physical);
    // render offscreen
    view->_camGroup->setOffscreenRenderEnabled(true);
    view->_camGroup->setOffscreenRenderColor( rw::sensor::Image::RGB );
    view->_camGroup->setOffscreenRenderSize(_width, _height);
    view->_camGroup->setCopyToImage( _img );
    view->_camGroup->setCopyToScan25D( _scan );

    view->_camGroup->setEnabled(true);
    _view = view;
}


void SimulatedKinect::open(){
    if(_drawer==NULL)
        RW_THROW("The SimulatedKinect sensor has not been properly initialized, please call init(...)");
	_isOpenned = true;
    _dtsum = 0;
}

bool SimulatedKinect::isOpen(){
	return _isOpenned;
}

void SimulatedKinect::close(){
	_isOpenned = false;
}

void SimulatedKinect::acquire(){
	if(!_isOpenned)
		RW_THROW("Scanner has not been openned yet!");
	_isAcquired = false;
	_grabSingleFrame = true;
}

bool SimulatedKinect::isDataReady(){
	return _isAcquired;
}

std::pair<double,double> SimulatedKinect::getRange() const{
	return std::make_pair(_near, _far);
}

double SimulatedKinect::getFrameRate() const {
	return _frameRate;
}

const Image& SimulatedKinect::getImage(){
    return *_img;
}

const rw::geometry::PointCloud& SimulatedKinect::getScan(){
    return *_scan;
}

namespace {

    double calcSigma( double r, double d ){
        const double p00 = 2.344;
        const double p10 = -1.202E-2;
        const double p01 = -1.734E-3;
        const double p20 =  1.818E-5;
        const double p11 =  6.516E-6;
        const double p02 =  1.233E-6;
        return p00 + p10*r + p01*d + p20*r*r + p11*r*d + p02*d*d;
    }

    void applyNoise(rw::geometry::PointCloud::Ptr scan){

        // walk through data and change z accordingly
        std::vector<Vector3D<float> > &data = scan->getData();

        double center_x = scan->getWidth()/2.0;
        double center_y = scan->getHeight()/2.0;

        for(size_t y=0;y<scan->getHeight();y++){
            for(size_t x=0;x<scan->getWidth();x++){
                Vector3D<float> &v = data[y*scan->getWidth() + x];
                double r = std::sqrt( Math::sqr(x - center_x) + Math::sqr(y - center_y) );
                // we convert to mm
                double d = (double)fabs(v[2])*1000.0;
                double sigma = calcSigma(r, d);
                // this will produce the error in m
                double noise_err = Math::ranNormalDist( 0 , sigma )/1000.0;
                v[2] += (float)noise_err;

            }
        }
    }

}

void SimulatedKinect::update(const Simulator::UpdateInfo& info, rw::kinematics::State& state){
    if(!_isOpenned)
        return;
    if( _frameRate<0.00001){
        if(_grabSingleFrame){
            _grabSingleFrame=false;
            rw::math::Transform3D<> wTf = rw::kinematics::Kinematics::worldTframe(getFrame(), state);

            Image::Ptr img = _camModel->getImage(state);
            if(img==NULL){
            	img = ownedPtr( new rw::sensor::Image( _width, _height, rw::sensor::Image::RGB, rw::sensor::Image::Depth8U ) );
            	_camModel->setImage(img, state);
            }

            _view->_viewCamera->setTransform( wTf );
            // the image is grabbed in the negative z-axis
            _drawer->renderView(_view);

            if(_noiseEnabled)
                applyNoise(_scan);

            _isAcquired = true;
        }
        return;
    }


    _dtsum += info.dt;

    if( _dtsum>1.0/_frameRate ){
    	_dtsum = 0;

        rw::math::Transform3D<> wTf = rw::kinematics::Kinematics::worldTframe(getFrame(), state);
        _view->_viewCamera->setTransform( wTf );
        Image::Ptr img = _camModel->getImage(state);
        if(img==NULL){
        	img = ownedPtr( new rw::sensor::Image( _width, _height, rw::sensor::Image::RGB, rw::sensor::Image::Depth8U ) );
        	_camModel->setImage(img, state);
        }

        _view->_camGroup->setCopyToImage( img ); // get the image from the relevant state
        // the image is grabbed in the negative z-axis
        _drawer->renderView(_view);

        if(_noiseEnabled)
            applyNoise(_scan);

    	_isAcquired = true;
    }
}

void SimulatedKinect::reset(const rw::kinematics::State& state){

}

rw::sensor::Sensor::Ptr SimulatedKinect::getSensorHandle(rwlibs::simulation::Simulator::Ptr simulator){
	return _rsensor;
}



