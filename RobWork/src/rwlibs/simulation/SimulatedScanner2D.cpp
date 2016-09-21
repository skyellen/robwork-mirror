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

#include "SimulatedScanner2D.hpp"
#include "FrameGrabber25D.hpp"

using namespace rwlibs::simulation;
using namespace rw::sensor;
using namespace rw::common;

namespace {

    class Sensor2DWrapper: public rw::sensor::Scanner2D {
    public:
        SimulatedScanner2D *_simscanner;

        Sensor2DWrapper(SimulatedScanner2D *scanner):
            Scanner2D(scanner->getSensorModel()->getName()),
            _simscanner(scanner)
        {
            setSensorModel( scanner->getSensorModel() );
        }

        const rw::geometry::PointCloud& getScan() const{ return _simscanner->getScan(); };
        double getAngularRange() const { return _simscanner->getAngularRange(); };
        size_t getMeasurementCount() const { return _simscanner->getMeasurementCount(); };


        void open(){ _simscanner->open(); }
        bool isOpen(){ return  _simscanner->isOpen(); }
        void close() {_simscanner->close(); }
        void acquire(){  _simscanner->acquire(); }
        bool isScanReady() { return _simscanner->isScanReady(); }
        std::pair<double,double> getRange() { return _simscanner->getRange(); }
        double getFrameRate(){ return  _simscanner->getFrameRate(); }

    };


}


SimulatedScanner2D::SimulatedScanner2D(const std::string& name, rw::kinematics::Frame* frame, FrameGrabber25D::Ptr framegrabber):
   SimulatedSensor( rw::common::ownedPtr( new Scanner2DModel(name, framegrabber->getFieldOfViewY(), static_cast<int>(framegrabber->getWidth()), frame) )),
   _framegrabber(framegrabber),
    _frameRate(30),
    _dtsum(0)
{
}

SimulatedScanner2D::SimulatedScanner2D(const std::string& name,
                                       const std::string& desc,
                                       rw::kinematics::Frame* frame,
                                       FrameGrabber25D::Ptr framegrabber):
       SimulatedSensor( ownedPtr( new Scanner2DModel(name, _framegrabber->getFieldOfViewY(), static_cast<int>(framegrabber->getWidth()),frame) )),
		_framegrabber(framegrabber),
		_frameRate(30),
        _dtsum(0)
{
}

SimulatedScanner2D::~SimulatedScanner2D(){}


void SimulatedScanner2D::open(){
	_isOpenned = true;
    _dtsum = 0;
}

bool SimulatedScanner2D::isOpen(){
	return _isOpenned;
}

void SimulatedScanner2D::close(){
	_isOpenned = false;
}



const rw::geometry::PointCloud& SimulatedScanner2D::getScan() const {
    return _framegrabber->getImage();
}


void SimulatedScanner2D::acquire(){
	if(!_isOpenned)
		RW_THROW("Scanner has not been openned yet!");
	_isAcquired = false;
}

bool SimulatedScanner2D::isScanReady(){
	return _isAcquired;
}

std::pair<double,double> SimulatedScanner2D::getRange(){
	return std::make_pair(_framegrabber->getMinDepth(),_framegrabber->getMaxDepth());
}

double SimulatedScanner2D::getFrameRate(){
	return _frameRate;
}


void SimulatedScanner2D::update(const Simulator::UpdateInfo& info, rw::kinematics::State& state){
    if(!_isOpenned || _isAcquired)
        return;
    if( _frameRate<0.00001 )
    	return;

    _dtsum += info.dt;

    if( _dtsum>1.0/_frameRate ){
    	_dtsum = 0;
    	_framegrabber->grab(getFrame(), state);
    	_isAcquired = true;
    }

}

void SimulatedScanner2D::reset(const rw::kinematics::State& state){

}

rw::sensor::Scanner2DModel::Ptr SimulatedScanner2D::getSensorModel(){
	return _smodel;
}

rw::sensor::Scanner2D::Ptr SimulatedScanner2D::getScanner2DSensor(rwlibs::simulation::Simulator* instance){
	if( instance->hasHandle(this) )
		return instance->getSensorHandle(this).cast<rw::sensor::Scanner2D>();

	rw::common::Ptr<Sensor2DWrapper> handle =
			rw::common::ownedPtr( new Sensor2DWrapper(this) );

	instance->addHandle(this, handle );
	return handle;
}

double SimulatedScanner2D::getAngularRange() {
    return _framegrabber->getFieldOfViewY();
}

size_t SimulatedScanner2D::getMeasurementCount() const {
    return _framegrabber->getWidth()*_framegrabber->getHeight();
}
