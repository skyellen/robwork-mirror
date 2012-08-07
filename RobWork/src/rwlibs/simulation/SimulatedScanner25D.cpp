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

#include "SimulatedScanner25D.hpp"

using namespace rwlibs::simulation;
using namespace rw::sensor;

namespace {

class Scanner25DWrapper: public rw::sensor::Scanner25D {
public:
    SimulatedScanner25D *_simscanner;

    Scanner25DWrapper(SimulatedScanner25D *scanner,rw::kinematics::Frame *sframe, const std::string& name):
        Scanner25D(name),
        _simscanner(scanner)
    {
        attachTo(sframe);
    }

    //const Image25D& getImage() const{ return _simscanner->getScan(); };
    virtual const Image25D& getScan(){ return _simscanner->getScan();} ;

    void open(){ _simscanner->open(); }
    bool isOpen(){ return  _simscanner->isOpen(); }
    void close() {_simscanner->close(); }
    void acquire(){  _simscanner->acquire(); }
    bool isScanReady() { return _simscanner->isScanReady(); }
    std::pair<double,double> getRange() { return _simscanner->getRange(); }
    double getFrameRate(){ return  _simscanner->getFrameRate(); }

};


}

SimulatedScanner25D::SimulatedScanner25D(const std::string& name,
                                         rw::kinematics::Frame *frame,
                                         FrameGrabber25D::Ptr framegrabber):
		SimulatedSensor(name),
		_framegrabber(framegrabber),
		_frameRate(30),
        _dtsum(0)
{
    _rsensor = rw::common::ownedPtr( new Scanner25DWrapper(this, frame,  name) );
    attachTo(frame);
}

SimulatedScanner25D::SimulatedScanner25D(const std::string& name,
		const std::string& desc,
		rw::kinematics::Frame *frame,
		FrameGrabber25D::Ptr framegrabber):
        SimulatedSensor(name),
		_framegrabber(framegrabber),
		_frameRate(30),
		_dtsum(0)
{
    _rsensor = rw::common::ownedPtr( new Scanner25DWrapper(this, frame,  name) );
    attachTo(frame);
}

SimulatedScanner25D::~SimulatedScanner25D(){}


void SimulatedScanner25D::open(){
	_isOpenned = true;
    _dtsum = 0;
}

bool SimulatedScanner25D::isOpen(){
	return _isOpenned;
}

void SimulatedScanner25D::close(){
	_isOpenned = false;
}

void SimulatedScanner25D::acquire(){
	if(!_isOpenned)
		RW_THROW("Scanner has not been openned yet!");
	_isAcquired = false;
}

bool SimulatedScanner25D::isScanReady(){
	return _isAcquired;
}

std::pair<double,double> SimulatedScanner25D::getRange(){
	return std::make_pair(_framegrabber->getMinDepth(), _framegrabber->getMaxDepth());
}

double SimulatedScanner25D::getFrameRate(){
	return _frameRate;
}

const Image25D& SimulatedScanner25D::getScan(){
    return _framegrabber->getImage();
}

void SimulatedScanner25D::update(const Simulator::UpdateInfo& info, rw::kinematics::State& state){
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

void SimulatedScanner25D::reset(const rw::kinematics::State& state){

}


