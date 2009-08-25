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


#include "VirtualCamera.hpp"

#include <rw/kinematics/Frame.hpp>
#include <rw/sensor/Image.hpp>
#include <rw/math/Transform3D.hpp>

#include <cmath>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rwlibs::simulation;

VirtualCamera::VirtualCamera(
    const std::string& name,
    FrameGrabber& frameGrabber,
    Frame *frame)
    :
    Camera(frame,name,"Virtual Camera"),
    _frameGrabber(&frameGrabber),
    _isAcquired(false),
    _dtSum(0.0)
{
    std::cout << "virtual cam:";
    std::cout << " initialized" << std::endl;
}

VirtualCamera::~VirtualCamera()
{
    if (_started)
        stop();
}

bool VirtualCamera::initialize()
{
    if (_started)
        stop();

    // next: allocate memory for the image
    //_imgBuffer = new char[(_width*_height*colorTypeWidth)];
    //_imgData.resize( _width, _height, bitsPerPixel);
    //_frameGrapper.resize( _width, _height, bitsPerPixel);
    _initialized = true;
    return true;
}

void VirtualCamera::stop()
{
    if (!_initialized){
    	RW_THROW("Camera was not initialized!");
    }
    if (!_started){
    	RW_THROW("Camera was not started!");
    }
    //
    _started = false;
}

bool VirtualCamera::start()
{
    if (_initialized == false)
        initialize();
    if (_initialized == false)
        return false;
    _started = true;
    return true;
}

void VirtualCamera::acquire()
{
    _isAcquired = false;
}

void VirtualCamera::update(double dt, const rw::kinematics::State& state){
    if(!_started || _isAcquired)
        return;
    if( _frameRate=0.0 )
    	return;

    _dtSum += dt;

    if( _dtSum>1/_frameRate ){
    	_dtSum = 0;
    	_frameGrabber->grab(getFrame(), state);
    	_isAcquired = true;
    }
}

bool VirtualCamera::isImageReady()
{
    return _isAcquired;
}

const Image* VirtualCamera::getImage()
{
    return &( _frameGrabber->getImage() );
}

double VirtualCamera::getFrameRate()
{
    return _frameRate;
}

void VirtualCamera::setFrameRate(double framerate)
{
    _frameRate = framerate;
}

