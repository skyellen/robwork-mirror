/*********************************************************************
 * RobWork Version 0.2
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
    _policy(SINGLE_SHOT),
    _mode(M640x480),
    _frameGrabber(&frameGrabber),
    _error(SUCCES),
    _isAcquired(false)
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
        _error = NOT_INITIALIZED;
        return;
    }
    if (!_started){
        _error = NOT_STARTED;
        return;
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

void VirtualCamera::acquire(const rw::kinematics::State& state)
{
    _isAcquired = false;
    _frameGrabber->grab(getFrame(), state);
    _isAcquired = true;
}

bool VirtualCamera::isImageReady()
{
    return _isAcquired;
}

const Image* VirtualCamera::getImage()
{
    // TODO: change so only last acquired image is returned
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

std::pair<unsigned int,unsigned int> VirtualCamera::getDimension()
{
    unsigned int width = _frameGrabber->getWidth();
    unsigned int height = _frameGrabber->getHeight();
    return std::make_pair(width, height);
}

Camera::CapturePolicy VirtualCamera::getCapturePolicy()
{
    return _policy;
}

Camera::CaptureMode VirtualCamera::getCaptureMode()
{
    return _mode;
}

bool VirtualCamera::setCaptureMode(Camera::CaptureMode mode)
{
    int height,width;

    switch(_mode){
    case(M160x120)  : width = 160;  height = 120; break;
    case(M320x240)  : width = 320;  height = 240; break;
    case(M640x480)  : width = 640;  height = 480; break;
    case(M800x600)  : width = 800;  height = 600; break;
    case(M1024x768) : width = 1024; height = 768; break;
    case(M1280x960) : width = 1280; height = 960; break;
    case(M1600x1200): width = 1600; height = 1200; break;
    default:
        _error = UNSUPPORTED_CAPTURE_MODE;
        return false;
    }

    _frameGrabber->resize(width,height);
    CaptureMode tmpMode = _mode;
    _mode = mode;

    stop();

    // if cam can't initialize then fall back to old mode
    if (!initialize()){
        _error = UNSUPPORTED_CAPTURE_MODE;
        _mode = tmpMode;
        return false;
    }

    return true;
}
