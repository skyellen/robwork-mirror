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

#include "DC1394Camera.hpp"

#include <rw/math/Transform3D.hpp>

using namespace rw::sensor;
using namespace rw::math;

using namespace rwlibs::sensors;

#define MAX_CAMERAS 12
#define MAX_PORTS 12

/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 0

using namespace rwlibs::sensors;
using namespace rw::kinematics;

namespace
{
    raw1394handle_t handles[MAX_CAMERAS];
    dc1394_cameracapture cameras[MAX_CAMERAS];

    std::string getCameraName(
        raw1394handle_t handle, dc1394_cameracapture dc1394cam)
    {
       dc1394_camerainfo info;
       dc1394_get_camera_info(handle,dc1394cam.node,&info);
       return std::string(info.model);
    }

    std::string getCameraVendor(
        raw1394handle_t handle, dc1394_cameracapture dc1394cam)
    {
        dc1394_camerainfo info;
        dc1394_get_camera_info(handle,dc1394cam.node,&info);
        return std::string(info.vendor);
    }
}

DC1394Camera::DC1394Camera(
    Frame* parent,
    raw1394handle_t handle,
    dc1394_cameracapture dc1394cam)
    :
    Camera(parent, getCameraName(handle, dc1394cam), getCameraVendor(handle, dc1394cam)),
    _policy(CONTINUES),
    _isAcquired(false),
    _handle(handle),
    _dccamera(dc1394cam),
    _captureMode(MODE_640x480_MONO),
    _frameRate(FRAMERATE_30)
{}

DC1394Camera::~DC1394Camera()
{
    raw1394_destroy_handle(_handle);
}

bool DC1394Camera::initialize()
{
    // check the iso channel number and speed
    unsigned int channel=0,speed=0;
    if(dc1394_get_iso_channel_and_speed(_handle,_dccamera.node,
                                        &channel,&speed)!= DC1394_SUCCESS ) {

        RW_WARN("Unable to get the iso channel number!!");
        return false;
    }
    char *device_name=NULL;
    std::cout << "Channel: "<< channel << " speed: " << speed << std::endl;
    // initialize DMA capture
    if (dc1394_dma_setup_capture(
            _handle,_dccamera.node,1,
            FORMAT_VGA_NONCOMPRESSED,_captureMode,
            SPEED_400, _frameRate, 10, DROP_FRAMES,
            device_name, &_dccamera) != DC1394_SUCCESS)
    {
        RW_WARN(
            "Unable to setup camera for DMA capture.\n"
            "Perhaps the framerate or videomode is not supported by the camera!!");

        return false;
    }

    _initialized = true;
    return true;
}

void DC1394Camera::stop()
{
    if(!_started)
        return;

    if( dc1394_stop_iso_transmission(_handle,_dccamera.node)!=DC1394_SUCCESS){
        RW_WARN("Unable to stop camera iso transmission.");
        return;
    }

    delete _image;
    _image = 0;

    _started = false;
}

bool DC1394Camera::start()
{
    if(_started)
        return false;
    if(!_initialized)
        return false;

    int width=0,height=0;
    CaptureMode mode = getCaptureMode();
    switch(mode){
    case(M160x120): width=160; height=120; break;
    case(M320x240): width=320; height=240; break;
    case(M640x480): width=640; height=480; break;
    case(M800x600): width=800; height=600; break;
    case(M1024x768): width=1024; height=768; break;
    case(M1280x960): width=1280; height=960; break;
    case(M1600x1200): width=1600; height=1200; break;
    default:
        return false;
    }

    if( dc1394_start_iso_transmission(_handle,_dccamera.node)!=DC1394_SUCCESS ){
        RW_WARN("Unable to start camera iso transmission.");
        return false;
    }

    Image::ColorCode encoding = Image::MONO8;

    _image = new Image(width,height,encoding);
    _started = true;
    return true;
}

void DC1394Camera::acquire()
{
    if(!_started)
        return;

    _isAcquired = false;

    if( dc1394_dma_single_capture_poll(&_dccamera) == DC1394_NO_FRAME) {
        return;
    }

    memcpy(
        _image->getImageData(),
        _dccamera.capture_buffer,
        _image->getDataSize());

    dc1394_dma_done_with_buffer(&_dccamera);

    _isAcquired = true;
}

bool DC1394Camera::isImageReady()
{
    return _isAcquired;
}

const Image* DC1394Camera::getImage()
{
    if(!_started || !_isAcquired)
        return NULL;
    return _image;
}

double DC1394Camera::getFrameRate()
{
    double res = 0.0;
    switch(_frameRate){
    case(FRAMERATE_1_875): res = 1.875; break;
    case(FRAMERATE_3_75): res = 3.75; break;
    case(FRAMERATE_7_5): res = 7.5; break;
    case(FRAMERATE_15): res = 15.0; break;
    case(FRAMERATE_30): res = 30.0; break;
    case(FRAMERATE_60): res = 60.0; break;
    case(FRAMERATE_120): res = 120.0; break;
    case(FRAMERATE_240): res = 240.0; break;
    default: res = 0;
    }
    return res;
}

void DC1394Camera::setFrameRate(double framerate)
{
    int res = FRAMERATE_1_875;
    if(framerate==1.875)  res = FRAMERATE_1_875;
    else if(framerate==3.75) res = FRAMERATE_3_75;
    else if(framerate==7.5) res = FRAMERATE_7_5;
    else {
        int fps = (int)framerate;
        switch(fps){
        case(15): res = FRAMERATE_15; break;
        case(30): res = FRAMERATE_30; break;
        case(60): res = FRAMERATE_60; break;
        case(120): res = FRAMERATE_120; break;
        case(240): res = FRAMERATE_240; break;
        default: res = 0;
        }
    }
    _frameRate = res;
}

Camera::CaptureMode DC1394Camera::getCaptureMode()
{
    CaptureMode mode = M160x120;
    switch(_captureMode){
    case(MODE_160x120_YUV444): mode = M160x120; break;
    case(MODE_320x240_YUV422): mode = M320x240; break;
    case(MODE_640x480_YUV411):
    case(MODE_640x480_YUV422):
    case(MODE_640x480_RGB):
    case(MODE_640x480_MONO):
    case(MODE_640x480_MONO16): mode = M640x480; break;
    case(MODE_800x600_YUV422):
    case(MODE_800x600_RGB):
    case(MODE_800x600_MONO):
    case(MODE_800x600_MONO16): mode = M800x600; break;
    case(MODE_1024x768_YUV422):
    case(MODE_1024x768_RGB):
    case(MODE_1024x768_MONO):
    case(MODE_1024x768_MONO16): mode = M1024x768; break;
    case(MODE_1280x960_YUV422):
    case(MODE_1280x960_RGB):
    case(MODE_1280x960_MONO):
    case(MODE_1280x960_MONO16): mode = M1280x960; break;
    case(MODE_1600x1200_YUV422):
    case(MODE_1600x1200_RGB):
    case(MODE_1600x1200_MONO):
    case(MODE_1600x1200_MONO16): mode = M1600x1200; break;
    default:
        RW_THROW("Unsupported capturemode!!");
    }
    return mode;
}

bool DC1394Camera::setCaptureMode(CaptureMode mode)
{
    int res = MODE_640x480_MONO;
    switch(mode){
    case(M160x120): res = MODE_160x120_YUV444; break;
    case(M320x240): res = MODE_320x240_YUV422; break;
    case(M640x480): res = MODE_640x480_MONO; break;
    case(M800x600): res = MODE_800x600_MONO; break;
    case(M1024x768): res = MODE_1024x768_MONO; break;
    case(M1280x960): res = MODE_1280x960_MONO; break;
    case(M1600x1200): res = MODE_1600x1200_MONO; break;
    default:
        return false;
    }
    _captureMode = res;
    return true;
}

std::pair<unsigned int,unsigned int> DC1394Camera::getDimension()
{
    return std::make_pair(0,0);
}

/**
 * @return a list of available cameras
 */
const std::vector<DC1394Camera*> DC1394Camera::getCameraHandles()
{
    std::vector<DC1394Camera::DC1394Camera*> result;

    int numCameras = 0;
    struct raw1394_portinfo ports[MAX_PORTS];
    // get the number of ports (cards)
    raw1394handle_t raw_handle = raw1394_new_handle();
    if (raw_handle==NULL) {
        RW_THROW("Unable to aquire a raw1394 handle\n Did you load the drivers?");
    }

    int numPorts = raw1394_get_port_info(raw_handle, ports, MAX_PORTS);
    raw1394_destroy_handle(raw_handle);
    std::cout << "number of ports = " << numPorts << std::endl;

    // get dc1394 handle to each port
    for (int p = 0; p < numPorts; p++)
    {
        std::cout<<"p = "<<p<<std::endl;
        // get the camera nodes and describe them as we find them
        raw_handle = dc1394_create_handle(p);
        // Tries to use cd1394_create_handle because of problems with raw1394_new_handle
        //raw_handle = raw1394_new_handle();

        //Don't need this when using cd1394_create_handle
        //raw1394_set_port( raw_handle, p );

        int camCount;

        nodeid_t *camera_nodes = dc1394_get_camera_nodes(raw_handle, &camCount, 1);

        std::cout<<"get camera nodes"<<std::endl;
        raw1394_destroy_handle(raw_handle);
        std::cout<<"raw destroyed"<<std::endl;

        // setup cameras for capture
        for (int i = 0; i < camCount; i++)
        {
            handles[numCameras] = dc1394_create_handle(p);
            if (handles[numCameras] == NULL) {
                RW_WARN("Unable to aquire a raw1394 handle.\n"
                        "Did you load the drivers?");
                continue;
            }

            cameras[numCameras].node = camera_nodes[i];
            result.push_back(
                new DC1394Camera(
                    NULL, handles[numCameras], cameras[numCameras]));

            numCameras++;
        }

        dc1394_free_camera_nodes(camera_nodes);
    }
    return result;
}

Camera::CapturePolicy DC1394Camera::getCapturePolicy()
{
    return _policy;
}


double DC1394Camera::getFeature(Camera::CameraFeature setting) {
    int value = 0;
    //dc1394_feature_get_value(&_dccamera, DC1394_FEATURE_SHUTTER, &value);
    return value;
}

bool DC1394Camera::setFeature(Camera::CameraFeature setting, double value) {
    //int val = (int) val;
    //dc1394_feature_set_value(&_dccamera, DC1394_FEATURE_SHUTTER, val);
    return true;
}