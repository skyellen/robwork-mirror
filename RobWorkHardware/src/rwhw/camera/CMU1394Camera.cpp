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

#include "CMU1394Camera.hpp"

#include <rw/common/macros.hpp>
#include <vector>

#include <windows.h>
#include <1394Camera.h>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rwhw;

std::vector<CMU1394Camera*> CMU1394Camera::_cameras;

namespace
{
    std::string getCameraName(C1394Camera* cam)
    {
        RW_ASSERT(cam);

        std::vector<char> name(256);
//        cam->GetCameraName(&name.front(), name.size());
        return std::string(name.begin(), name.end());
    }

    std::string getCameraVendor(C1394Camera* cam)
    {
        RW_ASSERT(cam);

        std::vector<char> name(256);
//        cam->GetCameraVendor(&name.front(), name.size());
        return std::string(name.begin(), name.end());
    }
}

CMU1394Camera::CMU1394Camera(C1394Camera* cmuCam)
    :
    CameraFirewire(
        getCameraName(cmuCam),
        getCameraVendor(cmuCam)),
    _policy(CONTINUES),
    _isAquired(false),
    _cmuCam(cmuCam)
{}

CMU1394Camera::~CMU1394Camera()
{}

bool CMU1394Camera::initialize(){
    C1394Camera cam0,cam1;

    _initialized = true;
    return true;
}

void CMU1394Camera::stop(){
    _started = false;
}

bool CMU1394Camera::start(){
    return true;
}

void CMU1394Camera::acquire(){
    _isAquired = false;
}


bool CMU1394Camera::isImageReady(){
    return _isAquired;
}

const Image* CMU1394Camera::getImage(){
    if(!_started)
        return NULL;
    return _image;
}

double CMU1394Camera::getFrameRate(){
    return _cmuCam->GetVideoFrameRate();
}

void CMU1394Camera::setFrameRate(double framerate){
    _frameRate = framerate;
}

CameraFirewire::CaptureMode CMU1394Camera::getCaptureMode()
{
    return _captureMode;
}

bool CMU1394Camera::setCaptureMode(CaptureMode mode){
    _captureMode = mode;
    return true;
}

std::pair<unsigned int,unsigned int> CMU1394Camera::getDimension(){
    return std::make_pair(0, 0);
}

const std::vector<CMU1394Camera*> CMU1394Camera::getCameraHandles()
{
//     C1394Camera cam;
//     cam.RefreshCameraList();
//     int numCams = cam.GetNumberCameras();
//     std::vector<CMU1394Camera*> tmpList;
//     // update the camList
//     for(int i=0; i<numCams; i++){
//         cam.SelectCamera(i);
//         char buff[100];
//         cam.GetCameraName(buff,100);
//         std::string camName(buff);
//         bool found = false;
//         std::vector<CMU1394Camera*>::iterator iter = _cameras.begin();
//         for(;iter!=_cameras.end();++iter){
//             if( (**iter).getName() == camName){
//                 tmpList.push_back( *iter );
//                 _cameras.erase( iter );
//                 found = true;
//                 break;
//             }
//         }
//         if(found==false){
//             C1394Camera *newCam = new C1394Camera();
//             newCam->RefreshCameraList();
//             newCam->SelectCamera(i);
//             tmpList.push_back( new CMU1394Camera(newCam) );
//         }
//     }
//     // delete cmu cameraes that no longer exist, and update the state
//     // of the wincamera
//     std::vector<CMU1394Camera*>::iterator i_cam = _cameras.begin();
//     for(;i_cam!=_cameras.end();++i_cam){
//         (*i_cam)->setConnected(false);
//         delete (*i_cam)->getCMUCamera();
//     }
//     // move cameras from tmp list to camList
//     _cameras = tmpList;
//     // construct the cam list

    return _cameras;
}

CameraFirewire::CapturePolicy CMU1394Camera::getCapturePolicy()
{
    return _policy;
}
