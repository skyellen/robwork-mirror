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

#include "CMU1394Camera_c.hpp"

#define CAM_DELAY_SLEEP 1

#include <rw/common/macros.hpp>
#include <rw/common/Log.hpp>
#include <rw/common/TimerUtil.hpp>
#include <vector>

#include <windows.h>

using namespace rw::sensor;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rwhw;

std::vector<CameraID> CMU1394Camera::_cameras;
std::vector<CMU1394Camera*> CMU1394Camera::_connectedCameras;
bool CMU1394Camera::_queryCameras = false;

const std::string CMU1394Camera::sFrameRates[6] = {"1.875 fps","3.75 fps","7.5 fps","15 fps","30 fps","60 fps"};
const double CMU1394Camera::FrameRates[6] = {1.875, 3.75, 7.5, 15, 30, 60};


namespace
{
    std::string getCameraName(CameraID camID)
    {
        std::vector<char> name(256);
        GetCameraName_c(camID, &name.front(), name.size());
        return std::string(name.begin(), name.end());
    }

    std::string getCameraVendor(CameraID camID)
    {
        std::vector<char> name(256);
        GetCameraVendor_c(camID, &    name.front(), name.size());
        return std::string(name.begin(), name.end());
    }
}

CMU1394Camera::CMU1394Camera(CameraID cmuCam, std::string camName, std::string vendorName)
    :
    CameraFirewire(NULL, camName, vendorName),
    _policy(CONTINUES),
    _isAquired(false),
    _camID(cmuCam),
    _image(NULL)
{

    GetCameraUniqueID_c(cmuCam, &_uniqueID);
}

CMU1394Camera::~CMU1394Camera()
{
    stop();
    releaseCameraInstance_c(_camID);
}

bool CMU1394Camera::initialize(){
    // make sure to allocate memory for the image...
    unsigned long width, height;
    GetVideoFrameDimensions_c(_camID,&width,&height);
    unsigned short depth;
    GetVideoDataDepth_c(_camID, &depth);
    int mode = GetVideoMode_c(_camID);
    int format = GetVideoFormat_c(_camID);

    InitCamera_c(_camID);

    std::cout << "Initializing camera: "
              << "width: " << width
              << "\nheight: " << height
              << "\ndepth: " << depth
              << "\nmode: " << mode
              << "\nformat: " << format << std::endl;

    _width = width;
    _height = height;
    _image = new Image(width,height,Image::GRAY, Image::Depth8U);

    _initialized = true;
    return true;
}

void CMU1394Camera::stop(){
    bool status = false;
    if(!_started)
    {
        Log::errorLog() << "start() has not been successfully called. \n";
        //return false;
    }

    TimerUtil::sleepMs(CAM_DELAY_SLEEP);

    switch( StopImageCapture_c(_camID) ) // edr: changed from StopImageAcquisition() to StopImageCapture()
    //switch( StopImageAcquisition_c(_camID) ) // edr: changed from StopImageAcquisition() to StopImageCapture()
    {
    case CAM_SUCCESS:
        _started = false;
        status = true;
        break;
    case CAM_ERROR_NOT_INITIALIZED:
        //_started = false;
        Log::errorLog()<< "start() has not been successfully called.";
        break;
    }
    //return status;
}

bool CMU1394Camera::start(){
    bool status = false;
    switch( StartImageCapture_c(_camID) ){
    //switch( StartImageAcquisition_c(_camID) ){
    case CAM_SUCCESS:{
        // capture a dummy image to initialize buffer
        TimerUtil::sleepMs(CAM_DELAY_SLEEP);
        switch(CaptureImage_c(_camID)) // edr: Changed from AcquireImage() to CaptureImage()!
        //switch(AcquireImage_c(_camID)) // edr: Changed from AcquireImage() to CaptureImage()!
        {
            case CAM_SUCCESS:
                status = true;
                break;
            case CAM_ERROR_NOT_INITIALIZED:
                Log::errorLog() << "Camera not initialized";
                break;
            default:
                Log::errorLog() << "Initialization of image buffer failed";
                break;
        }
        _started = true;
        status = true;
        break;
    }
    case CAM_ERROR_NOT_INITIALIZED:
        Log::errorLog() << "Must call Initialize() first.";
        break;
    case CAM_ERROR_INVALID_VIDEO_SETTINGS:
        Log::errorLog() << "The current video settings (format, mode, and/or rate) are not supported.";
        break;
    case CAM_ERROR_BUSY:
        Log::errorLog() << "The camera is already capturing or acquiring images.";
        break;
    case CAM_ERROR_INSUFFICIENT_RESOURCES:
        Log::errorLog() << "Not enough 1394 bus resoures available to complete the operation.";
        break;
    case ERROR_OUTOFMEMORY:
        Log::errorLog() << "Out of memory - Allocation of buffers has failed.";
        break;
    default:
        Log::errorLog() << "Error in device driver.";
        break;
    }
    return status;
}

void CMU1394Camera::acquire(){
	grab();
    _isAquired = true;
}

bool CMU1394Camera::selectCameraFromID(){
    for(size_t i=0;i<_connectedCameras.size();i++){
        CameraID cam = _connectedCameras[i]->getCMUCamera();
        LARGE_INTEGER uniqueID;
        GetCameraUniqueID_c(cam, &uniqueID);
        if( uniqueID.u.LowPart==_uniqueID.u.LowPart &&
                 uniqueID.u.HighPart==_uniqueID.u.HighPart){
            _camID = cam;
            setName( getCameraName(_camID));
            _modelInfo = getCameraVendor(_camID);
            return true;
        }
    }
    return false;
}

bool CMU1394Camera::setWithCurrentValues(){
    if (_queryCameras==false)
        getCameraHandles();

    // now we select the camera with the cameraID that this class has
    if (selectCameraFromID()==false){
        Log::errorLog() << "CCameraFirewireCMU::SetWithCurrentValues() Failed selecting camera.";
        return false;
    }

    if (isStarted()==false)
        return start();
    return true;
}

const rw::sensor::Image* CMU1394Camera::grab(){
    bool status = false;

    if ((_queryCameras==false) /*&& (SettingsInitialized()==true)*/)
    {
        getCameraHandles();
        TimerUtil::sleepMs(CAM_DELAY_SLEEP);
        setWithCurrentValues();
        TimerUtil::sleepMs(CAM_DELAY_SLEEP);
    }else if (/*(SettingsInitialized()==true) &&*/
            (isStarted()==false))
    {
        setWithCurrentValues();
        TimerUtil::sleepMs(CAM_DELAY_SLEEP);
        TimerUtil::sleepMs(CAM_DELAY_SLEEP);
    }
    if (!_initialized){
        Log::errorLog() << "CCameraFirewireCMU::Acquire() Must call Initialize() first";
        return false;
    }

    if(!_started){
        start();
    }
    std::cout << "Capturing image" << std::endl;
    //switch( CaptureImage_c(_camID) )
    switch( AcquireImage_c(_camID) )
    {
    case CAM_SUCCESS:
        status = true;
        break;
    case CAM_ERROR_NOT_INITIALIZED:
        Log::errorLog() << "CCameraFirewireCMU::Acquire() StartImageCapture() has not been successfully called." ;
        break;
    default:
        Log::errorLog() << "CCameraFirewireCMU::Acquire() Error acquiring frame." ;
        break;
    }

    // make sure the image has the right dimensions
    //if ((Image.GetWidth()!=m_Width) || (Image.GetHeight()!=m_Height) || (Image.GetBits()!=m_BitsPerPixel))
    //{
    //    if (Image.AllocFast(m_Width,m_Height,m_BitsPerPixel)==false)
    //    {
    //        Log::errorLog() << "CCameraFirewire::Acquire() Failed allocating image" ;
    //        return false;
    //    }
    //}

    if(status){
    	//std::cout << "Copying image from source...." << std::endl;
        size_t width = _image->getWidth();
        size_t height = _image->getHeight();

        //std::cout << width << ";:" << height << std::endl;

        unsigned long dataLen;
        unsigned char *source = GetRawData_c(_camID, &dataLen);
        //std::cout << dataLen << ">" << width*height << std::endl;
        if( dataLen>width*height ){
            std::cout << "Image is toooo small: " << width << " " << height << " _-> " << dataLen << std::endl;
            // resize image
            unsigned long pWidth,pHeight;
            GetVideoFrameDimensions_c(_camID, &pWidth, &pHeight);
            _image->resize((int)pWidth, (int)pHeight);
        }

        if(_image->getBitsPerPixel() == 8){
            unsigned char *destination = (unsigned char *)_image->getImageData().get();
            for(size_t lineNo = 0; lineNo < height; lineNo++){
                //copies a line at a time
                memcpy(&destination[lineNo*width], &source[(height-lineNo-1)*width], width);
            }
        }
//      else if(m_BitsPerPixel == 24)
//      {
//          unsigned char* tempData=(unsigned char*)malloc(m_Width * m_Height * 3);
//          m_phCamera->YUV411toRGB(tempData);
////            m_phCamera->getDIB(tempData);
//          TImage *destination = Image.GetTImagePtr();
//          unsigned char *source = m_phCamera->m_pData;
//          for(int lineNo = 0; lineNo < m_Height; lineNo++)
//              //copies a line at a time
//              memcpy(destination->ppAllScanLines[lineNo], &tempData[(m_Height-lineNo-1)*m_Width*3], m_Width*3);
//          free(tempData);
//      }
        //else if(m_BitsPerPixel == 24)
        //    m_phCamera->getDIB(Image.GetTImagePtr()->pPixelStream);
    }
    return _image;

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
    int frameRateIdx = GetVideoFrameRate_c(_camID);
    return FrameRates[frameRateIdx];
}

void CMU1394Camera::setFrameRate(double framerate){
    unsigned long closestIdx=0;
    double dist = fabs(framerate-FrameRates[closestIdx]);
    for(int i=1;i<6;i++){
        double tmpdist = fabs(framerate-FrameRates[i]);
        if(tmpdist<dist){
            dist = tmpdist;
            closestIdx = i;
        }
    }
    SetVideoFrameRate_c(_camID, closestIdx);
    //_frameRate = framerate;

}

CameraFirewire::CaptureMode CMU1394Camera::getCaptureMode()
{
    return _captureMode;
}

bool CMU1394Camera::setCaptureMode(CaptureMode mode){
    _captureMode = mode;
    return true;
}

void CMU1394Camera::close(){
    stop();
    /*if (ImageCaptureStarted()==true)
        StopImageCapture();
    else if (ImageAcquisitionExtendedStarted()==true)
        StopImageAcquisitionExtended();
    m_Initialized = false;
    */
}

const std::vector<CMU1394Camera*> CMU1394Camera::getCameraHandles()
{
    // first close all existing cameras
    for(int i=0; i<_connectedCameras.size(); i++)
        _connectedCameras[i]->close();

    CameraID camID = createCameraInstance_c();
    //C1394Camera CamClass;
    CheckLink_c( camID ); //Refresh Camera List
    RefreshCameraList_c( camID );
    int totalCams = GetNumberCameras_c(camID);

    for(int i=0; i<_connectedCameras.size();i++)
    {
        //delete m_ConnectedCams[i]->m_phCamera;
        delete _connectedCameras[i];
    }
    _connectedCameras.resize(totalCams);

    //collect information for each camera
    for(int node = 0; node < totalCams; node++){
        CameraID currentCamID = createCameraInstance_c();
        std::ostringstream ost;
        CheckLink_c(currentCamID);

        if(SelectCamera_c(currentCamID, node) != CAM_SUCCESS){
            RW_THROW("failed querying node");
        }

        InitCamera_c(currentCamID);
        //InquireControlRegisters_c(currentCamID);

        std::string cameraName = getCameraName(currentCamID);
        std::string vendorName = getCameraVendor(currentCamID);

        CMU1394Camera *cam = new CMU1394Camera(currentCamID, cameraName, vendorName);

        _connectedCameras[node] = cam;
    }
    _queryCameras = true;
   //return totalCams;

//    C1394Camera cam;
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

    return _connectedCameras;
}

CameraFirewire::CapturePolicy CMU1394Camera::getCapturePolicy()
{
    return _policy;
}


bool CMU1394Camera::isShutterAvailable() const{
	return HasFeature_c(_camID, FEATURE_SHUTTER);
}

double CMU1394Camera::getShutter() const{
	CONTROLHANDLE handle = GetCameraControl_c(_camID, FEATURE_SHUTTER);
	if(handle==NULL)
		RW_THROW("Shutter not supported!");

	Inquire_c(handle);

	SetAutoMode_c(handle, FALSE);
	SetAbsControl_c(handle, TRUE);

	std::cout << "HasAbsControl_c(handle): " << HasAbsControl_c(handle) << std::endl;
	std::cout << "HasOnOff_c(handle): " << HasOnOff_c(handle) << std::endl;
	std::cout << "HasAutoMode_c(handle): " << HasAutoMode_c(handle) << std::endl;
	std::cout << "HasManualMode_c(handle): " << HasManualMode_c(handle) << std::endl;

	unsigned short val=1;
	GetValue_c(handle, &val);
	return val;
}

void CMU1394Camera::setShutter(double Value){
	CONTROLHANDLE handle = GetCameraControl_c(_camID, FEATURE_SHUTTER);
	if(handle==NULL)
		RW_THROW("Shutter not supported!");
	float val = (float)Value;
	SetValueAbsolute_c(handle, val);
}

std::pair<double,double> CMU1394Camera::getShutterBounds() const{
	CONTROLHANDLE handle = GetCameraControl_c(_camID, FEATURE_SHUTTER);
	if(handle==NULL)
		RW_THROW("Shutter not supported!");
	unsigned short min, max;
	GetRange_c(handle, &min, &max);
	return std::make_pair((double)min,(double)max);
}
