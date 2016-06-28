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

#include "DC1394Camera.hpp"
#include <rw/math/Transform3D.hpp>
#include "DC1394CameraFactory.hpp"

using namespace rw::sensor;
using namespace rw::math;
using namespace rw::kinematics;

#define MAX_CAMERAS 12
#define MAX_PORTS 12

/* uncomment the following to drop frames to prevent delays */
#define DROP_FRAMES 0


namespace rwhw { namespace camera {

DC1394Camera::DC1394Camera(
    dc1394camera_t* dc1394cam)
    :
    CameraFirewire(dc1394cam->model, dc1394cam->vendor),
    _policy(SINGLE_SHOT),
    _isAcquired(false),
    _dccamera(dc1394cam),
    _captureMode(M640x480),
    _colorMode(MONO8),
    _frameRate(DC1394_FRAMERATE_30),
    _bufferSize(10)
{
	_initialized=false;
	_started=false;
	_f7PosLeft=0;
	_f7PosTop=0;
}

DC1394Camera::~DC1394Camera()
{
	stop();
	DC1394CameraFactory::getInstance()->freeCamera(this);
}

std::string DC1394Camera::getCameraName()
{
   return std::string(_dccamera->model);
}

std::string DC1394Camera::getCameraVendor()
{
    return std::string(_dccamera->vendor);
}

bool DC1394Camera::initialize(){
	dc1394video_mode_t res = DC1394_VIDEO_MODE_640x480_MONO8;
	_initialized=false;
	unsigned int packetSize=0;

//	if(dc1394_video_set_operation_mode(_dccamera,DC1394_OPERATION_MODE_1394B) != DC1394_SUCCESS) {
//		fprintf(stderr,"Unable to setup camera in OPERATION_MODE_1394B\n");
//		return false;
//	}

//	if(dc1394_video_set_iso_speed(_dccamera, DC1394_ISO_SPEED_800) != DC1394_SUCCESS){
//		fprintf(stderr,"Unable to setup camera speed\n");
//		return false;
//	}

//	//soft reset cameras
//	/* ON / OFF : Bit 6 */
//	dc1394_set_adv_control_register(_dccamera,REG_CAMERA_AVT_SOFT_RESET,(1<<25));
//
//	printf("\nResetting(soft) camera!\n");
//	sleep(3);

    switch(_captureMode){
    case(M160x120):
		switch(_colorMode){
		case(YUV444):	res = DC1394_VIDEO_MODE_160x120_YUV444;	break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M320x240):
		switch(_colorMode){
		case(YUV422):	res = DC1394_VIDEO_MODE_320x240_YUV422;	break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M640x480):
		switch(_colorMode){
		case(YUV411):	res = DC1394_VIDEO_MODE_640x480_YUV411;	break;
		case(YUV422):	res = DC1394_VIDEO_MODE_640x480_YUV422;	break;
		case(MONO8):	res = DC1394_VIDEO_MODE_640x480_MONO8;	break;
		case(MONO16):	res = DC1394_VIDEO_MODE_640x480_MONO16; break;
		case(RGB8):		res = DC1394_VIDEO_MODE_640x480_RGB8;	break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M800x600):
		switch(_colorMode){
		case(YUV422):	res = DC1394_VIDEO_MODE_800x600_YUV422;	break;
		case(MONO8):	res = DC1394_VIDEO_MODE_800x600_MONO8;	break;
		case(MONO16):	res = DC1394_VIDEO_MODE_800x600_MONO16; break;
		case(RGB8):		res = DC1394_VIDEO_MODE_800x600_RGB8;	break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M1024x768):
		switch(_colorMode){
		case(YUV422):	res = DC1394_VIDEO_MODE_1024x768_YUV422;	break;
		case(MONO8):	res = DC1394_VIDEO_MODE_1024x768_MONO8;		break;
		case(MONO16):	res = DC1394_VIDEO_MODE_1024x768_MONO16; 	break;
		case(RGB8):		res = DC1394_VIDEO_MODE_1024x768_RGB8;		break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M1280x960):
		switch(_colorMode){
		case(YUV422):	res = DC1394_VIDEO_MODE_1280x960_YUV422;	break;
		case(MONO8):	res = DC1394_VIDEO_MODE_1280x960_MONO8;		break;
		case(MONO16):	res = DC1394_VIDEO_MODE_1280x960_MONO16; 	break;
		case(RGB8):		res = DC1394_VIDEO_MODE_1280x960_RGB8;		break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(M1600x1200):
		switch(_colorMode){
		case(YUV422):	res = DC1394_VIDEO_MODE_1600x1200_YUV422;	break;
		case(MONO8):	res = DC1394_VIDEO_MODE_1600x1200_MONO8;	break;
		case(MONO16):	res = DC1394_VIDEO_MODE_1600x1200_MONO16; 	break;
		case(RGB8):		res = DC1394_VIDEO_MODE_1600x1200_RGB8;		break;
		default: 		RW_WARN("Unable to setup camera resolution with selected color mode."); return false;
		}
		break;

    case(CameraFirewire::MFORMAT7):
		res=modeConverter(_f7Mode);
		if(!getFormat7RecommendedPacketSize(_f7Mode, packetSize)) {
			return false;
		}
		//if(dc1394_format7_set_color_coding(_dccamera, (dc1394video_mode_t) res, ColorCodeConverter(_colorMode)) != DC1394_SUCCESS){
		//	RW_WARN("Unable to send the selected Format7 color mode");
		//	return false;
		//}
		if(dc1394_format7_set_roi(_dccamera, res, ColorCodeConverter(_colorMode), packetSize, _f7PosLeft, _f7PosTop, _f7width, _f7height)!= DC1394_SUCCESS){
			RW_WARN("Unable to send the selected Format7 resolution, color mode, image size and position\nCheck the settings and connection!");
			return false;
		}
		std::cout << "dc1394 Format7 set "<< _f7width << "x" << _f7height << " Pos: " << _f7PosLeft << "x" << _f7PosTop<< std::endl;
		break;

    default: 		RW_WARN("Unknown resolution selected."); return false;
    }

    std::cout << "set video mode" << std::endl;
	if(dc1394_video_set_mode(_dccamera, res)!= DC1394_SUCCESS){
		RW_WARN("Unable to send the selected resolution and color mode\nCheck the settings and connection!");
		return false;
	}

//	//something with the package size!!
//	unsigned int pmin,pmax,pact;
//	if(dc1394_format7_get_packet_parameters(_dccamera, (dc1394video_mode_t) res, &pmin, &pmax) != DC1394_SUCCESS){
//		RW_WARN("Unable to get packet size parameters!");
//		return false;
//	}
//
//	if(dc1394_format7_get_packet_size(_dccamera, (dc1394video_mode_t) res, &pact)){
//		RW_WARN("Unable to get packet size!");
//		return false;
//	}
//	printf("min Packet Size = %d\n",pmin);
//	printf("max Packet Size = %d\n",pmax);
//	printf("act Packet Size = %d\n",pact);

	_initialized=true;
	return _initialized;
}

void DC1394Camera::stop()
{
    if(!_started)
        return;

    if(_policy!=SINGLE_SHOT){
    	if(dc1394_video_set_transmission(_dccamera, DC1394_OFF)!=DC1394_SUCCESS){
            RW_WARN("Unable to stop camera transmission.");
            return;
        }
    }

    if( dc1394_capture_stop(_dccamera)!=DC1394_SUCCESS){
		RW_WARN("Unable to stop capturing.");
		return;
    }

    delete _image;
    _image = 0;

    _started = false;
}

bool DC1394Camera::start()
{
	std::cout << "DC1394Camera start, stared=" << _started << " initialized="<< _initialized;
    if(_started)
        return false;
    if(!_initialized)
        return false;

    std::cout << "Check passed!" << std::endl;

    _width=0,_height=0;
    CaptureMode mode = getCaptureMode();
    switch(mode){
    case(M160x120): _width=160; _height=120; break;
    case(M320x240): _width=320; _height=240; break;
    case(M640x480): _width=640; _height=480; break;
    case(M800x600): _width=800; _height=600; break;
    case(M1024x768): _width=1024; _height=768; break;
    case(M1280x960): _width=1280; _height=960; break;
    case(M1600x1200): _width=1600; _height=1200; break;
    case(MFORMAT7):
    		getFormat7ImageSize(_f7Mode,_width,_height); break;
    default:
    	RW_WARN("Unsupported capturing mode.");
        return false;
    }

    std::cout << "dc1394 start: image size " << _width << "x" << _height << std::endl;

    Image::ColorCode encoding =Image::GRAY;
    Image::PixelDepth depth = Image::Depth8U;
    switch(getColorMode()) {
    case(CameraFirewire::MONO8):	encoding = Image::GRAY; depth = Image::Depth8U;		break;
    case(CameraFirewire::MONO16):	encoding = Image::GRAY; depth = Image::Depth16U;	break;
    case(CameraFirewire::MONO16S):	encoding = Image::GRAY; depth = Image::Depth16S;	break;
    case(CameraFirewire::RGB8): 	encoding = Image::RGB; 	depth = Image::Depth8U;		break;
    case(CameraFirewire::RGB16): 	encoding = Image::RGB; 	depth = Image::Depth16U;	break;
    case(CameraFirewire::RGB16S): 	encoding = Image::RGB; 	depth = Image::Depth16S;	break;
    case(CameraFirewire::RGB24): 	encoding = Image::RGB; 	depth = Image::Depth32S;	break;
    case(CameraFirewire::YUV444):	encoding = Image::RGB; 	depth = Image::Depth8U;		break;
    case(CameraFirewire::YUV411):	encoding = Image::RGB; 	depth = Image::Depth8U;		break;
    case(CameraFirewire::YUV422):	encoding = Image::RGB; 	depth = Image::Depth8U;		break;
    case(CameraFirewire::RAW8):		encoding = Image::RGB; 	depth = Image::Depth8U;		break;
    case(CameraFirewire::RAW16):	encoding = Image::RGB; 	depth = Image::Depth16U;	break;
    default:
    	std::cout << " No valid image mode selected - mode: " << getColorMode()<< std::endl;
    	return false;
    }

//    unsigned int buffer = 0;
//    if(_policy!=CONTINUES_BUFFERED){
//    	buffer=10;
//    }


	/* Setup capture */
	if(dc1394_capture_setup(_dccamera, _bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS )
	{
		RW_WARN(
		                "Unable to setup camera for DMA capture.\n"
		                "Perhaps the framerate or videomode is not supported by the camera!!"
				);
		std::cout << "Camera initializing return false" << std::endl;
		return false;
	}

	if(_policy!=SINGLE_SHOT){
		if(dc1394_video_set_transmission(_dccamera, DC1394_ON)!=DC1394_SUCCESS){
			RW_WARN("Unable to stop camera transmission.");
			return false;
		}
	}

    _image = new Image(_width,_height,encoding,depth);
    _started = true;
    return true;
}

void DC1394Camera::acquire()
{
	switch(_policy){
		case(SINGLE_SHOT):
			acquireOneShot();
			break;
		case(CONTINUES):
		case(CONTINUES_BUFFERED):
			acquireContinues();
			break;
		default:
			RW_WARN("Unknown capture policy selected.");
			break;
	}
}

void DC1394Camera::acquireOneShot()
{
	dc1394video_frame_t* frame;

    if(!_started)
        return;

    _isAcquired = false;

    if(_dccamera->one_shot_capable) {
    	if(dc1394_video_set_one_shot(_dccamera, DC1394_ON)!=DC1394_SUCCESS)
		{
			 RW_WARN("Unable to start one shot mode.");
			 return;
		}

    	if(dc1394_capture_dequeue(_dccamera, DC1394_CAPTURE_POLICY_WAIT, &frame)!=DC1394_SUCCESS)
		{
			RW_WARN("Unable to dequeue a frame.");
			return;
		}

    	memcpy(
			_image->getImageData(),
			frame->image,
			_image->getDataSize());

    	if(dc1394_capture_enqueue(_dccamera, frame)!=DC1394_SUCCESS){
			RW_WARN("Unable to free camera frame.");
			return;
		}

    }
    else {
		if(dc1394_video_set_transmission(_dccamera, DC1394_ON)!=DC1394_SUCCESS) {
			 RW_WARN("Unable to start camera iso transmission.");
			 return;
		}

		if(dc1394_video_set_transmission(_dccamera, DC1394_OFF)!=DC1394_SUCCESS) {
			RW_WARN("Unable to stop camera transmission.");
			return;
		}

		if(dc1394_capture_dequeue(_dccamera, DC1394_CAPTURE_POLICY_WAIT, &frame)!=DC1394_SUCCESS) {
			RW_WARN("Unable to dequeue a frame.");
			return;
		}

		memcpy(
				_image->getImageData(),
				frame->image,
				_image->getDataSize());

		if(dc1394_capture_enqueue(_dccamera, frame)!=DC1394_SUCCESS) {
			RW_WARN("Unable to free camera frame.");
			return;
		}


    }

    _isAcquired = true;
}

void DC1394Camera::acquireContinues()
{
	dc1394video_frame_t* frame;

    if(!_started)
        return;

    _isAcquired = false;

	if(dc1394_capture_dequeue(_dccamera, DC1394_CAPTURE_POLICY_WAIT, &frame)!=DC1394_SUCCESS) {
		RW_WARN("Unable to dequeue a frame.");
		return;
	}

	memcpy(
			_image->getImageData(),
			frame->image,
			_image->getDataSize());

	if(dc1394_capture_enqueue(_dccamera, frame)!=DC1394_SUCCESS) {
		RW_WARN("Unable to free camera frame.");
		return;
	}

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

    _isAcquired=false;
    return _image;
}

double DC1394Camera::getFrameRate()
{
	dc1394framerate_t frameRate=DC1394_FRAMERATE_1_875;
	double res=0;
	if(dc1394_video_get_framerate(_dccamera, &frameRate)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read framerate from camera.");
	}
    switch(_frameRate){
    case(DC1394_FRAMERATE_1_875): res = 1.875; break;
    case(DC1394_FRAMERATE_3_75): res = 3.75; break;
    case(DC1394_FRAMERATE_7_5): res = 7.5; break;
    case(DC1394_FRAMERATE_15): res = 15.0; break;
    case(DC1394_FRAMERATE_30): res = 30.0; break;
    case(DC1394_FRAMERATE_60): res = 60.0; break;
    case(DC1394_FRAMERATE_120): res = 120.0; break;
    case(DC1394_FRAMERATE_240): res = 240.0; break;
    default: res = 0;
    }
    return res;
}

void DC1394Camera::setFrameRate(double framerate)
{
	dc1394framerate_t res = DC1394_FRAMERATE_1_875;
    if(framerate==1.875)  res = DC1394_FRAMERATE_1_875;
    else if(framerate==3.75) res = DC1394_FRAMERATE_3_75;
    else if(framerate==7.5) res = DC1394_FRAMERATE_7_5;
    else {
        int fps = (int)framerate;
        switch(fps){
        case(15): res = DC1394_FRAMERATE_15; break;
        case(30): res = DC1394_FRAMERATE_30; break;
        case(60): res = DC1394_FRAMERATE_60; break;
        case(120): res = DC1394_FRAMERATE_120; break;
        case(240): res = DC1394_FRAMERATE_240; break;
        default: res = DC1394_FRAMERATE_1_875;
        }
    }

    if(dc1394_video_get_framerate(_dccamera, &res)!=DC1394_SUCCESS) {
		RW_WARN("Unable to set framerate to camera.");
	}
}

CameraFirewire::CaptureMode DC1394Camera::getCaptureMode()
{
    CaptureMode mode = M160x120;
    dc1394video_mode_t captureMode;
    if(dc1394_video_get_mode(_dccamera, &captureMode)!=DC1394_SUCCESS) {
    	RW_WARN("Unable to read mode from camera.");
    }

    switch(captureMode){
    case(DC1394_VIDEO_MODE_160x120_YUV444): mode = M160x120; break;
    case(DC1394_VIDEO_MODE_320x240_YUV422): mode = M320x240; break;
    case(DC1394_VIDEO_MODE_640x480_YUV411):
    case(DC1394_VIDEO_MODE_640x480_YUV422):
    case(DC1394_VIDEO_MODE_640x480_RGB8):
    case(DC1394_VIDEO_MODE_640x480_MONO8):
    case(DC1394_VIDEO_MODE_640x480_MONO16): mode = M640x480; break;
    case(DC1394_VIDEO_MODE_800x600_YUV422):
    case(DC1394_VIDEO_MODE_800x600_RGB8):
    case(DC1394_VIDEO_MODE_800x600_MONO8):
    case(DC1394_VIDEO_MODE_800x600_MONO16): mode = M800x600; break;
    case(DC1394_VIDEO_MODE_1024x768_YUV422):
    case(DC1394_VIDEO_MODE_1024x768_RGB8):
    case(DC1394_VIDEO_MODE_1024x768_MONO8):
    case(DC1394_VIDEO_MODE_1024x768_MONO16): mode = M1024x768; break;
    case(DC1394_VIDEO_MODE_1280x960_YUV422):
    case(DC1394_VIDEO_MODE_1280x960_RGB8):
    case(DC1394_VIDEO_MODE_1280x960_MONO8):
    case(DC1394_VIDEO_MODE_1280x960_MONO16): mode = M1280x960; break;
    case(DC1394_VIDEO_MODE_1600x1200_YUV422):
    case(DC1394_VIDEO_MODE_1600x1200_RGB8):
    case(DC1394_VIDEO_MODE_1600x1200_MONO8):
    case(DC1394_VIDEO_MODE_1600x1200_MONO16): mode = M1600x1200; break;

    case DC1394_VIDEO_MODE_FORMAT7_0:
	case DC1394_VIDEO_MODE_FORMAT7_1:
	case DC1394_VIDEO_MODE_FORMAT7_2:
	case DC1394_VIDEO_MODE_FORMAT7_3:
	case DC1394_VIDEO_MODE_FORMAT7_4:
	case DC1394_VIDEO_MODE_FORMAT7_5:
	case DC1394_VIDEO_MODE_FORMAT7_6:
	case DC1394_VIDEO_MODE_FORMAT7_7:	mode = MFORMAT7;
	std::cout << "getCaptureMode=Format7 - "<<  mode << std::endl; break;

    case DC1394_VIDEO_MODE_EXIF:
    default:
        RW_THROW("Unsupported capturemode!!");
    }
    return mode;
}

CameraFirewire::ColorCode DC1394Camera::getColorMode() {
	dc1394video_mode_t mode = DC1394_VIDEO_MODE_640x480_MONO8;
	CameraFirewire::ColorCode colorCode = CameraFirewire::MONO8;
	if(dc1394_video_get_mode(_dccamera, &mode)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read mode from camera.");
	}
	switch(mode) {
	case DC1394_VIDEO_MODE_640x480_MONO8:
	case DC1394_VIDEO_MODE_800x600_MONO8:
	case DC1394_VIDEO_MODE_1024x768_MONO8:
	case DC1394_VIDEO_MODE_1280x960_MONO8:
	case DC1394_VIDEO_MODE_1600x1200_MONO8: colorCode= MONO8; break;

	case DC1394_VIDEO_MODE_640x480_MONO16:
	case DC1394_VIDEO_MODE_800x600_MONO16:
	case DC1394_VIDEO_MODE_1024x768_MONO16:
	case DC1394_VIDEO_MODE_1280x960_MONO16:
	case DC1394_VIDEO_MODE_1600x1200_MONO16: colorCode= MONO16; break;

	case DC1394_VIDEO_MODE_640x480_RGB8:
	case DC1394_VIDEO_MODE_800x600_RGB8:
	case DC1394_VIDEO_MODE_1024x768_RGB8:
	case DC1394_VIDEO_MODE_1280x960_RGB8:
	case DC1394_VIDEO_MODE_1600x1200_RGB8: colorCode= RGB8; break;

	case DC1394_VIDEO_MODE_160x120_YUV444: colorCode= YUV444; break;

	case DC1394_VIDEO_MODE_640x480_YUV411: colorCode= YUV411; break;

	case DC1394_VIDEO_MODE_320x240_YUV422:
	case DC1394_VIDEO_MODE_640x480_YUV422:
	case DC1394_VIDEO_MODE_800x600_YUV422:
	case DC1394_VIDEO_MODE_1024x768_YUV422:
	case DC1394_VIDEO_MODE_1280x960_YUV422: colorCode= YUV422; break;

	case DC1394_VIDEO_MODE_FORMAT7_0:
	case DC1394_VIDEO_MODE_FORMAT7_1:
	case DC1394_VIDEO_MODE_FORMAT7_2:
	case DC1394_VIDEO_MODE_FORMAT7_3:
	case DC1394_VIDEO_MODE_FORMAT7_4:
	case DC1394_VIDEO_MODE_FORMAT7_5:
	case DC1394_VIDEO_MODE_FORMAT7_6:
	case DC1394_VIDEO_MODE_FORMAT7_7:	getFormat7ColorCoding(colorCode); break;

	case DC1394_VIDEO_MODE_EXIF:
	default:
		RW_THROW("Unsupported colormode!!");
		break;
	}

	return colorCode;
}

bool DC1394Camera::setCaptureMode(CameraFirewire::CaptureMode mode)
{
    _captureMode = mode;
    return true;
}

bool DC1394Camera::setColorMode(CameraFirewire::ColorCode ColorCode)
{
    _colorMode = ColorCode;
    return true;
}

CameraFirewire::CapturePolicy DC1394Camera::getCapturePolicy()
{
    return _policy;
}

bool DC1394Camera::setCapturePolicy(CameraFirewire::CapturePolicy policy)
{
	if(_started){
		RW_WARN("Unable to change capture policy while the camera are started");
		return false;
	}
	_policy=policy;
	return true;
}

bool DC1394Camera::setBufferSize(unsigned int bufferSize)
{
	_bufferSize = bufferSize;
	return true;
}

dc1394feature_t DC1394Camera::settingsConverter(CameraFirewire::CameraFeature setting) {
	switch (setting) {
		case CameraFirewire::SHUTTER   	 	: return DC1394_FEATURE_SHUTTER;
		case CameraFirewire::ZOOM    		: return DC1394_FEATURE_ZOOM;
		case CameraFirewire::GAIN    		: return DC1394_FEATURE_GAIN;
		case CameraFirewire::FOCUS    		: return DC1394_FEATURE_FOCUS;
		case CameraFirewire::IRIS    		: return DC1394_FEATURE_IRIS;
		case CameraFirewire::HUE    		: return DC1394_FEATURE_HUE;
		case CameraFirewire::WHITEBALANCE   : return DC1394_FEATURE_WHITE_BALANCE;
		case CameraFirewire::SHARPNESS    	: return DC1394_FEATURE_SHARPNESS;
		case CameraFirewire::SATURATION    	: return DC1394_FEATURE_SATURATION;
		case CameraFirewire::GAMMA    		: return DC1394_FEATURE_GAMMA;
		case CameraFirewire::BRIGHTNESS    	: return DC1394_FEATURE_BRIGHTNESS;
		case CameraFirewire::AUTOEXPOSURE   : return DC1394_FEATURE_EXPOSURE;
//		case CameraFirewire::TEMPERATURE    : return DC1394_FEATURE_TEMPERATURE;
//		case CameraFirewire::TRIGGER    	: return DC1394_FEATURE_TRIGGER;
//		case CameraFirewire::TRIGGER_DELAY  : return DC1394_FEATURE_TRIGGER_DELAY;
//		case CameraFirewire::WHITE_SHADING  : return DC1394_FEATURE_WHITE_SHADING;
//		case CameraFirewire::FRAME_RATE    	: return DC1394_FEATURE_FRAME_RATE;
//		case CameraFirewire::PAN    		: return DC1394_FEATURE_PAN;
//		case CameraFirewire::TILT    		: return DC1394_FEATURE_TILT;
//		case CameraFirewire::OPTICAL_FILTER : return DC1394_FEATURE_OPTICAL_FILTER;
//		case CameraFirewire::CAPTURE_SIZE   : return DC1394_FEATURE_CAPTURE_SIZE;
//		case CameraFirewire::CAPTURE_QUALITY: return DC1394_FEATURE_CAPTURE_QUALITY;
		default:
			RW_THROW("Receive a unsupported feature, return GAIN instead");
			return DC1394_FEATURE_GAIN;
	}
}

CameraFirewire::CameraFeature DC1394Camera::settingsConverter(dc1394feature_t setting) {
	switch (setting) {
		case DC1394_FEATURE_SHUTTER   	 	: return CameraFirewire::SHUTTER;
		case DC1394_FEATURE_ZOOM    		: return CameraFirewire::ZOOM;
		case DC1394_FEATURE_GAIN    		: return CameraFirewire::GAIN;
		case DC1394_FEATURE_FOCUS    		: return CameraFirewire::FOCUS;
		case DC1394_FEATURE_IRIS    		: return CameraFirewire::IRIS;
		case DC1394_FEATURE_HUE    			: return CameraFirewire::HUE;
		case DC1394_FEATURE_WHITE_BALANCE   : return CameraFirewire::WHITEBALANCE;
		case DC1394_FEATURE_SHARPNESS    	: return CameraFirewire::SHARPNESS;
		case DC1394_FEATURE_SATURATION    	: return CameraFirewire::SATURATION;
		case DC1394_FEATURE_GAMMA    		: return CameraFirewire::GAMMA;
		case DC1394_FEATURE_BRIGHTNESS    	: return CameraFirewire::BRIGHTNESS;
		case DC1394_FEATURE_EXPOSURE   		: return CameraFirewire::AUTOEXPOSURE;
//		case DC1394_FEATURE_TEMPERATURE    : return DC1394_FEATURE_TEMPERATURE;
//		case DC1394_FEATURE_TRIGGER    	: return DC1394_FEATURE_TRIGGER;
//		case DC1394_FEATURE_TRIGGER_DELAY  : return DC1394_FEATURE_TRIGGER_DELAY;
//		case DC1394_FEATURE_WHITE_SHADING  : return DC1394_FEATURE_WHITE_SHADING;
//		case DC1394_FEATURE_FRAME_RATE    	: return DC1394_FEATURE_FRAME_RATE;
//		case DC1394_FEATURE_PAN    		: return DC1394_FEATURE_PAN;
//		case DC1394_FEATURE_TILT    		: return DC1394_FEATURE_TILT;
//		case DC1394_FEATURE_OPTICAL_FILTER : return DC1394_FEATURE_OPTICAL_FILTER;
//		case DC1394_FEATURE_CAPTURE_SIZE   : return DC1394_FEATURE_CAPTURE_SIZE;
//		case DC1394_FEATURE_CAPTURE_QUALITY: return DC1394_FEATURE_CAPTURE_QUALITY;
		default:
			RW_WARN("Receive a unsupported feature, return GAIN instead");
			return CameraFirewire::GAIN;
	}
}

double DC1394Camera::getFeature(CameraFirewire::CameraFeature setting) {
	uint32_t value = 0;
    if(dc1394_feature_get_value(_dccamera, settingsConverter(setting), &value)!=DC1394_SUCCESS) {
    		RW_WARN("Unable to read a feature setting from camera. return -1");
    		return -1;
    }
    return (double) value;
}

bool DC1394Camera::setFeature(CameraFirewire::CameraFeature setting, double value) {
  return setFeature(setting, std::vector<double>(1, value));
}

bool DC1394Camera::setFeature(CameraFirewire::CameraFeature setting, std::vector<double> values) {
  if(!values.size()) {
    RW_WARN("Empty feature vector.");
    return false;
  }

  //Cast the input feature vector
  std::vector<uint32_t> vals(values.size());
  for(unsigned int i = 0; i < values.size(); ++i)
    vals[i] = (uint32_t)(values[i]+0.5);

	//Turn the feature on
	dc1394bool_t is_switchable = DC1394_FALSE;
	if(dc1394_feature_is_switchable(_dccamera, settingsConverter(setting), &is_switchable)!=DC1394_SUCCESS) {
			RW_WARN("Unable to read if the feature is switchable.");
			return false;
	}
	if(is_switchable==DC1394_TRUE) {
		if(dc1394_feature_set_power(_dccamera, settingsConverter(setting), DC1394_ON)!=DC1394_SUCCESS) {
				RW_WARN("Unable to turn feature on.");
				return false;
		}
	}

	//Set the feature to manually mode
	if(dc1394_feature_set_mode(_dccamera, settingsConverter(setting), DC1394_FEATURE_MODE_MANUAL)!=DC1394_SUCCESS) {
			RW_WARN("Unable to enable feature to be manually adjusted.");
			return false;
	}

	//Set the feature value
  switch(setting) {
    case WHITEBALANCE:
      if(vals.size() != 2) {
        RW_WARN("Invalid feature vector size for setting white balance.");
        return false;
      }
      return dc1394_feature_whitebalance_set_value(_dccamera, vals[0], vals[1]) == DC1394_SUCCESS;
    default:
      return dc1394_feature_set_value(_dccamera, settingsConverter(setting), vals[0]) == DC1394_SUCCESS;
  }

  RW_WARN("Unable to set a feature setting to camera.");

  return false;
}

bool DC1394Camera::setFormat7Mode(Format7Mode mode){
	_f7Mode=mode;
	_captureMode = MFORMAT7;
	return true;
}

dc1394video_mode_t DC1394Camera::modeConverter(CameraFirewire::Format7Mode color) {
	switch(color) {
	case CameraFirewire::F7MODE0: 	return DC1394_VIDEO_MODE_FORMAT7_0;
	case CameraFirewire::F7MODE1:	return DC1394_VIDEO_MODE_FORMAT7_1;
	case CameraFirewire::F7MODE2:	return DC1394_VIDEO_MODE_FORMAT7_2;
	case CameraFirewire::F7MODE3:	return DC1394_VIDEO_MODE_FORMAT7_3;
	case CameraFirewire::F7MODE4:	return DC1394_VIDEO_MODE_FORMAT7_4;
	case CameraFirewire::F7MODE5:	return DC1394_VIDEO_MODE_FORMAT7_5;
	case CameraFirewire::F7MODE6:	return DC1394_VIDEO_MODE_FORMAT7_6;
	case CameraFirewire::F7MODE7:	return DC1394_VIDEO_MODE_FORMAT7_7;

	default:
		RW_THROW("Received unsupported Format 7 mode!!");
		return DC1394_VIDEO_MODE_FORMAT7_0;
	}
}

CameraFirewire::Format7Mode DC1394Camera::modeConverter(dc1394video_mode_t color) {
	switch(color) {
	case DC1394_VIDEO_MODE_FORMAT7_0: 	return CameraFirewire::F7MODE0;
	case DC1394_VIDEO_MODE_FORMAT7_1:	return CameraFirewire::F7MODE1;
	case DC1394_VIDEO_MODE_FORMAT7_2:	return CameraFirewire::F7MODE2;
	case DC1394_VIDEO_MODE_FORMAT7_3:	return CameraFirewire::F7MODE3;
	case DC1394_VIDEO_MODE_FORMAT7_4:	return CameraFirewire::F7MODE4;
	case DC1394_VIDEO_MODE_FORMAT7_5:	return CameraFirewire::F7MODE5;
	case DC1394_VIDEO_MODE_FORMAT7_6:	return CameraFirewire::F7MODE6;
	case DC1394_VIDEO_MODE_FORMAT7_7:	return CameraFirewire::F7MODE7;

	default:
		RW_THROW("Received unsupported Format 7 mode!!");
		return CameraFirewire::F7MODE0;
	}
}

CameraFirewire::Format7Mode DC1394Camera::getFormat7Mode()
{
	//CameraFirewire::Format7Mode mode = F7MODE0;
    dc1394video_mode_t captureMode;
    if(dc1394_video_get_mode(_dccamera, &captureMode)!=DC1394_SUCCESS) {
    	RW_WARN("Unable to read mode from camera.");
    }

    return modeConverter(captureMode);
}



bool DC1394Camera::setFormat7ImageSize(const unsigned int width, const unsigned int heigth){
	_f7width=width;
	_f7height= heigth;
	return true;
}

bool DC1394Camera::getFormat7ImageSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth){
	if(dc1394_format7_get_image_size(_dccamera, modeConverter(mode), &width, &heigth)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read size from camera.");
		return false;
	}

	return true;
}

bool DC1394Camera::setFormat7ImageSizeToMax(const CameraFirewire::Format7Mode mode){
	unsigned int width, heigth;
	if(!getFormat7ImageMaxSize(mode, width, heigth))
		return false;

	if(!setFormat7ImageSize(width, heigth))
		return false;

	setFormat7Mode(mode);
	return true;
}

bool DC1394Camera::getFormat7ImageMaxSize(const CameraFirewire::Format7Mode mode, unsigned int &width, unsigned int &heigth){
	if(dc1394_format7_get_max_image_size(_dccamera, modeConverter(mode), &width, &heigth)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read max size from camera.");
		return false;
	}

	std::cout << "Received max size: " << width << "x" << heigth << std::endl;
	return true;
}

bool DC1394Camera::setFormat7ImagePos(const unsigned int left, const unsigned int top){
	_f7PosLeft =left;
	_f7PosTop  = top;
	return true;
}

bool DC1394Camera::getFormat7ImagePos(unsigned int &left, unsigned int &top){
	dc1394video_mode_t captureMode;
	if(dc1394_video_get_mode(_dccamera, &captureMode)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read mode from camera.");
		return false;
	}

	if(dc1394_format7_get_image_position(_dccamera, captureMode, &left, &top)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read image position from camera.");
		return false;
	}
	return true;
}

dc1394color_coding_t DC1394Camera::ColorCodeConverter(const CameraFirewire::ColorCode color) {
	switch(color) {
	case CameraFirewire::MONO8: 	return  DC1394_COLOR_CODING_MONO8;
	case CameraFirewire::YUV411:	return DC1394_COLOR_CODING_YUV411;
	case CameraFirewire::YUV422:	return DC1394_COLOR_CODING_YUV422;
	case CameraFirewire::YUV444:	return DC1394_COLOR_CODING_YUV444;
	case CameraFirewire::RGB8:		return DC1394_COLOR_CODING_RGB8;
	case CameraFirewire::MONO16:	return DC1394_COLOR_CODING_MONO16;
	case CameraFirewire::RGB16:		return DC1394_COLOR_CODING_RGB16;
	case CameraFirewire::MONO16S:	return DC1394_COLOR_CODING_MONO16S;
	case CameraFirewire::RGB16S:	return DC1394_COLOR_CODING_RGB16S;
	case CameraFirewire::RAW8:		return DC1394_COLOR_CODING_RAW8;
	case CameraFirewire::RAW16:		return DC1394_COLOR_CODING_RAW16;
	default:
		RW_THROW("Received unsupported Format 7 color coding!!");
		return DC1394_COLOR_CODING_MONO8;
	}
}

CameraFirewire::ColorCode DC1394Camera::ColorCodeConverter(const dc1394color_coding_t color) {
	switch(color) {
	case DC1394_COLOR_CODING_MONO8: 	return CameraFirewire::MONO8;
	case DC1394_COLOR_CODING_YUV411:	return CameraFirewire::YUV411;
	case DC1394_COLOR_CODING_YUV422:	return CameraFirewire::YUV422;
	case DC1394_COLOR_CODING_YUV444:	return CameraFirewire::YUV444;
	case DC1394_COLOR_CODING_RGB8:		return CameraFirewire::RGB8;
	case DC1394_COLOR_CODING_MONO16:	return CameraFirewire::MONO16;
	case DC1394_COLOR_CODING_RGB16:		return CameraFirewire::RGB16;
	case DC1394_COLOR_CODING_MONO16S:	return CameraFirewire::MONO16S;
	case DC1394_COLOR_CODING_RGB16S:	return CameraFirewire::RGB16S;
	case DC1394_COLOR_CODING_RAW8:		return CameraFirewire::RAW8;
	case DC1394_COLOR_CODING_RAW16:		return CameraFirewire::RAW16;
	default:
		RW_THROW("Received unsupported Format 7 color coding!!");
		return CameraFirewire::MONO8;
	}
}

bool DC1394Camera::getFormat7ColorCoding(CameraFirewire::ColorCode &color){
	dc1394video_mode_t captureMode;
	if(dc1394_video_get_mode(_dccamera, &captureMode)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read mode from camera.");
		return false;
	}
 	dc1394color_coding_t colorCode;
	if(dc1394_format7_get_color_coding(_dccamera, captureMode, &colorCode)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read color coding from camera.");
		return false;
	}

	color = ColorCodeConverter(colorCode);
	return true;
}



bool DC1394Camera::setFormat7PacketSize(const unsigned int packetSize){
	_f7PacketSize = packetSize;
	return true;
}

bool DC1394Camera::getFormat7PacketSize(unsigned int &packetSize){
	dc1394video_mode_t captureMode;
	if(dc1394_video_get_mode(_dccamera, &captureMode)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read mode from camera.");
		return false;
	}

	if(dc1394_format7_get_packet_size(_dccamera, captureMode, &packetSize)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read packet size from camera.");
		return false;
	}
	return true;
}

bool DC1394Camera::getFormat7RecommendedPacketSize(const CameraFirewire::Format7Mode mode, unsigned int &packetSize){

	if(dc1394_format7_get_recommended_packet_size(_dccamera, modeConverter(mode), &packetSize)!=DC1394_SUCCESS) {
		RW_WARN("Unable to read packet size from camera.");
		return false;
	}
	return true;
}

}} // End namespaces
