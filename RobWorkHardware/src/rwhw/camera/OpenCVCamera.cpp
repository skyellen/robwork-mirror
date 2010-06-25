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

#include "OpenCVCamera.hpp"

#include <windows.h>

#include <cvcam.h>

#include <rw/common/macros.hpp>
#include <vector>

using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rwhw;

std::vector<OpenCVCamera*> OpenCVCamera::_cameras;

namespace
{
    std::string getCameraName(CvCapture* cam)
    {
        RW_ASSERT(cam);

        std::vector<char> name(256);

        return std::string(name.begin(), name.end());
    }

    std::string getCameraVendor(CvCapture* cam)
    {
        RW_ASSERT(cam);

        std::vector<char> name(256);
//        cam->GetCameraVendor(&name.front(), name.size());
        return std::string(name.begin(), name.end());
    }
    Image::PixelDepth getDepth(IplImage *img){
    	switch(img->depth){
    	case(IPL_DEPTH_8U): return Image::Depth8U;
    	case(IPL_DEPTH_8S): return Image::Depth8S;
    	case(IPL_DEPTH_16S): return Image::Depth16S;
    	case(IPL_DEPTH_32S): return Image::Depth32S;
    	case(IPL_DEPTH_32F): return Image::Depth32F;
    	default: RW_THROW("Unsupported Image depth!");
    	}
    	return Image::Depth8S;
    }
    Image::ColorCode getColorCode(IplImage *img){
    	switch(img->nChannels){
    	case(1): return Image::GRAY;
    	case(3): return Image::RGB;
    	default: RW_THROW("Unsupported ColorCode!");
    	}
    	return Image::GRAY;
    }
}

OpenCVCamera::OpenCVCamera(CvCapture* cam)
    :
    Camera(
        NULL,
        "noname", //getCameraName(cam),
        "NoVendor"),//getCameraVendor(cam)),
    _isAquired(false),
    _cam(cam),
    _image(NULL)
{}

OpenCVCamera::~OpenCVCamera()
{
	cvReleaseCapture(&_cam);
}

bool OpenCVCamera::initialize(){
    IplImage *iplimage = cvRetrieveFrame(_cam);
    _width = iplimage->width;
    _height = iplimage->height;

    _initialized = true;
    return true;
}

void OpenCVCamera::stop(){
    _started = false;
}

bool OpenCVCamera::start(){
	_started = true;
    return true;
}

void OpenCVCamera::acquire(){
    _isAquired = false;
    if( cvGrabFrame(_cam)==1)
    	_isAquired = true;
}


bool OpenCVCamera::isImageReady(){
	return _isAquired;
}

const Image* OpenCVCamera::getImage(){
	if(!_started){
		 return NULL;
	}

	//IplImage *iplimage = cvQueryFrame(_cam);
	IplImage *iplimage = cvRetrieveFrame(_cam);
	RW_ASSERT(iplimage);

	cvSaveImage("c:/SAMEIMAGE.jpg", iplimage);
	if(_image==NULL){
		std::cout << "widthStep: " << iplimage->widthStep << std::endl;
		std::cout << "widthStep: " << iplimage->align << std::endl;
		std::cout << "widthStep: " << iplimage->dataOrder << std::endl;

		_image = new Image( iplimage->width, iplimage->height,
						    getColorCode(iplimage), getDepth(iplimage));
	}

	for(int y=0;y<iplimage->height;y++){
		for(int x=0;x<iplimage->width;x++){
			CvScalar s = cvGet2D(iplimage,y,x);
			int idx = y*iplimage->width*iplimage->nChannels+x*iplimage->nChannels;
			_image->getImageData()[idx] = (unsigned char)s.val[2];
			_image->getImageData()[idx+1] = (unsigned char) s.val[1];
			_image->getImageData()[idx+2] = (unsigned char) s.val[0];
		}
	}

    return _image;
}

double OpenCVCamera::getFrameRate(){

	return cvGetCaptureProperty(_cam, CV_CAP_PROP_FPS);
}

void OpenCVCamera::setFrameRate(double framerate){
	_frameRate = framerate;
	cvSetCaptureProperty(_cam, CV_CAP_PROP_FPS, framerate);
}

void callback(IplImage* image)//Draws blue horizontal lines across the image
{
	std::cout << "CB\n";

	IplImage* image1 = image;
	int i, j;

	assert(image);

	for (i = 0; i < image1->height; i += 10) {
		for (j = (image1->widthStep) * i; j < (image1->widthStep) * (i + 1); j
				+= image1->nChannels) {
			image1->imageData[j] = (char) 255;
			image1->imageData[j + 1] = 0;
			image1->imageData[j + 2] = 0;
		}

	}
}

const std::vector<OpenCVCamera*> OpenCVCamera::getCameraHandles()
{
	/*
	int ncams = cvcamGetCamerasCount( );//returns the number of available cameras in the

	std::cout << "NR OF CAMS: " << ncams << std::endl;
	for(int i=0;i<ncams;i++){
		CameraDescription desc;
		cvcamGetProperty(i,CVCAM_DESCRIPTION,(void*)&desc);
		std::cout << desc.DeviceDescription << std::endl;


		cvcamSetProperty(i, CVCAM_PROP_ENABLE, &i); //Selects the 1-st found
											        //camera
		cvcamSetProperty(i, CVCAM_PROP_RENDER, &i);  //We�ll render stream
											        //from this source

		cvcamSetProperty(i, CVCAM_PROP_CALLBACK, (void*)callback);//this callback will
									  //process every frame

	}*/

	//cvNamedWindow("mywindow", CV_WINDOW_AUTOSIZE);

	std::cout << "Run through all cams" << std::endl;
	while( CvCapture *cvcam = cvCaptureFromCAM(CV_CAP_ANY) ){
		std::cout << "CAM" << std::endl;

		_cameras.push_back( new OpenCVCamera( cvcam ));

		//IplImage *frame = cvQueryFrame(cvcam);
		//cvSaveImage("c:/firstImageCapture.jpg", frame );
		//if(frame){
		//	cvShowImage("mywindow",frame);
		//}
		//_cameras.push_back(  );
		//if((cvWaitKey(10)&255)==27) break;
	}
	std::cout << "Cam out... " << std::endl;
//     CvCapture cam;
//     cam.RefreshCameraList();
//     int numCams = cam.GetNumberCameras();
//     std::vector<OpenCVCamera*> tmpList;
//     // update the camList
//     for(int i=0; i<numCams; i++){
//         cam.SelectCamera(i);
//         char buff[100];
//         cam.GetCameraName(buff,100);
//         std::string camName(buff);
//         bool found = false;
//         std::vector<OpenCVCamera*>::iterator iter = _cameras.begin();
//         for(;iter!=_cameras.end();++iter){
//             if( (**iter).getName() == camName){
//                 tmpList.push_back( *iter );
//                 _cameras.erase( iter );
//                 found = true;
//                 break;
//             }
//         }
//         if(found==false){
//             CvCapture *newCam = new CvCapture();
//             newCam->RefreshCameraList();
//             newCam->SelectCamera(i);
//             tmpList.push_back( new OpenCVCamera(newCam) );
//         }
//     }
//     // delete cmu cameraes that no longer exist, and update the state
//     // of the wincamera
//     std::vector<OpenCVCamera*>::iterator i_cam = _cameras.begin();
//     for(;i_cam!=_cameras.end();++i_cam){
//         (*i_cam)->setConnected(false);
//         delete (*i_cam)->getCMUCamera();
//     }
//     // move cameras from tmp list to camList
//     _cameras = tmpList;
//     // construct the cam list

    return _cameras;
}
