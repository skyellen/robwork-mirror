/*
 * DC1394CameraFactory.cpp
 *
 *  Created on: Feb 3, 2010
 *      Author: dr
 */

#include <rw/common/macros.hpp>

#include "DC1394CameraFactory.hpp"


#include <iostream>;


namespace rwhw { namespace camera {

DC1394CameraFactory* DC1394CameraFactory::_instance = 0; // definition outside class declaration

DC1394CameraFactory* DC1394CameraFactory::getInstance() {
	if (_instance == 0)
		_instance = new DC1394CameraFactory();

	return _instance;
}

const std::vector<std::string>& DC1394CameraFactory::getCameraList() {
	return _cameraNames;
}

void DC1394CameraFactory::freeCameraByIndex(unsigned int index) {
	RW_ASSERT(_cameraList.at(index)!=NULL);
//	delete _cameraList.at(index);
	_cameraList.at(index)=NULL;
}

void DC1394CameraFactory::freeCamera(DC1394Camera* camera) {
	for(unsigned int i=0; i<_cameraList.size(); i++) {
		if(camera==_cameraList.at(i)) {
			freeCameraByIndex(i);
			return;
		}
	}
	RW_THROW("Could not find camera pointer to free");
}

DC1394Camera* DC1394CameraFactory::getCamera(rw::kinematics::Frame* frame, unsigned int index) {
	if(_cameraList.size()>=index){
		RW_ASSERT("Wrong camera index");
		return NULL;
	}

	if (_cameraList.at(index) == NULL) {
		// Create DC1394 Camera and add to list
		_cameraList.at(index) = new DC1394Camera(frame, _cameraDriverList.at(index));
	}

	// Return _cameraList(index);
	return _cameraList.at(index);
}

void DC1394CameraFactory::initialize()
{
	dc1394camera_list_t * list;
	if(dc1394_camera_enumerate (_dc1394, &list)!=DC1394_SUCCESS) {
		RW_THROW("Could not enumerate cameras");
		dc1394_camera_free_list(list);
	}
	_cameraDriverList.clear();
	_cameraNames.clear();
	_cameraList.clear();

	for(unsigned int i = 0; i < list->num; i++) {
		//Initialise camera memory
		dc1394camera_t * camera = dc1394_camera_new(_dc1394, list->ids[i].guid);
		dc1394_reset_bus(camera);
		_cameraDriverList.push_back(camera);

		//Save camera name
		std::ostringstream ost;
		ost << camera->vendor << " - ";
		ost << camera->model << " - ";
		ost << camera->guid;
		_cameraNames.push_back(ost.str());
		_cameraList.push_back(NULL);

		std::cout << _cameraNames.at(i) << std::endl;
	}
}

DC1394CameraFactory::DC1394CameraFactory() {
	_dc1394 = dc1394_new();
	initialize();
}

DC1394CameraFactory::~DC1394CameraFactory() {
	//release camera memory
	for(unsigned int i = 0; i < _cameraDriverList.size(); i++) {
		dc1394_camera_free(_cameraDriverList.at(i));
	}

	//Release dc1394 driver
	dc1394_free(_dc1394);
}


}} // End namespaces
