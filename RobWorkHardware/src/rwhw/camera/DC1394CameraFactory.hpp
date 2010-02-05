/*
 * DC1394CameraFactory.h
 *
 *  Created on: Feb 3, 2010
 *      Author: dr
 */

#ifndef RWHW_CAMERA_DC1394CAMERAFACTORY_HPP
#define RWHW_CAMERA_DC1394CAMERAFACTORY_HPP

#include <rw/kinematics/Frame.hpp>

#include <dc1394/dc1394.h>
#include <string>
#include <vector>

#include "DC1394Camera.hpp"

namespace rwhw { namespace camera {

//class DC1394Camera : public rw::sensor::CameraFirewire {};

class DC1394CameraFactory {
public:
	static DC1394CameraFactory* getInstance();
	const std::vector<std::string>& getCameraList();
	DC1394Camera* getCamera(rw::kinematics::Frame* frame, unsigned int index);
	void freeCamera(DC1394Camera* camera);


protected:
	DC1394CameraFactory();
	virtual ~DC1394CameraFactory();
	void freeCameraByIndex(unsigned int index);

	void initialize();

	static DC1394CameraFactory* _instance;
	dc1394_t * _dc1394;
	std::vector<std::string> _cameraNames;
	std::vector<dc1394camera_t *> _cameraDriverList;
	std::vector<DC1394Camera*> _cameraList;
};

} } // End namespaces

#endif /* RWHW_CAMERA_DC1394CAMERAFACTORY_HPP */
