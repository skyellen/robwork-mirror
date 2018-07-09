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


#include "CameraModel.hpp"

using namespace rw::sensor;
using namespace rw::kinematics;

CameraModel::CameraModel(
        const rw::math::ProjectionMatrix& projection,
    const std::string& name,
    rw::kinematics::Frame* frame,
    const std::string& modelInfo)
    :
    SensorModel(name, frame, modelInfo),
    _pmatrix(projection),
    _sdata(1, rw::common::ownedPtr( new CameraModelCache()).cast<StateCache>())
{
	add(_sdata);
}

CameraModel::~CameraModel()
{}

double CameraModel::getFarClippingPlane() const{
    return _pmatrix.getClipPlanes().second;
}

double CameraModel::getNearClippingPlane() const{
    return _pmatrix.getClipPlanes().first;
}

Image::Ptr CameraModel::getImage(const rw::kinematics::State& state){
	return 	_sdata.getStateCache<CameraModelCache>(state)->_image;
}

void CameraModel::setImage(Image::Ptr img, rw::kinematics::State& state){
	_sdata.getStateCache<CameraModelCache>(state)->_image = img;
}


rw::math::ProjectionMatrix CameraModel::getProjectionMatrix() const { return _pmatrix; }

double CameraModel::getFieldOfViewX() const{
	double fovy,aspect,znear,zfar;
	_pmatrix.getPerspective(fovy,aspect,znear,zfar);
	return fovy*aspect;
}

double CameraModel::getFieldOfViewY() const{
	double fovy,aspect,znear,zfar;
	_pmatrix.getPerspective(fovy,aspect,znear,zfar);
	return fovy;
}


