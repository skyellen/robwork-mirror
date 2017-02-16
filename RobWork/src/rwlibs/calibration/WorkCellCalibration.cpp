/********************************************************************************
* Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#include "WorkCellCalibration.hpp"

#include <rw/models/SerialDevice.hpp>

#include <boost/foreach.hpp>



using namespace rw::math;
using namespace rw::common;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rwlibs::calibration;


WorkCellCalibration::WorkCellCalibration() 
{
}

		

WorkCellCalibration::~WorkCellCalibration() {

}

void WorkCellCalibration::addCalibration(Calibration::Ptr calibration)
{
	_calibrations.push_back(calibration);
}

void WorkCellCalibration::doApply()
{
	for (Calibration::Ptr calib : _calibrations) {
		calib->apply();
	}
}

void WorkCellCalibration::doRevert()
{
	for (Calibration::Ptr calib : _calibrations) {
		calib->revert();
	}
}

const std::vector<Calibration::Ptr>& WorkCellCalibration::getCalibrations() const
{
	return _calibrations;
}