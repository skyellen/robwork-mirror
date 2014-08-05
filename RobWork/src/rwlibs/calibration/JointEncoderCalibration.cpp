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


#include "JointEncoderCalibration.hpp"

using namespace rw::common;
using namespace rwlibs::calibration;

namespace {
	class EncoderTauFunction: public rw::math::Function<> { public: virtual double x(double q) { return -sin(q); }; };
	class EncoderSigmaFunction: public rw::math::Function<> { public: virtual double x(double q) { return -cos(q); }; };
}

class JointEncoderMapping : public rw::math::Function1Diff<> {
public:
	JointEncoderMapping(const std::vector<rw::math::Function<>::Ptr>& correctionFunctions, const CalibrationParameterSet& parameterSet) : _correctionFunctions(correctionFunctions), _parameterSet(parameterSet) {
	}

	virtual double x(double q) {
		double correctedQ = q;
		for (int parameterIndex = 0; parameterIndex < _parameterSet.getCount(); parameterIndex++) {
			if (_parameterSet(parameterIndex).isEnabled()) {
				correctedQ += _parameterSet(parameterIndex) * _correctionFunctions[parameterIndex]->x(q);
			}
		}
		return correctedQ;
	}

	virtual double dx(double q) {
		return 0;
	}

private:
	std::vector<rw::math::Function<>::Ptr> _correctionFunctions;
	CalibrationParameterSet _parameterSet;
};

JointEncoderCalibration::JointEncoderCalibration(rw::models::JointDevice::Ptr device,
		rw::models::Joint::Ptr joint) :
				CalibrationBase(CalibrationParameterSet(2)),
				_device(device),
				_joint(joint)
{
	_correctionFunctions.push_back( ownedPtr( new EncoderTauFunction() ) );
	_correctionFunctions.push_back( ownedPtr( new EncoderSigmaFunction() ) );
}

JointEncoderCalibration::~JointEncoderCalibration() {

}

rw::models::JointDevice::Ptr JointEncoderCalibration::getDevice() const {
	return _device;
}

rw::models::Joint::Ptr JointEncoderCalibration::getJoint() const {
	return _joint;
}

std::vector<rw::math::Function<>::Ptr> JointEncoderCalibration::getCorrectionFunctions() const {
	return _correctionFunctions;
}

void JointEncoderCalibration::doApply() {
	CalibrationParameterSet parameterSet = getParameterSet();
	if (parameterSet.getEnabledCount() > 0) {
		rw::math::Function1Diff<>::Ptr jointMapping = rw::common::ownedPtr(new JointEncoderMapping(_correctionFunctions, parameterSet)).cast<rw::math::Function1Diff<> >();
		_joint->setJointMapping(jointMapping);
	}
}

void JointEncoderCalibration::doRevert() {
	_joint->removeJointMapping();
}
