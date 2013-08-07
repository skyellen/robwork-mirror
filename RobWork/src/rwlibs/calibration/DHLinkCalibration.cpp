/*
* DHLinkCalibration.cpp
*
*  Created on: Aug 28, 2012
*      Author: bing
*/

#include "DHLinkCalibration.hpp"

namespace rwlibs {
	namespace calibration {

		DHLinkCalibration::DHLinkCalibration(rw::models::Joint::Ptr joint) : CalibrationBase(CalibrationParameterSet(6)), _joint(joint), _originalSet(*rw::models::DHParameterSet::get(joint.get())), _isParallel(_originalSet.isParallel()) {
			CalibrationParameterSet parameterSet = getParameterSet();
			if (_isParallel) {
				parameterSet(PARAMETER_D).setEnabled(false);
				parameterSet(PARAMETER_THETA).setEnabled(false);
				setParameterSet(parameterSet);
			} else {
				parameterSet(PARAMETER_B).setEnabled(false);
				parameterSet(PARAMETER_BETA).setEnabled(false);
				setParameterSet(parameterSet);
			
				// Warn if b/beta representation is not used for close to parallel joints.
				const double alphaParallelThreshold = 10 * rw::math::Deg2Rad;
				if (abs(_originalSet.alpha()) < alphaParallelThreshold)
					RW_WARN("DH alpha parameter close to zero for joint \"" << joint->getName() << "\". Singularities might occur, consider using b/beta parameters instead of d/theta.");
			}
		}

		DHLinkCalibration::~DHLinkCalibration() {

		}

		rw::models::Joint::Ptr DHLinkCalibration::getJoint() const {
			return _joint;
		}

		//rw::math::Transform3D<> DHLinkCalibration::getCorrectionTransform() const {
		//	rw::math::Transform3D<> original, corrected;
		//	CalibrationParameterSet parameterSet = getParameterSet();
		//	const double a = parameterSet(PARAMETER_A).isEnabled() ? (_originalSet.a() + parameterSet(PARAMETER_A)) : _originalSet.a();
		//	const double alpha = parameterSet(PARAMETER_ALPHA).isEnabled() ? (_originalSet.alpha() + parameterSet(PARAMETER_ALPHA)) : _originalSet.alpha();
		//	if (_originalSet.isParallel()) {
		//		const double b = parameterSet(PARAMETER_B).isEnabled() ? (_originalSet.b() + parameterSet(PARAMETER_B)) : _originalSet.b();
		//		const double beta = parameterSet(PARAMETER_BETA).isEnabled() ? (_originalSet.beta() + parameterSet(PARAMETER_BETA)) : _originalSet.beta();
		//		const rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
		//		rw::models::DHParameterSet::set(correctedSet, _joint.get());
		//		_joint->setFixedTransform(rw::math::Transform3D<double>::DHHGP(alpha, a, beta, b));
		//	} else {
		//		const double d = parameterSet(PARAMETER_D).isEnabled() ? (_originalSet.d() + parameterSet(PARAMETER_D)) : _originalSet.d();
		//		const double theta = parameterSet(PARAMETER_THETA).isEnabled() ? (_originalSet.theta() + parameterSet(PARAMETER_THETA)) : _originalSet.theta();
		//		const rw::models::DHParameterSet correctedSet(alpha, a, d, theta, _originalSet.getType());
		//		rw::models::DHParameterSet::set(correctedSet, _joint.get());
		//		_joint->setFixedTransform(rw::math::Transform3D<double>::DH(alpha, a, d, theta));
		//	}
		//}

		void DHLinkCalibration::doApply() {
			CalibrationParameterSet parameterSet = getParameterSet();
			const double a = parameterSet(PARAMETER_A).isEnabled() ? (_originalSet.a() + parameterSet(PARAMETER_A)) : _originalSet.a();
			const double alpha = parameterSet(PARAMETER_ALPHA).isEnabled() ? (_originalSet.alpha() + parameterSet(PARAMETER_ALPHA)) : _originalSet.alpha();
			if (_originalSet.isParallel()) {
				const double b = parameterSet(PARAMETER_B).isEnabled() ? (_originalSet.b() + parameterSet(PARAMETER_B)) : _originalSet.b();
				const double beta = parameterSet(PARAMETER_BETA).isEnabled() ? (_originalSet.beta() + parameterSet(PARAMETER_BETA)) : _originalSet.beta();
				const rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
				rw::models::DHParameterSet::set(correctedSet, _joint.get());
				const rw::math::Transform3D<> correctedTransform = computeTransform(correctedSet);
				_joint->setFixedTransform(correctedTransform);
			} else {
				const double d = parameterSet(PARAMETER_D).isEnabled() ? (_originalSet.d() + parameterSet(PARAMETER_D)) : _originalSet.d();
				const double theta = parameterSet(PARAMETER_THETA).isEnabled() ? (_originalSet.theta() + parameterSet(PARAMETER_THETA)) : _originalSet.theta();
				const rw::models::DHParameterSet correctedSet(alpha, a, d, theta, _originalSet.getType());
				rw::models::DHParameterSet::set(correctedSet, _joint.get());
				const rw::math::Transform3D<> correctedTransform = computeTransform(correctedSet);
				_joint->setFixedTransform(correctedTransform);
			}
		}

		void DHLinkCalibration::doRevert() {
			rw::models::DHParameterSet::set(_originalSet, _joint.get());
			const rw::math::Transform3D<> correctedTransform = computeTransform(_originalSet);
			_joint->setFixedTransform(correctedTransform);
		}

		rw::math::Transform3D<> DHLinkCalibration::computeTransform(const rw::models::DHParameterSet& dhParameterSet) {
			if (dhParameterSet.isParallel())
				return rw::math::Transform3D<double>::DHHGP(dhParameterSet.alpha(), dhParameterSet.a(), dhParameterSet.beta(), dhParameterSet.b());
			else
				return rw::math::Transform3D<double>::DH(dhParameterSet.alpha(), dhParameterSet.a(), dhParameterSet.d(), dhParameterSet.theta());
		}
	}
}
