/*
* ParallelAxisDHCalibration.cpp
*/

#include "ParallelAxisDHCalibration.hpp"

#include <rw/models/DHParameterSet.hpp>

using namespace rw::math;

namespace rwlibs {
	namespace calibration {

		ParallelAxisDHCalibration::ParallelAxisDHCalibration(rw::models::Joint::Ptr joint) : CalibrationBase(CalibrationParameterSet(6)), _joint(joint) {
			_originalTransform = _joint->getFixedTransform();
			CalibrationParameterSet parameterSet = getParameterSet();
			parameterSet(PARAMETER_D).setEnabled(false);
			parameterSet(PARAMETER_THETA).setEnabled(false);
			setParameterSet(parameterSet);
		}

		ParallelAxisDHCalibration::~ParallelAxisDHCalibration() {

		}

		rw::models::Joint::Ptr ParallelAxisDHCalibration::getJoint() const {
			return _joint;
		}

		//rw::math::Transform3D<> ParallelAxisDHCalibration::getCorrectionTransform() const {
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

		void ParallelAxisDHCalibration::doApply() {
			CalibrationParameterSet parameterSet = getParameterSet();
			const double a = parameterSet(PARAMETER_A).isEnabled() ? parameterSet(PARAMETER_A).getValue(): 0;
			const double alpha = parameterSet(PARAMETER_ALPHA).isEnabled() ? parameterSet(PARAMETER_ALPHA).getValue() : 0;
			const double b = parameterSet(PARAMETER_B).isEnabled() ? parameterSet(PARAMETER_B).getValue() : 0;
			const double beta = parameterSet(PARAMETER_BETA).isEnabled() ? parameterSet(PARAMETER_BETA).getValue() : 0;
			const rw::models::DHParameterSet correctedSet(alpha, a, beta, b, true);
			rw::models::DHParameterSet::set(correctedSet, _joint.get());
			const rw::math::Transform3D<> correctedTransform = computeTransform(correctedSet);
			//std::cout<<"OriginalTransform = "<<_originalTransform<<std::endl;
			_joint->setFixedTransform(_originalTransform*correctedTransform);			 
		}

		void ParallelAxisDHCalibration::doRevert() {
			rw::models::DHParameterSet::set(rw::models::DHParameterSet(0,0,0,0), _joint.get());
			//const rw::math::Transform3D<> correctedTransform = Transform3D<>::identity();
			_joint->setFixedTransform(_originalTransform);
		}

		rw::math::Transform3D<> ParallelAxisDHCalibration::computeTransform(const rw::models::DHParameterSet& dhParameterSet) {
			return rw::math::Transform3D<double>::DHHGP(dhParameterSet.alpha(), dhParameterSet.a(), dhParameterSet.beta(), dhParameterSet.b());
		}
	}
}
