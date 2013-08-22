/*
 * ParallelAxisDH.hpp
 *
 */

#ifndef RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/models.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class ParallelAxisDHCalibration: public CalibrationBase {
public:

	enum{PARAMETER_A = 0,
		 PARAMETER_B,
		 PARAMETER_D,
		 PARAMETER_ALPHA,
		 PARAMETER_BETA,
		 PARAMETER_THETA
	};

	typedef rw::common::Ptr<ParallelAxisDHCalibration> Ptr;

	ParallelAxisDHCalibration(rw::models::Joint::Ptr joint);

	virtual ~ParallelAxisDHCalibration();

	rw::models::Joint::Ptr getJoint() const;

private:
	virtual void doApply();

	virtual void doRevert();

	static rw::math::Transform3D<> computeTransform(const rw::models::DHParameterSet& dhParameterSet);

private:
	rw::models::Joint::Ptr _joint;

	rw::math::Transform3D<> _originalTransform;
};

}
}

#endif /* RWLIBS_CALIBRATION_PARALLELAXISDHCALIBRATION_HPP_ */
