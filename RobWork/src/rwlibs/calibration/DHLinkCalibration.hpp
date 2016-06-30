/*
 * DHLinkCalibration.hpp
 *
 *  Created on: Aug 28, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_DHLINKCALIBRATION_HPP_
#define RWLIBS_CALIBRATION_DHLINKCALIBRATION_HPP_

#include "CalibrationBase.hpp"
#include <rw/models/Joint.hpp>
#include <rw/models/DHParameterSet.hpp>

namespace rwlibs {
namespace calibration {

class DHLinkCalibration: public CalibrationBase {
public:

	enum{PARAMETER_A = 0,
		 PARAMETER_B,
		 PARAMETER_D,
		 PARAMETER_ALPHA,
		 PARAMETER_BETA,
		 PARAMETER_THETA
	};

	typedef rw::common::Ptr<DHLinkCalibration> Ptr;

	DHLinkCalibration(rw::models::Joint::Ptr joint);

	virtual ~DHLinkCalibration();

	rw::models::Joint::Ptr getJoint() const;

private:
	virtual void doApply();

	virtual void doRevert();

	static rw::math::Transform3D<> computeTransform(const rw::models::DHParameterSet& dhParameterSet);

private:
	rw::models::Joint::Ptr _joint;
	rw::models::DHParameterSet _originalSet;
	bool _isParallel;
};

}
}

#endif /* RWLIBS_CALIBRATION_DHLINKCALIBRATION_HPP_ */
