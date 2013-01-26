/*
 * CalibrationParameter.hpp
 *
 *  Created on: Dec 18, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_CALIBRATIONPARAMETER_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONPARAMETER_HPP_

#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class CalibrationParameter {
public:
	CalibrationParameter();

	CalibrationParameter(double initialValue);

	~CalibrationParameter();

	bool isEnabled() const;

	void setEnabled(bool isEnabled);

	double getValue() const;

	void setValue(double value);

	bool hasVariance() const;

	double getVariance() const;

	void setVariance(double variance);

	double getStandardDeviation() const;
	
	operator double() const;
	
	CalibrationParameter& operator =(const CalibrationParameter& other);
	
	CalibrationParameter& operator =(const double& value);
	
	double operator +(const double& value);
	
	CalibrationParameter operator +=(const double& value);

private:
	bool _isEnabled;
	double _value;
	double _variance;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATIONPARAMETER_HPP_ */
