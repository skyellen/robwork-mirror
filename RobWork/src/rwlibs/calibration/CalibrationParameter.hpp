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

#ifndef RWLIBS_CALIBRATION_CALIBRATIONPARAMETER_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONPARAMETER_HPP_

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @brief Storage class for a calibration parameter.
 *
 * A parameter can be set to enabled or disable, has a double value and may have an associated variance. 
 */
class CalibrationParameter {
public:
	/**
	 * @brief Constructs empty calibration parameter
	 * Defaults to enabled, value = 0 and variance = 0.
	 */
	CalibrationParameter();

	/**
	 * @brief Constructs calibration parameter and initialized the value to \b initialValue
	 * Defaults to enabled and variance = 0.
	 */
	CalibrationParameter(double initialValue);

	/**
	 * @brief Destructor
	 */
	~CalibrationParameter();

	/** 
	 * @brief Returns whether the parameter is enabled
	 */
	bool isEnabled() const;

	/**
	 * @brief Sets whether the parameter is enabled
	 * @param isEnabled [in] True to enable, false to disable.
	 */
	void setEnabled(bool isEnabled);

	/**
	 * @brief Returns the value of the parameter
	 */
	double getValue() const;

	/**
	 * @brief Sets the value of the parameter
	 * @param value [in] Value to set
	 */
	void setValue(double value);

	/** 
	 * @brief Returns true if the variance is not zero.
	 */
	bool hasVariance() const;

	/** 
	 * @brief Return variance of the parameter
	 */
	double getVariance() const;

	/**
	 * @brief Sets the variance
	 * @param variance [in] Variance to set
	 */
	void setVariance(double variance);

	/**
	 * @brief Returns standard deviation of the parameter
	 * Computes the standard deviation by taking the squareroot of the variance.
	 */
	double getStandardDeviation() const;
	
	/** 
	 * @brief Operator to access the value of the parameter
	 */
	operator double() const;
	
	/**
	 * @brief Overloaded assignment operator
	 * Copies all states of the parameter
	 */
	CalibrationParameter& operator =(const CalibrationParameter& other);
	
	/**
	 * @brief Overloaded assignment operator taking a double as the right side argument
	 * Sets \bvalue as the value of the parameter. The enabled property and the variance is not changed.
	 */
	CalibrationParameter& operator =(const double& value);
	
	/**
	 * @brief Overloaded add operator
	 * Adds a parameter and a double and returns a double value.
	 */
	double operator +(const double& value);
	
	/**
	 * @brief Overloaded add_assign operator
	 * Adds a double to a parameter.
	 */
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
