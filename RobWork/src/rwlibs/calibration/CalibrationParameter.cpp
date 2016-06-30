/*
* CalibrationParameter.cpp
*
*  Created on: Dec 18, 2012
*      Author: bing
*/

#include "CalibrationParameter.hpp"

#include <cmath>

namespace rwlibs {
	namespace calibration {

		CalibrationParameter::CalibrationParameter() : _isEnabled(true), _value(0.0), _variance(0.0) {

		}

		CalibrationParameter::CalibrationParameter(double initialValue) : _isEnabled(true), _value(initialValue), _variance(0.0) {

		}

		CalibrationParameter::~CalibrationParameter() {

		}

		bool CalibrationParameter::isEnabled() const {
			return _isEnabled;
		}

		void CalibrationParameter::setEnabled(bool isEnabled) {
			_isEnabled = isEnabled;
		}

		double CalibrationParameter::getValue() const {
			return _value;
		}

		void CalibrationParameter::setValue(double value) {
			_value = value;
		}

		bool CalibrationParameter::hasVariance() const {
			return _variance != 0.0;
		}

		double CalibrationParameter::getVariance() const {			
			return _variance;
		}

		void CalibrationParameter::setVariance(double variance) {
			_variance = variance;
		}

		double CalibrationParameter::getStandardDeviation() const {
			return sqrt(getVariance());
		}

		CalibrationParameter::operator double() const {
			return getValue();
		}

		CalibrationParameter& CalibrationParameter::operator =(const CalibrationParameter& other) {
			_isEnabled = other._isEnabled;
			_value = other._value;
			_variance = other._variance;
			return *this;
		}

		CalibrationParameter& CalibrationParameter::operator =(const double& value) {
			setValue(value);
			return *this;
		}

		double CalibrationParameter::operator +(const double& value) {
			return getValue() + value;
		}

		CalibrationParameter CalibrationParameter::operator +=(const double& value) {
			this->operator=(this->operator+(value));
			return *this;
		}


	}
}
