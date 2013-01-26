/*
* CalibrationBase.cpp
*
*  Created on: Nov 26, 2012
*      Author: bing
*/

#include "CalibrationBase.hpp"

namespace rwlibs {
	namespace calibration {

		CalibrationBase::~CalibrationBase() {

		}
		
		bool CalibrationBase::isEnabled() const {
			return _isEnabled;
		}

		void CalibrationBase::setEnabled(bool isEnabled) {
			RW_ASSERT(!isApplied());

			_isEnabled = isEnabled;
		}

		CalibrationParameterSet CalibrationBase::getParameterSet() const {
			return _parameterSet;
		}

		void CalibrationBase::setParameterSet(const CalibrationParameterSet& parameterSet) {
			_parameterSet = parameterSet;
		}

		bool CalibrationBase::isApplied() const {
			return _isApplied;
		}

		void CalibrationBase::apply() {
			RW_ASSERT(isEnabled());
			RW_ASSERT(!isApplied());

			doApply();

			_isApplied = true;

			RW_ASSERT(isApplied());
		}

		void CalibrationBase::revert() {
			RW_ASSERT(isEnabled());
			RW_ASSERT(isApplied());

			doRevert();

			_isApplied = false;

			RW_ASSERT(!isApplied());
		}

		CalibrationBase::CalibrationBase(const CalibrationParameterSet& parameterSet) :
			_parameterSet(parameterSet), _isEnabled(true), _isApplied(false) {

		}

	}
}
