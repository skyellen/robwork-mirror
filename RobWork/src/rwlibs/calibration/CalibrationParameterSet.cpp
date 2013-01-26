/*
* CalibrationParameterSet.cpp
*
*  Created on: Dec 18, 2012
*      Author: bing
*/

#include "CalibrationParameterSet.hpp"

namespace rwlibs {
	namespace calibration {

		CalibrationParameterSet::CalibrationParameterSet(int parameterCount) : _parameters(parameterCount) {

		}

		int CalibrationParameterSet::getCount() const {
			return _parameters.size();
		}

		int CalibrationParameterSet::getEnabledCount() const {
			const int count = getCount();
			int enabledCount = 0;
			for (int parameterIndex = 0; parameterIndex < count; parameterIndex++)
				if (getParameter(parameterIndex).isEnabled())
					enabledCount++;
			return enabledCount;
		}

		CalibrationParameter& CalibrationParameterSet::getParameter(int parameterIndex) {
			RW_ASSERT(parameterIndex < getCount());
			return _parameters(parameterIndex);
		}

		const CalibrationParameter& CalibrationParameterSet::getParameter(int parameterIndex) const {
			RW_ASSERT(parameterIndex < getCount());
			return _parameters(parameterIndex);
		}

		CalibrationParameter& CalibrationParameterSet::operator ()(int parameterIndex) {
			return getParameter(parameterIndex);
		}

		const CalibrationParameter& CalibrationParameterSet::operator ()(int parameterIndex) const {
			return getParameter(parameterIndex);
		}

	}
}
