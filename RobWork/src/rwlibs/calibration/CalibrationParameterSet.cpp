/*
* CalibrationParameterSet.cpp
*
*  Created on: Dec 18, 2012
*      Author: bing
*/

#include "CalibrationParameterSet.hpp"

#include <rw/common/macros.hpp>

namespace rwlibs {
	namespace calibration {

		CalibrationParameterSet::CalibrationParameterSet(int parameterCount) : _parameters(parameterCount) {
			for (size_t i = 0; (int)i<parameterCount; i++)
				_parameters(i) = 0;

		}

		int CalibrationParameterSet::getCount() const {
			return (int)_parameters.size();
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

		std::ostream& operator<<(std::ostream& out, const CalibrationParameterSet& set) {
			out<<"Calibration Parameter Set: "<<std::endl;
			for (int i = 0; i<set.getCount(); i++) {
				out<<"\t"<<set.getParameter(i).getValue()<<" enabled: "<<set.getParameter(i).isEnabled();
				if (i != set.getCount()-1)
					out<<std::endl;
			}
			return out;
		}


	}
}
