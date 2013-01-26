/*
* CalibrationParameterSet.hpp
*
*  Created on: Dec 18, 2012
*      Author: bing
*/

#ifndef RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_

#include "CalibrationParameter.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
	namespace calibration {
		/** @addtogroup calibration */
		/*@{*/

		class CalibrationParameterSet {
		public:
			CalibrationParameterSet(int parameterCount);

			int getCount() const;

			int getEnabledCount() const;

			CalibrationParameter& getParameter(int parameterIndex);

			const CalibrationParameter& getParameter(int parameterIndex) const;

			CalibrationParameter& operator ()(int parameterIndex);

			const CalibrationParameter& operator ()(int parameterIndex) const;

		private:
			Eigen::Matrix<CalibrationParameter, Eigen::Dynamic, 1> _parameters;
		};

		/*@}*/
	}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_ */
