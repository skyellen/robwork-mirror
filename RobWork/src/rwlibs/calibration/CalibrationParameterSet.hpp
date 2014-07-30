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


#ifndef RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_

#include "CalibrationParameter.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
	namespace calibration {
		/** @addtogroup calibration */
		/*@{*/

		/**
		 * @brief Container for handling a set of CalibrationParameter objects
		 */
		class CalibrationParameterSet {
		public:
			/**
			 * @brief Constructs a set with \bparameterCount CalibrationParameter objects
			 */
			CalibrationParameterSet(int parameterCount);

			/**
			 * @brief Returns the number of parameters in the set
			 */
			int getCount() const;

			/**
			 * @brief Returns how many of the parameters that are enabled
			 */
			int getEnabledCount() const;

			/**
			 * @brief Returns the parameter with \bparameterIndex
			 * @param parameterIndex [in] The index goes from 0 to getCount()-1
			 */
			CalibrationParameter& getParameter(int parameterIndex);

			/**
			 * @brief Returns the parameter with \bparameterIndex
			 * @param parameterIndex [in] The index goes from 0 to getCount()-1
			 */
			const CalibrationParameter& getParameter(int parameterIndex) const;

			/**
			 * @brief Overloaded operator for accessing the parameter with \bparameterIndex
			 * @param parameterIndex [in] The index goes from 0 to getCount()-1
			 */
			CalibrationParameter& operator ()(int parameterIndex);

			/**
			 * @brief Overloaded operator for accessing the parameter with \bparameterIndex
			 * @param parameterIndex [in] The index goes from 0 to getCount()-1
			 */
			const CalibrationParameter& operator ()(int parameterIndex) const;

			/**
			 * @brief Stream the parameter set \bset, to the stream \bout.
			 */
			friend std::ostream& operator<<(std::ostream& out, const CalibrationParameterSet& set);	

		private:
			Eigen::Matrix<CalibrationParameter, Eigen::Dynamic, 1> _parameters;
		};

		

		/*@}*/
	}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATIONPARAMETERSET_HPP_ */
