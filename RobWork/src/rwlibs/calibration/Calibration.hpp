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

#ifndef RWLIBS_CALIBRATION_CALIBRATION_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_HPP_

#include "CalibrationParameterSet.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
	namespace calibration {
		/** @addtogroup calibration */
		/*@{*/

		/**
		* @brief Calibration represents a kinematic correction.
		*
		* Calibrations can be applied or reverted. When applied the kinematics of the workcell will be modified according to the calibration model.
		*/
		class Calibration {
		public:
			typedef rw::common::Ptr<Calibration> Ptr;

			/**
			* @brief Destructor.
			*/ 
			virtual ~Calibration();

			/** 
			 * @brief Returns true if the calibraiton is enabled
			 */
			virtual bool isEnabled() const = 0;

			/** 
			 * @brief Sets whether the calibration is enabled
			 */
			virtual void setEnabled(bool isEnabled) = 0;

			/**
			 * @brief Returns the parameter set for the calibration
			 */
			virtual CalibrationParameterSet getParameterSet() const = 0;

			/** 
			 * @brief Sets the parameter set for the calibration
			 */

			virtual void setParameterSet(const CalibrationParameterSet& parameterSet) = 0;

			/**
			* @brief Test if calibration is applied.
			* @return True if applied, false otherwise
			*/
			virtual bool isApplied() const = 0;


			/**
			* @brief Apply calibration.
			*
			* Exception is thrown if calibration is already applied.
			*/
			virtual void apply() = 0;

			/**
			* @brief Revert calibration.
			*
			* Exception is thrown if calibration is not applied.
			*/
			virtual void revert() = 0;
		};

		/*@}*/
	}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
