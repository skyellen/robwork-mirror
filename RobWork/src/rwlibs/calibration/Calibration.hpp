/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
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

#ifndef RWLIBS_CALIBRATION_CALIBRATION_HPP
#define RWLIBS_CALIBRATION_CALIBRATION_HPP

#include <rw/common/Ptr.hpp>
#include <rw/common/macros.hpp>

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
			 * @brief Default constructor
			 */
			Calibration();

			/**
			* @brief Destructor.
			*/ 
			virtual ~Calibration();

			/**
			* @brief Test if calibration is applied.
			* @return True if applied, false otherwise
			*/
			bool isApplied() const;


			/**
			* @brief Apply calibration.
			*
			* Exception is thrown if calibration is already applied.
			*/
			void apply();

			/**
			* @brief Revert calibration.
			*
			* Exception is thrown if calibration is not applied.
			*/
			void revert();

		protected:
			virtual void doApply() = 0;
			virtual void doRevert() = 0;
		private:
			bool _isApplied;
		};
	

		/*@}*/
	}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP */
