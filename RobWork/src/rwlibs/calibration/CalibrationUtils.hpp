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

#ifndef RWLIBS_CALIBRATION_CALIBRATION_UTILS_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_UTILS_HPP_

#include "CalibrationMeasurement.hpp"

namespace rw { namespace models { class WorkCell; } }
namespace rw { namespace kinematics { class State; } }

namespace rwlibs {
	namespace calibration {
		/** @addtogroup calibration */
		/*@{*/

		/**
		* @brief CalibrationUtils Range of functionality to help use the calibration.
		*
		*/
		class CalibrationUtils {
		public:
			/**
			 * @brief Prints a summary of the errors between model and measurements.
			 * Writes information to \b output specifying the minimum, average and maximal errors in position and orientation.
			 *
			 * @param measurements [in] Measurements to print summary for.
			 * @param workcell [in] The workcell to use for the model.
			 * @param workcellState [in] The state of the workcell to use.
			 * @param output [in/out] The log write to which the information is written.
			 */
			static void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::common::Ptr<rw::models::WorkCell> workcell, const rw::kinematics::State& workcellState, rw::common::LogWriter& output);
			
			/**
			 * @brief Prints a summary of the errors between model and measurements.
			 * Writes information to \b output specifying the minimum, average and maximal errors in position and orientation.
			 *
			 * @param measurements [in] Measurements to print summary for.
			 * @param workcell [in] The workcell to use for the model.
			 * @param workcellState [in] The state of the workcell to use.
			 * @param output [in/out] The output stream to which the information is written.
			 */
			static void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::common::Ptr<rw::models::WorkCell> workcell, const rw::kinematics::State& workcellState, std::ostream& outstream);
		private:

			/**
			 * @brief Default constructor made private to avoid wrong usage of the class.
			 */
			CalibrationUtils() {};
		};

		/*@}*/
	}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATION_HPP_ */
