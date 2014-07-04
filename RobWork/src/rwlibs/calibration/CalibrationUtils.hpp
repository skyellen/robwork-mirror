#ifndef RWLIBS_CALIBRATION_CALIBRATION_UTILS_HPP_
#define RWLIBS_CALIBRATION_CALIBRATION_UTILS_HPP_

#include <rw/kinematics.hpp>
#include <rw/models.hpp>
#include "CalibrationMeasurement.hpp"

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
			static void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::WorkCell::Ptr workcell, const rw::kinematics::State& workcellState, rw::common::LogWriter& output);
			
			/**
			 * @brief Prints a summary of the errors between model and measurements.
			 * Writes information to \b output specifying the minimum, average and maximal errors in position and orientation.
			 *
			 * @param measurements [in] Measurements to print summary for.
			 * @param workcell [in] The workcell to use for the model.
			 * @param workcellState [in] The state of the workcell to use.
			 * @param output [in/out] The output stream to which the information is written.
			 */
			static void printMeasurementSummary(const std::vector<CalibrationMeasurement::Ptr>& measurements, rw::models::WorkCell::Ptr workcell, const rw::kinematics::State& workcellState, std::ostream& outstream);
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
