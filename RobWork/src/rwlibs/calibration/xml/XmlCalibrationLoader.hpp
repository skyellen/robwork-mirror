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

#ifndef RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_
#define RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_


#include <rwlibs/calibration/WorkCellCalibration.hpp>

namespace rw { namespace models { class WorkCell; } }

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/


	/**
	 * @brief Loads a calibration file for a workcell.
	 * 
	 * The file needs to satisfy the XML format of the calibration
	 */
	class XmlCalibrationLoader {
	public:
		/**
		* @brief Loads calibration file.
		* @param workcell [in] Workcell to which the calibration are associated
		* @param filename [in] Name of the file to load
		*/
		static WorkCellCalibration::Ptr load(rw::common::Ptr<rw::models::WorkCell> workcell, std::string fileName);
	};

	/* @} */

}
}

#endif /* RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_ */
