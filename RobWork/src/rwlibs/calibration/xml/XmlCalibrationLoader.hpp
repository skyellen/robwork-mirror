/*
 * XmlCalibrationLoader.hpp
 *
 *  Created on: Sep 18, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_
#define RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_

#include "../SerialDeviceCalibration.hpp"

namespace rwlibs {
namespace calibration {

class XmlCalibrationLoader {
public:
	static SerialDeviceCalibration::Ptr load(rw::kinematics::StateStructure::Ptr stateStructure, rw::models::SerialDevice::Ptr device, std::string fileName);
};

}
}

#endif /* RWLIBS_CALIBRATION_XMLCALIBRATIONLOADER_HPP_ */
