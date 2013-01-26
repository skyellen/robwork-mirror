/*
 * XmlMeasurementFile.hpp
 *
 *  Created on: May 22, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_XMLMEASUREMENTFILE_HPP
#define RWLIBS_CALIBRATION_XMLMEASUREMENTFILE_HPP

#include "../SerialDevicePoseMeasurement.hpp"

namespace rwlibs {
namespace calibration {

class XmlMeasurementFile {
public:
	static void save(const std::vector<SerialDevicePoseMeasurement>& measurements, std::string fileName);

	static std::vector<SerialDevicePoseMeasurement> load(std::string fileName);
};

}
}

#endif /* RWLIBS_CALIBRATION_XMLMEASUREMENTFILE_HPP */
