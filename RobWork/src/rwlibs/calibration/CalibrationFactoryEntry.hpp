/*
 * CalibrationFactoryEntry.hpp
 *
 */

#ifndef RWLIBS_CALIBRATION_FACTORYENTRY_HPP_
#define RWLIBS_CALIBRATION_FACTORYENTRY_HPP_

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

class CalibrationFactoryEntry {
public:
	CalibrationFactoryEntry() {};

	void WorkCellCalibration::Ptr load(const std::string& filename);
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_FACTORYENTRY_HPP_ */
