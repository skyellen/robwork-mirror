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

#ifndef RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_
#define RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_

#include "Calibration.hpp"
#include <Eigen/Core>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration {
/** @addtogroup calibration */
/*@{*/

/**
 * @copydoc Calibration
 */
class CalibrationBase: public Calibration {
public:
	typedef rw::common::Ptr<CalibrationBase> Ptr;

	/**
	 * @brief Destructor.
	 */
	virtual ~CalibrationBase();
	
	virtual bool isEnabled() const;

	virtual void setEnabled(bool isEnabled);

	virtual CalibrationParameterSet getParameterSet() const;

	virtual void setParameterSet(const CalibrationParameterSet& parameterSet);

	/**
	 * @copydoc Calibration::isApplied()
	 */
	virtual bool isApplied() const;

	/**
	 * @copydoc Calibration::apply()
	 */
	virtual void apply();

	/**
	 * @copydoc Calibration::revert()
	 */
	virtual void revert();

protected:
	/**
	 * @brief Constructor.
	 */
	CalibrationBase(const CalibrationParameterSet& parameterSet);

	/**
	 * @brief Subclass implementation of apply().
	 */
	virtual void doApply() = 0;

	/**
	 * @brief Subclass implementation of revert().
	 */
	virtual void doRevert() = 0;

private:
	CalibrationParameterSet _parameterSet;
	bool _isEnabled;
	bool _isApplied;
};

/*@}*/
}
}

#endif /* RWLIBS_CALIBRATION_CALIBRATIONBASE_HPP_ */
