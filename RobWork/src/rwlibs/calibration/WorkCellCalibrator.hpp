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

#ifndef RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP
#define RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP

#include <rw/math.hpp>
//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "nlls/NLLSSolver.hpp"
#include "Calibration.hpp"
#include "Jacobian.hpp"
#include "CalibrationMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

/** @addtogroup calibration */
/*@{*/


/**
 * @brief The WorkCellCalibrator provides the routines required to calibrate perform both intrinsic and extrinsic 
 * calibration of devices as well as marker and sensor frames in the work cell
 */
class WorkCellCalibrator {
public:
	/** @brief Smart pointer declaration */ 
	typedef rw::common::Ptr<WorkCellCalibrator> Ptr;

	/**
	 * @brief Constructs a WorkCellCalibrator for \bworkcell with the calibration given by \bcalibration and the Jacobian \bjacobian.
	 *
	 * @param workcell [in] The workcell to be calibrated
	 * @param calibration [in] The calibration to use. This calibration is updated when calling calibrate()
	 * @param jacobian [in] The jacobian for the system to calibrate
	 */
	WorkCellCalibrator(rw::models::WorkCell::Ptr workcell, Calibration::Ptr calibration, Jacobian::Ptr jacobian);

	/**
	 * @brief Destructor
	 */
	virtual ~WorkCellCalibrator();

	/**
	 * @brief Returns the workcell used in the calibration
	 */
	rw::models::WorkCell::Ptr getWorkCell() const;

	
	/**
	 * @brief Returns the minimm number of measurements required for the calibration
	 */
	unsigned int getMinimumMeasurementCount() const;

	/**
	 * @brief Returns the number of measurements added to the calibration
	 */
	int getMeasurementCount() const;

	/**
	 * @brief Add a measurement to the calibration
	 * @param measurement [in] The calibration measurement
	 */
	void addMeasurement(CalibrationMeasurement::Ptr measurement);

	/**
	 * @brief Sets the measurements to be used in the calibration
	 */
	void setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements);

	/** 
	 * @brief Returns whether to use weighted measurements
	 */
	bool useWeightedMeasurements() const;

	/** 
	 * @brief Sets whether to use weighted measurements
	 * @param useWeightedMeasurements [in] Set to true to use weighted measurements
	 */
	void setUseWeightedMeasurements(bool useWeightedMeasurements);

	/**
	 * @brief Run the calibration procedure.
	 *
	 * @param workCellState [in] State of the work cell to be calibrated
	 */
	void calibrate(const rw::kinematics::State& workCellState);
	
	/**
	 * @brief Returns the NLLSSolver used in the calibration
	 */
	NLLSSolver::Ptr getSolver() const;

private:
	rw::models::WorkCell::Ptr _workcell;
	std::vector<CalibrationMeasurement::Ptr> _measurements;
	Calibration::Ptr _calibration;
	Jacobian::Ptr _jacobian;
	bool _useWeightedMeasurements;
	NLLSSolver::Ptr _solver;

};

/* @} */
}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP */
