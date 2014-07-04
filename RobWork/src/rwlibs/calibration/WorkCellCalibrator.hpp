/*
 * WorkCellCalibrator.hpp
 *
 *  Created on: Feb 15, 2012
 *      Author: lpe
 */

#ifndef RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP
#define RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP

#include <rw/math.hpp>
#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "nlls/NLLSSolver.hpp"
#include "Calibration.hpp"
#include "Jacobian.hpp"
#include "CalibrationMeasurement.hpp"
#include <Eigen/Geometry>
#include <rw/models.hpp>

namespace rwlibs {
namespace calibration {

class WorkCellCalibrator {
public:
	typedef rw::common::Ptr<WorkCellCalibrator> Ptr;

	WorkCellCalibrator(rw::models::WorkCell::Ptr workcell,/* rw::kinematics::Frame::Ptr referenceFrame,
			rw::kinematics::Frame::Ptr measurementFrame, */Calibration::Ptr calibration, Jacobian::Ptr jacobian);

	virtual ~WorkCellCalibrator();

	rw::models::WorkCell::Ptr getWorkCell() const;

	//rw::kinematics::Frame::Ptr getReferenceFrame() const;

	//void setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame);

	//rw::kinematics::Frame::Ptr getMeasurementFrame() const;

	//void setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame);

	unsigned int getMinimumMeasurementCount() const;

	int getMeasurementCount() const;

	void addMeasurement(CalibrationMeasurement::Ptr measurement);

	void addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform,
			const Eigen::Matrix<double, 6, 6>& covarianceMatrix =
			(Eigen::Matrix<double, 6, 6>::Identity()));

	inline void addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform){
			const Eigen::Matrix<double, 6, 6>& covarianceMatrix = Eigen::Matrix<double, 6, 6>::Identity();
			addMeasurement(q,transform,covarianceMatrix);
	}

	void setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements);


	bool isWeightingMeasurements() const;

	void setWeightingMeasurements(bool isWeightingMeasurements);

	//void setUsePreCalibration(bool usePrecalibration);

	//bool isUsingPreCalibration() const;

	void calibrate(const rw::kinematics::State& workCellState);
	
	NLLSSolver::Ptr getSolver() const;

private:
	rw::models::WorkCell::Ptr _workcell;
	//rw::kinematics::Frame::Ptr _referenceFrame;
	//rw::kinematics::Frame::Ptr _measurementFrame;
	std::vector<CalibrationMeasurement::Ptr> _measurements;
	Calibration::Ptr _calibration;
	Jacobian::Ptr _jacobian;
	bool _isWeightingMeasurements;
	NLLSSolver::Ptr _solver;
//	bool _usePreCalibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATOR_HPP */
