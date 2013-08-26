/*
* WorkCellCalibrator.cpp
*
*  Created on: Feb 27, 2012
*      Author: bing
*/

#include "WorkCellCalibrator.hpp"

#include "nlls/NLLSNewtonSolver.hpp"

#include <rw/math/EAA.hpp>

#include <Eigen/Eigenvalues>

using namespace rw::math;

namespace rwlibs {
	namespace calibration {

		class WorkCellCalibrationSystem: public NLLSSystem {
		public:
			typedef rw::common::Ptr<WorkCellCalibrationSystem> Ptr;

			WorkCellCalibrationSystem(rw::models::SerialDevice::Ptr device, rw::kinematics::State workCellState, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, const std::vector<SerialDevicePoseMeasurement>& measurements, Calibration::Ptr calibration, Jacobian::Ptr jacobian, bool isWeightingMeasurements) : _device(device), _workCellState(workCellState), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _measurements(measurements), _calibration(calibration), _jacobian(jacobian), _isWeightingMeasurements(isWeightingMeasurements) {

			}

			virtual ~WorkCellCalibrationSystem() {

			}

		private:
			virtual void computeJacobian(Eigen::MatrixXd& stackedJacobians) {
				const int measurementCount = _measurements.size();
				const int rowCount = 6 * measurementCount;
				const int columnCount = _jacobian->getColumnCount();
				RW_ASSERT(columnCount > 0);
				RW_ASSERT(rowCount >= columnCount);

				stackedJacobians.resize(rowCount, columnCount);
				rw::kinematics::State workCellState = _workCellState;
				for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const SerialDevicePoseMeasurement measurement = _measurements[measurementIndex];

					// Update state according to current measurement.
					const rw::math::Q q = measurement.getQ();
					_device->setQ(q, workCellState);

					// Compute Jacobian.
					const int rowIndex = 6 * measurementIndex;
					stackedJacobians.block(rowIndex, 0, 6, columnCount) = _jacobian->computeJacobian(_referenceFrame, _measurementFrame, workCellState);

					// Weight Jacobian according to covariances.
					if (_isWeightingMeasurements && measurement.hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
							measurement.getCovarianceMatrix()).operatorInverseSqrt();
						stackedJacobians.block(rowIndex, 0, 6, columnCount) = weightMatrix
							* stackedJacobians.block(rowIndex, 0, 6, columnCount);
					}
				}
				
			}

			virtual void computeResiduals(Eigen::VectorXd& stackedResiduals) {
				const int measurementCount = _measurements.size();

				const int rowCount = 6 * measurementCount;
				stackedResiduals.resize(6 * measurementCount);
				rw::kinematics::State workCellState = _workCellState;
				for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const SerialDevicePoseMeasurement measurement = _measurements[measurementIndex];

					// Update state according to current measurement.
					const rw::math::Q q = measurement.getQ();
					_device->setQ(q, workCellState);

					// Compute residuals.
					const int rowIndex = 6 * measurementIndex;
					const rw::math::Transform3D<> tfmMeasurement = measurement.getTransform();
					//std::cout<<"Measurement = "<<tfmMeasurement<<std::endl;
					const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(_referenceFrame.get(), _measurementFrame.get(), workCellState);
					//std::cout<<"Model Transform = "<<tfmModel<<std::endl;
						
					rw::math::Vector3D<> dP = tfmModel.P() - tfmMeasurement.P();
//					std::cout<<"dP = "<<dP<<std::endl;
					//dP = -dP;
					stackedResiduals(rowIndex + 0) = dP(0);
					stackedResiduals(rowIndex + 1) = dP(1);
					stackedResiduals(rowIndex + 2) = dP(2);
					rw::math::Rotation3D<> dR = tfmModel.R() * rw::math::inverse(tfmMeasurement.R());

					//std::cout<<"dR = "<<EAA<>(dR)<<std::endl;
					//dR = Rotation3D<>::identity();

					stackedResiduals(rowIndex + 3) = (dR(2, 1) - dR(1, 2)) / 2;
					stackedResiduals(rowIndex + 4) = (dR(0, 2) - dR(2, 0)) / 2;
					stackedResiduals(rowIndex + 5) = (dR(1, 0) - dR(0, 1)) / 2;

					// Weight residuals according to covariances.
					if (_isWeightingMeasurements && measurement.hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(
							measurement.getCovarianceMatrix()).operatorInverseSqrt();
						stackedResiduals.segment<6>(rowIndex) = weightMatrix * stackedResiduals.segment<6>(rowIndex);
					}
				}
				//std::cout<<"Residuals: "<<stackedResiduals.norm()<<std::endl;
				//std::cout<<"Residuals Vector Length = "<<stackedResiduals.size()<<std::endl;
				//std::cout<<"Residuals: "<<stackedResiduals<<std::endl;
				//std::cout<<"Press enter to continue..."<<std::endl;
				//char ch[4];
				//std::cin.getline(ch, 1);
			}

			virtual void takeStep(const Eigen::VectorXd& step) {
				_jacobian->takeStep(step);
			}

		private:
			rw::models::SerialDevice::Ptr _device;
			rw::kinematics::State _workCellState;
			rw::kinematics::Frame::Ptr _referenceFrame;
			rw::kinematics::Frame::Ptr _measurementFrame;
			std::vector<SerialDevicePoseMeasurement> _measurements;
			Calibration::Ptr _calibration;
			Jacobian::Ptr _jacobian;
			bool _isWeightingMeasurements;
		};

		WorkCellCalibrator::WorkCellCalibrator(rw::models::SerialDevice::Ptr device,
			rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, Calibration::Ptr calibration, Jacobian::Ptr jacobian) :
		_device(device), _referenceFrame(referenceFrame), _measurementFrame(measurementFrame), _calibration(calibration), _jacobian(jacobian), _isWeightingMeasurements(true) {

		}

		WorkCellCalibrator::~WorkCellCalibrator() {

		}

		rw::models::SerialDevice::Ptr WorkCellCalibrator::getDevice() const {
			return _device;
		}

		rw::kinematics::Frame::Ptr WorkCellCalibrator::getReferenceFrame() const {
			return _referenceFrame;
		}

		void WorkCellCalibrator::setReferenceFrame(rw::kinematics::Frame::Ptr referenceFrame) {
			_referenceFrame = referenceFrame;
		}

		rw::kinematics::Frame::Ptr WorkCellCalibrator::getMeasurementFrame() const {
			return _measurementFrame;
		} 

		void WorkCellCalibrator::setMeasurementFrame(rw::kinematics::Frame::Ptr measurementFrame) {
			_measurementFrame = measurementFrame;
		}

		unsigned int WorkCellCalibrator::getMinimumMeasurementCount() const {
			return ceil(float(_jacobian->getColumnCount()) / 6);
		}

		int WorkCellCalibrator::getMeasurementCount() const {
			return _measurements.size();
		}

		void WorkCellCalibrator::addMeasurement(const SerialDevicePoseMeasurement& measurement) {
			_measurements.push_back(measurement);
		}

		void WorkCellCalibrator::addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) {
			_measurements.push_back(SerialDevicePoseMeasurement(q, transform, covarianceMatrix));
		}

		void WorkCellCalibrator::setMeasurements(const std::vector<SerialDevicePoseMeasurement>& measurements) {
			_measurements = measurements;
		}

		bool WorkCellCalibrator::isWeightingMeasurements() const {
			return _isWeightingMeasurements;
		}

		void WorkCellCalibrator::setWeightingMeasurements(bool isWeightingMeasurements) {
			_isWeightingMeasurements = isWeightingMeasurements;
		}

		void WorkCellCalibrator::calibrate(const rw::kinematics::State& workCellState) {
			RW_ASSERT(_calibration->isEnabled());

			const int measurementCount = getMeasurementCount();
			const int minimumMeasurementCount = getMinimumMeasurementCount();
			if (measurementCount < minimumMeasurementCount)
				RW_THROW(measurementCount << " measurements was provided but " << minimumMeasurementCount << " is required.");

			// Apply calibration if not applied.
			const bool wasApplied = _calibration->isApplied();
			if (!wasApplied)
				_calibration->apply();

			// Solve non-linear least square system.
			try {
				WorkCellCalibrationSystem::Ptr system = rw::common::ownedPtr(new WorkCellCalibrationSystem(_device, workCellState, _referenceFrame, _measurementFrame, _measurements, _calibration, _jacobian, _isWeightingMeasurements));
				_solver = rw::common::ownedPtr(new NLLSNewtonSolver(system));
				_solver->solve();
				
				// Compute variances if weighting was enabled.
				if (_isWeightingMeasurements) {
					Eigen::MatrixXd covarianceMatrix = _solver->estimateCovarianceMatrix();
					CalibrationParameterSet parameterSet = _calibration->getParameterSet();
					RW_ASSERT(covarianceMatrix.rows() == covarianceMatrix.cols());
					RW_ASSERT(covarianceMatrix.rows() == parameterSet.getEnabledCount());
					const int parameterCount = parameterSet.getCount();
					const int enabledParameterCount = parameterSet.getEnabledCount();
					int enabledParameterIndex = 0;
					for (int parameterIndex = 0; parameterIndex < parameterCount; parameterIndex++) {
						if (parameterSet(parameterIndex).isEnabled()) {
							double variance = covarianceMatrix(enabledParameterIndex, enabledParameterIndex);
							parameterSet(parameterIndex).setVariance(variance);
							enabledParameterIndex++;
						}
					}
					RW_ASSERT(enabledParameterIndex == enabledParameterCount);
					_calibration->setParameterSet(parameterSet);
				}
			} catch (rw::common::Exception& exception) {
				// Revert calibration if it was not applied.
				if (!wasApplied)
					_calibration->revert();

				// Re-trow exception.
				throw exception;
			}

			// Revert calibration if it was not applied.
			if (!wasApplied)
				_calibration->revert();
		}

		NLLSSolver::Ptr WorkCellCalibrator::getSolver() const {
			RW_ASSERT(!_solver.isNull());
			return _solver;
		}

	}
}
