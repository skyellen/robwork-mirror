#include "WorkCellCalibrator.hpp"
#include "WorkCellExtrinsicCalibrator.hpp"
#include "nlls/NLLSNewtonSolver.hpp"

#include <rw/math/EAA.hpp>

#include <Eigen/Eigenvalues>

using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;


namespace rwlibs {
	namespace calibration {

		class WorkCellCalibrationSystem: public NLLSSystem {
		public:
			typedef rw::common::Ptr<WorkCellCalibrationSystem> Ptr;

			WorkCellCalibrationSystem(rw::models::WorkCell::Ptr workcell, rw::kinematics::State workCellState, const std::vector<CalibrationMeasurement::Ptr>& measurements, Calibration::Ptr calibration, Jacobian::Ptr jacobian, bool isWeightingMeasurements) : 
				_workcell(workcell), 
				_workCellState(workCellState), /*
				_referenceFrame(referenceFrame), 
				_measurementFrame(measurementFrame), */
				_measurements(measurements), 
				_calibration(calibration), 
				_jacobian(jacobian), 
				_isWeightingMeasurements(isWeightingMeasurements) 
			{

			}

			virtual ~WorkCellCalibrationSystem() {

			}

		private:

			Device::Ptr getDevice(CalibrationMeasurement::Ptr measurement) {
				Device::Ptr device = _workcell->findDevice(measurement->getDeviceName());
				if (device.isNull())
					RW_THROW("No device in work cell which matches the name: '"<<measurement->getDeviceName());
				return device;
			}

			Frame* getSensorFrame(CalibrationMeasurement::Ptr measurement) {
				Frame* frame = _workcell->findFrame(measurement->getSensorFrameName());
				if (frame == NULL)
					RW_THROW("No frame in work cell which matches the name: '"<<measurement->getSensorFrameName());
				return frame;
			}

			Frame* getMarkerFrame(CalibrationMeasurement::Ptr measurement) {
				Frame* frame = _workcell->findFrame(measurement->getMarkerFrameName());
				if (frame == NULL)
					RW_THROW("No frame in work cell which matches the name: '"<<measurement->getMarkerFrameName());
				return frame;
			}


			virtual void computeJacobian(Eigen::MatrixXd& stackedJacobians) {
				const int measurementCount = _measurements.size();
				const int rowCount = 6 * measurementCount;
				const int columnCount = _jacobian->getColumnCount();
				RW_ASSERT(columnCount > 0);
				RW_ASSERT(rowCount >= columnCount);

				stackedJacobians.resize(rowCount, columnCount);
				rw::kinematics::State workCellState = _workCellState;
				for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const CalibrationMeasurement::Ptr measurement = _measurements[measurementIndex];

					Device::Ptr device = getDevice(measurement);
					Frame* sensorFrame = getSensorFrame(measurement);
					Frame* markerFrame = getMarkerFrame(measurement);
					// Update state according to current measurement.
					const rw::math::Q q = measurement->getQ();
					device->setQ(q, workCellState);

					// Compute Jacobian.
					const int rowIndex = 6 * measurementIndex;
					stackedJacobians.block(rowIndex, 0, 6, columnCount) = _jacobian->computeJacobian(sensorFrame, markerFrame, workCellState);
					//if (measurementIndex == 0) {
					//	std::cout<<"Jacobian[0] = "<<_jacobian->computeJacobian(sensorFrame, markerFrame, workCellState)<<std::endl;
					//}
					// Weight Jacobian according to covariances.
					if (_isWeightingMeasurements && measurement->hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(measurement->getCovarianceMatrix()).operatorInverseSqrt();
						//const Eigen::MatrixXd weightMatrix = measurement->getCovarianceMatrix();
						stackedJacobians.block(rowIndex, 0, 6, columnCount) = weightMatrix * stackedJacobians.block(rowIndex, 0, 6, columnCount);
					}
				}
				
			}

			virtual void computeResiduals(Eigen::VectorXd& stackedResiduals) {
				const int measurementCount = _measurements.size();

				const int rowCount = 6 * measurementCount;
				stackedResiduals.resize(6 * measurementCount);
				rw::kinematics::State workCellState = _workCellState;
				for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
					const CalibrationMeasurement::Ptr measurement = _measurements[measurementIndex];

					Device::Ptr device = getDevice(measurement);
					Frame* sensorFrame = getSensorFrame(measurement);
					Frame* markerFrame = getMarkerFrame(measurement);

					// Update state according to current measurement.
					const rw::math::Q q = measurement->getQ();
					device->setQ(q, workCellState);

					// Compute residuals.
					const int rowIndex = 6 * measurementIndex;
					const rw::math::Transform3D<> tfmMeasurement = measurement->getTransform();
					//std::cout<<"Measurement = "<<tfmMeasurement<<std::endl;
					//std::cout<<"Error Reference Frame = "<<_referenceFrame->getName()<<std::endl;
					const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(sensorFrame, markerFrame, workCellState);
					//std::cout<<"Model Transform = "<<tfmModel<<std::endl;
						
					//rw::math::Vector3D<> dP = tfmModel.P() - tfmMeasurement.P();
					rw::math::Vector3D<> dP = tfmMeasurement.P() - tfmModel.P();
					//std::cout<<"dP = "<<dP<<std::endl;
					//dP = -dP;
					stackedResiduals(rowIndex + 0) = dP(0);
					stackedResiduals(rowIndex + 1) = dP(1);
					stackedResiduals(rowIndex + 2) = dP(2);
					//rw::math::Rotation3D<> dR = tfmModel.R() * rw::math::inverse(tfmMeasurement.R());
					rw::math::Rotation3D<> dR = tfmMeasurement.R()*inverse(tfmModel.R());

					//std::cout<<"dR = "<<EAA<>(dR)<<std::endl;
					//dR = Rotation3D<>::identity();

					//if (measurementIndex == 0) {
					//	std::cout<<" dp = "<<dP<<std::endl;
					//	std::cout<<"dr = "<<(dR(2, 1) - dR(1, 2)) / 2<<"  "<<(dR(0, 2) - dR(2, 0)) / 2<<"  "<<(dR(1, 0) - dR(0, 1)) / 2<<std::endl;
					//}

					stackedResiduals(rowIndex + 3) = (dR(2, 1) - dR(1, 2)) / 2;
					stackedResiduals(rowIndex + 4) = (dR(0, 2) - dR(2, 0)) / 2;
					stackedResiduals(rowIndex + 5) = (dR(1, 0) - dR(0, 1)) / 2;

					// Weight residuals according to covariances.
					if (_isWeightingMeasurements && measurement->hasCovarianceMatrix()) {
						const Eigen::Matrix<double, 6, 6> weightMatrix = Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> >(measurement->getCovarianceMatrix()).operatorInverseSqrt();
						//const Eigen::MatrixXd weightMatrix = measurement->getCovarianceMatrix();
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
			rw::models::WorkCell::Ptr _workcell;
			//rw::models::SerialDevice::Ptr _device;
			rw::kinematics::State _workCellState;
			//rw::kinematics::Frame::Ptr _referenceFrame;
			//rw::kinematics::Frame::Ptr _measurementFrame;
			std::vector<CalibrationMeasurement::Ptr> _measurements;
			Calibration::Ptr _calibration;
			Jacobian::Ptr _jacobian;
			bool _isWeightingMeasurements;
		};

		WorkCellCalibrator::WorkCellCalibrator(rw::models::WorkCell::Ptr workcell,
			/*rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, */Calibration::Ptr calibration, Jacobian::Ptr jacobian) :
		_workcell(workcell), /*_referenceFrame(referenceFrame), _measurementFrame(measurementFrame), */_calibration(calibration), _jacobian(jacobian), _isWeightingMeasurements(true)/*, _usePreCalibration(true)*/ {

		}

		WorkCellCalibrator::~WorkCellCalibrator() {

		}

		rw::models::WorkCell::Ptr WorkCellCalibrator::getWorkCell() const {
			return _workcell;
		}

/*		rw::kinematics::Frame::Ptr WorkCellCalibrator::getReferenceFrame() const {
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
		}*/

		unsigned int WorkCellCalibrator::getMinimumMeasurementCount() const {
			return static_cast<unsigned int>(ceil(float(_jacobian->getColumnCount()) / 6));
		}

		int WorkCellCalibrator::getMeasurementCount() const {
			return _measurements.size();
		}

		void WorkCellCalibrator::addMeasurement(CalibrationMeasurement::Ptr measurement) {
			_measurements.push_back(measurement);
		}

		void WorkCellCalibrator::addMeasurement(const rw::math::Q& q, const rw::math::Transform3D<>& transform, const Eigen::Matrix<double, 6, 6>& covarianceMatrix) {
			_measurements.push_back(rw::common::ownedPtr(new CalibrationMeasurement(q, transform, covarianceMatrix)));
		}

		void WorkCellCalibrator::setMeasurements(const std::vector<CalibrationMeasurement::Ptr>& measurements) {
			_measurements = measurements;
		}

		bool WorkCellCalibrator::isWeightingMeasurements() const {
			return _isWeightingMeasurements;
		}

		void WorkCellCalibrator::setWeightingMeasurements(bool isWeightingMeasurements) {
			_isWeightingMeasurements = isWeightingMeasurements;
		}

		/*
		void WorkCellCalibrator::setUsePreCalibration(bool usePrecalibration) {
			_usePreCalibration = usePrecalibration;
		}

		bool WorkCellCalibrator::isUsingPreCalibration() const {
			return _usePreCalibration;
		}
		*/


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
				WorkCellCalibrationSystem::Ptr system = rw::common::ownedPtr(new WorkCellCalibrationSystem(_workcell, workCellState, _measurements, _calibration, _jacobian, _isWeightingMeasurements));
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
