/*
* CalibrationTest.cpp
*
*  Created on: Sep 7, 2012
*      Author: bing
*/

#include "../TestSuiteConfig.hpp"
#include "MultivariateNormalDistribution.hpp"
#include <rw/models.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;
using namespace rw::math;

std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise);

class EncoderTauFunction: public rw::math::Function<> { public: virtual double x(double q) { return -sin(q); }; };
class EncoderSigmaFunction: public rw::math::Function<> { public: virtual double x(double q) { return -cos(q); }; };

BOOST_AUTO_TEST_CASE( CalibratorTest ) {
	//_CrtSetDbgFlag(0);

	const std::string testFilesPath = testFilePath();
	BOOST_REQUIRE(!testFilesPath.empty());

	const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string referenceFrameName("SomeSensorFrame");
	const std::string measurementFrameName("SomeDevice.Marker");
	const std::string calibrationFilePath(testFilesPath + "calibration/SomeCalibration.xml");
	const int measurementCount = 40;

	// Load workcell.
	rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	BOOST_REQUIRE(!workCell.isNull());
	rw::kinematics::State state = workCell->getDefaultState();

	// Find device and frames.
	rw::models::Device::Ptr device = workCell->findDevice(deviceName);
	BOOST_REQUIRE(!device.isNull());
	rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();
	rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	BOOST_REQUIRE(!referenceFrame.isNull());
	rw::kinematics::Frame::Ptr measurementFrame = workCell->findFrame(measurementFrameName);
	BOOST_REQUIRE(!measurementFrame.isNull());

	// Find existing calibration.
	SerialDeviceCalibration::Ptr serialDeviceCalibrationExisting = SerialDeviceCalibration::get(serialDevice);
	//BOOST_CHECK(!serialDeviceCalibrationExisting.isNull());
	if (!serialDeviceCalibrationExisting.isNull())
		serialDeviceCalibrationExisting->revert();

	// Setup artificial calibration.
	const int ENCODER_PARAMETER_TAU = 0, ENCODER_PARAMETER_SIGMA = 1;
	std::vector<rw::math::Function<>::Ptr> encoderCorrectionFunctions;

	encoderCorrectionFunctions.push_back(rw::common::ownedPtr(new EncoderTauFunction()));
	encoderCorrectionFunctions.push_back(rw::common::ownedPtr(new EncoderSigmaFunction()));
	SerialDeviceCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice, encoderCorrectionFunctions)));
	artificialCalibration->getBaseCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(7.0 / 100.0, -8.0 / 100.0, 9.0 / 100.0), rw::math::RPY<>(1.9 * rw::math::Deg2Rad, -1.8 * rw::math::Deg2Rad, 1.7 * rw::math::Deg2Rad)));
	artificialCalibration->getEndCalibration()->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(1.0 / 100.0, 2.0 / 100.0, -3.0 / 100.0), rw::math::RPY<>(-0.3 * rw::math::Deg2Rad, 0.2 * rw::math::Deg2Rad, 0.1 * rw::math::Deg2Rad)));
	CompositeCalibration<DHLinkCalibration>::Ptr artificialCompositeLinkCalibration = artificialCalibration->getCompositeLinkCalibration();
	for (int calibrationIndex = 0; calibrationIndex < artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		DHLinkCalibration::Ptr artificialLinkCalibration = artificialCompositeLinkCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialLinkCalibration->getParameterSet();
		if (parameterSet(DHLinkCalibration::PARAMETER_A).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_A) = 0.3 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_B).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_B) = -0.2 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_D).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_D) = -0.1 / 100.0;
		if (parameterSet(DHLinkCalibration::PARAMETER_ALPHA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_ALPHA) = -0.6 * rw::math::Deg2Rad;
		if (parameterSet(DHLinkCalibration::PARAMETER_BETA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_BETA) = 0.5 * rw::math::Deg2Rad;
		if (parameterSet(DHLinkCalibration::PARAMETER_THETA).isEnabled())
			parameterSet(DHLinkCalibration::PARAMETER_THETA) = 0.4 * rw::math::Deg2Rad;
		artificialLinkCalibration->setParameterSet(parameterSet);
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr artificialCompositeJointCalibration = artificialCalibration->getCompositeJointCalibration();
	for (int calibrationIndex = 0; calibrationIndex < artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr artificialJointCalibration = artificialCompositeJointCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialJointCalibration->getParameterSet();
		if (parameterSet(ENCODER_PARAMETER_TAU).isEnabled())
			parameterSet(ENCODER_PARAMETER_TAU) = 0.003;
		if (parameterSet(ENCODER_PARAMETER_SIGMA).isEnabled())
			parameterSet(ENCODER_PARAMETER_SIGMA) = -0.002;
		artificialJointCalibration->setParameterSet(parameterSet);
	}

	artificialCalibration->apply();

	// Load robot pose measurements from file.
	const std::vector<SerialDevicePoseMeasurement> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame, state, measurementCount, false);
	BOOST_CHECK_EQUAL(measurements.size(), measurementCount);

	artificialCalibration->revert();

	// Initialize calibration, jacobian and calibrator.
	SerialDeviceCalibration::Ptr calibration(rw::common::ownedPtr(new SerialDeviceCalibration(serialDevice, encoderCorrectionFunctions)));
	SerialDeviceJacobian::Ptr jacobian(rw::common::ownedPtr(new SerialDeviceJacobian(calibration)));
	SerialDeviceCalibrator::Ptr calibrator(rw::common::ownedPtr(new SerialDeviceCalibrator(serialDevice, referenceFrame, measurementFrame, calibration, jacobian)));
	calibrator->setMeasurements(measurements);

	try {
		// Run calibrator.
		calibrator->calibrate(state);
	} catch (rw::common::Exception& ex) {
		BOOST_ERROR(ex.getMessage());
	}

	const int iterationCount = calibrator->getSolver()->getIterationLogs().back().getIterationNumber();
	BOOST_CHECK_EQUAL(iterationCount, 6);

	// Verify that the calibration match the artificial calibration.
	FixedFrameCalibration::Ptr baseCalibration = calibration->getBaseCalibration();
	if (baseCalibration->isEnabled()) {
		const rw::math::Transform3D<> artificialBaseCorrectionTransform = artificialCalibration->getBaseCalibration()->getCorrectionTransform();
		const rw::math::Transform3D<> baseCorrectionTransform = baseCalibration->getCorrectionTransform();
		double baseDistanceError = (artificialBaseCorrectionTransform.P() - baseCorrectionTransform.P()).norm2();
		BOOST_CHECK_SMALL(baseDistanceError, 10e-5);
		double baseAngleError = rw::math::EAA<>(artificialBaseCorrectionTransform.R() * rw::math::inverse(baseCorrectionTransform.R())).angle();
		BOOST_CHECK_SMALL(baseAngleError, 10e-5);
	}
	FixedFrameCalibration::Ptr endCalibration = calibration->getEndCalibration();
	if (endCalibration->isEnabled()) {
		const rw::math::Transform3D<> artificialEndCorrectionTransform = artificialCalibration->getEndCalibration()->getCorrectionTransform();
		const rw::math::Transform3D<> endCorrectionTransform = endCalibration->getCorrectionTransform();
		double endDistanceError = (artificialEndCorrectionTransform.P() - endCorrectionTransform.P()).norm2();
		BOOST_CHECK_SMALL(endDistanceError, 10e-5);
		double endAngleError = rw::math::EAA<>(artificialEndCorrectionTransform.R() * rw::math::inverse(endCorrectionTransform.R())).angle();
		BOOST_CHECK_SMALL(endAngleError, 10e-5);
	}
	CompositeCalibration<DHLinkCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	if (compositeLinkCalibration->isEnabled()) {
		for (int calibrationIndex = 0; calibrationIndex < artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
			DHLinkCalibration::Ptr calibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			if (calibration->isEnabled()) {
				const CalibrationParameterSet parameterSet = calibration->getParameterSet();
				const CalibrationParameterSet artificialParameterSet = artificialCompositeLinkCalibration->getCalibration(calibrationIndex)->getParameterSet();
				for (int parameterIndex = 0; parameterIndex < artificialParameterSet.getCount(); parameterIndex++) {
					if (artificialParameterSet(parameterIndex).isEnabled()) {
						BOOST_CHECK_CLOSE(parameterSet(parameterIndex).getValue(), artificialParameterSet(parameterIndex).getValue(), 10e-5);
					}
				}
			}
		}
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointCalibration();
	if (compositeJointCalibration->isEnabled()) {
		for (int calibrationIndex = 0; calibrationIndex < artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
			JointEncoderCalibration::Ptr calibration = compositeJointCalibration->getCalibration(calibrationIndex);
			if (calibration->isEnabled()) {
				const CalibrationParameterSet parameterSet = calibration->getParameterSet();
				const CalibrationParameterSet artificialParameterSet = artificialCompositeJointCalibration->getCalibration(calibrationIndex)->getParameterSet();
				for (int parameterIndex = 0; parameterIndex < artificialParameterSet.getCount(); parameterIndex++) {
					if (artificialParameterSet(parameterIndex).isEnabled()) {
						BOOST_CHECK_CLOSE(parameterSet(parameterIndex).getValue(), artificialParameterSet(parameterIndex).getValue(), 10e-5);
					}
				}
			}
		}
	}

	calibration->apply();

	// Verify that calibration fits measurements.
	for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex].getQ(), state);

		const rw::math::Transform3D<> measurementTransform = measurements[measurementIndex].getTransform();
		const rw::math::Transform3D<> modelTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);

		double measurementDistanceError = (modelTransform.P() - measurementTransform.P()).norm2();
		BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
		double measurementAngleError = rw::math::EAA<>(modelTransform.R() * rw::math::inverse(measurementTransform.R())).angle();
		BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}

	calibration->revert();

	//XmlCalibrationSaver::save(calibration, calibrationFilePath);
	XmlCalibrationSaver::save(*calibration, std::string("SomeCalibration.xml"));
	SerialDeviceCalibration::Ptr calibrationLoaded = XmlCalibrationLoader::load(workCell->getStateStructure(), serialDevice, "SomeCalibration.xml");
	//SerialDeviceCalibration::Ptr calibrationLoaded = calibration;
	calibrationLoaded->apply();

	// Verify that the loaded calibration match the artificial calibration.
	BOOST_CHECK(toEigen(calibrationLoaded->getBaseCalibration()->getCorrectionTransform()).isApprox(
			toEigen(artificialCalibration->getBaseCalibration()->getCorrectionTransform()), 10e-5));
	BOOST_CHECK(toEigen(calibrationLoaded->getEndCalibration()->getCorrectionTransform()).isApprox(
			toEigen(artificialCalibration->getEndCalibration()->getCorrectionTransform()), 10e-5));
	CompositeCalibration<DHLinkCalibration>::Ptr internalLinkCalibrationsLoaded =
			calibrationLoaded->getCompositeLinkCalibration();
	for (unsigned int calibrationIndex = 0; calibrationIndex < internalLinkCalibrationsLoaded->getCalibrationCount(); calibrationIndex++){
		for (int parameterIndex = 0; parameterIndex < 4; parameterIndex++){
			/*
			if (artificialInternalLinkCalibrations[calibrationIndex]->isParameterEnabled(parameterIndex)) {
				double diff = abs(abs(internalLinkCalibrationsLoaded[calibrationIndex]->getParameterValue(parameterIndex)) - abs(artificialInternalLinkCalibrations[calibrationIndex]->getParameterValue(parameterIndex)));
				BOOST_CHECK_CLOSE(diff, 0.0, 10e-5);
			}
			*/
		}
	}

	// Verify that loaded calibration fits measurements.
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex].getQ(), state);

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex].getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		const rw::math::Transform3D<> tfmError(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));

		double measurementDistanceError = (tfmModel.P() - tfmMeasurement.P()).norm2();
		BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
		double measurementAngleError = rw::math::EAA<>(tfmModel.R() * rw::math::inverse(tfmMeasurement.R())).angle();
		BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}
}


BOOST_AUTO_TEST_CASE( CalibrationMeasureSaveLoad ) {
	//
	//const std::string testFilesPath = testFilePath();
	//BOOST_REQUIRE(!testFilesPath.empty());	
	//const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	//const std::string deviceName("SomeDevice");
	//const std::string referenceFrameName("SomeSensorFrame");
	//const std::string measurementFrameName("SomeDevice.Marker");
	//const std::string measurementFilePath(testFilesPath + "calibration/TestMeasurements.xml");
	//const int measurementCount = 2;

	//// Load workcell.
	//rw::models::WorkCell::Ptr workCell = rw::loaders::XMLRWLoader::load(workCellFilePath);
	//BOOST_REQUIRE(!workCell.isNull());
	//rw::kinematics::State state = workCell->getDefaultState();

	//// Find device and frames.
	//rw::models::Device::Ptr device = workCell->findDevice(deviceName);
	//BOOST_REQUIRE(!device.isNull());
	//rw::models::SerialDevice::Ptr serialDevice = device.cast<rw::models::SerialDevice>();
	//rw::kinematics::Frame::Ptr referenceFrame = workCell->findFrame(referenceFrameName);
	//BOOST_REQUIRE(!referenceFrame.isNull());
	//rw::kinematics::Frame::Ptr measurementFrame = workCell->findFrame(measurementFrameName);
	//BOOST_REQUIRE(!measurementFrame.isNull());


	//
	//const std::vector<SerialDevicePoseMeasurement> measurements = generateMeasurements(serialDevice, referenceFrame, measurementFrame, state, measurementCount, false);
	//BOOST_CHECK_EQUAL(measurements.size(), measurementCount);

	//XmlMeasurementFile::save(measurements, measurementFilePath);

	//std::vector<SerialDevicePoseMeasurement> loadedMeasurements = XmlMeasurementFile::load(measurementFilePath);

	//BOOST_CHECK_EQUAL(measurements.size(), loadedMeasurements.size());

	//for (size_t i = 0; i<measurements.size(); i++) 
	//{
	//	//Check the configuration
	//	BOOST_CHECK_SMALL((measurements[i].getQ() - loadedMeasurements[i].getQ()).normInf(), 1e-9);
	//	
	//	//Check the transform
	//	Transform3D<> t3d1 = measurements[i].getTransform();
	//	Transform3D<> t3d2 = loadedMeasurements[i].getTransform();
	//	BOOST_CHECK_SMALL((t3d1.P() - t3d2.P()).normInf(), 1e-9);

	//	Rotation3D<> r3d = t3d1.R() * inverse(t3d2.R());
	//	BOOST_CHECK_SMALL(EAA<>(r3d).angle(), 1e-9);

	//}


}


std::vector<SerialDevicePoseMeasurement> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, rw::kinematics::Frame::Ptr referenceFrame, rw::kinematics::Frame::Ptr measurementFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise) {
	MultivariateNormalDistribution<double, 6> mvnd(time(0));

	std::vector<SerialDevicePoseMeasurement> measurements;
	for (unsigned int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
		serialDevice->setQ(q, state);

		rw::math::Transform3D<> transform = rw::kinematics::Kinematics::frameTframe(referenceFrame.get(), measurementFrame.get(), state);
		Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
		if (addNoise) {
			Eigen::Matrix<double, 6, 6> random = Eigen::Matrix<double, 6, 6>::Random();
			covariance = random.transpose() * random;
			covariance.block<3, 3>(0, 0) /= 50.0;
			covariance.block<3, 3>(3, 3) /= 5.0;
			covariance.block<3, 3>(3, 0) /= 1000.0;
			covariance.block<3, 3>(0, 3) /= 1000.0;
			covariance /= 10e14;

			Eigen::Matrix<double, 6, 1> mvndVector = mvnd.draw(covariance);
			transform.P() = rw::math::Vector3D<>(mvndVector(0), mvndVector(1), mvndVector(2)) + transform.P();
			transform.R() = rw::math::RPY<>(mvndVector(3), mvndVector(4), mvndVector(5)).toRotation3D() * transform.R();
		}

		measurements.push_back(SerialDevicePoseMeasurement(q, transform, covariance));
	}

	return measurements;
}
