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


#include "../TestSuiteConfig.hpp"
#include "MultivariateNormalDistribution.hpp"
#include <rw/models/SerialDevice.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rwlibs/calibration.hpp>

using namespace rwlibs::calibration;
using namespace rw::math;
using namespace rw::kinematics;
using rw::models::SerialDevice;

std::vector<CalibrationMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, const std::vector<Frame*>& sensorFrames, Frame::Ptr markerFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise);

BOOST_AUTO_TEST_CASE( CalibratorTest ) {
#ifdef _WIN32
	_CrtSetDbgFlag(0);
#endif
	const std::string testFilesPath = testFilePath();
	BOOST_REQUIRE(!testFilesPath.empty());

	const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string sensorFrameName1("SomeSensorFrame1");
	const std::string sensorFrameName2("SomeSensorFrame2");
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
	rw::kinematics::Frame::Ptr sensorFrame1 = workCell->findFrame(sensorFrameName1);
	rw::kinematics::Frame::Ptr sensorFrame2 = workCell->findFrame(sensorFrameName2);
	BOOST_REQUIRE(!sensorFrame1.isNull());
	BOOST_REQUIRE(!sensorFrame2.isNull());
	rw::kinematics::Frame::Ptr measurementFrame = workCell->findFrame(measurementFrameName);
	BOOST_REQUIRE(!measurementFrame.isNull());

	// Find existing calibration.
	//WorkCellCalibration::Ptr workCellCalibrationExisting = WorkCellCalibration::get(serialDevice);
	//BOOST_CHECK(!serialDeviceCalibrationExisting.isNull());
	//if (!workCellCalibrationExisting.isNull())
	//	workCellCalibrationExisting->revert();

	// Setup artificial calibration.
	std::vector<rw::kinematics::Frame*> sensorFrames;
	sensorFrames.push_back(sensorFrame1.get());
	sensorFrames.push_back(sensorFrame2.get());

	std::vector<WorkCellCalibration::DeviceMarkerPair> deviceMarkerPairs;
	deviceMarkerPairs.push_back(std::make_pair(serialDevice, measurementFrame.get()));

	WorkCellCalibration::Ptr artificialCalibration(rw::common::ownedPtr(new WorkCellCalibration(deviceMarkerPairs, sensorFrames)));
	artificialCalibration->getFixedFrameCalibrations()->getCalibration(0)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(10.0 / 1000.0, -8.0 / 1000.0, 7 / 1000.0),
																														  rw::math::RPY<>(1.7 * rw::math::Deg2Rad, 0.7 * rw::math::Deg2Rad, -2.0 * rw::math::Deg2Rad)));
	artificialCalibration->getFixedFrameCalibrations()->getCalibration(1)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(-9.0 / 1000.0, 11.0 / 1000.0, 17.0 / 1000.0),
																														  rw::math::RPY<>(1 * rw::math::Deg2Rad, 1.3 * rw::math::Deg2Rad, -1.4 * rw::math::Deg2Rad)));
	//artificialCalibration->getFixedFrameCalibrations()->getCalibration(1)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(3.0 / 1000.0, -11.0 / 1000.0, -3.0 / 1000.0), rw::math::RPY<>(1.3 * rw::math::Deg2Rad, -1.2 * rw::math::Deg2Rad, 0.7 * rw::math::Deg2Rad)));
	artificialCalibration->getFixedFrameCalibrations()->getCalibration(2)->setCorrectionTransform(rw::math::Transform3D<>(rw::math::Vector3D<>(1.0 / 1000.0, 2.0 / 1000.0, -3.0 / 1000.0), rw::math::RPY<>(-0.3 * rw::math::Deg2Rad, 0.2 * rw::math::Deg2Rad, 0.1 * rw::math::Deg2Rad)));
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr artificialCompositeLinkCalibration = artificialCalibration->getCompositeLinkCalibration();
	for (int calibrationIndex = 0; calibrationIndex < artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
		ParallelAxisDHCalibration::Ptr artificialLinkCalibration = artificialCompositeLinkCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialLinkCalibration->getParameterSet();
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_A).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_A) = 0.3 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_B).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_B) = -0.2 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_D).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_D) = -0.1 / 100.0;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_ALPHA) = -0.6 * rw::math::Deg2Rad;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_BETA) = 0.5 * rw::math::Deg2Rad;
		if (parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA).isEnabled())
			parameterSet(ParallelAxisDHCalibration::PARAMETER_THETA) = 0.4 * rw::math::Deg2Rad;
		artificialLinkCalibration->setParameterSet(parameterSet);
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr artificialCompositeJointCalibration = artificialCalibration->getCompositeJointEncoderCalibration();
	for (int calibrationIndex = 0; calibrationIndex < artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
		JointEncoderCalibration::Ptr artificialJointCalibration = artificialCompositeJointCalibration->getCalibration(calibrationIndex);
		CalibrationParameterSet parameterSet = artificialJointCalibration->getParameterSet();
		if (parameterSet(JointEncoderCalibration::PARAMETER_TAU).isEnabled())
			parameterSet(JointEncoderCalibration::PARAMETER_TAU) = 0.003;
		if (parameterSet(JointEncoderCalibration::PARAMETER_SIGMA).isEnabled())
			parameterSet(JointEncoderCalibration::PARAMETER_SIGMA) = -0.002;
		artificialJointCalibration->setParameterSet(parameterSet);
	}

	artificialCalibration->apply();

	// Load robot pose measurements from file.
	const std::vector<CalibrationMeasurement::Ptr> measurements = generateMeasurements(serialDevice, sensorFrames, measurementFrame, state, measurementCount, false);
	BOOST_CHECK_EQUAL(measurements.size(), measurementCount);

	artificialCalibration->revert();
	//// Initialize calibration, jacobian and calibrator.
	WorkCellCalibration::Ptr calibration(rw::common::ownedPtr(new WorkCellCalibration(deviceMarkerPairs, sensorFrames)));
	WorkCellJacobian::Ptr jacobian(rw::common::ownedPtr(new WorkCellJacobian(calibration)));
	WorkCellCalibrator::Ptr calibrator(rw::common::ownedPtr(new WorkCellCalibrator(workCell, calibration, jacobian)));
	calibrator->setMeasurements(measurements);


	calibration->getFixedFrameCalibrations()->getCalibration(0)->setEnabled(true);
	calibration->getFixedFrameCalibrations()->getCalibration(1)->setEnabled(true);
	calibration->getFixedFrameCalibrations()->getCalibration(2)->setEnabled(true);
	calibration->getCompositeLinkCalibration()->setEnabled(true);
	calibration->getCompositeJointEncoderCalibration()->setEnabled(true);


	try {
		// Run calibrator.
		calibrator->calibrate(state);
	} catch (rw::common::Exception& ex) {
		BOOST_ERROR(ex.getMessage());
	}

	// const int iterationCount = calibrator->getSolver()->getIterationLogs().back().getIterationNumber();  // not used
	//BOOST_CHECK_EQUAL(iterationCount, 6);

	//// Verify that the calibration match the artificial calibration.
	for (int i = 0; i < calibration->getFixedFrameCalibrations()->getCalibrationCount(); i++) {
		FixedFrameCalibration::Ptr ffCalibration = calibration->getFixedFrameCalibrations()->getCalibration(i);
		if (ffCalibration->isEnabled()) {
			FixedFrameCalibration::Ptr artificialFFCalibration = artificialCalibration->getFixedFrameCalibrations()->getCalibration(i);
			const rw::math::Transform3D<> artificialCorrection = artificialFFCalibration->getCorrectionTransform();
			const rw::math::Transform3D<> actualCorrection = ffCalibration->getCorrectionTransform();

			std::cout<<"FixedFrameCalibration: "<<artificialFFCalibration->getFrame()->getName()<<" Artificial: "<<artificialFFCalibration->getParameterSet()<<" Calibrated: "<<ffCalibration->getParameterSet()<<std::endl;
			std::cout<<"FixedFrameCalibration: "<<artificialFFCalibration->getFrame()->getName()<<" Artificial: "<<artificialCorrection.P()<<" Calibrated: "<<actualCorrection.P()<<std::endl;
			//std::cout<<"Artificial.P() = "<<correctionSensorFrame1Transform.P()<<" vs. "<<baseCorrectionTransform.P()<<" Distance = "<<(artificialBaseCorrectionTransform.P() - baseCorrectionTransform.P()).norm2()<<std::endl;
			double baseDistanceError = (artificialCorrection.P() - actualCorrection.P()).norm2();
			BOOST_CHECK_SMALL(baseDistanceError, 10e-5);
			double baseAngleError = rw::math::EAA<>(artificialCorrection.R() * rw::math::inverse(actualCorrection.R())).angle();
			std::cout<<"Angle Error = "<<baseAngleError<<std::endl;
			BOOST_CHECK_SMALL(baseAngleError, 10e-5);
		}
	}
	return;

	CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration = calibration->getCompositeLinkCalibration();
	if (compositeLinkCalibration->isEnabled()) {
		for (int calibrationIndex = 0; calibrationIndex < artificialCompositeLinkCalibration->getCalibrationCount(); calibrationIndex++) {
			ParallelAxisDHCalibration::Ptr calibration = compositeLinkCalibration->getCalibration(calibrationIndex);
			if (calibration->isEnabled()) {
				const CalibrationParameterSet parameterSet = calibration->getParameterSet();
				const CalibrationParameterSet artificialParameterSet = artificialCompositeLinkCalibration->getCalibration(calibrationIndex)->getParameterSet();
				for (int parameterIndex = 0; parameterIndex < artificialParameterSet.getCount(); parameterIndex++) {
					if (artificialParameterSet(parameterIndex).isEnabled()) {
						std::cout<<"Link Check Parameter = "<<parameterSet(parameterIndex).getValue()<<" and "<<artificialParameterSet(parameterIndex).getValue()<<std::endl;
						BOOST_CHECK_CLOSE(parameterSet(parameterIndex).getValue(), artificialParameterSet(parameterIndex).getValue(), 10e-5);
					}
				}
			}
		}
	}
	CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration = calibration->getCompositeJointEncoderCalibration();
	if (compositeJointCalibration->isEnabled()) {
		for (int calibrationIndex = 0; calibrationIndex < artificialCompositeJointCalibration->getCalibrationCount(); calibrationIndex++) {
			JointEncoderCalibration::Ptr calibration = compositeJointCalibration->getCalibration(calibrationIndex);
			if (calibration->isEnabled()) {
				const CalibrationParameterSet parameterSet = calibration->getParameterSet();
				const CalibrationParameterSet artificialParameterSet = artificialCompositeJointCalibration->getCalibration(calibrationIndex)->getParameterSet();
				for (int parameterIndex = 0; parameterIndex < artificialParameterSet.getCount(); parameterIndex++) {
					if (artificialParameterSet(parameterIndex).isEnabled()) {
						std::cout<<"JointCheck Parameter = "<<parameterSet(parameterIndex).getValue()<<" and "<<artificialParameterSet(parameterIndex).getValue()<<std::endl;
						BOOST_CHECK_CLOSE(parameterSet(parameterIndex).getValue(), artificialParameterSet(parameterIndex).getValue(), 10e-5);
					}
				}
			}
		}
	}

	calibration->apply();

	//// Verify that calibration fits measurements.
	for (int measurementIndex = 0; measurementIndex < measurementCount; measurementIndex++) {
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		const rw::math::Transform3D<> measurementTransform = measurements[measurementIndex]->getTransform();

		Frame* referenceFrame = workCell->findFrame(measurements[measurementIndex]->getSensorFrameName());

		const rw::math::Transform3D<> modelTransform = rw::kinematics::Kinematics::frameTframe(referenceFrame, measurementFrame.get(), state);

		double measurementDistanceError = (modelTransform.P() - measurementTransform.P()).norm2();
		BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
		double measurementAngleError = rw::math::EAA<>(modelTransform.R() * rw::math::inverse(measurementTransform.R())).angle();
		BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}

	calibration->revert();

	//XmlCalibrationSaver::save(calibration, calibrationFilePath);
	XmlCalibrationSaver::save(calibration, std::string("SomeCalibration.xml"));
	WorkCellCalibration::Ptr calibrationLoaded = XmlCalibrationLoader::load(workCell, "SomeCalibration.xml");
	//SerialDeviceCalibration::Ptr calibrationLoaded = calibration;
	calibrationLoaded->apply();

	// Verify that the loaded calibration match the artificial calibration.
	BOOST_CHECK(toEigen(calibrationLoaded->getFixedFrameCalibrations()->getCalibration(0)->getCorrectionTransform()).isApprox(
			toEigen(artificialCalibration->getFixedFrameCalibrations()->getCalibration(0)->getCorrectionTransform()), 10e-5));
	BOOST_CHECK(toEigen(calibrationLoaded->getFixedFrameCalibrations()->getCalibration(1)->getCorrectionTransform()).isApprox(
			toEigen(artificialCalibration->getFixedFrameCalibrations()->getCalibration(1)->getCorrectionTransform()), 10e-5));
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr internalLinkCalibrationsLoaded =
			calibrationLoaded->getCompositeLinkCalibration();
	for (size_t calibrationIndex = 0; calibrationIndex < static_cast<size_t>(internalLinkCalibrationsLoaded->getCalibrationCount()); calibrationIndex++){
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
		serialDevice->setQ(measurements[measurementIndex]->getQ(), state);

		Frame* referenceFrame = workCell->findFrame(measurements[measurementIndex]->getSensorFrameName());

		const rw::math::Transform3D<> tfmMeasurement = measurements[measurementIndex]->getTransform();
		const rw::math::Transform3D<> tfmModel = rw::kinematics::Kinematics::frameTframe(referenceFrame, measurementFrame.get(), state);
		const rw::math::Transform3D<> tfmError(tfmModel.P() - tfmMeasurement.P(), tfmModel.R() * rw::math::inverse(tfmMeasurement.R()));

		double measurementDistanceError = (tfmModel.P() - tfmMeasurement.P()).norm2();
		BOOST_CHECK_SMALL(measurementDistanceError, 10e-5);
		double measurementAngleError = rw::math::EAA<>(tfmModel.R() * rw::math::inverse(tfmMeasurement.R())).angle();
		BOOST_CHECK_SMALL(measurementAngleError, 10e-5);
	}
}


BOOST_AUTO_TEST_CASE( CalibrationMeasureSaveLoad ) {

	const std::string testFilesPath = testFilePath();
	BOOST_REQUIRE(!testFilesPath.empty());
	const std::string workCellFilePath(testFilesPath + "calibration/Scene/SomeScene.wc.xml");
	const std::string deviceName("SomeDevice");
	const std::string referenceFrameName("SomeSensorFrame1");
	const std::string measurementFrameName("SomeDevice.Marker");
	const std::string measurementFilePath("TestMeasurements.xml");
	const int measurementCount = 2;

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


	std::vector<Frame*> sensorFrames;
	sensorFrames.push_back(referenceFrame.get());
	const std::vector<CalibrationMeasurement::Ptr> measurements = generateMeasurements(serialDevice, sensorFrames, measurementFrame, state, measurementCount, false);
	BOOST_CHECK_EQUAL(measurements.size(), measurementCount);

	std::cout<<"Ready to try to save data"<<std::endl;
	XMLCalibrationMeasurementFile<>::save(measurements, measurementFilePath);
	std::cout<<"Data Saved"<<std::endl;
	std::vector<CalibrationMeasurement::Ptr> loadedMeasurements = XMLCalibrationMeasurementFile<>::load(measurementFilePath);
	std::cout<<"Loaded Data"<<std::endl;
	BOOST_CHECK_EQUAL(measurements.size(), loadedMeasurements.size());

	for (size_t i = 0; i<measurements.size(); i++)
	{
		//Check the configuration
		BOOST_CHECK_SMALL((measurements[i]->getQ() - loadedMeasurements[i]->getQ()).normInf(), 1e-9);

		//Check the transform
		Transform3D<> t3d1 = measurements[i]->getTransform();
		Transform3D<> t3d2 = loadedMeasurements[i]->getTransform();
		BOOST_CHECK_SMALL((t3d1.P() - t3d2.P()).normInf(), 1e-9);

		Rotation3D<> r3d = t3d1.R() * inverse(t3d2.R());
		BOOST_CHECK_SMALL(EAA<>(r3d).angle(), 1e-9);

		BOOST_CHECK_EQUAL(measurements[i]->getDeviceName() , loadedMeasurements[i]->getDeviceName());
		BOOST_CHECK_EQUAL(measurements[i]->getMarkerFrameName() , loadedMeasurements[i]->getMarkerFrameName());
		BOOST_CHECK_EQUAL(measurements[i]->getSensorFrameName() , loadedMeasurements[i]->getSensorFrameName());

	}


}


std::vector<CalibrationMeasurement::Ptr> generateMeasurements(rw::models::SerialDevice::Ptr serialDevice, const std::vector<Frame*>& sensorFrames, rw::kinematics::Frame::Ptr markerFrame, rw::kinematics::State state, unsigned int measurementCount, bool addNoise) {
	MultivariateNormalDistribution<double, 6> mvnd(static_cast<unsigned int>(time(0)));

	std::vector<CalibrationMeasurement::Ptr> measurements;

	BOOST_FOREACH(Frame* sensorFrame, sensorFrames) {
		for (unsigned int measurementIndex = 0; measurementIndex < measurementCount/sensorFrames.size(); measurementIndex++) {
			rw::math::Q q = rw::math::Math::ranQ(serialDevice->getBounds());
			serialDevice->setQ(q, state);

			rw::math::Transform3D<> transform = rw::kinematics::Kinematics::frameTframe(sensorFrame, markerFrame.get(), state);
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

			measurements.push_back(rw::common::ownedPtr(new CalibrationMeasurement(q, transform, covariance, serialDevice->getName(), sensorFrame->getName(), markerFrame->getName())));
		}
	}
	return measurements;
}
