/*
 * WorkCellCalibration.hpp
 *
 *  Created on: Jul 4, 2012
 *      Author: bing
 */

#ifndef RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP
#define RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP

#include "CompositeCalibration.hpp"
#include "ParallelAxisDHCalibration.hpp"
#include "JointEncoderCalibration.hpp"
#include "FixedFrameCalibration.hpp"

#include <rw/models.hpp>
#include <rw/kinematics.hpp>

namespace rwlibs {
namespace calibration { 

class WorkCellCalibration: public CompositeCalibration<CalibrationBase> {
public:
	typedef rw::common::Ptr<WorkCellCalibration> Ptr;

	WorkCellCalibration(rw::models::SerialDevice::Ptr device,
			const std::vector<rw::kinematics::Frame*>& sensorFrames,
			const std::vector<rw::math::Function<>::Ptr>& encoderCorrectionFunctions = std::vector<rw::math::Function<>::Ptr>());

	WorkCellCalibration(/*rw::models::SerialDevice::Ptr device,*/
			/*FixedFrameCalibration::Ptr sensorFrameCalibration,
			FixedFrameCalibration::Ptr endCalibration,*/
			CompositeCalibration<FixedFrameCalibration>::Ptr fixedFrameCalibrations,
			CompositeCalibration<ParallelAxisDHCalibration>::Ptr compositeLinkCalibration,
			CompositeCalibration<JointEncoderCalibration>::Ptr compositeJointCalibration);
			
	virtual ~WorkCellCalibration();

	//rw::models::SerialDevice::Ptr getDevice() const;

	//FixedFrameCalibration::Ptr getBaseCalibration() const;

	//FixedFrameCalibration::Ptr getEndCalibration() const;

	CompositeCalibration<FixedFrameCalibration>::Ptr getFixedFrameCalibrations() const;

	CompositeCalibration<ParallelAxisDHCalibration>::Ptr getCompositeLinkCalibration() const;

	CompositeCalibration<JointEncoderCalibration>::Ptr getCompositeJointCalibration() const;

	static WorkCellCalibration::Ptr make(rw::models::SerialDevice::Ptr device);

	static WorkCellCalibration::Ptr get(rw::models::SerialDevice::Ptr device);

	static WorkCellCalibration::Ptr get(const rw::common::PropertyMap& propertyMap);

	static void set(WorkCellCalibration::Ptr calibration, rw::models::SerialDevice::Ptr device);

	static void set(WorkCellCalibration::Ptr calibration, rw::common::PropertyMap& propertyMap);

private:
	//rw::models::SerialDevice::Ptr _device;
	//FixedFrameCalibration::Ptr _sensorFrameCalibration;
	//FixedFrameCalibration::Ptr _endCalibration;
	CompositeCalibration<FixedFrameCalibration>::Ptr _fixedFrameCalibrations;
	CompositeCalibration<ParallelAxisDHCalibration>::Ptr _compositeLinkCalibration;
	CompositeCalibration<JointEncoderCalibration>::Ptr _compositeJointCalibration;
};

}
}

#endif /* RWLIBS_CALIBRATION_WORKCELLCALIBRATION_HPP */
