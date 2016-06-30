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

#ifndef RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_
#define RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_

//#define EIGEN_TRANSFORM_PLUGIN "rwlibs/calibration/EigenTransformPlugin.hpp"

#include "JacobianBase.hpp"
#include <Eigen/Geometry>

namespace rw { namespace models { class Joint; } }

namespace rwlibs {
namespace calibration {
class ParallelAxisDHCalibration;

/** @addtogroup calibration */
/*@{*/

/**
 * @brief Implementation of Jacobian for the ParallelAxisDHCalibration
 */
class ParallelAxisDHJacobian: public JacobianBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/** @brief Smart pointer declaration */
	typedef rw::common::Ptr<ParallelAxisDHJacobian> Ptr;

	/**
	 * @brief Constructs Jacobian based on \bcalibration
	 */
	ParallelAxisDHJacobian(rw::common::Ptr<ParallelAxisDHCalibration> calibration);

	/**
	 * @brief Destructor
	 */
	virtual ~ParallelAxisDHJacobian();

protected:
	/**
	 * @copydoc Jacobian::doComputeJacobian()
	 */
	virtual Eigen::MatrixXd doComputeJacobian(rw::common::Ptr<rw::kinematics::Frame> referenceFrame, rw::common::Ptr<rw::kinematics::Frame> targetFrame, const rw::kinematics::State& state);

private:
	rw::common::Ptr<ParallelAxisDHCalibration> _calibration;
	rw::common::Ptr<rw::models::Joint> _joint;
};

/* @} */

}
}

#endif /* RWLIBS_CALIBRATION_PARALLELAXISDHJACOBIAN_HPP_ */
