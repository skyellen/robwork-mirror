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

#ifndef RWLIBS_ALGORITHMS_STABLEPLANEPOSE_HPP_
#define RWLIBS_ALGORITHMS_STABLEPLANEPOSE_HPP_

#include "StablePose.hpp"
#include <vector>

namespace rwlibs {
namespace algorithms {
/**
 * @brief calculate the stable poses of an object lying on a planar support
 * structure.
 */
class StablePlanePose {

	//! constructor
	StablePlanePose();

	/**
	 * @brief constructor
	 * @param geom [in] the geometry to find stable poses for
	 */
	StablePlanePose(rw::geometry::Geometry::Ptr geom);

	/**
	 * @brief constructor
	 * @param geom [in] the geometry to find stable poses for
	 * @param cm [in] center of mass of geometry
	 */
	StablePlanePose(rw::geometry::Geometry::Ptr geom, const rw::math::Vector3D<>& CM);

	/**
	 * @brief set a new geometry of this
	 * @param geom [in]
	 */
	void setGeometry(rw::geometry::Geometry::Ptr geom);

	/**
	 * @brief set the geometry to process and include the specification of the center of mass
	 * @param geom [in] geometry
	 * @param CM [in] center of mass
	 */
	void setGeometry(rw::geometry::Geometry::Ptr geom, const rw::math::Vector3D<>& CM);

	/**
	 * @brief calculate all stable poses for the geometry lying on a planar
	 * support structure.
	 * @param dist [in] the minimum distance from the edge of the supporting
	 * polygon to the point defined by CM projected onto the supporting polygon.
	 */
	std::vector<StablePose> calcStablePoses(bool includePoseDistributions=false);

	/**
	 * @brief calculate the pose distributions eg. the probabilities of an object
	 * falling into a specific stable pose when starting in a uniformly selected random
	 * orientation.
	 * @return
	 */
	std::vector<StablePose> calcPoseDistributions();

};

}
}

#endif /* STABLEPLANEPOSE_HPP_ */
