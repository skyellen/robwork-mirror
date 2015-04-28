/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELDATA_HPP_
#define RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELDATA_HPP_

/**
 * @file TNTFrictionModelData.hpp
 *
 * \copydoc rwsimlibs::tntphysics::TNTFrictionModelData
 */

namespace rwsimlibs {
namespace tntphysics {
//! @addtogroup rwsimlibs_tntphysics

//! @{
/**
 * @brief Interface for data structures used by TNTFrictionModel.
 */
class TNTFrictionModelData {
public:
	//! @brief Constructor.
	TNTFrictionModelData() {}

	//! @brief Destructor.
	virtual ~TNTFrictionModelData() {}

	/**
	 * @brief Clone the data.
	 * @return a pointer to cloned data owned by the caller.
	 */
	virtual TNTFrictionModelData* clone() = 0;
};
//! @}
} /* namespace tntphysics */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_TNTPHYSICS_TNTFRICTIONMODELDATA_HPP_ */
