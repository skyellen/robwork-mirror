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

#ifndef RWSIMLIBS_RWPE_RWPEFRICTIONMODELDATA_HPP_
#define RWSIMLIBS_RWPE_RWPEFRICTIONMODELDATA_HPP_

/**
 * @file RWPEFrictionModelData.hpp
 *
 * \copydoc rwsimlibs::rwpe::RWPEFrictionModelData
 */

namespace rwsimlibs {
namespace rwpe {
//! @addtogroup rwsimlibs_rwpe

//! @{
/**
 * @brief Interface for data structures used by RWPEFrictionModel.
 */
class RWPEFrictionModelData {
public:
	//! @brief Constructor.
	RWPEFrictionModelData() {}

	//! @brief Destructor.
	virtual ~RWPEFrictionModelData() {}

	/**
	 * @brief Clone the data.
	 * @return a pointer to cloned data owned by the caller.
	 */
	virtual RWPEFrictionModelData* clone() = 0;
};
//! @}
} /* namespace rwpe */
} /* namespace rwsimlibs */
#endif /* RWSIMLIBS_RWPE_RWPEFRICTIONMODELDATA_HPP_ */
