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

#ifndef RWSIM_DYNAMICS_RIGIDBODY_HPP_
#define RWSIM_DYNAMICS_RIGIDBODY_HPP_

#include "Body.hpp"

namespace rwsim {
namespace dynamics {
	//! @addtogroup rwsim_dynamics
	//! @{
    /**
     * @brief The composite body allow multiple objects to be grouped together in
     * a fixed configuration.
     *
     * @deprecated This class is marked for removal as it has never been implemented and is not used.
     */
    class CompositeBody : public Body
    {
    public:
    	/**
    	 * @brief Constructor.
    	 * @param base
    	 * @param b1
    	 */
        CompositeBody(Body* base, Body* b1);

        //! @brief Destructor.
        virtual ~CompositeBody(){}

    private:
        Body* _base;
        std::vector<Body*> _bodies;

    };
    //! @}
} // namespace dynamics
}

#endif /*DYNAMICS_RIGIDBODY_HPP_*/
