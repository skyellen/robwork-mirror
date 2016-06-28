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


#ifndef RW_MODELS_JACOBIANCALCULATOR_HPP
#define RW_MODELS_JACOBIANCALCULATOR_HPP

#include <rw/common/Ptr.hpp>
#include <rw/math/Jacobian.hpp>

namespace rw { namespace kinematics { class State; } }

namespace rw {
namespace models {


/**
 * @brief JacobianCalculator provides an interface for obtaining a Jacobian
 *
 */
class JacobianCalculator
{
public:
	//! smart pointer type
	typedef rw::common::Ptr<JacobianCalculator> Ptr;

    /**
     * @brief Destructor
     */
    virtual ~JacobianCalculator();

    /**
     * @brief Returns the Jacobian associated to \b state
     * @param state [in] State for which to calculate the Jacobian
     * @return Jacobian for \b state
     */
    virtual rw::math::Jacobian get(const rw::kinematics::State& state) const = 0;

    /**
     * @brief Returns the Jacobian calculated based on the content of \b fk
     * @param fk [in] Forward kinematics table based on which to calculate the Jacobian
     * @return Jacobian for \b fk
     */
    //virtual rw::math::Jacobian get(const rw::kinematics::FKTable& fk) const = 0;


};

//! deprecated smart pointer type
typedef rw::common::Ptr<JacobianCalculator> JacobianCalculatorPtr;


} //end namespace models
} //end namespace rw


#endif //end include guard
