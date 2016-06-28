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


#ifndef RW_MODELS_RIGIDBODYINFO_HPP
#define RW_MODELS_RIGIDBODYINFO_HPP

#include <rw/math/InertiaMatrix.hpp>

namespace rw{
namespace models{

	/** @addtogroup models */
	/* @{ */

	/**
	 * @brief A class to wrap rigid body information.
	 */
	class RigidBodyInfo
	{
	public:

	    /**
	     * @brief constructs a RigidBodyInfo with a mass, inertia matrix, initial
	     * pose and velocity.
	     */
	    RigidBodyInfo(double mass, const rw::math::InertiaMatrix<>& Ibody);

	    /**
	     * @brief destructor
	     */
	    virtual ~RigidBodyInfo();

	    /**
	     * @brief returns the mass of this RigidBodyInfo
	     * @return the mass
	     */
	    double getMass(){return _mass;};

	    /**
	     * @brief returns the inertia matrix of this rigid body
	     */
	    rw::math::InertiaMatrix<> getInertia(){
	        return _Ibody;
	    };

	private:
	    /* Constant quantities */
	    double _mass;
	    rw::math::InertiaMatrix<> _Ibody;
	};

	/* @} */
}
}

#endif /*RW_MODELS_RIGIDBODYINFO_HPP*/
