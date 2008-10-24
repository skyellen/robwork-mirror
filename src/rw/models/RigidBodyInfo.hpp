/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MODELS_RIGIDBODYINFO_HPP
#define RW_MODELS_RIGIDBODYINFO_HPP

#include <rw/kinematics/Frame.hpp>

#include <rw/math/Transform3D.hpp>
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
