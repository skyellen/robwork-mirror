#ifndef RigidBodyInfo_HPP_
#define RigidBodyInfo_HPP_

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

#endif /*RigidBodyInfo_HPP_*/
