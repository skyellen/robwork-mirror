#ifndef RW_SANDBOX_BLEND_HPP
#define RW_SANDBOX_BLEND_HPP

/**
 * @file Blend.hpp
 */

#include <rw/math/Q.hpp>

namespace rw {
namespace sandbox {

/** @addtogroup interpolator */
/*@{*/
    
/**
 * @brief Interface for blending 
 * 
 * A Blend describes a way to blend between to consecutive interpolators. If we let 
 * \f$t_1\f$ be the time switching from one interpolator to the next, then the blend
 * control the path in the interval \f$[t_1-\tau_1;t_1+\tau_2]\f$.
 * 
 * See the specific implementations for at description of which template 
 * arguments that are valid. 
 */
template <class T>
class Blend
{
public:
    /**
     * @brief Destructor
     */
	virtual ~Blend() {}
	
	/**
	 * @brief The position for a given time t
	 * @param t [in] \f$t\in[0,\tau_1+\tau_2} \f$
	 * @return Position at time \b t 
	 */
	virtual T x(double t) = 0;

    /**
     * @brief The velocity for a given time t
     * @param t [in] \f$t\in[0,\tau_1+\tau_2} \f$
     * @return Velocity at time \b t 
     */
	virtual T dx(double t) = 0;
	
    /**
     * @brief The acceleration for a given time t
     * @param t [in] \f$t\in[0,\tau_1+\tau_2} \f$
     * @return Acceleration at time \b t 
     */
	virtual T ddx(double t) = 0;
	
	/**
	 * @brief The time \f$\tau_1\f$ as defined in class definition
	 * @return \f$\tau_1\f$
	 */
	virtual double tau1() = 0;

    /**
     * @brief The time \f$\tau_2\f$ as defined in class definition
     * @return \f$\tau_2\f$
     */
	virtual double tau2() = 0;
	
};

/** @} */

} //end namespace sandbox
} //end namespace rw

#endif //RW_SANDBOX_BLEND_HPP
