#ifndef RW_TRAJECTORY_INTERPOLATOR_HPP
#define RW_TRAJECTORY_INTERPOLATOR_HPP

/**
 * @file Interpolator.hpp
 */

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/
    
/**
 * @brief Interface for interpolators
 * 
 * See the specific implementations for more details 
 */
template <class T>
class Interpolator
{
public:
    /**
     * @brief Virtual destructor
     */
	virtual ~Interpolator() {}
	
	/**
	 * @brief Position at time t
	 * @param t [in] time between \b 0 and \b length
	 * @return Position
	 */
	virtual T x(double t) const = 0;

    /**
     * @brief Velocity at time t
     * @param t [in] time between \b 0 and \b length
     * @return Velocity
     */
	virtual T dx(double t) const = 0;
	
    /**
     * @brief Acceleration at time t
     * @param t [in] time between \b 0 and \b length
     * @return Acceleration
     */
	virtual T ddx(double t) const = 0;
	
	/**
	 * @brief Returns the duration of the interpolator
	 * 
	 * The duration is defined as the time it takes to move from one end 
	 * of the interpolator to the other.
	 * @return duration 
	 */
	virtual double duration() const = 0;
};

/* @} */

} //end namespace trajectory
} //end namespace rw

#endif //RW_SANDBOX_INTERPOLATOR_HPP
