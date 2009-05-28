/*
 * Trajectory.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

template <class T>
class Trajectory
{
public:


    /**
     * @brief Destructor
     */
    virtual ~Trajectory() {}

    /**
     * @brief Position of trajectory at time \b t
     *
     * Returns the position of the trajectory at time \b t \f$\in[startTime(), endTime()]\f$.
     *
     * @param t [in] time between startTime() and endTime()
     * @return Position
     */
    T x(double t) const = 0;

    /**
     * @brief Velocity of trajectory at time \b t
     *
     * Returns the velocity of the trajectory at time \b t \f$\in[startTime(), endTime()]\f$.
     *
     * @param t [in] time between startTime() and endTime()
     * @return Velocity
     */
    T dx(double t) const = 0;

   /**
     * @brief Acceleration of trajectory at time \b t
     *
     * Returns the acceleration of the trajectory at time \b t \f$\in[startTime(), endTime()]\f$.
     *
     * @param t [in] time between startTime() and endTime()
     * @return Acceleration
     */
    T ddx(double t) = 0;

    /**
     * @brief Total duration of the trajectory.
     *
     * The duration of the Trajectory corresponds to the time it takes to
     *  run through it.
     *
     *  If the trajectory is empty, then -1 is returned.
     */
    double duration() const = 0;


    /**
     * @brief Returns the startTime of the trajectory
     *
     * @return Start time
     */
    double startTime() const = 0;

    /**
     * @brief Returns the endTime of the trajectory.
     *
     * The end time equals startTime() + duration().
     *
     * @return The end time
     */
    double endTime() const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};


#endif /* TRAJECTORY_HPP_ */
