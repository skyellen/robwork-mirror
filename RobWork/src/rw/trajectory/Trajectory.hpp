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

#ifndef RW_TRAJECTORY_TRAJECTORY_HPP
#define RW_TRAJECTORY_TRAJECTORY_HPP

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>

#include <rw/trajectory/TrajectoryIterator.hpp>

namespace rw {
namespace trajectory {

/** @addtogroup trajectory */
/*@{*/


/**
 * @brief Interface for Trajectories in RobWork
 */
template <class T>
class Trajectory
{
public:
    //! @brief smart pointer type
    typedef rw::common::Ptr<Trajectory<T> > Ptr;

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
    virtual T x(double t) const = 0;

    /**
     * @brief Velocity of trajectory at time \b t
     *
     * Returns the velocity of the trajectory at time \b t \f$\in[startTime(), endTime()]\f$.
     *
     * @param t [in] time between startTime() and endTime()
     * @return Velocity
     */
    virtual T dx(double t) const = 0;

   /**
     * @brief Acceleration of trajectory at time \b t
     *
     * Returns the acceleration of the trajectory at time \b t \f$\in[startTime(), endTime()]\f$.
     *
     * @param t [in] time between startTime() and endTime()
     * @return Acceleration
     */
    virtual T ddx(double t) const = 0;

    /**
     * @brief Total duration of the trajectory.
     *
     * The duration of the Trajectory corresponds to the time it takes to
     *  run through it.
     *
     *  If the trajectory is empty, then -1 is returned.
     */
    virtual double duration() const = 0;


    /**
     * @brief Returns the startTime of the trajectory
     *
     * @return Start time
     */
    virtual double startTime() const = 0;

    /**
     * @brief Returns the endTime of the trajectory.
     *
     * The end time equals startTime() + duration().
     *
     * @return The end time
     */
    virtual double endTime() const {
        return startTime() + duration();
    }

    /**
     * @brief Constructs a discrete path based on the trajectory.
     *
     * If \b uniform = true the path will be divided into the smallest number of 
     * uniform steps for which the time stepsize <= \b dt. 
     *
     * If \b uniform = false the path is divided into steps of duration dt, except the
     * last interval which may be shorter to include the end point.
     *
     * @param dt [in] Step size
     * @param uniform [in] Whether to sample the path uniformly
     * @return The discrete path.
     */
    std::vector<T> getPath(double dt, bool uniform = true) {
        std::vector<T> path;
        if (uniform) {
            int steps = (int)std::ceil(duration()/dt);
            double delta = duration()/steps;
            for (double t = 0; t<=duration(); t += delta) {
                path.push_back(x(t));
            }
        } else {
            for (double t = 0; t<duration(); t += dt) {
                path.push_back(x(t));                
            }
            path.push_back(x(duration()));
        }
        
        return path;
    }

	/**
	 * @brief Returns a bi-directional interator for running through the trajectory.
	 *
	 * For some trajectory types it may be significantly more efficient to run through 
	 * using an iterator, rather than using random access.
	 *
	 * @param dt [in] The default time step used when using the ++ or -- operators in the iterator
	 * @brief Pointer to the iterator. The pointer has ownership.
	 */
	virtual typename TrajectoryIterator<T>::Ptr getIterator(double dt = 1) const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};


//! A trajectory on rw::kinematics::State.
typedef Trajectory<rw::kinematics::State> StateTrajectory;

//! A trajectory on rw::math::Q
typedef Trajectory<math::Q> QTrajectory;


//! A trajectory on a Vector3D
typedef Trajectory<math::Vector3D<> > Vector3DTrajectory;


//! A trajectory on rw::math;:Rotation3D
typedef Trajectory<math::Rotation3D<> > Rotation3DTrajectory;



//! A trajectory on a rw::math::Transform3D
typedef Trajectory<math::Transform3D<> > Transform3DTrajectory;

/** @} */

} //end namespace trajectory
} //end namespace rw

#endif //end include guard
