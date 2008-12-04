/*
 * Trajectory.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#ifndef RW_TRAJECTORY_TRAJECTORY_HPP
#define RW_TRAJECTORY_TRAJECTORY_HPP

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/kinematics/State.hpp>


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

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};


//! A trajectory on rw::kinematics::State.
typedef Trajectory<rw::kinematics::State> StateTrajectory;

//! A pointer to a StateTrajectory.
typedef rw::common::Ptr<StateTrajectory> StateTrajectoryPtr;


//! A trajectory on rw::math::Q
typedef Trajectory<math::Q> QTrajectory;

//! A pointer to a QTrajectory.
typedef rw::common::Ptr<QTrajectory> QTrajectoryPtr;


//! A trajectory on a Vector3D
typedef Trajectory<math::Vector3D<> > Vector3DTrajectory;

//! A pointer to a Vector3DTrajectory.
typedef rw::common::Ptr<Vector3DTrajectory> Vector3DTrajectoryPtr;


//! A trajectory on rw::math;:Rotation3D
typedef Trajectory<math::Rotation3D<> > Rotation3DTrajectory;

//! A pointer to a Rotation3DTrajectory.
typedef rw::common::Ptr<Rotation3DTrajectory> Rotation3DTrajectoryPtr;


//! A trajectory on a rw::math::Transform3D
typedef Trajectory<math::Transform3D<> > Transform3DTrajectory;

//! A pointer to a Transform3DTrajectory.
typedef rw::common::Ptr<Transform3DTrajectory> Transform3DTrajectoryPtr;


/** @} */

} //end namespace trajectory
} //end namespace rw

#endif //end include guard
