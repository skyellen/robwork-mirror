/*********************************************************************
 * RobWork Version 0.2
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

#ifndef RW_TRAJECTORY_TIMED_HPP
#define RW_TRAJECTORY_TIMED_HPP

/**
   @file Timed.hpp
   @brief Class rw::interpolator::Timed
*/

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>

namespace rw { namespace trajectory {

    /** @addtogroup trajectory */
    /*@{*/

    /**
       @brief A tuple of (time, value).
     */
    template <class T>
    class Timed
    {
    public:
        /**
           Constructor
         */
        Timed(double time, const T& value) :
            _time(time),
            _value(value)
        {}

        /**
           @brief The time
        */
        double getTime() const { return _time; }

        /**
           @brief The value
        */
        const T& getValue() const { return _value; }

    private:
        double _time;
        T _value;
    };

    /**
       @brief A tuple of (time, value).
    */
    template <class T>
    Timed<T> makeTimed(double time, const T& value) { return Timed<T>(time, value); }

    //! A tuple of (time, Q).
    typedef Timed<rw::math::Q> TimedQ;

    //! A tuple of (time, State).
    typedef Timed<rw::kinematics::State> TimedState;

    /*@}*/
}} // end namespaces

#endif // end include guard
