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
 * for detailed Actionrmation about these packages.
 *********************************************************************/

#ifndef RW_TRAJECTORY_INTERPOLATOR_HPP
#define RW_TRAJECTORY_INTERPOLATOR_HPP

/**
   @file Interpolator.hpp
*/

namespace rw { namespace trajectory {

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
}} // end namespaces
#endif // end include guard
