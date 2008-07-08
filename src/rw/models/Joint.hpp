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

#ifndef rw_models_Joint_HPP
#define rw_models_Joint_HPP

/**
 * @file Joint.hpp
 */

#include <rw/kinematics/Frame.hpp>
#include <cfloat>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
     * @brief A Joint is a single-joint-value Frame with assignable values for
     * position, velocity and acceleration limits.
     *
     * (This definition of a joint is pretty simplistic and may need change.)
     *
     * Joint is an interface: Subclasses should implement Frame::getTransform().
     */
    class Joint : public kinematics::Frame
    {
    public:
        /**
         * @brief Default constructor for the joint interface.
         *
         * @param parent [in] The parent frame (or NULL if attachable).
         *
         * @param name [in] The name of the frame.
         */
        Joint(const std::string& name) :
            Frame(1, name),
            _bounds(-DBL_MAX, DBL_MAX),
            _maxVelocity(1),
            _maxAcceleration(8)
        {}

        /**
         * @brief Virtual destructor
         */
        virtual ~Joint() {}

        /**
         * @brief Sets joint bounds
         * @param bounds [in] the lower and upper bounds of this joint
         */
        void setBounds(std::pair<double, double> bounds){ _bounds = bounds; }

        /**
         * @brief Gets joint bounds
         * @return the lower and upper bound of this joint
         */
        std::pair<double, double> getBounds() const { return _bounds; }

        /**
         * @brief Sets max velocity of joint
         * @param maxVelocity [in] the new maximum velocity of the joint
         */
        void setMaxVelocity(double maxVelocity)
        { _maxVelocity = maxVelocity; }

        /**
         * @brief Gets max velocity of joint
         * @return the maximum velocity of the joint
         */
        double getMaxVelocity() const
        { return _maxVelocity; }

        /**
         * @brief Sets max acceleration of joint
         * @param maxAcceleration [in] the new maximum acceleration of the joint
         */
        void setMaxAcceleration(double maxAcceleration)
        { _maxAcceleration = maxAcceleration; }

        /**
         * @brief Gets max acceleration of joint
         * @return the maximum acceleration of the joint
         */
        double getMaxAcceleration() const
        { return _maxAcceleration; }

    private:
        std::pair<double, double> _bounds;
        double _maxVelocity;
        double _maxAcceleration;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
