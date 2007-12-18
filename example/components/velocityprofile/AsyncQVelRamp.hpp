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

#ifndef rwlibs_components_velocityprofile_AsyncQVelRamp_HPP
#define rwlibs_components_velocityprofile_AsyncQVelRamp_HPP

/**
 * @file AsyncQVelRamp.hpp
 */

#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>

namespace rwlibs { namespace components {

    /** @addtogroup components */
    /** @{*/

    /**
     * @brief Provides an Orocos TaskContext component that generates
     * a asynchronous velocity ramp profile on the basis of the
     * velocity and acceleration limits of a device.
     */
    class AsyncQVelRamp: public RTT::TaskContext {
    public:
        /**
         * @brief constructor
         * @param name [in] name of TaskContext component
         * @param device [in] device to generate ramps for
         * @param state [in] state for the workcell
         * @param dt [in] time interval to use
         */
        AsyncQVelRamp(const std::string& name,
                      rw::models::Device* device,
                      const rw::kinematics::State& state,
                      double dt);

        /**
         * @brief deconstructor
         */
        ~AsyncQVelRamp();

    protected:
        /**
         * @brief [input] The target configuration.
         */
        RTT::ReadDataPort<rw::math::Q > _qTarget;

        /**
         * @brief [input] The current configuration.
         */
        RTT::ReadDataPort<rw::math::Q > _qCurrent;

        /**
         * @brief [input] The current velocity.
         */
        RTT::ReadDataPort<rw::math::Q > _qdotCurrent;

        /**
         * @brief [output] The configuration in next time step.
         */
        RTT::WriteDataPort< rw::math::Q > _qOut;

        /**
         * @brief [output] The velocity in next time step.
         */
        RTT::WriteDataPort<rw::math::Q > _qdotOut;

    private:
        bool startup();

        void update();

        void shutdown();

        rw::models::Device* _device;
        double _dt;
        rw::math::Q _acc;
        rw::math::Q _velmax;
        rw::kinematics::State _state;
    };

}} // end namespaces

#endif // end include guard
