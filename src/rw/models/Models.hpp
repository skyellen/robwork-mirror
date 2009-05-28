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
 * for detailed information about these packages.
 *********************************************************************/

#ifndef RW_MODELS_MODELS_HPP
#define RW_MODELS_MODELS_HPP

/**
 * @file Models.hpp
 */

#include "Joint.hpp"
#include "Device.hpp"
#include "WorkCell.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/trajectory/Path.hpp>
#include <rw/math/Q.hpp>

#include <utility>
#include <vector>
#include <list>

namespace rw { namespace models {

    /** @addtogroup models */
    /*@{*/

    /**
       @brief Utility functions for the rw::models module.
    */
    class Models
    {
    public:

        // Frames and workcells.

        /**
           @brief All frames of the workcell.
        */
		static std::vector<rw::kinematics::Frame*> findAllFrames(const WorkCell& workcell);

        /**
           @brief The frame named \b name of workcell \b workcell.

           An exception is thrown if the frame can not be found in the workcell.

           See WorkCell::findFrame() for a non-throwing version.
        */
        static
        rw::kinematics::Frame& getFrame(const WorkCell& workcell, const std::string& name);

        /**
           @brief The device named \b name of workcell \b workcell.

           An exception is thrown if the device can not be found in the workcell.

           See WorkCell::findDevice() for a non-throwing version.
        */
        static
        Device& getDevice(const WorkCell& workcell, const std::string& name);

        // Bounds checking

        /**
           @brief True iff the configuration \b q is within the box with lower and
           upper corners given by \b bounds. Each value of \b q is allowed to be
           outside of the box by the amount \b tolerance.
        */
        static bool inBounds(const rw::math::Q& q,
                             const Device::QBox& bounds,
                             double tolerance = 0);

        /**
           @brief True iff the configuration \b q is within the joint limits of the
           device \b device.
        */
        static bool inBounds(const rw::math::Q& q,
                             const Device& device,
                             double tolerance = 0);

        /**
           @brief True iff the joint value \b val is within the joint limits of the
           joint \b joint with a tolerance of \b tolerance.
        */
        static bool inBounds(const rw::math::Q& val,
                             const Joint& joint,
                             double tolerance = 0);

        /**
           @brief True iff the joint values of \b state are within the joint limits
           of the joints of \b workcell with a tolerance of \b tolerance.
        */
        static bool inBounds(
            const rw::kinematics::State& state,
            const WorkCell& workcell,
            double tolerance = 0);

        // Q path to state path conversion.

        /**
           @brief Convert a sequence of configurations to a sequence of states.

           The device configurations are assumed to belong to a common device
           and state.

           @param device [in] The device for the configurations.
           @param path [in] The sequence of device configurations.
           @param common_state [in] State to share for all configurations.
           @return Sequence of states - one state for each configuration.
        */
        static rw::trajectory::StatePath getStatePath(const Device& device,
                                                      const rw::trajectory::QPath& path,
                                                      const rw::kinematics::State& common_state);

        /**
           @brief Convert a sequence of configurations to a sequence of states.

           The device configurations are assumed to belong to a common device
           and state.

           @param device [in] The device for the configurations.
           @param path [in] The sequence of device configurations.
           @param common_state [in] State to share for all configurations.
           @param result [out] Sequence of states - one state for each configuration.
        */
        static void getStatePath(const Device& device,
                                 const rw::trajectory::QPath& path,
                                 const rw::kinematics::State& common_state,
                                 rw::trajectory::StatePath& result);

        /**
           @brief Construct a new device for which the base of the device equals
           \b base and the end of the device equals \b end.

           For changes in the configuration of \b device, \b base should be
           fixed relative to device->getBase() and \b end should be fixed
           relative to device->getEnd().

           If \b base is NULL, then device->getBase() is used as the default
           value.

           If \b end is NULL, then device->getEnd() is used as the default
           value.

           If \b base and \b end equal base and end for the device, then the
           original \b device is returned.

           @param device [in] Original device.

           @param state [in] The kinematic structure assumed for Jacobian
           computations.

           @param base [in] Base frame for the new device.

           @param end [in] End frame for the new device.
        */
        static rw::models::DevicePtr makeDevice(rw::models::DevicePtr device,
                                                const rw::kinematics::State& state,
                                                rw::kinematics::Frame* base = NULL,
                                                rw::kinematics::Frame* end = NULL);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
