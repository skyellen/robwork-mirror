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

#ifndef RWHW_FANUC_VELRAMPPROFILE_HPP
#define RWHW_FANUC_VELRAMPPROFILE_HPP

/**
 * @file VelRampProfile.hpp
 */

#include <vector>

#include <rw/math/Q.hpp>

namespace rwhw { namespace fanuc {

    /** @addtogroup rwhw */
    /*@{*/


    /**
     * @brief velocity ramp profile
     */
    class VelRampProfile
    {
    public:
        /** @brief Over and upper limit.
         */
        typedef std::pair<double, double> Range;

        /**
         * @brief constructor
         * @param poslimits [in] The lower and upper position limits
         * @param vellimits [in] The lower and upper velocity limits
         * @param acclimits [in] The lower and upper accelleration limits
         */
        VelRampProfile(
            const std::vector<Range>& poslimits,
            const std::vector<Range>& vellimits,
            const std::vector<Range>& acclimits);

        /**
         * @brief deconstructor
         */
        ~VelRampProfile();

        /**
         * @brief Returns the velocity needed for moving towards goal, when
         * having the current position \f$pos\f$ and velocity \f$vel\f$.
         *
         * @param goal [in] the goal
         * @param pos [in] the current position
         * @param vel [in] the current velocity
         * @param dt [in] time interval
         */
        rw::math::Q getVelocity(
            const rw::math::Q& goal,
            const rw::math::Q& pos,
            const rw::math::Q& vel,
            double dt) const;

    private:
        std::vector<Range> _poslimits;
        std::vector<Range> _vellimits;
        std::vector<Range> _acclimits;
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
