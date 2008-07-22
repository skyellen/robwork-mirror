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

#ifndef rw_pathplanning_PlannerUtil_HPP
#define rw_pathplanning_PlannerUtil_HPP

/**
 * @file PlannerUtil.hpp
 */

#include <rw/models/Device.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/proximity/CollisionDetector.hpp>

namespace rw { namespace pathplanning {

    /** @addtogroup pathplanning */
    /** @{*/

    /**
     * @brief PlannerUtil provides various utilities useful in path planning
     */
    class PlannerUtil {
    public:
        /**
         * @brief Description of the different estimation type possible in the
         * estimateMotionWeights(EsitmateType, size_t) method
         */
        enum EstimateType {
            WORSTCASE = 0, /** Estimate weights corresponding to the maximal distance */
            AVERAGE /** Estimate weights corresponding to the average distance */
        };

        /**
         * @brief Estimate the distance traveled by the frame, when moving the joints
         *
         * The estimate is based on sampling \b samples random configuration and
         * using the Jacobian to estimate the distance traveled when moving the
         * joints. This estimate can be used in the WeightedEuclideanMetric.
         *
         * @param frame [in] Frame to calculate weights for. If null the device end frame will be used
         * @param type [in] The estimation type
         * @param samples [in] The number of samples to use (default 1000)
         * @return Weights representing the distance
         */
        static rw::math::Q estimateMotionWeights(
            const rw::models::Device& device,
            const rw::kinematics::Frame* frame,
            const rw::kinematics::State& initialState,
            EstimateType type,
            size_t samples);

        /**
         * @brief Clamps values to be within bounds
         *
         * The method treats all joints individually and clamps them to be within the
         * position bounds of the device
         *
         * @param device [in] The device for the configurations.
         * @param q [in] Configuration to clamp
         * @return The clamped configuration
         */
        static rw::math::Q clampPosition(
            const rw::models::Device& device,
            const rw::math::Q& q);

        /**
         * @brief Clamps values to be within bounds
         *
         * The method treats all joints individually and clamps them to be
         * within the position bounds of the configuration space box.
         *
         * @param bounds [in] The bounds of the configuration space
         * @param q [in] Configuration to clamp
         * @return The clamped configuration
         */
        static rw::math::Q clampPosition(
            const std::pair<rw::math::Q, rw::math::Q>& bounds,
            const rw::math::Q& q);

    private:
        PlannerUtil();
        PlannerUtil(const PlannerUtil&);
        PlannerUtil& operator=(const PlannerUtil&);
    };

    /* @} */
}} // end namespaces

#endif // end include guard
