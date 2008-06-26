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
         * @brief Constructs PlannerUtil object
         *
         * The object does NOT take ownership of or copies the workcell
         * or collision detector. The caller must therefore make sure that
         * these stays valid while using the object
         *
         * @param device [in] the device
         * @param state [in] the default state to use
         * @param detector [in] the collision detector to use
         */
        PlannerUtil(
            rw::models::Device* device,
            const rw::kinematics::State& state,
            rw::proximity::CollisionDetector* detector);

        /**
         * @brief destructor
         */
        ~PlannerUtil();

        /**
         * @brief sets the default state
         * @param state [in] the state to use
         */
        void setState(rw::kinematics::State& state);

        /**
         * @brief Creates random joint configuration @f$ \mathbf{q}\in C @f$
         * @return a joint configuration
         * @pre the object must be initialized
         */
        rw::math::Q randomConfig() const;

        /**
         * @brief Checks to see if the robot is in collision in the given
         * joint configuration (@f$ \mathbf{q}\in C_{free} @f$)
         *
         * @param q [in] @f$ \mathbf{q} @f$ the configuration to check
         * @return true if the robot is in collision, false otherwise
         * @pre the object must be initialized
         */
        bool inCollision(const rw::math::Q& q) const;

        /**
         * @brief Check to see of the robot is in collision when moving on an edge
         * from \b q1 to \b q2.
         *
         * The algorithm constructs \b samples on the straight line from \b q1 to \b q2 and
         * tests these for collision. It is assumed that \b q1 and \b q2 is already tested.
         *
         * Example:
         * bool incollision = util.inCollision(q1, q2, (int)std::ceil(metric.distance(q1, q2)/resolution));
         *
         * In this example the samples will be placed no further than \b resolution apart, assuming
         * the metric satisfies that metric.distance(q) <= 2*metric.distance(q/2).
         */
        bool inCollision(const rw::math::Q& q1, const rw::math::Q& q2, int samples) const;

        /**
         * @brief Normalizes configuration
         * @param q [in] @f$ \mathbf{q} @f$
         * @return @f$ \mathbf{qn} @f$
         * @pre the object must be initialized
         */
        rw::math::Q normalize(const rw::math::Q& q) const;

        /**
         * @brief UnNormalizes configuration
         * @param qn [in] @f$ \mathbf{qn} @f$
         * @return @f$ \mathbf{q} @f$
         * @pre the object must be initialized
         */
        rw::math::Q unNormalize(const rw::math::Q& qn) const;

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
        rw::models::Device* _device;
        rw::kinematics::State _state;
        rw::proximity::CollisionDetector* _collisionDetector;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
