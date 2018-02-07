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

#ifndef RW_INVKIN_PARALLELIKSOLVER_HPP
#define RW_INVKIN_PARALLELIKSOLVER_HPP

/**
 * @file ParallelIKSolver.hpp
 */

#include "IterativeIK.hpp"

#include <rw/common/PropertyMap.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/kinematics/FramePairMap.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/math/VectorND.hpp>

namespace rw { namespace kinematics { class Frame; }}
namespace rw { namespace models { class ParallelDevice; }}
namespace rw { namespace models { class ParallelLeg; }}

namespace rw { namespace invkin {

    /** @addtogroup invkin */
    /*@{*/

    /**
     * @brief Inverse kinematics method for parallel devices.
     *
     * The method is based on solving for two simultaneous constraints.
     * First, the junctions defined in the ParallelDevice must remain connected.
     * Second, the target(s) given by the user should be fulfilled.
     *
     * A stacked Jacobian is used to form an equation system that includes these objectives.
     * The Singular Value Decomposition is used to find the solution for the joint values
     * in this equation system.
     */
    class ParallelIKSolver : public IterativeIK
    {
    public:
		//! @brief smart pointer type to this class
		typedef rw::common::Ptr<ParallelIKSolver> Ptr;

		/**
		 * @brief Construct new solver.
		 * @param device [in] pointer to the parallel device.
		 */
        ParallelIKSolver(const models::ParallelDevice* device);

		/**
		 * @brief Destructor
		 */
        virtual ~ParallelIKSolver();

        /**
         * @copybrief IterativeIK::solve
         *
         * Given a desired \f$\robabx{}{desired}{\mathbf{T}}\f$
         * and the current state, the method solves the inverse kinematics
         * problem.
         *
         * This algorithm will return a maximum of one solution, but only if
         * it is able to find one. Before returning a solution, it may be checked
         * to be within the bounds of the configuration space.
         * (See setCheckJointLimits(bool) )
         *
         * @param baseTend [in] Desired base to end transformation \f$
         * \robabx{}{desired}{\mathbf{T}}\f$
         *
         * @param state [in] State of the device from which to start the
         * iterations
         *
         * @return List with one or zero solutions.
         *
         * @note The targets \b baseTend must be defined relative to the base of the
         * robot/device. For a ParallelDevice loaded from an XML file, the base and
         * end frames will normally be the first and last frames of the first leg of
         * the first junction.
         */
        virtual std::vector<math::Q> solve(const math::Transform3D<>& baseTend,
                                           const kinematics::State &state) const;

        //! @brief A target definition used in the multi-target solve function.
        struct Target {
        	//! @brief Constructor with all directions enabled initially.
        	Target():
        		refFrame(NULL),
        		tcpFrame(NULL)
        	{
        		for (std::size_t i = 0; i < 6; i++)
        			enabled[i] = true;
        	}

        	/**
        	 * @brief Constructor with specification of a target transformation from the base to a given tcp \b frame.
        	 * @param frame [in] the end frame.
        	 * @param refTtcp [in] the target base to frame transformation.
        	 */
        	Target(const rw::kinematics::Frame* frame, const rw::math::Transform3D<>& refTtcp):
        		refFrame(NULL),
        		tcpFrame(frame),
				refTtcp(refTtcp)
        	{
        		for (std::size_t i = 0; i < 6; i++)
        			enabled[i] = true;
        	}

        	/**
        	 * @brief Constructor with specification of a target transformation where some directions are not \b enabled.
        	 * @param frame [in] the end frame.
        	 * @param refTtcp [in] the target base to frame transformation.
        	 * @param enabled [in] 6 values specifying if the x, y, z and EAA x, y, z directions should be enabled.
        	 */
        	Target(const rw::kinematics::Frame* frame, const rw::math::Transform3D<>& refTtcp, const rw::math::VectorND<6, bool>& enabled):
        		refFrame(NULL),
        		tcpFrame(frame),
				refTtcp(refTtcp),
				enabled(enabled)
        	{
        	}

        	/**
        	 * @brief Get the number directions enabled.
        	 * @return number of directions enabled.
        	 */
        	std::size_t dof() const {
        	    std::size_t dof = 0;
        	    for (std::size_t i = 0; i < 6; i++) {
        	    	if (enabled[i])
        	    		dof++;
        	    }
        	    return dof;
        	}

        	//! @brief The reference frame. If zero, this is equivalent to the device base frame.
        	const rw::kinematics::Frame* refFrame;

        	//! @brief The frame to specify target for.
        	const rw::kinematics::Frame* tcpFrame;

        	//! @brief The target transformation from \b refFrame to the \b tcpFrame.
        	rw::math::Transform3D<> refTtcp;

        	//! @brief Directions of \b baseTtcp to enable. The 6 values specify x, y, z and EAA x, y, z directions.
        	rw::math::VectorND<6, bool> enabled;
        };

        /**
         * @brief Version of solve that allows for definition of multiple targets.
         *
         * This is a more flexible version of solve than the one from the standard InvKinSolver interface.
         * As an example, it is useful to define multiple targets for a gripper where each of the fingers
         * can have different target configurations.
         * Furthermore, targets can be defined for different frames, and relative to different frames.
         * There should always be a minimum of one joint between the base and end frames, and the end frame
         * should be in the child tree of the given base frame. The reference frame given must either lie in
         * one of the legs of the junctions in the ParallelDevice, or it must be in the parent path in
         * the frame structure.
         * Finally, the target definitions do not need to be defined as full 6 DOF constraints.
         * It is possible to specify that the constraint should only be defined in some positional
         * or angular directions. In some parallel structures this is very useful, as it might not be possible
         * to do inverse kinematics with full 6 DOF constraints.
         *
         * @param targets [in] list of targets.
         * @param state [in] state containing the current configuration of the device.
         * @return vector with one solution if found, otherwise vector is empty.
         */
        virtual std::vector<rw::math::Q> solve(const std::vector<Target>& targets, const rw::kinematics::State &state) const;

        //! @copydoc InvKinSolver::setCheckJointLimits
        virtual void setCheckJointLimits(bool check);

        /**
         * @brief enables clamping of the solution such that solution always is within joint limits
         * @param enableClamping [in] true to enable clamping, false otherwise
         */
        void setClampToBounds(bool enableClamping);

    private:
        void updateDeltaX(const std::vector<Target>& targets, const rw::kinematics::FramePairMap<rw::common::Ptr<rw::models::ParallelLeg> >& targetLegs, const rw::kinematics::State& state, rw::math::Q& deltaX, const Eigen::MatrixXd::Index nCon) const;
        void updateJacobian(const std::vector<Target>& targets, const rw::kinematics::FramePairMap<rw::common::Ptr<rw::models::ParallelLeg> >& targetLegs, const rw::kinematics::State& state, rw::math::Jacobian& jacobian) const;

        const models::ParallelDevice* _device;
        std::vector<std::vector<rw::models::ParallelLeg*> > _junctions;

        rw::kinematics::FrameMap<Eigen::MatrixXd::Index> _jointIndex;

        bool _useJointClamping;
        bool _checkJointLimits;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
