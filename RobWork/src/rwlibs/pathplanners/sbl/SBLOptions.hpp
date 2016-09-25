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


#ifndef RWLIBS_PATHPLANNERS_SBL_SBLOPTIONS_HPP
#define RWLIBS_PATHPLANNERS_SBL_SBLOPTIONS_HPP

/**
   @file SBLOptions.hpp
*/

#include "SBLExpand.hpp"
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QEdgeConstraintIncremental.hpp>

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanners */
    /*@{*/

	//! @brief A SBL planner constraint.
	class SBLPlannerConstraint {
	public:
		/**
		 * @brief Constructor for a planner constrinct.
		 * @param qconstraint [in] a constraint giving the valid (collision free) configurations.
		 * @param edgeconstraint [in] a constraint for checking the edges in-between valid configurations.
		 */
		SBLPlannerConstraint(rw::pathplanning::QConstraint::Ptr qconstraint, 
			rw::pathplanning::QEdgeConstraintIncremental::Ptr edgeconstraint):
		_qconstraint(qconstraint),
		_edgeConstraint(edgeconstraint)
		{}

		/**
		 * @brief Get the part that checks for valid configurations.
		 * @return a reference to the constraint.
		 */
		const rw::pathplanning::QConstraint& getQConstraint() const {
			return *_qconstraint;
		}

		/**
		 * @brief Get the part that checks edges in-between valid configurations.
		 * @return a reference to the edge constraint.
		 */
		const rw::pathplanning::QEdgeConstraintIncremental& getEdgeConstraint() const {
			return *_edgeConstraint;
		}
	
	private:
		rw::pathplanning::QConstraint::Ptr _qconstraint;
		rw::pathplanning::QEdgeConstraintIncremental::Ptr _edgeConstraint;
	};

    /**
       @brief SBL planner setup.

       SBLOptions is the value stored in SBLSetup.

       SBLOptions is a seperate file so that we can keep SBLSetup as abstract as
       possible.

       SBLOptions is used by SBLInternal and is for internal use only.
    */
    class SBLOptions
    {
    public:
		/**
		 * @brief Construct a new set of options for the internal algorithms.
		 * @param constraint [in] a constraint on the valid configurations.
		 * @param edgeConstraint [in] a constraint on the edges between valid configurations.
		 * @param expansion [in] the policy for how to sample new configurations in the vicinity.
		 * @param metric [in] the distance metric for nearest neighbor searching.
		 * @param connectRadius [in] connect trees if the distance to the nearest neighbor is below this threshold.
		 */
        SBLOptions(
			rw::pathplanning::QConstraint::Ptr& constraint,
			rw::pathplanning::QEdgeConstraintIncremental::Ptr& edgeConstraint,
            SBLExpandPtr expansion,
			rw::math::QMetric::Ptr metric,
            double connectRadius);

        //! @brief The constraint that determined if a path or configuration is valid (collision free) or not.
        SBLPlannerConstraint constraint;
        //! @brief The expand policy used to sample new configurations in the vicinity.
        SBLExpandPtr expansion;
        //! @brief the distance metric for nearest neighbor searching.
		rw::math::QMetric::Ptr metric;
		//! @brief Attempt connection of the trees if the distance to the nearest neighbor is below this threshold.
        double connectRadius;

        //! @brief Policy for choosing a node in the vicinity of a given node, \b n.
        enum NearNodeSelection {
            UniformSelect,  //!< take a random node.
            UniformFromCell,//!< take a random node within the cell where node \b n lies.
            NearestFromCell,//!< take the nearest node from the cell where node \b n lies.
            NearestNode     //!< search for the nearest node (default)
        };

        //! @brief Policy for selecting a tree.
        enum TreeSelection {
            UniformTree, //!< randomly select one of the two trees (default)
            WeightedTree,//!< choose the tree randomly, but weighted according to the size of the tree.
            SmallestTree,//!< choose the smallest tree.
            LargestTree  //!< choose the largest tree.
        };

        //! @brief Policy for how often to connect trees.
        enum ConnectFrequency {
            ConnectAlways,//!< always connect (default)
            ConnectAtReset//!< connect only at reset.
        };

        //! @brief (default is 20).
        int resetCount;
        //! @brief (default is 25).
        int rootSampleInterval;
        //! @brief (default is 10).
        double nodesPerCell;
        //! @brief (default is NearestNode).
        NearNodeSelection nearNodeSelection;
        //! @brief (default is UniformTree).
        TreeSelection treeSelection;
        //! @brief (default is ConnectAlways).
        ConnectFrequency connectAt;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
