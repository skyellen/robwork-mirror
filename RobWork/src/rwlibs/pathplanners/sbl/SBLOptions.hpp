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


	class SBLPlannerConstraint {
	public:
		SBLPlannerConstraint(rw::pathplanning::QConstraint::Ptr qconstraint, 
			rw::pathplanning::QEdgeConstraintIncremental::Ptr edgeconstraint):
		_qconstraint(qconstraint),
		_edgeConstraint(edgeconstraint)
		{}

		const rw::pathplanning::QConstraint& getQConstraint() const {
			return *_qconstraint;
		}

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
        SBLOptions(
			rw::pathplanning::QConstraint::Ptr& constraint,
			rw::pathplanning::QEdgeConstraintIncremental::Ptr& edgeConstraint,
            SBLExpandPtr expansion,
			rw::math::QMetric::Ptr metric,
            double connectRadius);

        SBLPlannerConstraint constraint;		
        SBLExpandPtr expansion;
		rw::math::QMetric::Ptr metric;
        double connectRadius;

        enum NearNodeSelection {
            UniformSelect,
            UniformFromCell,
            NearestFromCell,
            NearestNode
        };

        enum TreeSelection {
            UniformTree,
            WeightedTree,
            SmallestTree,
            LargestTree
        };

        enum ConnectFrequency {
            ConnectAlways,
            ConnectAtReset
        };

        int resetCount;
        int rootSampleInterval;
        double nodesPerCell;
        NearNodeSelection nearNodeSelection;
        TreeSelection treeSelection;
        ConnectFrequency connectAt;
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
