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


#ifndef RWLIBS_PATHPLANNERS_ARVEXPAND_HPP
#define RWLIBS_PATHPLANNERS_ARVEXPAND_HPP

/**
   @file ARWExpand.hpp
*/

#include <rw/common/Ptr.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>
#include <rw/trajectory/Path.hpp>

namespace rw { namespace pathplanning { class PlannerConstraint; } }

namespace rwlibs { namespace pathplanners {

    /** @addtogroup pathplanning */
    /** @{*/

#ifdef RW_USE_DEPRECATED
    class ARWExpand;

    //! A pointer to a ARWExpand.
    typedef rw::common::Ptr<ARWExpand> ARWExpandPtr;
#endif

    /**
       @brief ARWExpand expands a random walk in the configuration space by one
       step.
    */
    class ARWExpand
    {
    public:
		//! @brief smart pointer type to this class
        typedef rw::common::Ptr<ARWExpand> Ptr;

        /**
           @brief Expand the path by one step and return true if a new
           configuration was added to the path.

           The current path can be retrieved with ARWExpand::getPath().

           @return True iff a node was added to the end of the path.
        */
        bool expand();

        /**
           @brief Construct a new random walk with start node at \b start.
        */
		ARWExpand::Ptr duplicate(const rw::math::Q& start) const;

        /**
           @brief Destructor
        */
        virtual ~ARWExpand() {}

        /**
           @brief The current path of the random walk.
        */
		const rw::trajectory::QPath& getPath() const { return _path; }

        /**
           @brief Constructor

           The expansion method computes the variance for the latest \b
           historySize elements of the path and performs one step sampled from a
           Gaussian distribution with the computed variances. Variances lower
           than \b minVariances are never used.

           If \b minVariances is empty, then a default value is chosen based on
           \b bounds.

           If \b historySize is negative, a default value is chosen.

           @param bounds [in] Configuration space bounds.

           @param constraint [in] Path planning constraint.

           @param minVariances [in] Minimum variances.

           @param historySize [in] Number of previous elements of the path to
           use for variance computation.
        */
		static ARWExpand::Ptr make(
            const rw::models::Device::QBox& bounds,
            const rw::pathplanning::PlannerConstraint& constraint,
            const rw::math::Q& minVariances = rw::math::Q(),
            int historySize = -1);

    protected:
        /**
           @brief Constructor
        */
        ARWExpand() {}

        /**
           @brief Subclass implementation of the expand() method.

           The doExpand() adds one or more nodes to \b _path if and only if the
           method returns true.
        */
        virtual bool doExpand() = 0;

        /**
           @brief Subclass implementation of the duplicate() method.
        */
		virtual ARWExpand::Ptr doDuplicate(const rw::math::Q& start) const = 0;

    private:
        ARWExpand(const ARWExpand&);
        ARWExpand& operator=(const ARWExpand&);

    protected:
        /**
           @brief The path of random walk.
        */
		rw::trajectory::QPath _path;
    };

    /* @} */
}} // end namespaces

#endif // end include guard
