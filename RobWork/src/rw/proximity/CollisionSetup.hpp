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

#ifndef RW_COLLISION_COLLISIONSETUP_HPP
#define RW_COLLISION_COLLISIONSETUP_HPP

#include <rw/common/Ptr.hpp>
#include <rw/common/StringUtil.hpp>

#include <set>
#include <string>
#include <vector>

namespace rw { namespace common { class PropertyMap; } }
namespace rw { namespace kinematics { class Frame; } }
namespace rw { namespace models { class WorkCell; } }

namespace rw {
namespace proximity {

/** @addtogroup proximity */
/*@{*/
//! @file rw/proximity/CollisionSetup.hpp
/**
 * @brief Setup for the collision checker
 *
 * The CollisionSetup contains information about
 * which frames, not be checked against each other
 */
class CollisionSetup
{
public:
    /**
     * @brief Default constructor for when no excludes are described
     */
    CollisionSetup();

    /**
     @brief Constructs CollisionSetup with list of exclusions

     @param exclude [in] pairs to be excluded
     */
    explicit CollisionSetup(const std::vector<std::pair<std::string,std::string> >& exclude);

    /**
     @brief CollisionSetup for a list of pairs to exclude and a sequence
     of volatile frames.

     @param exclude [in] pairs to be excluded

     @param volatileFrames [in] names of frames to treat as volatile.

     @param excludeStaticPairs [in] if true exclude statically related pairs.
     */
    CollisionSetup(const rw::common::StringPairList& exclude,
                   const std::set<std::string>& volatileFrames,
                   bool excludeStaticPairs);


	void addExcludePair(rw::common::StringPair& pair);

	void removeExcludePair(rw::common::StringPair& pair);

    /**
     * @brief Returns the exclude list
     * @return the exclude list
     */
    const rw::common::StringPairList& getExcludeList() const
    {
        return _exclude;
    }

    /**
     @brief True iff the collision setup for the frame can change over
     time.
     */
    bool isVolatile(const rw::kinematics::Frame& frame) const;

    /**
     @brief True iff all statically related pairs of frames should be
     excluded.

     Note that this will exclude also statically related pairs of frames
     for which one or both of the pairs are volatile.
     */
    bool excludeStaticPairs() const
    {
        return _excludeStaticPairs;
    }

    /**
     * @brief Combine setup of this and setup of \b b into this collision setup.
     */
    void merge(const CollisionSetup& b);

    /**
     * @brief Combine setup \b a and setup \b b into a single collision setup.
     */
    static CollisionSetup merge(const CollisionSetup& a, const CollisionSetup& b);

    static CollisionSetup get(const rw::models::WorkCell& wc);
    static CollisionSetup get(rw::common::Ptr<rw::models::WorkCell> wc);

    static CollisionSetup get(const rw::common::PropertyMap& map);

    static void set(const CollisionSetup& setup, rw::common::Ptr<rw::models::WorkCell> wc);

    static void set(const CollisionSetup& setup, rw::common::PropertyMap& map);


private:
    rw::common::StringPairList _exclude;
    std::set<std::string> _volatileFrames;
    bool _excludeStaticPairs;
};

/*@}*/
}
} // end namespaces

#endif // end include guard
