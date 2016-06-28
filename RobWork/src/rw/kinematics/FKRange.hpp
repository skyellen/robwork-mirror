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


#ifndef RW_KINEMATICS_FKRANGE_HPP
#define RW_KINEMATICS_FKRANGE_HPP

/**
 * @file FKRange.hpp
 */

#include <rw/math/Transform3D.hpp>

namespace rw { namespace kinematics {

    class Frame;
    class State;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Forward kinematics between a pair of frames.
     *
     * FKRange finds the relative transform between a pair of frames. FKRange
     * finds the path connecting the pair of frames and computes the total
     * transform of this path. Following initialization, FKRange assumes that
     * the path does not change structure because of uses of the attachFrame()
     * feature. If the structure of the path has changed, the FKRange will
     * produce wrong results.
     *
     * FKRange is guaranteed to select the \e shortest path connecting the
     * frames, i.e. the path doesn't go all the way down to the root if it can
     * be helped.
     */
    class FKRange
    {
    public:
        /**
         * @brief Forward kinematics for the path leading from \b from to \b to.
         *
         * If a frame of NULL is passed as argument, it is interpreted to mean
         * the WORLD frame.
         *
         * @param from [in] The start frame.
         *
         * @param to [in] The end frame.
         *
         * @param state [in] The path structure.
         */
        FKRange(const Frame* from, const Frame* to, const State& state);

        /**
         * @brief Default constructor
         *
         * Will always return an identity matrix as the transform
         */
        FKRange();

        /**
         * @brief The relative transform between the frames.
         *
         * @param state [in] Configuration values for the frames of the tree.
         */
        math::Transform3D<> get(const State& state) const;

    private:
        std::vector<const Frame*> _inverseBranch;
        std::vector<const Frame*> _forwardBranch;

    };

    /*@}*/
}} // end namespaces

#endif // end include guard
