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


#ifndef RW_KINEMATICS_FKTABLE_HPP
#define RW_KINEMATICS_FKTABLE_HPP

/**
 * @file FKTable.hpp
 */

#include "State.hpp"

#include <rw/math/Transform3D.hpp>
#include "FrameMap.hpp"

/*
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
*/

namespace rw { namespace kinematics {

    class Frame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Forward kinematics for a set of frames.
     *
     * FKTable finds transforms for frames for a given fixed work cell state.
     * The frame transforms are calculated relative to the world frame.
     */
    class FKTable
    {
    public:
        /**
         * @brief Forward kinematics for the work cell state \b state.
         *
         * @param state [in] The work state for which world transforms are to be
         * calculated.
         */
        FKTable(const State& state);

        /**
         * @brief Forward kinematics for the work cell state \b state.
         * @param state [in] The work state for which world transforms are to be
         * calculated.
        */
        FKTable(const State* state);

        /**
         * @brief The world transform for the frame \b frame.
         *
         * @param frame [in] The frame for which to find the world transform.
         *
         * @return The transform of the frame relative to the world.
         */
        const math::Transform3D<>& get(const Frame& frame) const;

        inline const math::Transform3D<>& get(const Frame* frame) const{ return get(*frame); }

        /**
         * @brief Returns State associated with the FKTable
         *
         * The State returned is the State used to calculate the forward kinematics.
         *
         * @return State used to calculate the forward kinematics
         */
        const State& getState() const { return *_sp; }

        /**
         * @brief resets the FKTable to \b state
         * @param state
         */
        void reset(const State& state);

    private:
        const State* _sp;
        State _state;

        /*
        struct Entry {
            Entry(const Frame* frame) : frame(frame), transform() {}

            Entry(const Frame* frame, const math::Transform3D<>& transform) :
                frame(frame),
                transform(transform)
            {}

            const Frame* frame;
            math::Transform3D<> transform;
        };
        */

        /*
        typedef boost::multi_index_container<
            Entry,
            boost::multi_index::indexed_by<
                boost::multi_index::hashed_unique<
                    boost::multi_index::member<Entry, const Frame*, &Entry::frame> > > >
        TransformMap;
        */

        typedef FrameMap<math::Transform3D<> > TransformMap;

        // typedef std::map<const Frame*, math::Transform3D<> > TransformMap;
        // typedef TransformMap::value_type Entry;

        mutable TransformMap _transforms;

        /*
        TransformMap::iterator _end;
        */

    private:
        FKTable(const FKTable&);
        //FKTable& operator=(const FKTable&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
