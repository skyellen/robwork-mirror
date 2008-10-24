/*********************************************************************
 * RobWork Version 0.3
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

#include <map>

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
           @brief Forward kinematics for the work cell state \b state.

           @param state [in] The work state for which world transforms are to be
           calculated.
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

    private:
        const State* _sp;
        State _state;

        const State& getState() const { return *_sp; }

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
        FKTable& operator=(const FKTable&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
