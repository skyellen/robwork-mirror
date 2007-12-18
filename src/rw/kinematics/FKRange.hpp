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

#ifndef rw_kinematics_FKRange_HPP
#define rw_kinematics_FKRange_HPP

/**
 * @file FKRange.hpp
 */

#include "State.hpp"

#include <rw/math/Transform3D.hpp>

#include <map>

namespace rw { namespace kinematics {

    class Frame;

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Forward kinematics between a pair of frames.
     *
     * FKRange finds the relative transform between a pair of frames. FKRange
     * finds the path connecting the pair of frames and computes the total
     * transform of this path. Following initialization, FKRange assumes that
     * the path does not change structure because of uses of the atttachFrame()
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
         * @brief Forward kinematics for the path leading from \a from to \a to.
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
         * @brief The relative transform between the frames.
         *
         * @param state [in] Configuration values for the frames of the tree.
         */
        math::Transform3D<> get(const State& state) const;

    private:
        std::vector<const Frame*> _inverseBranch;
        std::vector<const Frame*> _forwardBranch;

    private:
        FKRange(const FKRange&);
        FKRange& operator=(const FKRange&);
    };

    /*@}*/
}} // end namespaces

#endif // end include guard
