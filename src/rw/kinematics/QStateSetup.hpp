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

#ifndef rw_kinematics_QStateSetup_HPP
#define rw_kinematics_QStateSetup_HPP

/**
 * @file QStateSetup.hpp
 */

#include "Tree.hpp"
#include "Frame.hpp"
#include <boost/shared_ptr.hpp>

namespace rw { namespace kinematics {

    /** @addtogroup kinematics */
    /*@{*/

    /**
     * @brief Utility class to help construct a QState
     *
     * QStateSetup contains the data to share among QState objects, namely the
     * assignment of offsets to frames.
     */
    class QStateSetup
    {
    public:
        /**
         * @brief Creates an empty QState
         */
        QStateSetup(): _dof(0)
        {
        }
         
        /**
         * @brief Constructs QStateSetup
         */
        explicit QStateSetup(const Tree& tree)
        {
            const std::vector<Frame*>& frames = tree.getFrames();

            _offsets.resize(tree.getMaxCnt());

            // Traverse the frames and calculate the offsets.
            typedef std::vector<Frame*>::const_iterator I;
            int offset = 0;
            for (I p = frames.begin(); p != frames.end(); ++p) {
                const Frame& frame = **p;
                _offsets.at(frame.getID()) = offset;
                offset += frame.getDOF();
            }

            // The total sum of the dofs.
            _dof = offset;
        }

        /**
         * @brief The position in QState at which the configuration for \b frame
         * is stored.
         */
        int getOffset(const Frame& frame) const
        {
            return _offsets.at(frame.getID());
        }

        /**
         * @brief The total number of degrees of freedom of the frame tree.
         *
         * This number equals the length of the QState array.
         */
        int getDOF() const { return _dof; }

    private:
        // Offsets into the QState array.
        std::vector<int> _offsets;

        // The total sum of the dofs of the frames.
        int _dof;

    private:
        // You _can_ go around copying QStateSetup without memory leaks or other
        // infelicities, but we don't expect to do that so we disallow it.
        QStateSetup(const QStateSetup&);
        QStateSetup& operator=(const QStateSetup&);
    };
    /*@}*/
}} // end namespaces

#endif // end include guard
