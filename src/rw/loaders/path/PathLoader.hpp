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

#ifndef rw_loaders_PathLoader_HPP
#define rw_loaders_PathLoader_HPP

/**
 * @file PathLoader.hpp
 */

#include <string>
#include <memory>
#include <vector>

#include <rw/interpolator/Timed.hpp>
#include <rw/interpolator/TimedStatePath.hpp>
#include <rw/interpolator/StatePath.hpp>

#include <rw/pathplanning/Path.hpp>

// Forward declarations
namespace rw { namespace models { class WorkCell; }}
namespace rw { namespace kinematics { class State; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
       @brief Load and store for various types of paths.

       Probably, what we want to store probably is not paths, but trajectories.
       Perhaps each type of trajectory may have its own storage format. We will
       see. So far storeVelocityTimedStatePath() and loadTimedStatePath() are
       useful utilities to have as is.
     */
    class PathLoader
    {
    public:
        //! A tuple (time, state).
        typedef interpolator::Timed<kinematics::State> TimedState;

        static void storePath(
            const rw::pathplanning::Path& path,
            const std::string& file);

        /**
         * @brief Store the sequence \b path of \b workcell to the file named \b file.
         *
         *  See also loadTimedStatePath().
         */
        static void storeTimedStatePath(
            const rw::models::WorkCell& workcell,
            const rw::interpolator::TimedStatePath& path,
            const std::string& file);

        /**
         * @brief Time stamp the sequence \b path of \b workcell and store it to
         * the file named \b file.
         *
         * The states are given time stamps according to the maximum joint speed
         * velocities of \b workcell.
         *
         * The time stamped sequence can be loaded again with
         * loadTimedStatePath().
         */
        static void storeVelocityTimedStatePath(
            const models::WorkCell& workcell,
            const rw::interpolator::StatePath& path,
            const std::string& file);

        /**
         * @brief Loads a Path of robot configuration
         *
         * Load and return a path from a file.
         * Throws an exception if an error occurs
         *
         * @param file [in] file name
         * @return the path
         */
        static rw::pathplanning::Path loadPath(const std::string& file);

        /**
           @brief Load a sequence of states for \b workcell from the file named \b
           file.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.

           See also storeTimedStatePath().
        */
        static std::auto_ptr<rw::interpolator::TimedStatePath> loadTimedStatePath(
            const rw::models::WorkCell& workcell,
            const std::string& file);

    private:
        // Everything below we probably don't want to support: All we will
        // support is the loading and storing of time stamped paths.

        /**
         * @brief Store the sequence \b path of \b workcell to the file named \b file.
         *
         *  See also loadStatePath().
         */
        static void storeStatePath(
            const models::WorkCell& workcell,
            const rw::interpolator::StatePath& path,
            const std::string& file);

        /**
           @brief Load a sequence of states for \b workcell from the file named \b
           file.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.

           See also storeStatePath().
        */
        static std::auto_ptr<rw::interpolator::StatePath> loadStatePath(
            const models::WorkCell& workcell,
            const std::string& file);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
