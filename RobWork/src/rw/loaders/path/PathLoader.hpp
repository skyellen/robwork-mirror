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


#ifndef RW_LOADERS_PATHLOADER_HPP
#define RW_LOADERS_PATHLOADER_HPP

/**
 * @file PathLoader.hpp
 */

#include <string>
#include <memory>
#include <vector>

#include <rw/trajectory/Timed.hpp>
#include <rw/trajectory/Path.hpp>

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

    	/**
    	 * @brief store a QPath to file
    	 * @param path
    	 * @param file
    	 */
        static void storePath(
            const rw::trajectory::QPath& path,
            const std::string& file);

        /**
         * @brief Store the sequence \b path of \b workcell to the file named \b file.
         *
         *  See also loadTimedStatePath().
         */
        static void storeTimedStatePath(
            const rw::models::WorkCell& workcell,
            const rw::trajectory::TimedStatePath& path,
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
            const rw::trajectory::StatePath& path,
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
        static rw::trajectory::QPath loadPath(const std::string& file);

        /**
           @brief Load a sequence of states for \b workcell from the file named \b
           file.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.

           See also storeTimedStatePath().
        */
        static rw::trajectory::TimedStatePath loadTimedStatePath(
            const rw::models::WorkCell& workcell,
            const std::string& file);

    //private:
        // Everything below we probably don't want to support: All we will
        // support is the loading and storing of time stamped paths.
        // JIMMY: hmm, i can use it ;)

        /**
         * @brief Store the sequence \b path of \b workcell to the file named \b file.
         *
         *  See also loadStatePath().
         */
        static void storeStatePath(
            const models::WorkCell& workcell,
            const rw::trajectory::StatePath& path,
            const std::string& file);

        /**
           @brief Load a sequence of states for \b workcell from the file named \b
           file.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.

           See also storeStatePath().
        */
        static rw::trajectory::StatePath loadStatePath(
            const models::WorkCell& workcell,
            const std::string& file);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
