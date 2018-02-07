/********************************************************************************
 * Copyright 2017 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#ifndef RW_LOADERS_PATHLOADER_CSV_HPP
#define RW_LOADERS_PATHLOADER_CSV_HPP

/**
 * @file PathLoader.hpp
 */

#include <string>

#include <rw/trajectory/Timed.hpp>
#include <rw/trajectory/Path.hpp>

// Forward declarations
namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace loaders {

/** @addtogroup loaders */
/* @{*/

/**
       @brief Loader for .csv files.

       This loader differs from the standard PathLoader,
       in that it only considers devices in the workcell.

       The .csv format is implemented as follows. First 2 lines is setup:
       \verbatim
       3; <- first line is the path length. The loader exist after loading 3 lines of configurations.
       6;  <- second line is degrees of freedom (DOF) for all devices in the workcell. (Has to match total DOF in workcell).
       \endverbatim
       Then follows the path itself, one configuration-vector at the time:
       \verbatim
       -1.5,0.0,-1.5,0.0,-1.5,0.0;
       -1.0,-1.5,0.0,-1.5,0.0,-1.5;
       -0.8,0.0,-1.5,0.0,-1.5,0.0;
       \endverbatim
       If loadTimedStatePath is used, a timestamp (in seconds) is added in the beginning of each line.

       If a workcell contains more than 1 device, the first device reads the first n values, second device reads next n values etc.

       An example of a file with a path length of 3, DOF of six and a timestamp is given below:
       \verbatim
        11;
        6;
        0.0,-1.5,0.0,-1.5,0.0,-1.5,0.0;
        1.0,-1.0,-1.5,0.0,-1.5,0.0,-1.5;
        1.5,-0.8,0.0,-1.5,0.0,-1.5,0.0;
       \endverbatim

     */
class PathLoaderCSV
{
public:


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
           file. This loader includes timestamps in the path.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.
        */
    static rw::trajectory::TimedStatePath loadTimedStatePath(
            const rw::models::WorkCell& workcell,
            const std::string& file);


    /**
           @brief Load a sequence of states for \b workcell from the file named \b
           file.

           An exception is thrown if the file can't be read or is of the wrong
           format with respect to the work cell.
        */
    static rw::trajectory::StatePath loadStatePath(
            const models::WorkCell& workcell,
            const std::string& file);
};

/**@}*/
}} // end namespaces

#endif // end include guard
