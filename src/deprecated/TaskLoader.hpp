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


#ifndef RW_LOADERS_TASKLOADER_HPP
#define RW_LOADERS_TASKLOADER_HPP

/**
 * @file TaskLoader.hpp
 */

#include <string>
#include <rw/models/WorkCell.hpp>

// Forward declarations
namespace rw { namespace task { class Task; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

    /**
       @brief Loader for task files.
     */
    class TaskLoader
    {
    public:
        /**
           @brief Loads/imports a task from a file.

           An exception is thrown if the file can't be loaded.

           If \b optional_workcell is non-null this workcell is used for the
           Task object. Otherwise the workcell specified in the task file is
           used.

           @param filename [in] name of task file.
           @param optional_workcell [in] optional workcell.
         */
        static task::Task load(
            const std::string& filename, models::WorkCellPtr optional_workcell);

    private:
        TaskLoader() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
