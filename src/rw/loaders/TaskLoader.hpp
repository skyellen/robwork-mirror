/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.

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
