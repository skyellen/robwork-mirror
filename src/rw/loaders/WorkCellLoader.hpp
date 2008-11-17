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

#ifndef RW_LOADERS_WORKCELLLOADER_HPP
#define RW_LOADERS_WORKCELLLOADER_HPP

/**
 * @file WorkCellLoader.hpp
 */

#include <string>
#include <memory>

#include <rw/models/WorkCell.hpp>

// Forward declarations
//namespace rw { namespace models { class WorkCell; }}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Loader for workcell files.
     */
    class WorkCellLoader
    {
    public:
        /**
         * @brief Loads/imports a workcell from a file.
         *
         * An exception is thrown if the file can't be loaded.
         *
         * XML as well as TUL workcell formats are supported.
         *
         * @param filename [in] name of workcell file.
         */
        static models::WorkCellPtr load(const std::string& filename);

    private:
        WorkCellLoader() {}
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
