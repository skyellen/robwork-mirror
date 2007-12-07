/*********************************************************************
 * RobWork Version 0.2
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

#ifndef rw_loaders_tul_TULLoader_HPP
#define rw_loaders_tul_TULLoader_HPP

/**
 * @file TULLoader.hpp
 */

#include <string>
#include <memory>

// Forward declarations
namespace rw { namespace models {
    class WorkCell;
}}

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /* @{*/

    /**
     * @brief Loader for the AMROSE TUL format
     */
    class TULLoader
    {
    public:
        /**
         * @brief Loads/imports TUL file
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] filename of TUL file
         */
        static std::auto_ptr<models::WorkCell> LoadTUL(
            const std::string& filename);
    };

    /**@}*/
}} // end namespaces

#endif // end include guard
