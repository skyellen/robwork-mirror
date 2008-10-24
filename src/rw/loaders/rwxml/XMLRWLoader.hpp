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

#ifndef RW_LOADERS_XMLRWLOADER_HPP
#define RW_LOADERS_XMLRWLOADER_HPP

#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

namespace rw { namespace loaders {
	/** @addtogroup loaders */
	/*@{*/

	/**
	 * @brief this class loads a workcell in xml format from a filename.
	 *
	 */
    class XMLRWLoader
    {
    public:
        /**
         * @brief Loads/imports robwork workcell in XML file format
         *
         * An exception is thrown if the file can't be loaded.
         *
         * @param filename [in] filename of XML file
         */
        static std::auto_ptr<rw::models::WorkCell> loadWorkCell(
            const std::string& filename);
    };

	/*@}*/
}} // end namespaces

#endif // end include guard
