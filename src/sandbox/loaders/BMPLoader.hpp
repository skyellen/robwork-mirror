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

#ifndef RW_LOADERS_IMAGE_BMPLOADER_HPP
#define RW_LOADERS_IMAGE_BMPLOADER_HPP

#include <rw/sensor/Image.hpp>

namespace rw { namespace loaders {
	/**
	 * @brief
	 */
	class BMPLoader
	{
	public:
		static rw::sensor::ImagePtr load(const std::string& filename);

		static void save(rw::sensor::ImagePtr img, const std::string& filename);
	};
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
