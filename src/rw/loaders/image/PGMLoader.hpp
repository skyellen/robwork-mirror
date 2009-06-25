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

#ifndef RW_LOADERS_PGMLOADER_HPP
#define RW_LOADERS_PGMLOADER_HPP

/**
 * @file PGMLoader.hpp
 */

#include <rw/sensor/Image.hpp>

//#include "ImageLoader.hpp"

namespace rw { namespace loaders {

    /** @addtogroup loaders */
    /*@{*/

	/**
	 * @brief Loads images in Portable Gray Map (PGM) format.
	 *
	 * The image format is quite simple and editors like Gimp and Photoshop are
	 * able to view and edit this format.
	 */
	class PGMLoader //: public ImageLoader
	{
	public:

	    /**
	     * @param filename [in] name of the file that is to be loaded.
	     * @return if loaded successfully a pointer to the image is returned else NULL
	     */
		static rw::sensor::ImagePtr load(const std::string& filename);
	};
	/*@}*/
}}

#endif /*RW_LOADERS_PGMLOADER_HPP*/
