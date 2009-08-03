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

#include "WorkCellLoader.hpp"

#include <rw/loaders/tul/TULLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common;

WorkCellPtr WorkCellLoader::load(const std::string& file)
{
    const std::string ext = StringUtil::getFileExtension(file);
	if (ext == ".wu" || ext == ".wc" || ext == ".tag" || ext == ".dev") {
        return TULLoader::load(file);
	}
	else {
	    return XMLRWLoader::loadWorkCell(file);
	}
}
