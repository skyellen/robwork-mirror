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

#ifndef RWLIBS_DRAWABLE_RENDERPATH_HPP
#define RWLIBS_DRAWABLE_RENDERPATH_HPP

#include "Render.hpp"

class RenderPath : public Render
{
public:
	RenderPath(const std::string& filename);
	virtual ~RenderPath();

protected:
    virtual void update(UpdateType type);

};

#endif /*RWLIBS_DRAWABLE_RENDERPATH_HPP*/
