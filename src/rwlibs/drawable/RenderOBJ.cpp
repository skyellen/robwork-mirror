/*********************************************************************
 * RobWork Version 0.2
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

#include "RenderOBJ.hpp"


#include <rw/math/Vector3D.hpp>
#include <rw/common/macros.hpp>

#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

// --- TEST START ------------------------------------------------------------------------------------
#include <sys/timeb.h>
#include <time.h>
// --- TEST END --------------------------------------------------------------------------------------

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::geometry;

RenderOBJ::RenderOBJ(const std::string &filename)
{
	struct _timeb timebuffer;
	_ftime( &timebuffer );
	double __t1 = timebuffer.time + timebuffer.millitm / 1000.0;

	_reader.ParseFile(filename);
	//_reader.Test1();

	Render(1.0f, _displayListId);

	_ftime( &timebuffer );
	double __t2 = timebuffer.time + timebuffer.millitm / 1000.0;
	std::cout << "Time parsing obj:  " << __t2-__t1 << " [sec]" << std::endl;
}

void RenderOBJ::Render(float alpha, GLuint &displayListId) const
{
	if (displayListId != 0) {
		glDeleteLists(displayListId, 1);
	}

	displayListId = glGenLists(1);
	glNewList(displayListId, GL_COMPILE);
	glPushMatrix();

	_reader.Render(alpha);

	glPopMatrix();
	glEndList();
}
