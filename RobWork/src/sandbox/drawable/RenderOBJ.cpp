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
//#include <sys/timeb.h>
//#include <time.h>
// --- TEST END --------------------------------------------------------------------------------------

using namespace rwlibs::drawable;
using namespace rw::math;
using namespace rw::geometry;

RenderOBJ::RenderOBJ(const std::string &filename)
{
	//struct _timeb timebuffer;
//	_ftime( &timebuffer );
	//double __t1 = timebuffer.time + timebuffer.millitm / 1000.0;

	_reader.load(filename);
	//_reader.Test1();

	render(1.0f, _displayListId);

	//_ftime( &timebuffer );
	//double __t2 = timebuffer.time + timebuffer.millitm / 1000.0;
	//std::cout << "Time parsing obj:  " << __t2-__t1 << " [sec]" << std::endl;
}

void RenderOBJ::render(float alpha, GLuint &displayListId) const
{
	if (displayListId != 0) {
		glDeleteLists(displayListId, 1);
	}

	displayListId = glGenLists(1);
	glNewList(displayListId, GL_COMPILE);
	glPushMatrix();

	_reader.render(alpha);

	glPopMatrix();
	glEndList();
}
