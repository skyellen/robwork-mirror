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
 
#include "CSGUtil.hpp"

#include <rw/loaders/model3d/STLFile.hpp>
#include "CSGConvert.hpp"



using namespace std;
using namespace rw;
using namespace rw::common;
using namespace rw::math;
using namespace rw::geometry;
using namespace rwlibs::csg;
using namespace rw::loaders;

	
void CSGUtil::saveToStl(const std::string& filename, CSGModel& csgmodel) {
	STLFile::save(*csgmodel.getTriMesh(), filename);
}

