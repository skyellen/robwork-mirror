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

#include "XMLPathSaver.hpp"
#include "XMLPathLoader.hpp"

#include "XMLBasisTypes.hpp"


#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>


#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>


using namespace xercesc;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::math;
using namespace rw::kinematics;




void XMLPathSaver::save(const QPath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<Q,  QPath>(path, XMLPathFormat::QPathId, filename);
}

void XMLPathSaver::save(const Vector3DPath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<Vector3D<>,  Vector3DPath>(path, XMLPathFormat::V3DPathId, filename);
}

void XMLPathSaver::save(const Rotation3DPath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<Rotation3D<>,  Rotation3DPath>(path, XMLPathFormat::R3DPathId, filename);
}

void XMLPathSaver::save(const Transform3DPath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<Transform3D<>,  Transform3DPath>(path, XMLPathFormat::T3DPathId, filename);
}


void XMLPathSaver::save(const rw::trajectory::StatePath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<State, StatePath>(path, XMLPathFormat::StatePathId, filename);
}

void XMLPathSaver::save(const rw::trajectory::TimedQPath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<TimedQ, TimedQPath>(path, XMLPathFormat::TimedQPathId, filename);
}

void XMLPathSaver::save(const rw::trajectory::TimedStatePath& path, const std::string& filename) {
    XMLPathSaver::savePathImpl<TimedState, TimedStatePath>(path, XMLPathFormat::TimedStatePathId, filename);
}
