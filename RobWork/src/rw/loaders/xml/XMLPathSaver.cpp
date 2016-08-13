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

XMLPathSaver::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLBasisTypes::Initializer init1;
		XMLPathFormat::Initializer init2;
		done = true;
	}
}

const XMLPathSaver::Initializer XMLPathSaver::initializer;

void XMLPathSaver::save(const QPath& path, const std::string& filename) {
    XMLPathSaver::savePath<Q,  QPath>(path, XMLPathFormat::idQPath(), filename);
}

void XMLPathSaver::save(const Vector3DPath& path, const std::string& filename) {
    XMLPathSaver::savePath<Vector3D<>,  Vector3DPath>(path, XMLPathFormat::idV3DPath(), filename);
}

void XMLPathSaver::save(const Rotation3DPath& path, const std::string& filename) {
    XMLPathSaver::savePath<Rotation3D<>,  Rotation3DPath>(path, XMLPathFormat::idR3DPath(), filename);
}

void XMLPathSaver::save(const Transform3DPath& path, const std::string& filename) {
    XMLPathSaver::savePath<Transform3D<>,  Transform3DPath>(path, XMLPathFormat::idT3DPath(), filename);
}

void XMLPathSaver::save(const rw::trajectory::StatePath& path, const std::string& filename) {
    XMLPathSaver::savePath<State, StatePath>(path, XMLPathFormat::idStatePath(), filename);
}

void XMLPathSaver::save(const rw::trajectory::TimedQPath& path, const std::string& filename) {
    XMLPathSaver::savePath<TimedQ, TimedQPath>(path, XMLPathFormat::idTimedQPath(), filename);
}

void XMLPathSaver::save(const rw::trajectory::TimedStatePath& path, const std::string& filename) {
    XMLPathSaver::savePath<TimedState, TimedStatePath>(path, XMLPathFormat::idTimedStatePath(), filename);
}




void XMLPathSaver::write(const QPath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<Q,  QPath>(path, XMLPathFormat::idQPath(), outstream);
}

void XMLPathSaver::write(const Vector3DPath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<Vector3D<>,  Vector3DPath>(path, XMLPathFormat::idV3DPath(), outstream);
}

void XMLPathSaver::write(const Rotation3DPath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<Rotation3D<>,  Rotation3DPath>(path, XMLPathFormat::idR3DPath(), outstream);
}

void XMLPathSaver::write(const Transform3DPath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<Transform3D<>,  Transform3DPath>(path, XMLPathFormat::idT3DPath(), outstream);
}


void XMLPathSaver::write(const rw::trajectory::StatePath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<State, StatePath>(path, XMLPathFormat::idStatePath(), outstream);
}

void XMLPathSaver::write(const rw::trajectory::TimedQPath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<TimedQ, TimedQPath>(path, XMLPathFormat::idTimedQPath(), outstream);
}

void XMLPathSaver::write(const rw::trajectory::TimedStatePath& path, std::ostream& outstream) {
    XMLPathSaver::writePath<TimedState, TimedStatePath>(path, XMLPathFormat::idTimedStatePath(), outstream);
}
