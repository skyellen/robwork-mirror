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

#include "DOMPathLoader.hpp"

#include <iostream>
#include <rw/math/Q.hpp>

#include <rw/common/DOMParser.hpp>

#include <rw/models/WorkCell.hpp>

#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include "DOMBasisTypes.hpp"


using namespace rw;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::kinematics;
using namespace rw::models;

const std::string DOMPathLoader::QPathId("QPath");
const std::string DOMPathLoader::V3DPathId("V3DPath");
const std::string DOMPathLoader::R3DPathId("R3DPath");
const std::string DOMPathLoader::T3DPathId("T3DPath");
const std::string DOMPathLoader::StatePathId("StatePath");
const std::string DOMPathLoader::TimedQPathId("TimedQPath");
const std::string DOMPathLoader::TimedStatePathId("TimedStatePath");
const std::string DOMPathLoader::TimedStateId("TimedState");
const std::string DOMPathLoader::TimedQId("TimedQ");
const std::string DOMPathLoader::TimeId("Time");


DOMPathLoader::DOMPathLoader(const std::string& filename, rw::models::WorkCell::Ptr workcell, const std::string& schemaFileName)
{
    _workcell = workcell;
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(filename);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    readPath(elementRoot);
}


DOMPathLoader::DOMPathLoader(std::istream& instream, rw::models::WorkCell::Ptr workcell, const std::string& schemaFileName) {
    _workcell = workcell;
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load(instream);
    DOMElem::Ptr elementRoot = parser->getRootElement();
    readPath(elementRoot);
}


DOMPathLoader::DOMPathLoader(DOMElem::Ptr element) {
    readPath(element);
}

DOMPathLoader::~DOMPathLoader()
{
}


namespace {

    template <class T>
    class ElementReader {
    public:
		ElementReader(WorkCell::Ptr workcell = NULL) {
            _workcell = workcell;
        }

        T readElement(DOMElem::Ptr element);
    protected:
		WorkCell::Ptr _workcell;
    };


    template<> Q ElementReader<Q>::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readQ(element, true);
    }

    template<> Vector3D<> ElementReader<Vector3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readVector3D(element, true);
    }

    template<> Rotation3D<> ElementReader<Rotation3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readRotation3DStructure(element);
    }


    template<> Transform3D<> ElementReader<Transform3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readTransform3D(element, true);
    }

    template<> State ElementReader<State>::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readState(element, _workcell, true);
    }

    template<> TimedQ ElementReader<TimedQ>::readElement(DOMElem::Ptr element) {
        double time = 0.0;
        Q q;
        BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
			if (element->isName(DOMPathLoader::TimeId)) {
				time = element->getValueAsDouble();
			} else if (element->isName(DOMBasisTypes::QId)) {
				q = DOMBasisTypes::readQ(child, false);
			}
        }
        return makeTimed(time, q);
    }

    template<> TimedState ElementReader<TimedState>::readElement(DOMElem::Ptr element) {
        double time = 0.0;
        State state;
        BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
			if (element->isName(DOMPathLoader::TimeId)) {
				time = element->getValueAsDouble();
			} else if (element->isName(DOMBasisTypes::StateId)) {
				state = DOMBasisTypes::readState(child, _workcell, false);
			}
        }
        return makeTimed(time, state);
    }

    template <class T, class R>
	void read(DOMElem::Ptr element,R result, WorkCell::Ptr workcell = NULL) {
    	ElementReader<T> reader;
    	BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
                T val = reader.readElement(child);
                result->push_back(val);
        }
    }


} //end namespace




DOMPathLoader::Type DOMPathLoader::getType() {
    return _type;
}


QPath::Ptr DOMPathLoader::getQPath() {
    if (_type != QType)
        RW_THROW("The loaded Path is not of type QPath. Use DOMPathLoader::getType() to read its type");
    return _qPath;
}



Vector3DPath::Ptr DOMPathLoader::getVector3DPath() {
    if (_type != Vector3DType)
        RW_THROW("The loaded Path is not of type Vector3DPath. Use DOMPathLoader::getType() to read its type");
    return _v3dPath;
}

Rotation3DPath::Ptr DOMPathLoader::getRotation3DPath() {
    if (_type != Rotation3DType)
        RW_THROW("The loaded Path is not of type Rotation3DPath. Use DOMPathLoader::getType() to read its type");
    return _r3dPath;
}


Transform3DPath::Ptr DOMPathLoader::getTransform3DPath()
{
    if (_type != Transform3DType)
        RW_THROW("The loaded Path is not of type Transform3DPath. Use DOMPathLoader::getType() to read its type");
    return _t3dPath;
}


StatePath::Ptr DOMPathLoader::getStatePath() {
    if (_type != StateType)
        RW_THROW("The loaded Path is not of type StatePath. Use DOMPathLoader::getType() to read its type");
    return _statePath;
}




rw::trajectory::TimedQPath::Ptr DOMPathLoader::getTimedQPath() {
    if (_type != TimedQType)
        RW_THROW("The loaded Path is not of type TimedQPath. Use DOMPathLoader::getType() to read its type");
    return _timedQPath;
}

rw::trajectory::TimedStatePath::Ptr DOMPathLoader::getTimedStatePath() {
    if (_type != TimedStateType)
        RW_THROW("The loaded Path is not of type TimedStatePath. Use DOMPathLoader::getType() to read its type");
    return _timedStatePath;
}



void DOMPathLoader::readPath(DOMElem::Ptr element) {
    if (element->isName(QPathId)) {
        _qPath = ownedPtr(new QPath());
		read<Q, QPath::Ptr>(element, _qPath);
        _type = QType;
    } else if (element->isName(V3DPathId)) {
        _v3dPath = ownedPtr(new Vector3DPath());
		read<Vector3D<>, Vector3DPath::Ptr>(element, _v3dPath);
        _type = Vector3DType;
    } else if (element->isName(R3DPathId)) {
        _r3dPath = ownedPtr(new Rotation3DPath());
		read<Rotation3D<>, Rotation3DPath::Ptr>(element, _r3dPath);
        _type = Rotation3DType;
    } else if (element->isName(T3DPathId)) {
        _t3dPath = ownedPtr(new Transform3DPath());
		read<Transform3D<>, Transform3DPath::Ptr>(element, _t3dPath);
        _type = Transform3DType;
    } else if(element->isName(StatePathId)) {
        _statePath = ownedPtr(new StatePath());
		read<State, StatePath::Ptr>(element, _statePath, _workcell);
        _type = StateType;
    } else if(element->isName(TimedQPathId)) {
        _timedQPath = ownedPtr(new TimedQPath());
		read<TimedQ, TimedQPath::Ptr>(element, _timedQPath, _workcell);
        _type = TimedQType;
    } else if(element->isName(TimedStatePathId)) {
        _timedStatePath = ownedPtr(new TimedStatePath());
		read<TimedState, TimedStatePath::Ptr>(element, _timedStatePath, _workcell);
        _type = TimedStateType;
    } else {
        //The element is not one we are going to parse.
    }
}

