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

#include "DOMTrajectoryLoader.hpp"
#include "DOMBasisTypes.hpp"

#include <iostream>

#include <rw/common/DOMElem.hpp>
#include <rw/math/Q.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>
#include <rw/common/DOMParser.hpp>

using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::loaders;

const std::string DOMTrajectoryLoader::QTrajectoryId("QTrajectory");
const std::string DOMTrajectoryLoader::V3DTrajectoryId("V3DTrajectory");

const std::string DOMTrajectoryLoader::R3DTrajectoryId("R3DTrajectory");
const std::string DOMTrajectoryLoader::T3DTrajectoryId("T3DTrajectory");

const std::string DOMTrajectoryLoader::QLinearInterpolatorId("QLinearInterpolator");
const std::string DOMTrajectoryLoader::QCubicSplineInterpolatorId("QCubicSplineInterpolator");

const std::string DOMTrajectoryLoader::V3DLinearInterpolatorId("V3DLinearInterpolator");
const std::string DOMTrajectoryLoader::V3DCubicSplineInterpolatorId("V3DCubicSplineInterpolator");
const std::string DOMTrajectoryLoader::V3DCircularInterpolatorId("V3DCircularInterpolator");

const std::string DOMTrajectoryLoader::R3DLinearInterpolatorId("R3DLinearInterpolator");
const std::string DOMTrajectoryLoader::R3DCubicSplineInterpolatorId("R3DCubicSplineInterpolator");

const std::string DOMTrajectoryLoader::T3DLinearInterpolatorId("T3DLinearInterpolator");
const std::string DOMTrajectoryLoader::T3DCubicSplineInterpolatorId("T3DCubicSplineInterpolator");

const std::string DOMTrajectoryLoader::ParabolicBlendId("ParabolicBlend");
const std::string DOMTrajectoryLoader::LloydHaywardBlendId("LloydHaywardBlend");

const std::string DOMTrajectoryLoader::DurationAttributeId("duration");
const std::string DOMTrajectoryLoader::StartTimeAttributeId("starttime");
const std::string DOMTrajectoryLoader::TauAttributeId("tau");
const std::string DOMTrajectoryLoader::KappaAttributeId("kappa");



DOMTrajectoryLoader::DOMTrajectoryLoader(const std::string& filename, const std::string& schemaFileName)
{
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load( filename );
    DOMElem::Ptr elementRoot = parser->getRootElement();
    readTrajectory(elementRoot);
}


DOMTrajectoryLoader::DOMTrajectoryLoader(std::istream& instream, const std::string& schemaFileName) {
    DOMParser::Ptr parser = DOMParser::make();
    parser->setSchema(schemaFileName);
    parser->load( instream );
    DOMElem::Ptr elementRoot = parser->getRootElement();
    readTrajectory(elementRoot);
}

DOMTrajectoryLoader::~DOMTrajectoryLoader()
{
}



namespace {

    double readDuration(DOMElem::Ptr element) {
        return element->getAttributeValueAsDouble(DOMTrajectoryLoader::DurationAttributeId, 1.0);
    }

    double readStartTime(DOMElem::Ptr element) {
    	return element->getAttributeValueAsDouble(DOMTrajectoryLoader::StartTimeAttributeId, 0.0);
    }

    template <class T>
    class ElementReader {
    public:
        static T readElement(DOMElem::Ptr element);
    };

    template<> Q ElementReader<Q>::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readQ(element, false);
    }

    template<> Vector3D<> ElementReader<Vector3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readVector3D(element, false);
    }

    template<> Rotation3D<> ElementReader<Rotation3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readRotation3DStructure(element);
    }


    template<> Transform3D<> ElementReader<Transform3D<> >::readElement(DOMElem::Ptr element) {
        return DOMBasisTypes::readTransform3D(element, false);
    }

    template <class T>
    class LinearInterpolatorParser {
    public:
        static LinearInterpolator<T>* read(DOMElem::Ptr element) {
            double duration = readDuration(element);
            T vias[2];
            int index = 0;
            BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
            	if(index>1)
            		RW_THROW("Expected 2 via points for LinearInterpolator. Found: >2");
            	vias[index] =  ElementReader<T>::readElement(child);
            	index++;
            }
            if(index<1)
            	RW_THROW("Expected 3 via points for CircularInterpolator. Found:"<< index);

            return new LinearInterpolator<T>(vias[0], vias[1], duration);
        }
    };

    template <class T>
    class CircularInterpolatorParser {
    public:
        static CircularInterpolator<T>* read(DOMElem::Ptr element);
    };

    template<class T> CircularInterpolator<T>* CircularInterpolatorParser<T>::read(DOMElem::Ptr element) {
        RW_THROW("Only Vector3D is supported in CircularInterpolator");
    }

    template<> CircularInterpolator<Vector3D<> >* CircularInterpolatorParser<Vector3D<> >::read(DOMElem::Ptr element) {
        double duration = readDuration(element);
        Vector3D<> vias[3];
        int index = 0;
        BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
        	if(index>3)
        		RW_THROW("Expected 3 via points for CircularInterpolator. Found: >3");
        	vias[index] =  ElementReader<Vector3D<> >::readElement(child);
        	index++;
        }
        if(index<3)
        	RW_THROW("Expected 3 via points for CircularInterpolator. Found:"<< index);

        return new CircularInterpolator<Vector3D<> >(vias[0], vias[1], vias[2], duration);
    }

    template <class T>
    class ParabolicBlendParser {
    public:
        static ParabolicBlend<T>* read(DOMElem::Ptr element, LinearInterpolator<T>* int1, LinearInterpolator<T>* int2) {
        	double tau = element->getAttributeValueAsDouble(DOMTrajectoryLoader::TauAttributeId);
            return new ParabolicBlend<T>(int1, int2, tau);
        }
    };


    template <class T>
    class LloydHaywardBlendParser {
    public:
        static LloydHaywardBlend<T>* read(DOMElem::Ptr, Interpolator<T>* int1, Interpolator<T>* int2);
    };


    template<class T> LloydHaywardBlend<T>* LloydHaywardBlendParser<T>::read(DOMElem::Ptr element, Interpolator<T>* int1, Interpolator<T>* int2) {
        double tau = element->getAttributeValueAsDouble(DOMTrajectoryLoader::TauAttributeId);
        double kappa = element->getAttributeValueAsDouble(DOMTrajectoryLoader::KappaAttributeId);
        return new LloydHaywardBlend<T>(int1, int2, tau, kappa);
    }

    template <> LloydHaywardBlend<Rotation3D<> >* LloydHaywardBlendParser<Rotation3D<> >::read(DOMElem::Ptr, Interpolator<Rotation3D<> >* int1, Interpolator<Rotation3D<> >* int2) {
        RW_THROW("Rotation3D is not supported in LloydHaywardBlend");
    }

    class Identifiers {
    public:
        virtual const std::string& linearInterpolatorId() = 0;
        virtual const std::string& cubicSplineInterpolatorId() = 0;
        virtual const std::string& circularInterpolatorId() = 0;

        bool isInterpolator(const std::string& str) {
            return str == linearInterpolatorId() ||
                    str == cubicSplineInterpolatorId() ||
                    str == circularInterpolatorId();
        }
    };

    class QIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::QLinearInterpolatorId;
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::QCubicSplineInterpolatorId;
        }

        virtual const std::string& circularInterpolatorId() {
			const static std::string str = "";
            return str;
        }
    };

    class V3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::V3DLinearInterpolatorId;
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::V3DCubicSplineInterpolatorId;
        }
        virtual const std::string& circularInterpolatorId() {
            return DOMTrajectoryLoader::V3DCircularInterpolatorId;
        }
    };

    class R3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::R3DLinearInterpolatorId;
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::R3DCubicSplineInterpolatorId;
        }

        virtual const std::string& circularInterpolatorId() {
			const static std::string str = "";
            return str;
        }
    };

    class T3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::T3DLinearInterpolatorId;
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::T3DCubicSplineInterpolatorId;
        }

        virtual const std::string& circularInterpolatorId() {
			const static std::string str = "";
            return str;
        }
    };



    template <class T>
    Ptr<InterpolatorTrajectory<T> > read(DOMElem::Ptr element, Identifiers* ids) {
        double starttime = readStartTime(element);
        Ptr<InterpolatorTrajectory<T> > result = ownedPtr(new InterpolatorTrajectory<T>(starttime));

        std::vector<Interpolator<T>* > interpolators;

        //First we run through and finds the interpolators
        BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){
            if (element != NULL) {
                if (child->isName(ids->linearInterpolatorId()) ) {
                    LinearInterpolator<T>* linearinterpolator = LinearInterpolatorParser<T>::read(element);
                    interpolators.push_back(linearinterpolator);
                } else if (child->isName(ids->cubicSplineInterpolatorId() )) {
                    //TODO read in QCubicSplineInterpolator
                } else if (child->isName(ids->circularInterpolatorId() )) {
                    CircularInterpolator<T>* interpolator = CircularInterpolatorParser<T>::read(element);
                    interpolators.push_back(interpolator);
                }
            }
        }

        //Secondly we run through and finds the blends
        Blend<T>* blend = NULL;
        size_t interpolatorIndex = 0;
        BOOST_FOREACH(DOMElem::Ptr child, element->getChildren()){

			if (ids->isInterpolator( child->getName())) {
				if (blend == NULL) {
					result->add(ownedPtr(interpolators[interpolatorIndex]));
				} else {
					result->add(ownedPtr(blend), ownedPtr(interpolators[interpolatorIndex]));
					blend = NULL;
				}
				interpolatorIndex++;
			} else if (child->isName(DOMTrajectoryLoader::ParabolicBlendId) ) {
				LinearInterpolator<T>* linear1 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex-1]);
				LinearInterpolator<T>* linear2 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex]);

				if (linear1 == NULL || linear2 == NULL)
					RW_THROW("ParabolicBlends can only be constructed between LinearInterpolator's");

				blend = ParabolicBlendParser<T>::read(child, linear1, linear2);
			} else if (child->isName(DOMTrajectoryLoader::LloydHaywardBlendId) ) {
				Interpolator<T>* int1 = interpolators[interpolatorIndex-1];
				Interpolator<T>* int2 = interpolators[interpolatorIndex];
				blend = LloydHaywardBlendParser<T>::read(child, int1, int2);
			}
        }
        return result;
    }


} //end namespace

DOMTrajectoryLoader::Type DOMTrajectoryLoader::getType() {
    return _type;
}

QTrajectory::Ptr DOMTrajectoryLoader::getQTrajectory() {
    if (_type != QType)
        RW_THROW("The loaded Trajectory is not of type Q. Use DOMTrajectoryLoader::getType() to read its type");
    return _qTrajectory;
}

Vector3DTrajectory::Ptr DOMTrajectoryLoader::getVector3DTrajectory() {
    if (_type != Vector3DType)
        RW_THROW("The loaded Trajectory is not of type Vector3D<>. Use DOMTrajectoryLoader::getType() to read its type");

    return _v3dTrajectory;
}

Rotation3DTrajectory::Ptr DOMTrajectoryLoader::getRotation3DTrajectory() {
    if (_type != Rotation3DType)
        RW_THROW("The loaded Trajectory is not of type Rotation3D. Use DOMTrajectoryLoader::getType() to read its type");

    return _r3dTrajectory;
}

Transform3DTrajectory::Ptr DOMTrajectoryLoader::getTransform3DTrajectory() {
    if (_type != Transform3DType)
        RW_THROW("The loaded Trajectory is not of type Transform3D<>. Use DOMTrajectoryLoader::getType() to read its type");

    return _t3dTrajectory;
}


void DOMTrajectoryLoader::readTrajectory(DOMElem::Ptr element) {
    //Determine which type of trajectory we are using
    if (element->isName(DOMTrajectoryLoader::QTrajectoryId ) ) {
        QIdentifiers ids;
        _qTrajectory = read<Q>(element, &ids);
        _type = QType;
    } else if (element->isName( DOMTrajectoryLoader::V3DTrajectoryId ) ) {
        V3DIdentifiers ids;
        _v3dTrajectory = read<Vector3D<> >(element, &ids);
        _type = Vector3DType;
    } else if (element->isName(DOMTrajectoryLoader::R3DTrajectoryId) ) {
        R3DIdentifiers ids;
        _r3dTrajectory = read<Rotation3D<> >(element, &ids);
        _type = Rotation3DType;
    } else if (element->isName(DOMTrajectoryLoader::T3DTrajectoryId) ) {
        T3DIdentifiers ids;
        _t3dTrajectory = read<Transform3D<> >(element, &ids);
        _type = Transform3DType;
    } else {

    }
}

