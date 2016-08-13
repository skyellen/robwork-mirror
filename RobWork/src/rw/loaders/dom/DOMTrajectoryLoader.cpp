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

DOMTrajectoryLoader::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		DOMBasisTypes::Initializer init;
		idQTrajectory();
		idV3DTrajectory();
		idR3DTrajectory();
		idT3DTrajectory();
		idQLinearInterpolator();
		idQCubicSplineInterpolator();
		idV3DLinearInterpolator();
		idV3DCubicSplineInterpolator();
		idV3DCircularInterpolator();
		idR3DLinearInterpolator();
		idR3DCubicSplineInterpolator();
		idT3DLinearInterpolator();
		idT3DCubicSplineInterpolator();
		idParabolicBlend();
		idLloydHaywardBlend();
		idDurationAttribute();
		idStartTimeAttribute();
		idTauAttribute();
		idKappaAttribute();
		done = true;
	}
}

const DOMTrajectoryLoader::Initializer DOMTrajectoryLoader::initializer;

const std::string& DOMTrajectoryLoader::idQTrajectory() {
	static const std::string id("QTrajectory");
	return id;
}

const std::string& DOMTrajectoryLoader::idV3DTrajectory() {
	static const std::string id("V3DTrajectory");
	return id;
}

const std::string& DOMTrajectoryLoader::idR3DTrajectory() {
	static const std::string id("R3DTrajectory");
	return id;
}

const std::string& DOMTrajectoryLoader::idT3DTrajectory() {
	static const std::string id("T3DTrajectory");
	return id;
}

const std::string& DOMTrajectoryLoader::idQLinearInterpolator() {
	static const std::string id("QLinearInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idQCubicSplineInterpolator() {
	static const std::string id("QCubicSplineInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idV3DLinearInterpolator() {
	static const std::string id("V3DLinearInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idV3DCubicSplineInterpolator() {
	static const std::string id("V3DCubicSplineInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idV3DCircularInterpolator() {
	static const std::string id("V3DCircularInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idR3DLinearInterpolator() {
	static const std::string id("R3DLinearInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idR3DCubicSplineInterpolator() {
	static const std::string id("R3DCubicSplineInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idT3DLinearInterpolator() {
	static const std::string id("T3DLinearInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idT3DCubicSplineInterpolator() {
	static const std::string id("T3DCubicSplineInterpolator");
	return id;
}

const std::string& DOMTrajectoryLoader::idParabolicBlend() {
	static const std::string id("ParabolicBlend");
	return id;
}

const std::string& DOMTrajectoryLoader::idLloydHaywardBlend() {
	static const std::string id("LloydHaywardBlend");
	return id;
}

const std::string& DOMTrajectoryLoader::idDurationAttribute() {
	static const std::string id("duration");
	return id;
}

const std::string& DOMTrajectoryLoader::idStartTimeAttribute() {
	static const std::string id("starttime");
	return id;
}

const std::string& DOMTrajectoryLoader::idTauAttribute() {
	static const std::string id("tau");
	return id;
}

const std::string& DOMTrajectoryLoader::idKappaAttribute() {
	static const std::string id("kappa");
	return id;
}

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
        return element->getAttributeValueAsDouble(DOMTrajectoryLoader::idDurationAttribute(), 1.0);
    }

    double readStartTime(DOMElem::Ptr element) {
    	return element->getAttributeValueAsDouble(DOMTrajectoryLoader::idStartTimeAttribute(), 0.0);
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
        	double tau = element->getAttributeValueAsDouble(DOMTrajectoryLoader::idTauAttribute());
            return new ParabolicBlend<T>(int1, int2, tau);
        }
    };


    template <class T>
    class LloydHaywardBlendParser {
    public:
        static LloydHaywardBlend<T>* read(DOMElem::Ptr, Interpolator<T>* int1, Interpolator<T>* int2);
    };


    template<class T> LloydHaywardBlend<T>* LloydHaywardBlendParser<T>::read(DOMElem::Ptr element, Interpolator<T>* int1, Interpolator<T>* int2) {
        double tau = element->getAttributeValueAsDouble(DOMTrajectoryLoader::idTauAttribute());
        double kappa = element->getAttributeValueAsDouble(DOMTrajectoryLoader::idKappaAttribute());
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
            return DOMTrajectoryLoader::idQLinearInterpolator();
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::idQCubicSplineInterpolator();
        }

        virtual const std::string& circularInterpolatorId() {
			const static std::string str = "";
            return str;
        }
    };

    class V3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::idV3DLinearInterpolator();
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::idV3DCubicSplineInterpolator();
        }
        virtual const std::string& circularInterpolatorId() {
            return DOMTrajectoryLoader::idV3DCircularInterpolator();
        }
    };

    class R3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::idR3DLinearInterpolator();
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::idR3DCubicSplineInterpolator();
        }

        virtual const std::string& circularInterpolatorId() {
			const static std::string str = "";
            return str;
        }
    };

    class T3DIdentifiers: public Identifiers {
        virtual const std::string& linearInterpolatorId() {
            return DOMTrajectoryLoader::idT3DLinearInterpolator();
        }

        virtual const std::string& cubicSplineInterpolatorId() {
            return DOMTrajectoryLoader::idT3DCubicSplineInterpolator();
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
			} else if (child->isName(DOMTrajectoryLoader::idParabolicBlend()) ) {
				LinearInterpolator<T>* linear1 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex-1]);
				LinearInterpolator<T>* linear2 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex]);

				if (linear1 == NULL || linear2 == NULL)
					RW_THROW("ParabolicBlends can only be constructed between LinearInterpolator's");

				blend = ParabolicBlendParser<T>::read(child, linear1, linear2);
			} else if (child->isName(DOMTrajectoryLoader::idLloydHaywardBlend()) ) {
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
    if (element->isName(DOMTrajectoryLoader::idQTrajectory() ) ) {
        QIdentifiers ids;
        _qTrajectory = read<Q>(element, &ids);
        _type = QType;
    } else if (element->isName( DOMTrajectoryLoader::idV3DTrajectory() ) ) {
        V3DIdentifiers ids;
        _v3dTrajectory = read<Vector3D<> >(element, &ids);
        _type = Vector3DType;
    } else if (element->isName(DOMTrajectoryLoader::idR3DTrajectory()) ) {
        R3DIdentifiers ids;
        _r3dTrajectory = read<Rotation3D<> >(element, &ids);
        _type = Rotation3DType;
    } else if (element->isName(DOMTrajectoryLoader::idT3DTrajectory()) ) {
        T3DIdentifiers ids;
        _t3dTrajectory = read<Transform3D<> >(element, &ids);
        _type = Transform3DType;
    } else {

    }
}

