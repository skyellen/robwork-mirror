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

#include "XMLTrajectoryLoader.hpp"
#include "XMLTrajectoryFormat.hpp"
#include "XMLBasisTypes.hpp"

#include <iostream>

#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>
#include <xercesc/util/XMLDouble.hpp>


#include <rw/math/Q.hpp>
#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>


using namespace xercesc;
using namespace rw::common;
using namespace rw::math;
using namespace rw::trajectory;
using namespace rw::loaders;

XMLTrajectoryLoader::XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName)
{
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, filename, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    readTrajectory(elementRoot);
}





XMLTrajectoryLoader::XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName) {
    XercesDOMParser parser;
    xercesc::DOMDocument* doc = XercesDocumentReader::readDocument(parser, instream, schemaFileName);
    xercesc::DOMElement* elementRoot = doc->getDocumentElement();
    readTrajectory(elementRoot);
}

XMLTrajectoryLoader::~XMLTrajectoryLoader()
{
}



namespace {

    double readDuration(xercesc::DOMElement* element) {
        if (element->hasAttribute(XMLTrajectoryFormat::DurationAttributeId)) {
            const XMLCh* attr = element->getAttribute(XMLTrajectoryFormat::DurationAttributeId);
            XMLDouble xmlfloat(attr);
            return xmlfloat.getValue();
        }
        return 1;
    }

    double readStartTime(xercesc::DOMElement* element) {
        if (element->hasAttribute(XMLTrajectoryFormat::StartTimeAttributeId)) {
            const XMLCh* attr = element->getAttribute(XMLTrajectoryFormat::StartTimeAttributeId);
            XMLDouble xmlfloat(attr);
            return xmlfloat.getValue();
        }
        return 0;
    }


    double readAttribute(xercesc::DOMElement* element, const XMLCh* id) {
        if (element->hasAttribute(id)) {
            const XMLCh* attr = element->getAttribute(id);
            XMLDouble value(attr);
            return value.getValue();
        }
        RW_THROW("Unable to find attribute: \""<<XMLStr(id).str()<<"\"");
    }




    template <class T>
    class ElementReader {
    public:
        static T readElement(xercesc::DOMElement* element);
    };

    template<> Q ElementReader<Q>::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readQ(element, false);
    }

    template<> Vector3D<> ElementReader<Vector3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readVector3D(element, false);
    }

    template<> Rotation3D<> ElementReader<Rotation3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readRotation3DStructure(element);
    }


    template<> Transform3D<> ElementReader<Transform3D<> >::readElement(xercesc::DOMElement* element) {
        return XMLBasisTypes::readTransform3D(element, false);
    }

    template <class T>
    class LinearInterpolatorParser {
    public:
        static LinearInterpolator<T>* read(xercesc::DOMElement* element) {
            double duration = readDuration(element);
            DOMNodeList* children = element->getChildNodes();
            const  XMLSize_t nodeCount = children->getLength();
            T vias[2];
            int index = 0;
            for(XMLSize_t i = 0; i < nodeCount; ++i) {
                xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
                if (child != NULL) {
                    T tmp = ElementReader<T>::readElement(child);
                    vias[index] = tmp;
                    ++index;
                }
            }
            if (index != 2)
                RW_THROW("Expected 3 via points for CircularInterpolator. Found: "<<index);

            return new LinearInterpolator<T>(vias[0], vias[1], duration);
        }
    };



    template <class T>
    class CircularInterpolatorParser {
    public:
        static CircularInterpolator<T>* read(xercesc::DOMElement* element);
    };

    template<class T> CircularInterpolator<T>* CircularInterpolatorParser<T>::read(xercesc::DOMElement* element) {
        RW_THROW("Only Vector3D is supported in CircularInterpolator");
    }

    template<> CircularInterpolator<Vector3D<> >* CircularInterpolatorParser<Vector3D<> >::read(xercesc::DOMElement* element) {
        double duration = readDuration(element);
        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        Vector3D<> vias[3];
        int index = 0;
        for(XMLSize_t i = 0; i < nodeCount; ++i) {
            xercesc::DOMElement* child = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (child != NULL) {
                vias[index] = ElementReader<Vector3D<> >::readElement(child);
                ++index;
            }
        }
        if (index != 3)
            RW_THROW("Expected 3 via points for CircularInterpolator. Found: "<<index);

        return new CircularInterpolator<Vector3D<> >(vias[0], vias[1], vias[2], duration);
    }




    template <class T>
    class ParabolicBlendParser {
    public:
        static ParabolicBlend<T>* read(xercesc::DOMElement* element, LinearInterpolator<T>* int1, LinearInterpolator<T>* int2) {
            double tau = readAttribute(element, XMLTrajectoryFormat::TauAttributeId);
            return new ParabolicBlend<T>(int1, int2, tau);
        }
    };


    template <class T>
    class LloydHaywardBlendParser {
    public:
        static LloydHaywardBlend<T>* read(xercesc::DOMElement*, Interpolator<T>* int1, Interpolator<T>* int2);
    };


    template<class T> LloydHaywardBlend<T>* LloydHaywardBlendParser<T>::read(xercesc::DOMElement* element, Interpolator<T>* int1, Interpolator<T>* int2) {
        double tau = readAttribute(element, XMLTrajectoryFormat::TauAttributeId);
        double kappa = readAttribute(element, XMLTrajectoryFormat::KappaAttributeId);
        return new LloydHaywardBlend<T>(int1, int2, tau, kappa);
    }

    template <> LloydHaywardBlend<Rotation3D<> >* LloydHaywardBlendParser<Rotation3D<> >::read(xercesc::DOMElement*, Interpolator<Rotation3D<> >* int1, Interpolator<Rotation3D<> >* int2) {
        RW_THROW("Rotation3D is not supported in LloydHaywardBlend");
    }

    class Identifiers {
    public:
        virtual const XMLCh* linearInterpolatorId() = 0;
        virtual const XMLCh* cubicSplineInterpolatorId() = 0;
        virtual const XMLCh* circularInterpolatorId() = 0;

        bool isInterpolator(const XMLCh* str) {
            return XMLString::equals(str, linearInterpolatorId()) ||
                   XMLString::equals(str, cubicSplineInterpolatorId()) ||
                   XMLString::equals(str, circularInterpolatorId());
        }
    };

    class QIdentifiers: public Identifiers {
        virtual const XMLCh* linearInterpolatorId() {
            return XMLTrajectoryFormat::QLinearInterpolatorId;
        }

        virtual const XMLCh* cubicSplineInterpolatorId() {
            return XMLTrajectoryFormat::QCubicSplineInterpolatorId;
        }

        virtual const XMLCh* circularInterpolatorId() {
            return NULL;
        }
    };

    class V3DIdentifiers: public Identifiers {
        virtual const XMLCh* linearInterpolatorId() {
            return XMLTrajectoryFormat::V3DLinearInterpolatorId;
        }

        virtual const XMLCh* cubicSplineInterpolatorId() {
            return XMLTrajectoryFormat::V3DCubicSplineInterpolatorId;
        }
        virtual const XMLCh* circularInterpolatorId() {
            return XMLTrajectoryFormat::V3DCircularInterpolatorId;
        }
    };

    class R3DIdentifiers: public Identifiers {
        virtual const XMLCh* linearInterpolatorId() {
            return XMLTrajectoryFormat::R3DLinearInterpolatorId;
        }

        virtual const XMLCh* cubicSplineInterpolatorId() {
            return XMLTrajectoryFormat::R3DCubicSplineInterpolatorId;
        }

        virtual const XMLCh* circularInterpolatorId() {
            return NULL;
        }
    };

    class T3DIdentifiers: public Identifiers {
        virtual const XMLCh* linearInterpolatorId() {
            return XMLTrajectoryFormat::T3DLinearInterpolatorId;
        }

        virtual const XMLCh* cubicSplineInterpolatorId() {
            return XMLTrajectoryFormat::T3DCubicSplineInterpolatorId;
        }

        virtual const XMLCh* circularInterpolatorId() {
            return NULL;
        }
    };



    template <class T>
    Ptr<InterpolatorTrajectory<T> > read(xercesc::DOMElement* element, Identifiers* ids) {
        double starttime = readStartTime(element);
        Ptr<InterpolatorTrajectory<T> > result = ownedPtr(new InterpolatorTrajectory<T>(starttime));

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();
        std::vector<Interpolator<T>* > interpolators;



        //First we run through and finds the interpolators
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* element = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (element != NULL) {
                if (XMLString::equals(ids->linearInterpolatorId(), element->getNodeName())) {
                    LinearInterpolator<T>* linearinterpolator = LinearInterpolatorParser<T>::read(element);
                    interpolators.push_back(linearinterpolator);
                } else if (XMLString::equals(ids->cubicSplineInterpolatorId(), element->getNodeName())) {
                    //TODO read in QCubicSplineInterpolator
                } else if (XMLString::equals(ids->circularInterpolatorId(), element->getNodeName())) {
                    CircularInterpolator<T>* interpolator = CircularInterpolatorParser<T>::read(element);
                    interpolators.push_back(interpolator);
                }
            }
        }


        //Secondly we run through and finds the blends
        Blend<T>* blend = NULL;
        size_t interpolatorIndex = 0;
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            xercesc::DOMElement* element = dynamic_cast<xercesc::DOMElement*>(children->item(i));
            if (element != NULL) {
                if (ids->isInterpolator(element->getNodeName())) {
                    if (blend == NULL) {
                        result->add(ownedPtr(interpolators[interpolatorIndex]));
                    } else {
                        result->add(ownedPtr(blend), ownedPtr(interpolators[interpolatorIndex]));
                        blend = NULL;
                    }
                    interpolatorIndex++;
                } else if (XMLString::equals(XMLTrajectoryFormat::ParabolicBlendId, element->getNodeName())) {
                    LinearInterpolator<T>* linear1 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex-1]);
                    LinearInterpolator<T>* linear2 = dynamic_cast<LinearInterpolator<T>*>(interpolators[interpolatorIndex]);

                    if (linear1 == NULL || linear2 == NULL)
                        RW_THROW("ParabolicBlends can only be constructed between LinearInterpolator's");

                    blend = ParabolicBlendParser<T>::read(element, linear1, linear2);
                } else if (XMLString::equals(XMLTrajectoryFormat::LloydHaywardBlendId, element->getNodeName())) {
                    Interpolator<T>* int1 = interpolators[interpolatorIndex-1];
                    Interpolator<T>* int2 = interpolators[interpolatorIndex];
                    blend = LloydHaywardBlendParser<T>::read(element, int1, int2);
                }
            }
        }

        return result;
    }


} //end namespace

XMLTrajectoryLoader::Type XMLTrajectoryLoader::getType() {
    return _type;
}

QTrajectory::Ptr XMLTrajectoryLoader::getQTrajectory() {
    if (_type != QType)
        RW_THROW("The loaded Trajectory is not of type Q. Use XMLTrajectoryLoader::getType() to read its type");
    return _qTrajectory;
}

Vector3DTrajectory::Ptr XMLTrajectoryLoader::getVector3DTrajectory() {
    if (_type != Vector3DType)
        RW_THROW("The loaded Trajectory is not of type Vector3D<>. Use XMLTrajectoryLoader::getType() to read its type");

    return _v3dTrajectory;
}

Rotation3DTrajectory::Ptr XMLTrajectoryLoader::getRotation3DTrajectory() {
    if (_type != Rotation3DType)
        RW_THROW("The loaded Trajectory is not of type Rotation3D. Use XMLTrajectoryLoader::getType() to read its type");

    return _r3dTrajectory;
}

Transform3DTrajectory::Ptr XMLTrajectoryLoader::getTransform3DTrajectory() {
    if (_type != Transform3DType)
        RW_THROW("The loaded Trajectory is not of type Transform3D<>. Use XMLTrajectoryLoader::getType() to read its type");

    return _t3dTrajectory;
}


void XMLTrajectoryLoader::readTrajectory(xercesc::DOMElement* element) {
    //Determine which type of trajectory we are using
    if (XMLString::equals(XMLTrajectoryFormat::QTrajectoryId, element->getNodeName())) {
        QIdentifiers ids;
        _qTrajectory = read<Q>(element, &ids);
        _type = QType;
    } else if (XMLString::equals(XMLTrajectoryFormat::V3DTrajectoryId, element->getNodeName())) {
        V3DIdentifiers ids;
        _v3dTrajectory = read<Vector3D<> >(element, &ids);
        _type = Vector3DType;
    } else if (XMLString::equals(XMLTrajectoryFormat::R3DTrajectoryId, element->getNodeName())) {
        R3DIdentifiers ids;
        _r3dTrajectory = read<Rotation3D<> >(element, &ids);
        _type = Rotation3DType;
    } else if (XMLString::equals(XMLTrajectoryFormat::T3DTrajectoryId, element->getNodeName())) {
        T3DIdentifiers ids;
        _t3dTrajectory = read<Transform3D<> >(element, &ids);
        _type = Transform3DType;

    } else {

    }
}

