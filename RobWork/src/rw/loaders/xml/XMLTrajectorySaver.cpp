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

/*
 * XMLTrajectorySaver.cpp
 *
 *  Created on: Nov 28, 2008
 *      Author: lpe
 */

#include "XMLTrajectorySaver.hpp"
#include "XMLTrajectoryFormat.hpp"
#include "XMLBasisTypes.hpp"

#include <rw/trajectory/InterpolatorTrajectory.hpp>
#include <rw/trajectory/Interpolator.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/CubicSplineInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

#include <xercesc/dom/DOMImplementationLS.hpp>
//#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>

using namespace xercesc;
using namespace rw::common;
using namespace rw::math;
using namespace rw::loaders;
using namespace rw::trajectory;

XMLTrajectorySaver::Initializer::Initializer() {
	static bool done = false;
	if (!done) {
		XMLBasisTypes::Initializer init1;
		XMLTrajectoryFormat::Initializer init2;
		done = true;
	}
}

const XMLTrajectorySaver::Initializer XMLTrajectorySaver::initializer;

namespace {

    template <class T>
    class ElementCreator {
    public:
        static xercesc::DOMElement* createElement(const T& element, xercesc::DOMDocument* doc);
    };

    template<> xercesc::DOMElement* ElementCreator<Q>::createElement(const Q& element, xercesc::DOMDocument* doc) {
        return XMLBasisTypes::createQ(element, doc);
    }

    template<> xercesc::DOMElement* ElementCreator<Vector3D<> >::createElement(const Vector3D<>& element, xercesc::DOMDocument* doc) {
        return XMLBasisTypes::createVector3D(element, doc);
    }

    template<> xercesc::DOMElement* ElementCreator<Rotation3D<> >::createElement(const Rotation3D<>& element, xercesc::DOMDocument* doc) {
        return XMLBasisTypes::createRotation3D(element, doc);
    }

    template<> xercesc::DOMElement* ElementCreator<Transform3D<> >::createElement(const Transform3D<>& element, xercesc::DOMDocument* doc) {
        return XMLBasisTypes::createTransform3D(element, doc);
    }



    template <class T>
    class Identifiers {
     public:
         static const XMLCh* linearInterpolatorId();
         static const XMLCh* cubicSplineInterpolatorId();
         static const XMLCh* circularInterpolatorId();

     };

    template<> const XMLCh* Identifiers<Q>::linearInterpolatorId() {
        return XMLTrajectoryFormat::idQLinearInterpolator();
    }


    template<> const XMLCh* Identifiers<Q>::cubicSplineInterpolatorId() {
        return XMLTrajectoryFormat::idQCubicSplineInterpolator();
    }

    template<> const XMLCh* Identifiers<Q>::circularInterpolatorId() {
        return NULL;
    }



    template<> const XMLCh* Identifiers<Vector3D<> >::linearInterpolatorId() {
        return XMLTrajectoryFormat::idV3DLinearInterpolator();
    }

    template<> const XMLCh* Identifiers<Vector3D<> >::cubicSplineInterpolatorId() {
        return XMLTrajectoryFormat::idV3DCubicSplineInterpolator();
    }

    template<> const XMLCh* Identifiers<Vector3D<> >::circularInterpolatorId() {
        return XMLTrajectoryFormat::idV3DCircularInterpolator();
    }


    template<> const XMLCh* Identifiers<Rotation3D<> >::linearInterpolatorId() {
        return XMLTrajectoryFormat::idR3DLinearInterpolator();
    }


    template<> const XMLCh* Identifiers<Rotation3D<> >::cubicSplineInterpolatorId() {
        return XMLTrajectoryFormat::idR3DCubicSplineInterpolator();
    }

    template<> const XMLCh* Identifiers<Rotation3D<> >::circularInterpolatorId() {
        return NULL;
    }



    template<> const XMLCh* Identifiers<Transform3D<> >::linearInterpolatorId() {
        return XMLTrajectoryFormat::idT3DLinearInterpolator();
    }


    template<> const XMLCh* Identifiers<Transform3D<> >::cubicSplineInterpolatorId() {
        return XMLTrajectoryFormat::idT3DCubicSplineInterpolator();
    }

    template<> const XMLCh* Identifiers<Transform3D<> >::circularInterpolatorId() {
        return NULL;
    }



    template <class T>
    xercesc::DOMElement* writeInterpolator(const Ptr<Interpolator<T> > interpolator, xercesc::DOMDocument* doc) {
        LinearInterpolator<T>* linear = dynamic_cast<LinearInterpolator<T>*>(interpolator.get());
        if (linear != NULL) {
            T start = linear->getStart();
            T end = linear->getEnd();
            double duration = linear->duration();

            xercesc::DOMElement* element = doc->createElement(Identifiers<T>::linearInterpolatorId());
            DOMAttr* durationAttr = doc->createAttribute(XMLTrajectoryFormat::idDurationAttribute());
            durationAttr->setValue(XMLStr(duration).uni());
            element->setAttributeNode(durationAttr);
            xercesc::DOMElement* startElement = ElementCreator<T>::createElement(start, doc);
            element->appendChild(startElement);
            xercesc::DOMElement* endElement = ElementCreator<T>::createElement(end, doc);
            element->appendChild(endElement);
            return element;
        }

         CubicSplineInterpolator<T>* cspline = dynamic_cast<CubicSplineInterpolator<T>*>(interpolator.get());
         if (cspline != NULL) {
             //TODO Once implemented
             RW_THROW("Interpolator not supported by XMLTrajectorySaver");
         }

         CircularInterpolator<T>* circular = dynamic_cast<CircularInterpolator<T>*>(interpolator.get());
         if (circular != NULL) {
             T p1 = circular->getP1();
             T p2 = circular->getP2();
             T p3 = circular->getP3();
             double duration = circular->duration();

             xercesc::DOMElement* element = doc->createElement(Identifiers<T>::circularInterpolatorId());
             DOMAttr* durationAttr = doc->createAttribute(XMLTrajectoryFormat::idDurationAttribute());
             durationAttr->setValue(XMLStr(duration).uni());
             element->setAttributeNode(durationAttr);

             xercesc::DOMElement* element1 = ElementCreator<T>::createElement(p1, doc);
             element->appendChild(element1);
             xercesc::DOMElement* element2 = ElementCreator<T>::createElement(p2, doc);
             element->appendChild(element2);
             xercesc::DOMElement* element3 = ElementCreator<T>::createElement(p3, doc);
             element->appendChild(element3);
             return element;
         }
        RW_THROW("The Trajectory contains an interpolator not supported by XMLTrajectorySaver");

    };


    template <class T>
    xercesc::DOMElement* writeBlend(const Ptr<Blend<T> >& blend, xercesc::DOMDocument* doc) {
        const ParabolicBlend<T>* parabolic = dynamic_cast<const ParabolicBlend<T>*>(blend.get());
        if (parabolic != NULL) {
            double tau = parabolic->tau1();
            xercesc::DOMElement* element = doc->createElement(XMLTrajectoryFormat::idParabolicBlend());
            DOMAttr* tauAttr = doc->createAttribute(XMLTrajectoryFormat::idTauAttribute());
            tauAttr->setValue(XMLStr(tau).uni());
            element->setAttributeNode(tauAttr);
            return element;
        }

        const LloydHaywardBlend<T>* lloydHayward = dynamic_cast<const LloydHaywardBlend<T>*>(blend.get());
        if (lloydHayward != NULL) {
            double tau = lloydHayward->tau1();
            double kappa = lloydHayward->kappa();
            xercesc::DOMElement* element = doc->createElement(XMLTrajectoryFormat::idLloydHaywardBlend());
            DOMAttr* tauAttr = doc->createAttribute(XMLTrajectoryFormat::idTauAttribute());
            tauAttr->setValue(XMLStr(tau).uni());
            element->setAttributeNode(tauAttr);
            DOMAttr* kappaAttr = doc->createAttribute(XMLTrajectoryFormat::idKappaAttribute());
            kappaAttr->setValue(XMLStr(kappa).uni());
            element->setAttributeNode(kappaAttr);
            return element;
        }
        RW_THROW("The Trajectory contains a blend not supported by XMLTrajectorySaver");
    }


    template <class T, class TRAJ>
    xercesc::DOMDocument* createDOMDocument(TRAJ& trajectory, const XMLCh* trajectoryId) {
        XMLCh* features = XMLString::transcode("Core");
        DOMImplementation* impl =  DOMImplementationRegistry::getDOMImplementation(features);
        XMLString::release(&features);

        xercesc::DOMDocument* doc = NULL;
        if (impl != NULL)
        {
            try
            {
                doc = impl->createDocument(0,                    // root element namespace URI.
                                                        trajectoryId,         // root element name
                                                        0);                   // We do not wish to specify a document type

                xercesc::DOMElement* root = doc->getDocumentElement();

                typedef const InterpolatorTrajectory<T> Traj;
                Traj* traj = dynamic_cast<Traj*>(&trajectory);
                if (traj == NULL) {
                    doc->release();
                    RW_THROW("Unable to save trajectory which is not a InterpolatorTrajectory");
                }

                for (size_t i = 0; i<traj->getSegmentsCount(); i++) {
                    typedef std::pair<const Ptr<Blend<T> >, const Ptr<Interpolator<T> > > Segment;
                    Segment segment = traj->getSegment(i);

                    if (segment.first != NULL) {
                        xercesc::DOMElement* blendElement = writeBlend(segment.first, doc);
                        root->appendChild(blendElement);
                    }
                    xercesc::DOMElement* element = writeInterpolator(segment.second, doc);
                    root->appendChild(element);
                }




            }
            catch (const OutOfMemoryException&)
            {
                RW_THROW("XMLPathWriter: OutOfMemory");
            }
            catch (const DOMException& e)
            {
                RW_THROW("XMLPathWriter: DOMException:  " << XMLStr(e.getMessage()).str());
            }
            catch (const rw::common::Exception& exp) {
                throw exp;
            }
            catch (...)
            {
                RW_THROW("XMLPathWriter: Unknown Exception while creating saving path");
            }
        }
        else
        {
            RW_THROW("XMLPathWriter: Unable to find a suitable DOM Implementation");
        }
        return doc;

    }


    template <class T, class TRAJ>
    bool saveTrajectoryImpl(TRAJ& trajectory, const XMLCh* trajectoryId, const std::string& filename) {
        xercesc::DOMDocument* doc = createDOMDocument<T, TRAJ>(trajectory, trajectoryId);
        if (doc == NULL)
            return false;
        XercesDocumentWriter::writeDocument(doc, filename);
        doc->release();
        return true;
    }

    template <class T, class TRAJ>
    bool saveTrajectoryImpl(TRAJ& trajectory, const XMLCh* trajectoryId, std::ostream& outstream) {
        xercesc::DOMDocument* doc = createDOMDocument<T, TRAJ>(trajectory, trajectoryId);
        if (doc == NULL)
            return false;
        XercesDocumentWriter::writeDocument(doc, outstream);
        doc->release();
        return true;
    }
} //end cpp file's namespace


bool XMLTrajectorySaver::save(const rw::trajectory::QTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Q, const QTrajectory>(trajectory, XMLTrajectoryFormat::idQTrajectory(), filename);
}


bool XMLTrajectorySaver::save(const rw::trajectory::Vector3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Vector3D<>, const Vector3DTrajectory>(trajectory, XMLTrajectoryFormat::idV3DTrajectory(), filename);
}

bool XMLTrajectorySaver::save(const rw::trajectory::Rotation3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Rotation3D<>, const Rotation3DTrajectory>(trajectory, XMLTrajectoryFormat::idR3DTrajectory(), filename);
}

bool XMLTrajectorySaver::save(const rw::trajectory::Transform3DTrajectory& trajectory, const std::string& filename) {
    return saveTrajectoryImpl<Transform3D<>, const Transform3DTrajectory>(trajectory, XMLTrajectoryFormat::idT3DTrajectory(), filename);
}



bool XMLTrajectorySaver::write(const rw::trajectory::QTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Q, const QTrajectory>(trajectory, XMLTrajectoryFormat::idQTrajectory(), outstream);
}

bool XMLTrajectorySaver::write(const rw::trajectory::Vector3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Vector3D<>, const Vector3DTrajectory>(trajectory, XMLTrajectoryFormat::idV3DTrajectory(), outstream);
}

bool XMLTrajectorySaver::write(const rw::trajectory::Rotation3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Rotation3D<>, const Rotation3DTrajectory>(trajectory, XMLTrajectoryFormat::idR3DTrajectory(), outstream);
}

bool XMLTrajectorySaver::write(const rw::trajectory::Transform3DTrajectory& trajectory, std::ostream& outstream) {
    return saveTrajectoryImpl<Transform3D<>, const Transform3DTrajectory>(trajectory, XMLTrajectoryFormat::idT3DTrajectory(), outstream);
}
