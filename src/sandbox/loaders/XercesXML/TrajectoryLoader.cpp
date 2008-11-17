/*
 * TrajectoryLoader.cpp
 *
 *  Created on: Nov 12, 2008
 *      Author: lpe
 */

#include "TrajectoryLoader.hpp"

#include <iostream>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
#include <xercesc/util/XMLDouble.hpp>

#include <xercesc/validators/common/Grammar.hpp>
#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>


#include <rw/math/Q.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

#include "XercesErrorHandler.hpp"
#include "XercesXMLMathUtils.hpp"

using namespace xercesc;
using namespace rw::math;
using namespace rw::trajectory;

TrajectoryLoader::TrajectoryLoader(const std::string& filename)
{
    try
    {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
       char* message = XMLString::transcode( e.getMessage() );
       std::cerr << "XML toolkit initialization error: " << message << std::endl;
       XMLString::release( &message );
       // throw exception here to return ERROR_XERCES_INIT
    }

    XercesDOMParser parser;

    XercesErrorHandler errorHandler;


  parser.setDoNamespaces( true );
    parser.setDoSchema( true );
/*    parser.setExternalNoNamespaceSchemaLocation("rwxml_collisionsetup.xsd");
    */


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(XercesDOMParser::Val_Auto);

    parser.parse(filename.c_str() );
    std::cout<<"Error Count = "<<parser.getErrorCount()<<std::endl;
    std::cout<<"Error Handler = "<<parser.getErrorHandler()<<std::endl;

    // no need to free this pointer - owned by the parent parser object
    DOMDocument* xmlDoc = parser.getDocument();

    // Get the top-level element: NAme is "root". No attributes for "root"

    DOMElement* elementRoot = xmlDoc->getDocumentElement();



    readTrajectory(elementRoot);


}

TrajectoryLoader::~TrajectoryLoader()
{
    // TODO Auto-generated destructor stub
}


namespace {

    bool initializeXerces() {
        try
        {
           XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        }
        catch( XMLException& e )
        {
           char* message = XMLString::transcode( e.getMessage() );
           std::cerr << "XML toolkit initialization error: " << message << std::endl;
           XMLString::release( &message );
           // throw exception here to return ERROR_XERCES_INIT
        }

        return true;
    }

    //Small trick to make sure Xerces is initialized before we start using XMLString::transcode
    const bool initialized = initializeXerces();
    const XMLCh* QTRAJECTORY_ID = XMLString::transcode("QTrajectory");
    const XMLCh* V3DTRAJECTORY_ID = XMLString::transcode("V3DTrajectory");

    const XMLCh* R3DTRAJECTORY_ID = XMLString::transcode("R3DTrajectory");
    const XMLCh* T3DTRAJECTORY_ID = XMLString::transcode("T3DTrajectory");

    const XMLCh* QLINEARINTERPOLATOR_ID = XMLString::transcode("QLinearInterpolator");
    const XMLCh* QCUBICSPLINEINTERPOLATOR_ID = XMLString::transcode("QCubicSplineInterpolator");

    const XMLCh* DURATION_ATTRIBUTE_ID = XMLString::transcode("duration");

    //template <typename T>

    double readDuration(DOMElement* element) {
        if (element->hasAttribute(DURATION_ATTRIBUTE_ID)) {
            const XMLCh* attr = element->getAttribute(DURATION_ATTRIBUTE_ID);
            std::string strAttr = XMLString::transcode(attr);
            XMLDouble xmlfloat(attr);
            return xmlfloat.getValue();
        }
        return 1;
    }

    double readTau(DOMElement* element) {
        if (element->hasAttribute(TAU_ATTRIBUTE_ID)) {
            const XMLCh* attr = element->getAttribute(TAU_ATTRIBUTE_ID);
            std::string strAttr = XMLString::transcode(attr);
            XMLDouble xmlfloat(attr);
            return xmlfloat.getValue();
        }
        RW_THROW("Unable to find attribute: \""<<XMLString::transcode(TAU_ATTRIBUTE_ID)<<"\"");
    }

    template <class T>
    class LinearInterpolatorParser {
    public:
        virtual T readElement(DOMElement* element);

        LinearInterpolator<T>* readLinearInterpolator(DOMElement* element) {
            std::cout<<"ReadLinearInterpolator"<<std::endl;
            double duration = readDuration(element);
            std::cout<<"Duration = "<<duration<<std::endl;
            DOMNodeList* children = element->getChildNodes();
            const  XMLSize_t nodeCount = children->getLength();
            std::cout<<"NodeCount = "<<nodeCount<<std::endl;
            T vias[2];
            int index = 0;
            for(XMLSize_t i = 0; i < nodeCount; ++i) {
                DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
                if (child != NULL) {
                    std::cout<<"Read Element "<<XMLString::transcode(child->getNodeName())<<std::endl;
                    T tmp = readElement(child);
                    std::cout<<"Element = "<<tmp<<std::endl;
                    vias[index] = tmp;
                    ++index;
                }
            }

            return new LinearInterpolator<T>(vias[0], vias[1], duration);
        }
    };

    template<> Q LinearInterpolatorParser<Q>::readElement(DOMElement* element) {
        std::cout<<"Read Q"<<std::endl;
        return XercesXMLMathUtils::readQ(element, false);
    }

    template<> Vector3D<> LinearInterpolatorParser<Vector3D<> >::readElement(DOMElement* element) {
        return XercesXMLMathUtils::readVector3D(element, false);
    }


    template <class T>
    class ParabolicBlendParser {
    public:
        ParabolicBlend<T> readParabolicBlend(DOMElement* element) {
            std::cout<<"Read ParabolicBlend"<<std::endl;
            double tau = readTau(element);
            std::cout<<"Tau = "<<tau<<std::endl;



        }
    };



Trajectory<Q> readQTrajectory(DOMElement* element) {
    Trajectory<Q> result;

    DOMNodeList* children = element->getChildNodes();
    const  XMLSize_t nodeCount = children->getLength();
    std::vector<Intepolator<Q>* > interpolators;



    //First we run through and finds the interpolators
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        DOMElement* element = dynamic_cast<DOMElement*>(children->item(i));
        if (element != NULL) {
            if (XMLString::equals(QLINEARINTERPOLATOR_ID, element->getNodeName())) {
                //TODO read in QLinearInterpolator
                LinearInterpolatorParser<Q> reader;
                LinearInterpolator<Q>* linearinterpolator = reader.readLinearInterpolator(element);
                interpolators.push_back(linearinterpolator);
            } else if (XMLString::equals(QCUBICSPLINEINTERPOLATOR_ID, element->getNodeName())) {
                //TODO read in QCubicSplineInterpolator
            } else if (XMLString::equals(PARABOLICBLEND_ID, element->getNodeName())) {

                std::cout<<"Type "<<XMLString::transcode(element->getNodeName())<<" Not Supported yet"<<std::endl;
            }
        }
    }

    size_t interpolatorIndex = 0;
    for(XMLSize_t i = 0; i < nodeCount; ++i ) {
        DOMElement* element = dynamic_cast<DOMElement*>(children->item(i));
        if (element != NULL) {
            if (XMLString::equals(QLINEARINTERPOLATOR_ID, element->getNodeName()) ||
                XMLString::equals(QCUBICSPLINEINTERPOLATOR_ID, element->getNodeName())) {
                interpolatorIndex++;
            } else if (XMLString::equals(PARABOLICBLEND_ID, element->getNodeName())) {
                LinearInterpolator<Q> linear1 = dynamic_cast<LinearInterpolator<Q>*>(interpolators[interpolatorIndex-1]);
                LinearInterpolator<Q> linear2 = dynamic_cast<LinearInterpolator<Q>*>(interpolators[interpolatorIndex]);
                double tau = readTau(element);

                std::cout<<"Type "<<XMLString::transcode(element->getNodeName())<<" Not Supported yet"<<std::endl;
            }
        }
    }

    //Secondly we run through and finds the blends
    return result;
}



} //end namespace

rw::trajectory::Trajectory<Q> TrajectoryLoader::getQTrajectory() {
    return _qTrajectory;
}


void TrajectoryLoader::readTrajectory(DOMElement* element) {
    //TODO Determine which type of trajectory we are using
    if (XMLString::equals(QTRAJECTORY_ID, element->getNodeName())) {
        _qTrajectory = readQTrajectory(element);
    } else if (XMLString::equals(V3DTRAJECTORY_ID, element->getNodeName())) {

    } else if (XMLString::equals(R3DTRAJECTORY_ID, element->getNodeName())) {

    } else if (XMLString::equals(T3DTRAJECTORY_ID, element->getNodeName())) {
    } else {
        std::cout<<"no match"<<std::endl;
    }

}

