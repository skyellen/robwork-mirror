#include "XMLPathLoader.hpp"

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
#include <rw/trajectory/CircularInterpolator.hpp>
#include <rw/trajectory/ParabolicBlend.hpp>
#include <rw/trajectory/LloydHaywardBlend.hpp>

#include "XercesErrorHandler.hpp"
#include "XMLMathUtils.hpp"
#include "XMLPathFormat.hpp"

using namespace xercesc;
using namespace rw::math;
using namespace rw::common;
using namespace rw::trajectory;
using namespace rw::loaders;

XMLPathLoader::XMLPathLoader(const std::string& filename, const std::string& schemaFileName)
{
    try
    {
       XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
    }
    catch( XMLException& e )
    {
       RW_THROW("Xerces initialization Error"<<XMLStr(e.getMessage()).str());
    }

    XercesDOMParser parser;

    XercesErrorHandler errorHandler;

    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    if (schemaFileName.size() != 0)
        parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(XercesDOMParser::Val_Auto);

    parser.parse(filename.c_str() );
    if (parser.getErrorCount() != 0) {
        std::cerr<<std::endl<<std::endl<<"Error(s) = "<<std::endl<<errorHandler.getMessages()<<std::endl;
        RW_THROW(""<<parser.getErrorCount()<<" Errors: "<<errorHandler.getMessages());
    }


    // no need to free this pointer - owned by the parent parser object
    DOMDocument* xmlDoc = parser.getDocument();

    // Get the top-level element: NAme is "root". No attributes for "root"

    DOMElement* elementRoot = xmlDoc->getDocumentElement();


    readTrajectory(elementRoot);
}

XMLPathLoader::XMLPathLoader(DOMElement* element) {
    readTrajectory(element);
}

XMLPathLoader::~XMLPathLoader()
{
}


namespace {


    template <class T>
    class ElementReader {
    public:
        static T readElement(DOMElement* element);
    };

    template<> Q ElementReader<Q>::readElement(DOMElement* element) {
        return XMLMathUtils::readQ(element, true);
    }

    template<> Vector3D<> ElementReader<Vector3D<> >::readElement(DOMElement* element) {
        return XMLMathUtils::readVector3D(element, true);
    }

    template<> Rotation3D<> ElementReader<Rotation3D<> >::readElement(DOMElement* element) {
        return XMLMathUtils::readRotation3DStructure(element);
    }


    template<> Transform3D<> ElementReader<Transform3D<> >::readElement(DOMElement* element) {
        return XMLMathUtils::readTransform3D(element, true);
    }



    template <class T, class R>
    void read(DOMElement* element,R result) {

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();

        //Run through all elements and read in content
        for(XMLSize_t i = 0; i < nodeCount; ++i ) {
            DOMElement* child = dynamic_cast<DOMElement*>(children->item(i));
            if (child != NULL) {
                T val = ElementReader<T>::readElement(child);
                result->push_back(val);
            }
        }
    }


} //end namespace

XMLPathLoader::Type XMLPathLoader::getType() {
    return _type;
}


QPathPtr XMLPathLoader::getQPath() {
    if (_type != QType)
        RW_THROW("The loaded Path is not of type QPath. Use XMLPathLoader::getType() to read its type");
    return _qPath;
}



Vector3DPathPtr XMLPathLoader::getVector3DPath() {
    if (_type != Vector3DType)
        RW_THROW("The loaded Path is not of type Vector3DPath. Use XMLPathLoader::getType() to read its type");
    return _v3dPath;
}

Rotation3DPathPtr XMLPathLoader::getRotation3DPath() {
    if (_type != Rotation3DType)
        RW_THROW("The loaded Path is not of type Rotation3DPath. Use XMLPathLoader::getType() to read its type");
    return _r3dPath;
}


Transform3DPathPtr XMLPathLoader::getTransform3DPath()
{
    if (_type != Transform3DType)
        RW_THROW("The loaded Path is not of type Transform3DPath. Use XMLPathLoader::getType() to read its type");
    return _t3dPath;
}

void XMLPathLoader::readTrajectory(DOMElement* element) {
    if (XMLString::equals(XMLPathFormat::QPathId, element->getNodeName())) {
        _qPath = ownedPtr(new QPath());
        read<Q, QPathPtr>(element, _qPath);
        _type = QType;
    } else if (XMLString::equals(XMLPathFormat::V3DPathId, element->getNodeName())) {
        _v3dPath = ownedPtr(new Vector3DPath());
        read<Vector3D<>, Vector3DPathPtr>(element, _v3dPath);
        _type = Vector3DType;
    } else if (XMLString::equals(XMLPathFormat::R3DPathId, element->getNodeName())) {
        _r3dPath = ownedPtr(new Rotation3DPath());
        read<Rotation3D<>, Rotation3DPathPtr >(element, _r3dPath);
        _type = Rotation3DType;
    } else if (XMLString::equals(XMLPathFormat::T3DPathId, element->getNodeName())) {
        _t3dPath = ownedPtr(new Transform3DPath());
        read<Transform3D<>, Transform3DPathPtr>(element, _t3dPath);
        _type = Transform3DType;

    } else {
        //The element is not one we are going to parse.
    }
}

