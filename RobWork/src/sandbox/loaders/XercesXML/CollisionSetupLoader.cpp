/*
 * CollisionSetupLoader.cpp
 *
 *  Created on: Nov 5, 2008
 *      Author: lpe
 */

#include <xercesc/validators/common/Grammar.hpp>

#include "CollisionSetupLoader.hpp"
#include <rw/loaders/xml/XercesErrorHandler.hpp>
using namespace rw::proximity;

using namespace sandbox;
using namespace xercesc;


namespace {
    ProximityPair readFramePair(DOMElement* element) {
    std::cout<<"Read Frame Pair"<<std::endl;
        const XMLCh* FIRST = XMLString::transcode("first");
        const XMLCh* SECOND = XMLString::transcode("second");
        const XMLCh* frame1 = element->getAttribute(FIRST);
        const XMLCh* frame2 = element->getAttribute(SECOND);
        ProximityPair result(XMLString::transcode(frame1), XMLString::transcode(frame2));
        return result;
    }

    void readFramePairList(DOMElement* element, ProximityPairList& result) {

        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();

        for( XMLSize_t xx = 0; xx < nodeCount; ++xx )
        {
            DOMElement* element = dynamic_cast<DOMElement*>(children->item(xx));
            if (element != NULL) {
                std::string name = XMLString::transcode(element->getNodeName());
                std::cout<<"Element = "<<name<<std::endl;
                if (name == "FramePair")
                    result.push_back(readFramePair(element));
            }
        }
    }

    ProximityPairList readCollisionSetup(DOMElement* element) {
        ProximityPairList result;
        std::string str = XMLString::transcode(element->getNodeName());
        if (str != "CollisionSetup")
            RW_THROW("Not a CollisionSetup file");


        DOMNodeList* children = element->getChildNodes();
        const  XMLSize_t nodeCount = children->getLength();

        // For all nodes, children of "root" in the XML tree.

        for( XMLSize_t xx = 0; xx < nodeCount; ++xx )
        {
            DOMElement* element = dynamic_cast<DOMElement*>(children->item(xx));
            if (element != NULL) {
                std::string name = XMLString::transcode(element->getNodeName());
                if (name == "Include") {
                    readFramePairList(element, result);
                } else if (name == "Exclude")
                    readFramePairList(element, result);
            }
        }
        return result;
    }





}


ProximityPairList CollisionSetupLoader::load(const std::string& filename) {
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

    rw::loaders::XercesErrorHandler errorHandler;
/*
    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    parser.setExternalNoNamespaceSchemaLocation("rwxml_collisionsetup.xsd");
    */
    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(XercesDOMParser::Val_Auto);

    parser.parse(filename.c_str() );

    // no need to free this pointer - owned by the parent parser object
    DOMDocument* xmlDoc = parser.getDocument();

	if (xmlDoc == NULL) {
		RW_THROW("Unable to open and parse file: '"<<filename<<"'");
	}
    // Get the top-level element: NAme is "root". No attributes for "root"

    DOMElement* elementRoot = xmlDoc->getDocumentElement();
    return readCollisionSetup(elementRoot);


}
