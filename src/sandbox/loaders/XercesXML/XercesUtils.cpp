/*
 * XercesUtils.cpp
 *
 *  Created on: Nov 28, 2008
 *      Author: lpe
 */

#include "XercesUtils.hpp"


#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>

using namespace xercesc;

bool XercesDocumentWriter::WriteErrorHandler::handleError (const xercesc::DOMError &domError) {
    // Display whatever error message passed from the serializer
    if (domError.getSeverity() == xercesc::DOMError::DOM_SEVERITY_WARNING)
        std::cerr << "\nWarning Message: ";
    else if (domError.getSeverity() == xercesc::DOMError::DOM_SEVERITY_ERROR)
        std::cerr << "\nError Message: ";
    else
        std::cerr << "\nFatal Message: ";

    char *msg = xercesc::XMLString::transcode(domError.getMessage());
    std::cerr<< msg <<std::endl;
    xercesc::XMLString::release(&msg);

    // Instructs the serializer to continue serialization if possible.
    return true;
}

void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, const std::string& filename) {
    // get a serializer, an instance of DOMWriter
     XMLCh buf[100];
     xercesc::XMLString::transcode("LS", buf, 99);

     //Pointer to DOMImplementation in the registry. The pointer should therefore not be deleted
     xercesc::DOMImplementation *impl          = xercesc::DOMImplementationRegistry::getDOMImplementation(buf);
     xercesc::DOMWriter         *theSerializer = ((xercesc::DOMImplementationLS*)impl)->createDOMWriter();


     // Setup our error handler
     WriteErrorHandler errorHandler;
     theSerializer->setErrorHandler(&errorHandler);


     // set feature if the serializer supports the feature/mode
     //if (theSerializer->canSetFeature(XMLUni::fgDOMWRTSplitCdataSections, true))
     //    theSerializer->setFeature(XMLUni::fgDOMWRTSplitCdataSections, true);

     if (theSerializer->canSetFeature(xercesc::XMLUni::fgDOMWRTDiscardDefaultContent, true)) //Do not try to use any default stuff. We will write it ourselves
         theSerializer->setFeature(xercesc::XMLUni::fgDOMWRTDiscardDefaultContent, true);

     if (theSerializer->canSetFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true)) //Do pretty printing
         theSerializer->setFeature(XMLUni::fgDOMWRTFormatPrettyPrint, true);

     if (theSerializer->canSetFeature(XMLUni::fgDOMWRTBOM, false)) //Do not use Byte-Order-Mark
         theSerializer->setFeature(XMLUni::fgDOMWRTBOM, false);

     //Setup target to write to
     LocalFileFormatTarget target(filename.c_str());


     bool res = theSerializer->writeNode(&target, *doc);
     delete theSerializer; //We can now delete the serializer
     if (!res) {
         RW_THROW("Unable to serialize to file");
     }
};
