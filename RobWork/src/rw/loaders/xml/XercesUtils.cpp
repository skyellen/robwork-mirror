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

#include "XercesUtils.hpp"
#include "XercesErrorHandler.hpp"

#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>

using namespace xercesc;
using namespace rw::loaders;




void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, const std::string& filename) {
    // get a serializer, an instance of DOMWriter
     XMLCh buf[100];
     xercesc::XMLString::transcode("LS", buf, 99);

     //Pointer to DOMImplementation in the registry. The pointer should therefore not be deleted
     xercesc::DOMImplementation *impl          = xercesc::DOMImplementationRegistry::getDOMImplementation(buf);
     xercesc::DOMWriter         *theSerializer = ((xercesc::DOMImplementationLS*)impl)->createDOMWriter();


     // Setup our error handler
     XercesErrorHandler errorHandler;
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

     if (errorHandler.getErrorCount() != 0)
         RW_THROW("Error while trying to write XML to file \""<<filename<<"\" Error Messages: "<<errorHandler.getMessages());

     if (errorHandler.getWarningCount() != 0)
         RW_WARN("Warnings received while writing XML to file \""<<filename<<"\" Warning Messages: "<<errorHandler.getMessages());


     if (!res)
         RW_THROW("Unable to serialize XML to file \""<<filename<<"\"");

};
