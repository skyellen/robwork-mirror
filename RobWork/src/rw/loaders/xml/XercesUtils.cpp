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
//#include <xercesc/dom/DOMWriter.hpp>


#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/framework/XMLFormatter.hpp>

#include <xercesc/util/XMLUni.hpp>



#include <iostream>

using namespace xercesc;
using namespace rw::loaders;




xercesc::DOMDocument* XercesDocumentReader::readDocument(XercesDOMParser& parser, const std::string& filename, const std::string& schemaFileName) {
	static XercesInitializer initXerces;
    rw::loaders::XercesErrorHandler errorHandler;

    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    if (schemaFileName.size() != 0)
        parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(xercesc::XercesDOMParser::Val_Auto);

    parser.parse(filename.c_str());
    if (parser.getErrorCount() != 0) {
		std::cerr<<std::endl<<std::endl<<"Error(s) parsing file '"<<filename<<"': "<<std::endl<<XMLStr(errorHandler.getMessages()).str()<<std::endl;
        RW_THROW(""<<parser.getErrorCount()<<"Error(s) parsing "<<filename<<": "<<XMLStr(errorHandler.getMessages()).str());
    }
    return parser.getDocument();
}

xercesc::DOMDocument* XercesDocumentReader::readDocument(XercesDOMParser& parser, std::istream& instream, const std::string& schemaFileName) {
	InputStreamSource isource(instream);

    return readDocument(parser, isource, schemaFileName);
}

xercesc::DOMDocument* XercesDocumentReader::readDocument(XercesDOMParser& parser, const InputSource& source, const std::string& schemaFileName) {
	static XercesInitializer initXerces;
    rw::loaders::XercesErrorHandler errorHandler;

    parser.setDoNamespaces( true );
    parser.setDoSchema( true );
    if (schemaFileName.size() != 0)
        parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


    parser.setErrorHandler(&errorHandler);
    parser.setValidationScheme(xercesc::XercesDOMParser::Val_Auto);

    parser.parse(source);
    if (parser.getErrorCount() != 0) {
        std::cerr<<std::endl<<std::endl<<"Error(s) = "<<std::endl<<XMLStr(errorHandler.getMessages()).str()<<std::endl;
        RW_THROW(""<<parser.getErrorCount()<<" Errors: "<<XMLStr(errorHandler.getMessages()).str());
    }
    return parser.getDocument();
}



xercesc::DOMDocument* XercesDocumentWriter::createDocument(const XMLCh* rootName) {
	static XercesInitializer initXerces;
    xercesc::DOMImplementation* impl =  xercesc::DOMImplementationRegistry::getDOMImplementation(XMLStr("Core").uni());

    if (impl != NULL)
    {
        try
        {
            xercesc::DOMDocument* doc = impl->createDocument(0,                    // root element namespace URI.
                                                             rootName,         // root element name
                                                             0);                   // We do not wish to specify a document type
            return doc;
        }
        catch (const xercesc::OutOfMemoryException&)
        {
            RW_THROW("XMLPathWriter: OutOfMemory");
        }
        catch (const xercesc::DOMException& e)
        {
            RW_THROW("XMLPathWriter: DOMException code:  " << XMLStr(e.getMessage()).str());
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


}


void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, std::ostream& out) {
    OutStreamFormatTarget target(out);
    writeDocument(doc, &target);

}


void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, const std::string& filename) {
    if (filename.size() != 0) {
        try {
            LocalFileFormatTarget target(filename.c_str());
            writeDocument(doc, &target);
        } catch (std::exception& exp) {
            RW_THROW("Unable open output file: "<<exp.what());
        }

    } else {
        RW_THROW("No filename specified");
    }

}



#if XERCES_VERSION_MAJOR == 3


void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, XMLFormatTarget* formatTarget) {
	static XercesInitializer initXerces;
      //int retval;
      {

          try
          {
              // get a serializer, an instance of DOMLSSerializer
              XMLCh tempStr[3] = {chLatin_L, chLatin_S, chNull};
              DOMImplementation *impl          = DOMImplementationRegistry::getDOMImplementation(tempStr);
              DOMLSSerializer   *theSerializer = ((DOMImplementationLS*)impl)->createLSSerializer();
              DOMLSOutput       *theOutputDesc = ((DOMImplementationLS*)impl)->createLSOutput();

              // set user specified output encoding
              theOutputDesc->setEncoding(0);



              DOMConfiguration* serializerConfig=theSerializer->getDomConfig();

              // plug in user's own error handler
              XercesErrorHandler errorHandler;
              serializerConfig->setParameter(XMLUni::fgDOMErrorHandler, &errorHandler);

              // set feature if the serializer supports the feature/mode
              if (serializerConfig->canSetParameter(XMLUni::fgDOMWRTSplitCdataSections, true))
                  serializerConfig->setParameter(XMLUni::fgDOMWRTSplitCdataSections, true);

              if (serializerConfig->canSetParameter(XMLUni::fgDOMWRTDiscardDefaultContent, true))
                  serializerConfig->setParameter(XMLUni::fgDOMWRTDiscardDefaultContent, true);

              if (serializerConfig->canSetParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true))
                  serializerConfig->setParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true);

              if (serializerConfig->canSetParameter(XMLUni::fgDOMWRTBOM, false))
                  serializerConfig->setParameter(XMLUni::fgDOMWRTBOM, false);

              theOutputDesc->setByteStream(formatTarget);


              theSerializer->write(doc, theOutputDesc);

              theOutputDesc->release();
              theSerializer->release();

          }
          catch (const OutOfMemoryException&)
          {
              XERCES_STD_QUALIFIER cerr << "OutOfMemoryException" << XERCES_STD_QUALIFIER endl;
              //retval = 5;
          }
          catch (XMLException& e)
          {
              XERCES_STD_QUALIFIER cerr << "An error occurred during creation of output transcoder. Msg is:"
                  << XERCES_STD_QUALIFIER endl
                  << XMLStr(e.getMessage()).str() << XERCES_STD_QUALIFIER endl;
              //retval = 4;
          }

      }
      return;

}


#else



//void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, const std::string& filename) {
void XercesDocumentWriter::writeDocument(xercesc::DOMDocument* doc, XMLFormatTarget* formatTarget) {
    // get a serializer, an instance of DOMWriter
     XMLCh buf[100];
     xercesc::XMLString::transcode("LS", buf, 99);

     //Pointer to DOMImplementation in the registry. The pointer should therefore not be deleted
     xercesc::DOMImplementation *impl          = xercesc::DOMImplementationRegistry::getDOMImplementation(buf);
     xercesc::DOMWriter* theSerializer = ((xercesc::DOMImplementationLS*)impl)->createDOMWriter();


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
     //LocalFileFormatTarget target(filename.c_str());


     bool res = theSerializer->writeNode(formatTarget, *doc);
     delete theSerializer; //We can now delete the serializer

     if (errorHandler.getErrorCount() != 0)
         RW_THROW("Error while trying to write XML. Error Messages: "<<errorHandler.getMessages());

     if (errorHandler.getWarningCount() != 0)
         RW_WARN("Warnings received while writing XML, Warning Messages: "<<errorHandler.getMessages());


     if (!res)
         RW_THROW("Unable to serialize XML");

};
#endif
