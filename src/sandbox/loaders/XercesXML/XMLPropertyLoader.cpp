#include "XMLPropertyLoader.hpp"

using namespace rw::common;
using namespace rw::loaders;
//using namespace xercesc;

XMLPropertyLoader::XMLPropertyLoader()
{

}

XMLPropertyLoader::~XMLPropertyLoader()
{
}




PropertyMap XMLPropertyLoader::load(std::string& filename) {
  /*  try
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


    readProperties(elementRoot);*/
}
