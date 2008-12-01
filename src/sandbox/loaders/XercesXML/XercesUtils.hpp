/*
 * XercesUtils.hpp
 *
 *  Created on: Nov 27, 2008
 *      Author: lpe
 */

#ifndef RW_LOADERS_XERCESUTILS_HPP
#define RW_LOADERS_XERCESUTILS_HPP

#include <rw/common/macros.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>




#include <string>

// ---------------------------------------------------------------------------
//  This is a simple class that lets us do easy (though not terribly efficient)
//  trancoding of char* data to XMLCh data.
// ---------------------------------------------------------------------------
class XMLStr
{
public :
    // -----------------------------------------------------------------------
    //  Constructors and Destructor
    // -----------------------------------------------------------------------
    XMLStr(const char* const str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str);
    }

    XMLStr(const std::string& str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str.c_str());
    }

    XMLStr(double value) {
        char* buffer = new char[128];
        sprintf(buffer, "%f", value );
        _uniCodeForm = xercesc::XMLString::transcode(buffer);
    }

    XMLStr(const XMLCh* ch) {
        _uniCodeForm = NULL;
        char* buf = xercesc::XMLString::transcode(ch);
        _str = std::string(buf);
        xercesc::XMLString::release(&buf);
    }

    ~XMLStr()
    {
        xercesc::XMLString::release(&_uniCodeForm);
    }


    // -----------------------------------------------------------------------
    //  Getter methods
    // -----------------------------------------------------------------------
    const XMLCh* uni() const
    {
        return _uniCodeForm;
    }

    std::string str() const {
        return _str;
    }

private:
    XMLCh*   _uniCodeForm;
    std::string _str;
};




class XercesInitializer
{
public:
    XercesInitializer() {
        try
        {
           xercesc::XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        }
        catch(xercesc::XMLException& e) {
           std::string message = xercesc::XMLString::transcode( e.getMessage() );
           RW_THROW("Unable to initialize Xerces used for parsing XML-files. Error Msg: "<<message);
        }
    }
};


class XercesDocumentWriter
{
private:
    class WriteErrorHandler: public xercesc::DOMErrorHandler {
    public:
        virtual bool handleError (const xercesc::DOMError &domError);
    };

public:
    static void writeDocument(xercesc::DOMDocument* doc, const std::string& filename);
};


#endif //End include guard
