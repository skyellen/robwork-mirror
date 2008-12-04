#ifndef RW_LOADERS_XERCESUTILS_HPP
#define RW_LOADERS_XERCESUTILS_HPP

#include <rw/common/macros.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>

#include <string>


namespace rw {
namespace loaders {


/**
 * @brief Utility class to help convert between Xerces unicode XMLCh* and ordinary C/C++ strings.
 *
 * The class makes sure to clean up pointers
 */
class XMLStr
{
public :
    /**
     * @brief Constructs from char array.
     */
    XMLStr(const char* const str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str);
    }

    /**
     * @brief Constructs from std::string
     */
    XMLStr(const std::string& str)
    {
        _uniCodeForm = xercesc::XMLString::transcode(str.c_str());
    }

    /**
     * @brief Constructs from double.
     *
     * The methods uses an ordinary sprintf to convert into ch*, which
     * are then converted to unicode.
     */
    XMLStr(double value) {
        char buffer[128];
        sprintf(buffer, "%f", value );
        _uniCodeForm = xercesc::XMLString::transcode(buffer);
    }

    /**
     * @brief Constructs from unicode string
     */
    XMLStr(const XMLCh* ch) {
        _uniCodeForm = NULL;
        char* buf = xercesc::XMLString::transcode(ch);
        _str = std::string(buf);
        xercesc::XMLString::release(&buf);
    }

    /**
     * @brief Destructor
     */
    ~XMLStr()
    {
        xercesc::XMLString::release(&_uniCodeForm);
    }


    /**
     * @brief Returns the unicode value
     *
     * If constructed with XMLStr(const XMLCh* ch) no conversion has appeared and the method
     * returns NULL
     */
    const XMLCh* uni() const
    {
        return _uniCodeForm;
    }

    /**
     * @brief Returns std::string from unicode
     *
     * Only defined when the object is constructed using XMLStr(const XMLCh* ch).
     */
    std::string str() const {
        return _str;
    }

private:
    XMLCh*   _uniCodeForm;
    std::string _str;
};



/**
 * @brief Utility class which initializes Xerces upon creation
 */
class XercesInitializer
{
public:
    /**
     * @brief Initializes Xerces when constructed.
     *
     * The method may throw an exception if not able to initialize
     */
    XercesInitializer() {
        try
        {
           xercesc::XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        }
        catch(xercesc::XMLException& e) {
           RW_THROW("Unable to initialize Xerces used for parsing XML-files. Error Msg: "<<XMLStr(e.getMessage()).str());
        }
    }
};

/**
 * @brief Utility class for writing a DOMDocument to file
 */
class XercesDocumentWriter
{
/*private:
    class WriteErrorHandler: public xercesc::DOMErrorHandler {
    public:
        WriteErrorHandler();
        virtual bool handleError (const xercesc::DOMError &domError);

        int _errorCnt;
        int _warningCnt;

    private:
        void printMsg(const std::string& title, const xercesc::SAXParseException& exc);

        std::ostringstream _messages;
    };
*/
public:
    /**
     * @brief Writes the content of \b doc to file named \b filename
     *
     * This method throws an exception in case of errors.
     *
     * @param doc [in] DOMDocument to write
     * @param filename [in] Desired filename
     */
    static void writeDocument(xercesc::DOMDocument* doc, const std::string& filename);
};


} //end namespace loaders
} //end namespace rw

#endif //End include guard
