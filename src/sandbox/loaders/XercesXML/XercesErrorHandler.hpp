#ifndef RW_LOADERS_XERCESERRORHANDLER_HPP
#define RW_LOADERS_XERCESERRORHANDLER_HPP

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>

#include <iostream>
#include <sstream>

namespace rw {
namespace loaders {

/**
 * @brief Error handler for the Xerces parser.
 *
 * Implements the xercesc::ErrorHandler interface and receives callbacks when either
 * warnings or errors occurs in parsing and validation of the XML-document,
 */
class XercesErrorHandler: public xercesc::ErrorHandler {
public:
    /**
     * @brief Inherited from xercesc::ErrorHandler. Call when a warning occurs
     */
    virtual void warning (const xercesc::SAXParseException& exc);

    /**
     * @brief Inherited from xercesc::ErrorHandler. Call when an error occurs
     */
    virtual void error (const xercesc::SAXParseException& exc);

    /**
     * @brief Inherited from xercesc::ErrorHandler. Call when a fatal error occurs
     */
    virtual void fatalError (const xercesc::SAXParseException& exc);

    /**
     * @brief Inherited from xercesc::ErrorHandler. Resets the list of errors
     */
    virtual void resetErrors();

    /**
     * @brief Returns a string containing all error and warning information
     */
    std::string getMessages();

private:
    void printMsg(const std::string& title, const xercesc::SAXParseException& exc);

    std::ostringstream _messages;

};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
