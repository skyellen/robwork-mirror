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

#ifndef RW_LOADERS_XERCESERRORHANDLER_HPP
#define RW_LOADERS_XERCESERRORHANDLER_HPP

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/dom/DOM.hpp>

#include <sstream>

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


/**
 * @brief Error handler for the Xerces parser.
 *
 * Implements the xercesc::ErrorHandler and xercesc::DOMErrorHandler interfaces and receives callbacks when either
 * warnings or errors occur in parsing and validation of the XML-document or while trying to save it.
 */
class XercesErrorHandler: public xercesc::ErrorHandler, public xercesc::DOMErrorHandler {
public:
    /**
     * @brief Constructs XercesErrorHandler
     */
    XercesErrorHandler();

    virtual ~XercesErrorHandler();

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
     * @brief Inherited from xercesc::DOMErrorHandler
     */
    virtual bool handleError (const xercesc::DOMError &domError);

    /**
     * @brief Returns a string containing all error and warning information
     */
    std::string getMessages();

    /**
     * @brief Returns number of errors
     */
    int getErrorCount();

    /**
     * @brief Returns number of warning
     */
    int getWarningCount();

private:
    void printMsg(const std::string& title, const xercesc::SAXParseException& exc);
    void printMsg(const std::string& title, const xercesc::DOMError& domError);
    std::ostringstream _messages;
    int _errCnt;
    int _warCnt;

};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
