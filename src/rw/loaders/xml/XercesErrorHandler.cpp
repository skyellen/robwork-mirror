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

#include "XercesErrorHandler.hpp"
#include "XercesUtils.hpp"
using namespace rw::loaders;
using namespace xercesc;

XercesErrorHandler::XercesErrorHandler() {
    _errCnt = 0;
    _warCnt = 0;
}

void XercesErrorHandler::printMsg(const std::string& title, const xercesc::SAXParseException& exc) {
    _messages<<title
             <<": "
             <<xercesc::XMLString::transcode(exc.getSystemId());
             _messages<<" Line:"
             <<exc.getLineNumber()
             <<" Col:"
             <<exc.getColumnNumber()
             <<" "
             <<XMLString::transcode(exc.getMessage())
             <<std::endl<<std::endl;
}


void XercesErrorHandler::printMsg(const std::string& title, const xercesc::DOMError& domError) {
    _messages<<title
             <<":"
             <<XMLStr(domError.getMessage()).str()<<std::endl<<std::endl;
}

void XercesErrorHandler::warning (const xercesc::SAXParseException &exc) {
    printMsg("Warning", exc);
    _warCnt++;
}


void XercesErrorHandler::error (const xercesc::SAXParseException &exc) {
    printMsg("Error", exc);
    _errCnt++;
}

void XercesErrorHandler::fatalError (const xercesc::SAXParseException &exc) {
    printMsg("Fatal Error", exc);
    _errCnt++;
}

void XercesErrorHandler::resetErrors () {
    _messages.clear();
    _errCnt = 0;
    _warCnt = 0;
}


bool XercesErrorHandler::handleError (const xercesc::DOMError &domError) {
    if (domError.getSeverity() == xercesc::DOMError::DOM_SEVERITY_WARNING) {
        printMsg("Warning",domError);
        _warCnt++;
    }
    else if (domError.getSeverity() == xercesc::DOMError::DOM_SEVERITY_ERROR) {
        printMsg("Error",domError);
        _errCnt++;
    }
    else {
        printMsg("Fatal Error",domError);
        _errCnt++;
    }
    return true;
}


std::string XercesErrorHandler::getMessages() {
    return _messages.str();
}


int XercesErrorHandler::getErrorCount() {
    return _errCnt;
}

int XercesErrorHandler::getWarningCount() {
    return _warCnt;
}
