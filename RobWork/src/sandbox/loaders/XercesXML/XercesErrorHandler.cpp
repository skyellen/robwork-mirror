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
