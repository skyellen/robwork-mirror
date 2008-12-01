/*
 * XercesErrorHandler.cpp
 *
 *  Created on: Nov 5, 2008
 *      Author: lpe
 */

#include "XercesErrorHandler.hpp"

using namespace rw::loaders;
using namespace xercesc;


void XercesErrorHandler::printMsg(const std::string& title, const xercesc::SAXParseException& exc) {
    std::cout<<"Prints out stuff"<<std::endl;
    //std::string id = xercesc::XMLString::transcode(exc.getPublicId());
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

void XercesErrorHandler::warning (const xercesc::SAXParseException &exc) {
    printMsg("Warning", exc);
}


void XercesErrorHandler::error (const xercesc::SAXParseException &exc) {
    printMsg("Error", exc);
}

void XercesErrorHandler::fatalError (const xercesc::SAXParseException &exc) {
    printMsg("Critical Error", exc);
}

void XercesErrorHandler::resetErrors () {
    _messages.clear();
}

std::string XercesErrorHandler::getMessages() {
    return _messages.str();
}
