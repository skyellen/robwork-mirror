/*
 * XercesErrorHandler.hpp
 *
 *  Created on: Nov 5, 2008
 *      Author: lpe
 */

#ifndef XERCESERRORHANDLER_HPP_
#define XERCESERRORHANDLER_HPP_

#include <xercesc/sax/ErrorHandler.hpp>
#include <xercesc/sax/SAXParseException.hpp>
#include <xercesc/sax/SAXException.hpp>

#include <iostream>

class XercesErrorHandler: public xercesc::ErrorHandler {
    public:
        virtual void warning (const xercesc::SAXParseException &exc) {
            std::cout<<"Warning"<<std::endl;
            std::cout<<"Msg = "<<xercesc::XMLString::transcode(exc.getMessage())<<std::endl;
            std::cout<<"Column = "<<exc.getColumnNumber()<<std::endl;
            std::cout<<"Line= "<<exc.getLineNumber()<<std::endl;
            std::cout<<"PublicId = "<<xercesc::XMLString::transcode(exc.getPublicId());

        }
        virtual void error (const xercesc::SAXParseException &exc) {
            std::cout<<"Error"<<std::endl;
            std::cout<<"Msg = "<<xercesc::XMLString::transcode(exc.getMessage())<<std::endl;
            std::cout<<"Column = "<<exc.getColumnNumber()<<std::endl;
            std::cout<<"Line= "<<exc.getLineNumber()<<std::endl;
            std::cout<<"PublicId = "<<xercesc::XMLString::transcode(exc.getSystemId())<<std::endl;

        }

        virtual void fatalError (const xercesc::SAXParseException &exc) {
            std::cout<<"Fatal Error"<<std::endl;
            std::cout<<"Msg = "<<xercesc::XMLString::transcode(exc.getMessage())<<std::endl;
            std::cout<<"Column = "<<exc.getColumnNumber()<<std::endl;
            std::cout<<"Line= "<<exc.getLineNumber()<<std::endl;
            std::cout<<"PublicId = "<<xercesc::XMLString::transcode(exc.getSystemId())<<std::endl;

        }

        virtual void resetErrors () {
            std::cout<<"Reset Errors"<<std::endl;
        }
    };


#endif /* XERCESERRORHANDLER_HPP_ */
