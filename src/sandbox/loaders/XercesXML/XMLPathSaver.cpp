/*
 * XMLPathWriter.cpp
 *
 *  Created on: Nov 26, 2008
 *      Author: lpe
 */

#include "XMLPathSaver.hpp"
#include "XMLPathLoader.hpp"

#include "XMLMathUtils.hpp"
#include "XMLPathFormat.hpp"

#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>

#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMWriter.hpp>

#include <xercesc/framework/StdOutFormatTarget.hpp>
#include <xercesc/framework/LocalFileFormatTarget.hpp>
#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>


using namespace xercesc;
using namespace rw::trajectory;
using namespace rw::loaders;
using namespace rw::math;

XMLPathSaver::XMLPathSaver()
{
    // TODO Auto-generated constructor stub

}

XMLPathSaver::~XMLPathSaver()
{
    // TODO Auto-generated destructor stub
}





namespace {
    template <class T>
    class ElementCreator {
    public:
        static DOMElement* createElement(const T& element, DOMDocument* doc);
    };

    template<> DOMElement* ElementCreator<Q>::createElement(const Q& element, DOMDocument* doc) {
        return XMLMathUtils::createQ(element, doc);
    }

    template<> DOMElement* ElementCreator<Vector3D<> >::createElement(const Vector3D<>& element, DOMDocument* doc) {
        return XMLMathUtils::createVector3D(element, doc);
    }

    template<> DOMElement* ElementCreator<Rotation3D<> >::createElement(const Rotation3D<>& element, DOMDocument* doc) {
        return XMLMathUtils::createRotation3D(element, doc);
    }

    template<> DOMElement* ElementCreator<Transform3D<> >::createElement(const Transform3D<>& element, DOMDocument* doc) {
        return XMLMathUtils::createTransform3D(element, doc);
    }


    template<class T, class PATH>
    bool savePathImpl(const PATH& path, const XMLCh* pathId, const std::string& filename) {

        DOMImplementation* impl =  DOMImplementationRegistry::getDOMImplementation(XMLStr("Core").uni());

        if (impl != NULL)
        {
            try
            {
                DOMDocument* doc = impl->createDocument(0,                    // root element namespace URI.
                                                        pathId,         // root element name
                                                        0);                   // We do not wish to specify a document type

                DOMElement* root = doc->getDocumentElement();
                for (typename PATH::const_iterator it = path.begin(); it != path.end(); ++it) {
                    root->appendChild(ElementCreator<T>::createElement(*it, doc));
                }

                XercesDocumentWriter::writeDocument(doc, filename);

                doc->release();
            }
            catch (const OutOfMemoryException&)
            {
                RW_THROW("XMLPathWriter: OutOfMemory");
            }
            catch (const DOMException& e)
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
        return true;

    }

}



bool XMLPathSaver::savePath(const QPath& path, const std::string& filename) {

    return savePathImpl<Q,  QPath>(path, XMLPathFormat::QPathId, filename);
}

bool XMLPathSaver::savePath(const Vector3DPath& path, const std::string& filename) {

    return savePathImpl<Vector3D<>,  Vector3DPath>(path, XMLPathFormat::V3DPathId, filename);
}

bool XMLPathSaver::savePath(const Rotation3DPath& path, const std::string& filename) {

    return savePathImpl<Rotation3D<>,  Rotation3DPath>(path, XMLPathFormat::R3DPathId, filename);
}

bool XMLPathSaver::savePath(const Transform3DPath& path, const std::string& filename) {

    return savePathImpl<Transform3D<>,  Transform3DPath>(path, XMLPathFormat::T3DPathId, filename);
}



