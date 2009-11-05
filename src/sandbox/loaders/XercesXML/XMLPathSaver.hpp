#ifndef RW_LOADERS_XMLPATHWRITER_HPP
#define RW_LOADERS_XMLPATHWRITER_HPP


#include "XMLMathUtils.hpp"
#include "XercesUtils.hpp"
#include <rw/trajectory/Path.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>


#include <string>

namespace rw {
namespace loaders {

    namespace {
         template <class T>
         class ElementCreator {
         public:
             static xercesc::DOMElement* createElement(const T& element, xercesc::DOMDocument* doc);
         };

         template<> xercesc::DOMElement* ElementCreator<rw::math::Q>::createElement(const rw::math::Q& element, xercesc::DOMDocument* doc) {
             return XMLBasisTypes::createQ(element, doc);
         }

         template<> xercesc::DOMElement* ElementCreator<rw::math::Vector3D<> >::createElement(const rw::math::Vector3D<>& element, xercesc::DOMDocument* doc) {
             return XMLBasisTypes::createVector3D(element, doc);
         }

         template<> xercesc::DOMElement* ElementCreator<rw::math::Rotation3D<> >::createElement(const rw::math::Rotation3D<>& element, xercesc::DOMDocument* doc) {
             return XMLBasisTypes::createRotation3D(element, doc);
         }

         template<> xercesc::DOMElement* ElementCreator<rw::math::Transform3D<> >::createElement(const rw::math::Transform3D<>& element, xercesc::DOMDocument* doc) {
             return XMLBasisTypes::createTransform3D(element, doc);
         }
     } //end namespace

/**
 * @brief Class used for saving a Path using the RobWork XML Path Format
 *
 * Specification of the XML-format can be found in rwxml_path.xsd and
 * matching with XMLPathLoader
 */
class XMLPathSaver
{
public:

    /**
     * @brief Saves the rw::trajectory::QPath \b path to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::QPath& path, const std::string& filename);

    /**
     * @brief Saves the rw::trajectory::Vector3DPath \b path to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Vector3DPath& path, const std::string& filename);

    /**
     * @brief Saves the Rotation3DPath \b path to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Rotation3DPath& path, const std::string& filename);

    /**
     * @brief Saves the Transform3DPath \b path to file
     *
     * If an error occurs while saving an rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Transform3DPath& path, const std::string& filename);

    /**
     * @brief Create an element representing the path
     *
     * Create an element titles \b pathId representing \b path
     *
     * @param path [in] Path to save. Can be either QPath, Vector3DPath, Rotation3DPath or Transform3DPath
     * @param pathId [in] Id of the path
     * @param doc [in] Document for which to construct the element
     */
    template <class T, class PATH>
    static xercesc::DOMElement* createElement(const PATH& path, const XMLCh* pathId, xercesc::DOMDocument* doc) {
        xercesc::DOMElement* pathElement = doc->createElement(pathId);
        XMLPathSaver::insertElements<T, PATH>(path, pathElement, doc);
        return pathElement;
    }




private:
    XMLPathSaver() {};



    template <class T, class PATH>
    static void savePathImpl(const PATH& path, const XMLCh* pathId, const std::string& filename) {
       xercesc::DOMImplementation* impl =  xercesc::DOMImplementationRegistry::getDOMImplementation(XMLStr("Core").uni());

       if (impl != NULL)
       {
           try
           {
               xercesc::DOMDocument* doc = impl->createDocument(0,                    // root element namespace URI.
                                                                pathId,         // root element name
                                                                0);                   // We do not wish to specify a document type


               xercesc::DOMElement* root = doc->getDocumentElement();
               XMLPathSaver::insertElements<T, PATH>(path, root, doc);

               XercesDocumentWriter::writeDocument(doc, filename);

               doc->release();
           }
           catch (const xercesc::OutOfMemoryException&)
           {
               RW_THROW("XMLPathWriter: OutOfMemory");
           }
           catch (const xercesc::DOMException& e)
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
    }

    template<class T, class PATH>
    static void insertElements(const PATH& path, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
        for (typename PATH::const_iterator it = path.begin(); it != path.end(); ++it) {
            element->appendChild(ElementCreator<T>::createElement(*it, doc));
        }
    }

};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
