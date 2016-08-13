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

#ifndef RW_LOADERS_XMLPATHWRITER_HPP
#define RW_LOADERS_XMLPATHWRITER_HPP


#include "XMLBasisTypes.hpp"
#include "XercesUtils.hpp"
#include "XMLPathFormat.hpp"

#include <rw/trajectory/Path.hpp>
#include <rw/trajectory/Timed.hpp>

#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>


#include <string>

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


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

         template<> xercesc::DOMElement* ElementCreator<rw::kinematics::State>::createElement(const rw::kinematics::State& element, xercesc::DOMDocument* doc) {
             return XMLBasisTypes::createState(element, doc);
         }


         template<> xercesc::DOMElement* ElementCreator<rw::trajectory::TimedQ>::createElement(const rw::trajectory::TimedQ& timedQ, xercesc::DOMDocument* doc) {

             xercesc::DOMElement* element = doc->createElement(XMLPathFormat::idTimedQ());

             xercesc::DOMElement* timeElement = doc->createElement(XMLPathFormat::idTime());
             timeElement->appendChild(doc->createTextNode(XMLStr(timedQ.getTime()).uni()));
             element->appendChild(timeElement);
             element->appendChild(XMLBasisTypes::createQ(timedQ.getValue(), doc));

             return element;
         }

         template<> xercesc::DOMElement* ElementCreator<rw::trajectory::TimedState>::createElement(const rw::trajectory::TimedState& timedState, xercesc::DOMDocument* doc) {
             xercesc::DOMElement* element = doc->createElement(XMLPathFormat::idTimedState());
             xercesc::DOMElement* timeElement = doc->createElement(XMLPathFormat::idTime());
             timeElement->appendChild(doc->createTextNode(XMLStr(timedState.getTime()).uni()));
             element->appendChild(timeElement);
             element->appendChild(XMLBasisTypes::createState(timedState.getValue(), doc));

             return element;
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
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::QPath& path, const std::string& filename);

    /**
     * @brief Saves rw::trajectory::Vector3DPath \b path to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Vector3DPath& path, const std::string& filename);

    /**
     * @brief Saves rw::trajectory::Rotation3DPath \b path to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Rotation3DPath& path, const std::string& filename);

    /**
     * @brief Saves rw::trajectory::Transform3DPath \b path to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::Transform3DPath& path, const std::string& filename);

    /**
     * @brief Saves rw::trajectory::StatePath \b to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::StatePath& path, const std::string& filename);


    /**
     * @brief Saves rw::trajectory::TimedQPath \b to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::TimedQPath& path, const std::string& filename);

    /**
     * @brief Saves rw::trajectory::TimedStatePath \b to file
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Target filename
     */
    static void save(const rw::trajectory::TimedStatePath& path, const std::string& filename);



    /**
     * @brief Writes the rw::trajectory::QPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::QPath& path, std::ostream& outstream);

    /**
     * @brief Writes rw::trajectory::Vector3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save8
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::Vector3DPath& path, std::ostream& outstream);

    /**
     * @brief Writes rw::trajectory::Rotation3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::Rotation3DPath& path, std::ostream& outstream);

    /**
     * @brief Writes rw::trajectory::Rotation3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::Transform3DPath& path, std::ostream& outstream);

    /**
     * @brief Writes rw::trajectory::Rotation3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::StatePath& path, std::ostream& outstream);


    /**
     * @brief Writes rw::trajectory::Rotation3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param outstream [in] Stream to write to
     */
    static void write(const rw::trajectory::TimedQPath& path, std::ostream& outstream);

    /**
     * @brief Writes rw::trajectory::Rotation3DPath \b path to \b outstream
     *
     * If an error occurs while saving a rw::common::Exception is thrown
     *
     * @param path [in] Path to save
     * @param filename [in] Stream to write to
     */
    static void write(const rw::trajectory::TimedStatePath& path, std::ostream& outstream);


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

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLPathSaver is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLPathSaver is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLPathSaver() {};

    template <class T, class PATH>
    static xercesc::DOMDocument* createDOMDocument(const PATH& path, const XMLCh* pathId) {
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
                return doc;
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

    template <class T, class PATH>
    static void savePath(const PATH& path, const XMLCh* pathId, const std::string& filename) {

        xercesc::DOMDocument* doc = createDOMDocument<T, PATH>(path, pathId);
        XercesDocumentWriter::writeDocument(doc, filename);
        doc->release();

    }

    template <class T, class PATH>
    static void writePath(const PATH& path, const XMLCh* pathId, std::ostream& outstream) {

        xercesc::DOMDocument* doc = createDOMDocument<T, PATH>(path, pathId);
        XercesDocumentWriter::writeDocument(doc, outstream);
        doc->release();

    }
    /*
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
*/
    template<class T, class PATH>
    static void insertElements(const PATH& path, xercesc::DOMElement* element, xercesc::DOMDocument* doc) {
        for (typename PATH::const_iterator it = path.begin(); it != path.end(); ++it) {
            element->appendChild(ElementCreator<T>::createElement(*it, doc));
        }
    }
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
