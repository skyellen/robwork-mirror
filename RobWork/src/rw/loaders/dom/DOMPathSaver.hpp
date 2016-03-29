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

#ifndef RW_LOADERS_DOMPATHSAVER_HPP
#define RW_LOADERS_DOMPATHSAVER_HPP


#include <rw/loaders/dom/DOMBasisTypes.hpp>


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



/**
 * @brief Class used for saving a Path using the RobWork XML Path Format
 *
 * Specification of the XML-format can be found in rwxml_path.xsd and
 * matching with XMLPathLoader
 */
class DOMPathSaver
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

    //-------------------------------- Creating and writing DOMElem
    /**
     * @brief Creates a DOMElem to represent \b path
     *
     * Creates a DOMElement owned by \b doc and representing \b path
     *
     * @param path [in] path to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElem
     */
    static rw::common::DOMElem::Ptr createTransform3DPath(const rw::trajectory::Transform3DPath &path, rw::common::DOMElem::Ptr doc);

    /**
     * @brief Creates a DOMElem to represent \b path
     *
     * Creates a DOMElement owned by \b doc and representing \b path
     *
     * @param path [in] path to represent
     * @param doc [in] Document which should contain the element
     * @return Pointer to the newly created DOMElem
     */
    static rw::common::DOMElem::Ptr createQPath(const rw::trajectory::QPath &path, rw::common::DOMElem::Ptr doc);

private:
    DOMPathSaver() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //end include guard
