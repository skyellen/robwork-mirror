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

#ifndef RW_LOADERS_XMLPATHLOADER_HPP
#define RW_LOADERS_XMLPATHLOADER_HPP


#include <rw/trajectory/Path.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <xercesc/util/XercesDefs.hpp>

#include <string>

namespace rw { namespace models { class WorkCell; } }

XERCES_CPP_NAMESPACE_BEGIN
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rw {
namespace loaders {

/** @addtogroup loaders */
/*@{*/


/**
 * @brief Enables loading in path file specified in the RobWork Path XML format.
 *
 * The XMLPathLoader loads in a file containing a path specified according to the rwxml_path.xsd schema.
 * The XML-file can be parsed either with or without schema verification. The schema can either be specified in the
 * XML-file or given as argument to the constructor.
 *
 * A path can contain either rw::math::Q, rw::math::Vector3D, rw::math::Rotation3D or rw::math::Transform3D elements.
 * If the type of the path in the file in unknown it can be determined using the XMLPathLoader::getType after loading.
 *
 * If reading in a path fails an exception is thrown
 */
class XMLPathLoader
{
public:
    /**
     * @brief Constructs XMLPathLoader and parser \b filename
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the path fails an exception is thrown
     *
     * @param filename [in] The file to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
	XMLPathLoader(const std::string& filename, rw::common::Ptr<rw::models::WorkCell> workcell = NULL, const std::string& schemaFileName = "");


    /**
     * @brief Constructs XMLPathLoader and parser input from \b instream
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * Throw rw::common::Exception if reading the path fails
     *
     * @param instream [in] The input stream to read from
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
	XMLPathLoader(std::istream& instream, rw::common::Ptr<rw::models::WorkCell> workcell = NULL, const std::string& schemaFileName = "");


    /**
     * @brief Constructs XMLPathLoader and load in path in \b element.
     *
     * No validation is applied hence the syntax of the element is assumed correct.
     *
     * If loading the path fails an exception is thrown
     *
     * @param element [in] DOMElement representing the path
     */
    XMLPathLoader(xercesc::DOMElement* element);


    /**
     * @brief Destructor
     */
    virtual ~XMLPathLoader();

    /**
     * @brief Enumeration specifying which type of path, that has been loaded
     */
    enum Type { QType = 0,       /** @brief rw::trajectory::QPath */
                Vector3DType,    /** @brief rw::trajectory::Vector3DPath */
                Rotation3DType,  /** @brief rw::trajectory::Rotation3DPath */
                Transform3DType, /** @brief rw::trajectory::Transform3DPath */
                StateType,       /** @brief rw::trajectory::StatePath */
                TimedQType,      /** @brief rw::trajectory::TimedQPath */
                TimedStateType   /** @brief rw::trajectory::TimedStatePath */
                };

    /**
     * @brief Returns the type of the path loaded
     */
    Type getType();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type QPath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::QPath::Ptr getQPath();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type Vector3DPath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::Vector3DPath::Ptr getVector3DPath();

    /**
     * @brief Returns path loaded
     *
     * If the loaded path is not of type Rotation3DPath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::Rotation3DPath::Ptr getRotation3DPath();

    /**
     * @brief Returns loaded path
     *
     * If the loaded path is not of type Transform3DPatha rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::Transform3DPath::Ptr getTransform3DPath();

    /**
     * @brief Returns loaded path
     *
     * If the loaded path is not of type StatePath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::StatePath::Ptr getStatePath();


    /**
     * @brief Returns loaded path
     *
     * If the loaded path is not of type TimedQPath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::TimedQPath::Ptr getTimedQPath();


    /**
     * @brief Returns loaded path
     *
     * If the loaded path is not of type TimedStatePath a rw::common::Exception is thrown.
     *
     * @return Pointer to the path
     */
	rw::trajectory::TimedStatePath::Ptr getTimedStatePath();

private:


   void readPath(xercesc::DOMElement* element);

   rw::trajectory::QPath::Ptr _qPath;
   rw::trajectory::Vector3DPath::Ptr _v3dPath;
   rw::trajectory::Rotation3DPath::Ptr _r3dPath;
   rw::trajectory::Transform3DPath::Ptr _t3dPath;
   rw::trajectory::StatePath::Ptr _statePath;
   rw::trajectory::TimedQPath::Ptr _timedQPath;
   rw::trajectory::TimedStatePath::Ptr _timedStatePath;

    Type _type;
    rw::common::Ptr<rw::models::WorkCell> _workcell;
};

/** @} */


} //end namespace loaders
} //end namespace rw

#endif //enc include guard
