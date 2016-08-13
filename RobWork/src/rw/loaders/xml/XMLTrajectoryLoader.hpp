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

#ifndef RW_LOADERS_TRAJECTORYLOADER_HPP
#define RW_LOADERS_TRAJECTORYLOADER_HPP


#include <rw/common/Ptr.hpp>
#include <rw/trajectory/Trajectory.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/Transform3D.hpp>

#include <xercesc/util/XercesDefs.hpp>

//#include "XercesErrorHandler.hpp"

/*
#include <xercesc/dom/DOM.hpp>
#include <xercesc/dom/DOMDocument.hpp>
#include <xercesc/dom/DOMDocumentType.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/dom/DOMImplementation.hpp>
#include <xercesc/dom/DOMImplementationLS.hpp>
#include <xercesc/dom/DOMNodeIterator.hpp>
#include <xercesc/dom/DOMNodeList.hpp>
#include <xercesc/dom/DOMText.hpp>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLUni.hpp>
*/
#include <string>

XERCES_CPP_NAMESPACE_BEGIN
class DOMElement;
XERCES_CPP_NAMESPACE_END

namespace rw {
namespace loaders {


/** @addtogroup loaders */
/*@{*/


/**
 * @brief Enables loading in trajectories file specified in the RobWork Trajectory XML format.
 *
 * The XMLTrajectoryLoader loads in a file containing a trajectory specified according to the rwxml_trajectory.xsd schema.
 * The XML-file can be parsed either with or without schema verification. The schema can either be specified in the
 * XML-file or given as argument to the constructor.
 *
 * A trajectory can contain either rw::math::Q, rw::math::Vector3D, rw::math::Rotation3D or rw::math::Transform3D elements.
 * If the type of the trajectory in the file in unknown it can be determined using the XMLTrajectoryLoader::getType after loading.
 *
 * If reading in a trajectory fails an exception is thrown
 */
class XMLTrajectoryLoader
{
public:
    /**
     * @brief Constructs XMLTrajectoryLoader and parser \b filename
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the trajectory fails an exception is thrown
     *
     * @param filename [in] The file to load
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");


    /**
     * @brief Constract XMLTrajectoryLoader and parser input from \b instream
     *
     * It is possible to specify whether to use the default schema which is the default behavior. If a
     * schema is specified in the XML-file or no schema should be used set \b useDefaultSchema to false.
     *
     * If reading in the trajectory fails an exception is thrown
     *
     * @param instream [in] The istream to read from
     * @param schemaFileName [in] Name of the schema to use. If empty it will use the schema specified in the XML-file if available.
     */
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");


    /**
     * @brief Destructor
     */
    virtual ~XMLTrajectoryLoader();

    /**
     * @brief Enumeration specifying which type of trajectory, that has been loaded
     */
    enum Type { QType = 0,      /** @brief rw::trajectory::Trajectory<Q> */
                Vector3DType,   /** @brief rw::trajectory::Trajectory<Vector3D> */
                Rotation3DType, /** @brief rw::trajectory::Trajectory<Rotation3D> */
                Transform3DType /** @brief rw::trajectory::Trajectory<Transform3D> */
                };

    /**
     * @brief Returns the type of the trajectory loaded
     */
    Type getType();

    /**
     * @brief Returns trajectory with template type rw::math::Q.
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::QTrajectory::Ptr getQTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Vector3D<>
     *
     * If the loaded trajectory does not contain this type an exception is thrown.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Vector3DTrajectory::Ptr getVector3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Rotation3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Rotation3DTrajectory::Ptr getRotation3DTrajectory();

    /**
     * @brief Returns trajectory with template type rw::math::Transform3D<>
     *
     * If the loaded path is not of type Transform3DPath it throws an exception.
     *
     * @return Copy of trajectory
     */
	rw::trajectory::Transform3DTrajectory::Ptr getTransform3DTrajectory();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLTrajectoryLoader is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLTrajectoryLoader is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

   /* template<class T>
    void initialize(T t, const std::string& schemaFileName) {
        try
        {
           xercesc::XMLPlatformUtils::Initialize();  // Initialize Xerces infrastructure
        }
        catch(xercesc::XMLException& e )
        {
           RW_THROW("Xerces initialization Error"<<rw::loaders::XMLStr(e.getMessage()).str());
        }

        xercesc::XercesDOMParser parser;

        rw::loaders::XercesErrorHandler errorHandler;

        parser.setDoNamespaces( true );
        parser.setDoSchema( true );
        if (schemaFileName.size() != 0)
            parser.setExternalNoNamespaceSchemaLocation(schemaFileName.c_str());


        parser.setErrorHandler(&errorHandler);
        parser.setValidationScheme(xercesc::XercesDOMParser::Val_Auto);

        parser.parse(t);
        if (parser.getErrorCount() != 0) {
            std::cerr<<std::endl<<std::endl<<"Error(s) = "<<std::endl<<XMLStr(errorHandler.getMessages()).str()<<std::endl;
            RW_THROW(""<<parser.getErrorCount()<<" Errors: "<<XMLStr(errorHandler.getMessages()).str());
        }


        // no need to free this pointer - owned by the parent parser object
        xercesc::DOMDocument* xmlDoc = parser.getDocument();

        // Get the top-level element: Name is "root". No attributes for "root"
        xercesc::DOMElement* elementRoot = xmlDoc->getDocumentElement();

        readTrajectory(elementRoot);
    }*/



    void readTrajectory(xercesc::DOMElement* element);

    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Q> > _qTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Vector3D<> > > _v3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Rotation3D<> > > _r3dTrajectory;
    rw::common::Ptr<rw::trajectory::Trajectory<rw::math::Transform3D<> > > _t3dTrajectory;

    Type _type;
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //enc include guard
