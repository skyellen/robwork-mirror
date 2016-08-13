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

#ifndef RW_LOADERS_XMLPATHFORMAT_HPP
#define RW_LOADERS_XMLPATHFORMAT_HPP

#include <xercesc/util/XercesDefs.hpp>

namespace rw {
namespace loaders {
/** @addtogroup loaders */
/*@{*/


/**
 * @brief Class storing the identifiers used for paths in the XML Path Format
 */
class XMLPathFormat
{
public:
	/**
	 * @brief Identifier for rw::trajectory::QPath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idQPath();

	/**
	 * @brief Identifier for rw::trajectory::Vector3DPath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idV3DPath();

	/**
	 * @brief Identifier for rw::trajectory::Rotation3DPath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idR3DPath();

	/**
	 * @brief Identifier for rw::trajectory::Transform3DPath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idT3DPath();

	/**
	 * @brief Identifier for rw::trajectory::StatePath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idStatePath();

	/**
	 * @brief Identifier for rw::trajectory::TimedQPath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idTimedQPath();

	/**
	 * @brief Identifier for rw::trajectory::TimedState in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idTimedState();

	/**
	 * @brief Identifier for rw::trajectory::TimedQ in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idTimedQ();

	/**
	 * @brief Identifier for rw::trajectory::TimedStatePath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idTimedStatePath();

	/**
	 * @brief Identifier for time attribute used for rw::trajectory::TimedQPath and rw::trajectory::TimedStatePath in the XML format.
	 * @return the identifier.
	 */
	static const XMLCh* idTime();

	/**
	 * @brief Utility class which initializes local static variables.
	 *
	 * If the XMLPathFormat is used outside main (as a part of global initialization/destruction), the Initializer
	 * should be used explicitly to control the static initialization/destruction order.
	 *
	 * Notice that the Initializer is automatically defined as a global variable, hence it should not
	 * be necessary to specify the initializer explicitly if XMLPathFormat is to be used in local static
	 * initialization/destruction.
	 */
	class Initializer {
	public:
	    //! @brief Initializes when constructed.
		Initializer();
	};

private:
	static const Initializer initializer;

    XMLPathFormat() {};
};

/** @} */

} //end namespace loaders
} //end namespace rw

#endif //End include guard
