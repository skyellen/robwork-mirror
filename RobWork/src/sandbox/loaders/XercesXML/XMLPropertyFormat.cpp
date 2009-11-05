/*
 * XMLPropertyFormat.cpp
 *
 *  Created on: Jan 5, 2009
 *      Author: lpe
 */

#include "XMLPropertyFormat.hpp"

using namespace rw::loaders;
using namespace xercesc;

const XercesInitializer XMLPropertyFormat::initializer;

/** @brief Identifier for rw::common::PropertyMap in the XML format  */
const XMLCh* XMLPropertyFormat::PropertyMapId = XMLString::transcode("PropertyMap");

/** @brief Identifier for rw::common::Property in the XML format  */
const XMLCh* XMLPropertyFormat::PropertyId = XMLString::transcode("Property");

/** @brief Identifier for the name of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyNameId = XMLString::transcode("Name");

/** @brief Identifier for the description of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyDescriptionId = XMLString::transcode("Description");

/** @brief Identifier for the type of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyTypeId = XMLString::transcode("Type");

/** @brief Identifier for the value of a rw::common::Property */
const XMLCh* XMLPropertyFormat::PropertyValueId = XMLString::transcode("Value");
