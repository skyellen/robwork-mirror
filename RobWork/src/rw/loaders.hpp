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
/**
 * @file loaders.hpp
 *
 * this file includes all header files from the loaders namespace
 */

#ifndef RW_LOADERS_HPP_
#define RW_LOADERS_HPP_

#include "./loaders/colsetup/CollisionSetupLoader.hpp"
#include "./loaders/image/PGMLoader.hpp"
#include "./loaders/image/RGBLoader.hpp"
#include "./loaders/ImageLoader.hpp"
#include "./loaders/path/PathLoader.hpp"
#include "./loaders/rwxml/DependencyGraph.hpp"
#include "./loaders/rwxml/MultipleFileIterator.hpp"
#include "./loaders/rwxml/XMLParserUtil.hpp"
#include "./loaders/rwxml/XMLRWLoader.hpp"
#include "./loaders/rwxml/XMLRWParser.hpp"
#include "./loaders/rwxml/XMLRWPreParser.hpp"
#include "loaders/model3d/STLFile.hpp"
#include "loaders/GeometryFactory.hpp"
//#include "./loaders/TaskLoader.hpp"
#include "./loaders/tul/Tag.hpp"
#include "./loaders/tul/TULLoader.hpp"
#include "./loaders/WorkCellLoader.hpp"
#include "./loaders/rwxml/XML.hpp"
#include "./loaders/rwxml/XMLErrorHandler.hpp"
#include "./loaders/rwxml/XMLParser.hpp"
#include "./loaders/xml/XMLPropertyLoader.hpp"
#include "./loaders/xml/XMLPropertySaver.hpp"
#include "./loaders/xml/XMLPathLoader.hpp"
#include "./loaders/xml/XMLPathSaver.hpp"
#include "./loaders/xml/XMLTrajectoryLoader.hpp"
#include "./loaders/xml/XMLTrajectorySaver.hpp"

#include "./loaders/dom/DOMBasisTypes.hpp"
#include "./loaders/dom/DOMPathLoader.hpp"
#include "./loaders/dom/DOMPathSaver.hpp"
#include "./loaders/dom/DOMPropertyMapLoader.hpp"
#include "./loaders/dom/DOMProximitySetupLoader.hpp"
#include "./loaders/dom/DOMTrajectoryLoader.hpp"
#include "./loaders/dom/DOMTrajectorySaver.hpp"


#endif /* LOADERS_HPP_ */
