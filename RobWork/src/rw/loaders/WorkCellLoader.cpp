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


#include "WorkCellLoader.hpp"

#include <rw/loaders/tul/TULLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/common/StringUtil.hpp>
#include "WorkCellFactory.hpp"

using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common;

WorkCell::Ptr WorkCellLoader::load(const std::string& file)
{
    RW_WARN("1");
    return WorkCellFactory::load(file);
}

WorkCell::Ptr WorkCellLoader::load(const std::string& file, rw::graphics::WorkCellScene::Ptr wcscene)
{
    return WorkCellFactory::load(file, wcscene);
}
