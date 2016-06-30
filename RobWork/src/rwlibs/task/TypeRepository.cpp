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

#include "TypeRepository.hpp"

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

using namespace rwlibs::task;

TypeRepository* TypeRepository::_repository = NULL;

TypeRepository::TypeRepository() {
    _typeMap[typeid(rw::math::Q).name()] = Type::Q;
    _typeMap[typeid(rw::math::Transform3D<>).name()] = Type::Transform3D;
    _next = Type::User;
}
