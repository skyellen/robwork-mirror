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


#ifdef _MSC_VER
#include <boost/cstdint.hpp>
typedef boost::int8_t int8_t;
typedef boost::uint8_t uint8_t;
typedef boost::int16_t int16_t;
typedef boost::uint16_t uint16_t;
typedef boost::int32_t int32_t;
typedef boost::uint32_t uint32_t;
typedef boost::int64_t int64_t;
typedef boost::uint64_t uint64_t;

#else
#include <stdint.h>
#endif
