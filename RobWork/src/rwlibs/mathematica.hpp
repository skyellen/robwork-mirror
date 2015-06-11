/********************************************************************************
 * Copyright 2015 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#ifndef RWLIBS_MATHEMATICA_HPP_
#define RWLIBS_MATHEMATICA_HPP_

/**
 * @file mathematica.hpp
 *
 * @brief Include file for all Mathematica headers.
 */

#include "./mathematica/Mathematica.hpp"

// Packets
#include "./mathematica/EnterExpressionPacket.hpp"
#include "./mathematica/EnterTextPacket.hpp"
#include "./mathematica/EvaluatePacket.hpp"
#include "./mathematica/InputNamePacket.hpp"
#include "./mathematica/MessagePacket.hpp"
#include "./mathematica/OutputNamePacket.hpp"
#include "./mathematica/ReturnExpressionPacket.hpp"
#include "./mathematica/ReturnPacket.hpp"
#include "./mathematica/ReturnTextPacket.hpp"
#include "./mathematica/TextPacket.hpp"

// Functions
#include "./mathematica/FactorInteger.hpp"
#include "./mathematica/Image.hpp"
#include "./mathematica/List.hpp"
#include "./mathematica/ListPlot.hpp"
#include "./mathematica/RawArray.hpp"
#include "./mathematica/Rule.hpp"
#include "./mathematica/ToExpression.hpp"

#endif /* RWLIBS_MATHEMATICA_HPP_ */
