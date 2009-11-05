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


#include "XMLErrorHandler.hpp"

using namespace boost::spirit;
using namespace rw::loaders;

/*
assertion<XMLErrors> expect_(program_expected);
assertion<XMLErrors> expect_begin(begin_expected);
assertion<XMLErrors> expect_end(end_expected);
*/

/*assertion<XMLErrorHandler::XMLError> XMLErrorHandler::bad_end_elem(xml_bad_end_elem);
assertion<XMLErrorHandler::XMLError> XMLErrorHandler::missing_brac(xml_missing_brack);
assertion<XMLErrorHandler::XMLError> XMLErrorHandler::att_expected(xml_att_expected);
assertion<XMLErrorHandler::XMLError> XMLErrorHandler::att_unexpected(xml_att_unexpected);*/
guard<XMLErrorHandler::XMLError> XMLErrorHandler::XMLErrorGuard;
