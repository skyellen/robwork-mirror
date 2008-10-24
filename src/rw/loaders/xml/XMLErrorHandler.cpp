/*********************************************************************
 * RobWork Version 0.3
 * Copyright (C) Robotics Group, Maersk Institute, University of Southern
 * Denmark.
 *
 * RobWork can be used, modified and redistributed freely.
 * RobWork is distributed WITHOUT ANY WARRANTY; including the implied
 * warranty of merchantability, fitness for a particular purpose and
 * guarantee of future releases, maintenance and bug fixes. The authors
 * has no responsibility of continuous development, maintenance, support
 * and insurance of backwards capability in the future.
 *
 * Notice that RobWork uses 3rd party software for which the RobWork
 * license does not apply. Consult the packages in the ext/ directory
 * for detailed information about these packages.
 *********************************************************************/

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
