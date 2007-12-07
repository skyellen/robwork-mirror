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
