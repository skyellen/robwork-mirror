#include "AsciiPathSaver.hpp"
#include <rw/math/Q.hpp>

using namespace rw::loaders;
namespace {
	std::string getNewLine() {
		std::stringstream sstr;
		sstr<<std::endl;
		return sstr.str();
	}
}

AsciiPathSaver::Format AsciiPathSaver::Mathematica("", "", " ", getNewLine());
AsciiPathSaver::Format AsciiPathSaver::CommaSeparated("", "", " ", getNewLine());
	

void test() {
	rw::trajectory::QPath path;
	rw::loaders::AsciiPathSaver::save(std::cout, path);

	rw::trajectory::Transform3DPath patht3d;
	rw::loaders::AsciiPathSaver::save(std::cout, patht3d);
}
