#include "WorkCellLoader.hpp"

#include <rw/loaders/tul/TULLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>
#include <rw/common/StringUtil.hpp>

using namespace rw::loaders;
using namespace rw::models;
using namespace rw::common;

std::auto_ptr<WorkCell>
WorkCellLoader::load(const std::string& file)
{
	std::cout<<"WorkCellLoader="<<file<<std::endl;
    const std::string ext = StringUtil::getFileExtension(file);
	if (ext == ".wu" || ext == ".wc" || ext == ".tag" || ext == ".dev") {
		std::cout<<"TUL WorkCell"<<std::endl;
        return TULLoader::load(file);
	}
	else {
		std::cout<<"XML WorkCell"<<std::endl;
        return XMLRWLoader::loadWorkCell(file);
	}
}
