#include "WorkCellLoader.hpp"

#include <rw/loaders/tul/TULLoader.hpp>
#include <rw/loaders/rwxml/XMLRWLoader.hpp>

using namespace rw::loaders;
using namespace rw::models;

std::auto_ptr<WorkCell>
WorkCellLoader::load(const std::string& file)
{
    // We need a more robust getFileExtension() utility here.
    if (file.size() > 3) {
        const std::string suffix = file.substr(file.size() - 3);
        if (suffix == ".wu" || suffix == ".wc")
            return TULLoader::load(file);
    }

    if (file.size() > 4) {
        const std::string suffix = file.substr(file.size() - 4);
        if (suffix == ".dev")
            return TULLoader::load(file);
    }

    return XMLRWLoader::loadWorkCell(file);
}
