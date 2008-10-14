#include "GeometryFactory.hpp"

#include <rw/common/IOUtil.hpp>
#include <rw/common/StringUtil.hpp>

#include <sandbox/trimesh/STLFile.hpp>

using namespace rw::common;
using namespace rw::geometry;

namespace
{
    const std::string extensionsArray[] = {
        /*".TRI", ".AC", ".AC3D", ".3DS", ".IVG",*/ ".STL", ".STLA", ".STLB"
    };

    const int extensionCount = sizeof(extensionsArray) / sizeof(extensionsArray[0]);

    const std::vector<std::string> extensions(
        extensionsArray, extensionsArray + extensionCount);
}


Geometry* GeometryFactory::getGeometry(const std::string& raw_filename, bool useCache){
    const std::string& filename = IOUtil::resolveFileName(raw_filename, extensions);
    const std::string& filetype =
        StringUtil::toUpper(StringUtil::getFileExtension(filename));

    // if the file does not exist then throw an exception
    if (filetype.empty()) {
        RW_THROW(
            "No file type known for file "
            << StringUtil::quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    if (useCache && getCache().isInCache(filename)) {
        std::cout << "GeometryFactory - CACHE HIT" << std::endl;
        return new Geometry(getCache().get(filename));
    }

    // else check if the file has been loaded before
    if (filetype == ".STL" || filetype == ".STLA" || filetype == ".STLB") {
        GeometryData* data = STLFile::read(filename);
        if( data == NULL )
            RW_THROW("Reading of geometry failed!");
        getCache().add(filename, data);
        return new Geometry(getCache().get(filename));
    /*
    } else if (filetype == ".3DS") {
        return NULL;
    } else if (filetype == ".AC" || filetype == ".AC3D") {
        return NULL;
    } else if (filetype == ".TRI") {
        return NULL;
    } else if (filetype == ".IVG") {
        return NULL;
    */
    } else {
        RW_THROW(
            "Unknown extension "
            << StringUtil::quote(StringUtil::getFileExtension(filename))
            << " for file "
            << StringUtil::quote(raw_filename)
            << " that was resolved to file name "
            << filename);
    }

    RW_ASSERT(!"Impossible");
    return NULL; // To avoid a compiler warning.
}

GeometryFactory::Cache& GeometryFactory::getCache(){
    static Cache cache;
    return cache;
}
