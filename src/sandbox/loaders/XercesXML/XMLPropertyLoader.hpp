#ifndef RW_LOADERS_XMLPROPERTYLOADER_HPP
#define RW_LOADERS_XMLPROPERTYLOADER_HPP


#include <rw/common/PropertyMap.hpp>


namespace rw {
namespace loaders {


class XMLPropertyLoader
{
public:
    XMLPropertyLoader();
    virtual ~XMLPropertyLoader();

    static rw::common::PropertyMap load(std::string& filename);

};

} //end namespace loaders
} //end namespace rw

#endif //end include guard
