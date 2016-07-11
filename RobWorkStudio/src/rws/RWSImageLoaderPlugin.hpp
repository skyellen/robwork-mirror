#ifndef RWS_RWSIMAGELOADERPLUGIN_HPP_
#define RWS_RWSIMAGELOADERPLUGIN_HPP_

#include <rw/common/Plugin.hpp>

namespace rws {

/**
 * @brief A RobWork image loader factory plugin. It adds additional image loader functionality
 * to the rw::loaders::ImageFactory through RobWork plugin structure.
 */
class RWSImageLoaderPlugin: public rw::common::Plugin {
public:

    RWSImageLoaderPlugin();

    virtual ~RWSImageLoaderPlugin();

    std::vector<rw::common::Extension::Descriptor> getExtensionDescriptors();

    rw::common::Ptr<rw::common::Extension> makeExtension(const std::string& str);

private:
    rw::common::PropertyMap _map;

};

}

#endif /* RWSIMAGELOADERPLUGIN_HPP_ */
