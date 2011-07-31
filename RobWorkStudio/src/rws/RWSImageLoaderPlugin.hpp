#ifndef RWSIMAGELOADERPLUGIN_HPP_
#define RWSIMAGELOADERPLUGIN_HPP_

#include <rw/loaders/image/ImageLoader.hpp>
#include <rw/plugin/PluginFactory.hpp>

namespace rws {

/**
 * @brief A RobWork image loader factory plugin. It adds additional image loader functionality
 * to the rw::loaders::ImageFactory through RobWork plugin structure.
 */
class RWSImageLoaderPlugin: public rw::plugin::PluginFactory<rw::loaders::ImageLoader> {
public:

    RWSImageLoaderPlugin():
        rw::plugin::PluginFactory<rw::loaders::ImageLoader>("RWSImageLoaderPlugin")
    {}

    rw::loaders::ImageLoader::Ptr make();

    rw::loaders::ImageLoader::Ptr make(const std::string&);


private:

};

}

#endif /* RWSIMAGELOADERPLUGIN_HPP_ */
