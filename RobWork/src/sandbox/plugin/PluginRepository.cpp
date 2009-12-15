#include "PluginRepository.hpp"

#include <rwlibs/dll/DynamicLibraryLoader.hpp>


using namespace rw::plugin;
using namespace rwlibs::dll;


PluginRepository PluginRepository::_repository;


void PluginRepository::add(const std::string& filename) {
    DynamicLibraryLoader<PluginConstructor> loader(filename);
    PluginConstructorPtr constructor = loader.get();
    if (constructor != NULL) {
        std::cout<<"Identifier = "<<constructor->identifier()<<std::endl;
        _str2constructorMap[constructor->identifier()] = constructor;
    } else {
        std::cout<<"Not able to load"<<std::endl;
    }
}

PluginRepository& PluginRepository::instance() {
    return _repository;
}
