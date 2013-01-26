#include "Plugin.hpp"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <rw/common/StringUtil.hpp>

using namespace rw::common;

Plugin::Plugin(const std::string& id,
               const std::string& name,
               const std::string& version):
                       _id(id),_name(name),_version(version)
{

}
Plugin::~Plugin(){}


rw::common::Ptr<Plugin> Plugin::load(const std::string& filename){
    boost::filesystem::path file(filename);
    if(!exists(file))
        RW_THROW("The file does not exist: "<< filename );
    //std::cout << rw::common::StringUtil::toUpper(file.extension().string()) << std::endl;
    if( rw::common::StringUtil::toUpper(file.extension().string())==".XML"){
        return loadLazy(filename);
    }
    return loadDirect(filename);
}


#ifdef RW_WIN32



DynamicLibraryLoaderBase::DynamicLibraryLoaderBase(const std::string& fname)
{
    // Try to open the library now and get any error message.
    //h = LoadLibraryA((fname + getFileExtension()).c_str());
    h = LoadLibraryA((fname).c_str());

    //h = LoadLibraryA((fname).c_str());
    if (h == NULL)
    {
        LPTSTR buffer = NULL;
        std::cout<<"Ready to generate error message "<<GetLastError()<<std::endl;
        if(FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM,
                      NULL,             // Instance
                      GetLastError(),   // Message Number
                      0,                // Language
                      buffer,              // Buffer
                      0,                // Min/Max Buffer size
                      NULL))            // Arguments
        {
            RW_THROW(buffer);
        } else {
            RW_THROW("Unknown Error: Could not open library");
        }
    }
    else
    {
        _err = NULL;
    }
}


DynamicLibraryLoaderBase::~DynamicLibraryLoaderBase() {
    if (h != NULL)
        FreeLibrary(h);
}

bool DynamicLibraryLoaderBase::getSymbol(void** v,
                                         const char *sym_name) {
    // try extract a symbol from the library
    // get any error message is there is any

    if( h!=0 )
    {
        *v = (void*)GetProcAddress(h, sym_name);
        if (v != NULL)
          return true;
        else
        {
            LPTSTR buffer = NULL;
            FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
                      FORMAT_MESSAGE_FROM_SYSTEM |
                      FORMAT_MESSAGE_IGNORE_INSERTS,
                      NULL,             // Instance
                      GetLastError(),   // Message Number
                      0,                // Language
                      buffer,           // Buffer
                      0,                // Min/Max Buffer size
                      NULL);            // Arguments
            RW_WARN(buffer);
            return false;
        }
    }
    else
    {
        return false;
    }
}


#else //#ifdef RW_WIN32

#include <dlfcn.h>

rw::common::Ptr<Plugin> Plugin::loadDirect(const std::string& filename){
    void *handle = dlopen(filename.c_str(), RTLD_NOW /* RTLD_GLOBAL*/);
    char *err = dlerror();

    if (handle == NULL || err != NULL)
        RW_THROW("Unknown Error: Could not open library: "<<err);

    if (err != 0) { return NULL; }

    const char* func = "createplugin";

    //if (getSymbol((void**)&factory_func, func )) {

    void* (*factory_func)(void) = NULL;
    void **tmp = (void**)&factory_func;
    *tmp = dlsym( handle, func );
    err = dlerror();
    if( err == 0 ){
        Plugin *lplugin = (Plugin*)factory_func();
        // TODO set a handle on Plugin so that i is able to close the dll
        // dlclose(_handle);
        return rw::common::ownedPtr(lplugin);
    } else {
        RW_THROW("Error: Plugin is not valid! Unable to identify factory function in dynamic library: " << err);
    }
    return NULL;
}

#endif //#else

class LazyPlugin: public Plugin {
public:
    LazyPlugin(const std::string& id, const std::string& name, const std::string& version):
        Plugin(id,name,version)
    {
    }

    void setExtensionDescriptors(std::vector<Extension::Descriptor> descs){
        _descs = descs;
    }

    void setLibFile(const std::string& file){
        _libfile = file;
    }

    std::vector<Extension::Descriptor> getExtensionDescriptors(){ return _descs; }

    rw::common::Ptr<Extension> makeExtension(const std::string& id){
        Extension::Descriptor *desc=NULL;
        BOOST_FOREACH(Extension::Descriptor &desc_tmp, _descs){
            //std::cout << desc_tmp.id << "==" << id << std::endl;
            if(desc_tmp.id==id){
                desc = &desc_tmp;
                break;
            }
        }
        if(desc==NULL)
            RW_THROW("Not a valid id!");
        // now comes the loading part
        if(_srcPlugin==NULL)
            _srcPlugin = Plugin::load(_libfile);

        _descs = _srcPlugin->getExtensionDescriptors();
        return _srcPlugin->makeExtension(id);
    }

private:
    std::vector<Extension::Descriptor> _descs;
    rw::common::Ptr<Plugin> _srcPlugin;
    std::string _libfile;
};

rw::common::Ptr<Plugin> Plugin::loadLazy(const std::string& filename){
    using namespace boost::property_tree;
    // parse xml file
    ptree tree;
    read_xml(filename, tree);
    ptree plugin = tree.get_child("plugin");
    std::string id = plugin.get_child("<xmlattr>").get<std::string>("id");
    std::string name = plugin.get_child("<xmlattr>").get<std::string>("name");
    std::string version = plugin.get_child("<xmlattr>").get<std::string>("version");

    std::string runtimelib = plugin.get_child("runtime").get_child("<xmlattr>").get<std::string>("library");
    boost::filesystem::path libfile(runtimelib);
    if(!libfile.is_absolute()){
        std::string fname = boost::filesystem::path(filename).parent_path().string() + "/" + runtimelib;
        libfile = boost::filesystem::path(fname);
    }

    if(!exists(libfile))
        RW_THROW("The plugin file specified in \n" << filename << "\n does not exist.");

    std::vector<Extension::Descriptor> ext_descriptors;
    for (ptree::iterator p = plugin.begin(); p != plugin.end(); ++p) {
        if(p->first == "extension") {
            Extension::Descriptor extension;
            extension.id = p->second.get_child("<xmlattr>").get<std::string>("id");
            extension.name = p->second.get_child("<xmlattr>").get<std::string>("name");
            extension.point = p->second.get_child("<xmlattr>").get<std::string>("point");
            ext_descriptors.push_back(extension);
        }
    }

    rw::common::Ptr<LazyPlugin> lplugin =
            rw::common::ownedPtr( new LazyPlugin(id,name,version) );
    lplugin->setLibFile(libfile.string());
    lplugin->setExtensionDescriptors( ext_descriptors );
    return lplugin;
}
