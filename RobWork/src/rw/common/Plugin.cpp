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

std::vector<std::string> Plugin::getExtensionPointIDs() {
	std::vector<std::string> ids;
	const std::vector<Extension::Descriptor> desc = getExtensionDescriptors();
	for (std::size_t i = 0; i < desc.size(); i++)
		ids.push_back(desc[i].id);
	return ids;
}

rw::common::Ptr<Plugin> Plugin::load(const std::string& filename){
    boost::filesystem::path file(filename);
    if(!exists(file))
        RW_THROW("The file does not exist: "<< filename );
    //std::cout << rw::common::StringUtil::toUpper(file.extension().string()) << std::endl;
#if(BOOST_FILESYSTEM_VERSION==2)
    if( rw::common::StringUtil::toUpper(file.extension())==".XML"){
#else
    if( rw::common::StringUtil::toUpper(file.extension().string())==".XML"){
#endif

    	return loadLazy(filename);
    }
    return loadDirect(filename);
}


#ifdef RW_WIN32

#include <windows.h>
#include <winbase.h>

rw::common::Ptr<Plugin> Plugin::loadDirect(const std::string& filename){
	HINSTANCE h = LoadLibraryA((filename).c_str());
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

    // try extract a symbol from the library
    // get any error message if there is any
    void* (*factory_func)(void) = NULL;
    void **tmp = (void**)&factory_func;
	const char* sym_name = "createplugin";
    *tmp = (void*)GetProcAddress(h, sym_name);
	if (*tmp == NULL){
		LPTSTR buffer = NULL;
		const DWORD errId = GetLastError();
		const DWORD res = FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER |
				  FORMAT_MESSAGE_FROM_SYSTEM |
				  FORMAT_MESSAGE_IGNORE_INSERTS,
				  NULL,             // Instance
				  errId,            // Message Number
				  0,                // Language
				  (LPSTR)&buffer,   // Buffer
				  0,                // Min/Max Buffer size
				  NULL);            // Arguments
		if (!res) {
			RW_WARN("FormatMessage returned error: " << GetLastError() << "!");
		} else {
			RW_WARN("Error " << errId << ": " << buffer);
		}
		return NULL;
	}

	// call factory function and create plugin
    Plugin *lplugin = (Plugin*)factory_func();
    // TODO set a handle on Plugin so that it is able to close the dll
    //FreeLibrary(h);



    return rw::common::ownedPtr(lplugin);
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
        // TODO set a handle on Plugin so that it is able to close the dll
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

#if(BOOST_FILESYSTEM_VERSION==2)
	if( !libfile.has_root_path() ){
		std::string fname = boost::filesystem::path(filename).parent_path().string() + "/" + runtimelib;
		libfile = boost::filesystem::path(fname);
	}
#else
    if(!libfile.is_absolute()){
        std::string fname = boost::filesystem::path(filename).parent_path().string() + "/" + runtimelib;
        libfile = boost::filesystem::path(fname);
    }
#endif

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
