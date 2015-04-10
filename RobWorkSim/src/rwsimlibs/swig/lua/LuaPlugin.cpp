#include "LuaPlugin.hpp"

#include <rwlibs/swig/lua/LuaState.hpp>

using namespace rwsim::swig;
using namespace rw::common;

RW_ADD_PLUGIN(LuaPlugin)

#ifdef __cplusplus
extern "C" {
#endif

    int luaopen_rwsim(lua_State* L); // declare the wrapped module

#ifdef __cplusplus
}
#endif


int rwsim::swig::openLuaLibRWSim(lua_State* L){
    return luaopen_rwsim(L);
}


LuaPlugin::LuaPlugin():Plugin("LuaPlugin", "LuaPlugin", "0.1")
{
}

LuaPlugin::~LuaPlugin()
{
}

std::vector<rw::common::Extension::Descriptor> LuaPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("RWSimLua","rwlibs.swig.LuaState.LuaLibrary"));

    // todo: add posible properties to the extension descriptor
    exts.back().getProperties().set<std::string>("ID", "rwsim");
    //exts.back().getProperties().set<std::string>("engineID", "ODE");

    return exts;
}
namespace {
struct RWSLuaLibrary: rwlibs::swig::LuaState::LuaLibrary {
	virtual const std::string getId(){ return "RWSimLua"; }
	virtual bool initLibrary(rwlibs::swig::LuaState& state){
		luaopen_rwsim( state.get() );
	    //state.runCmd("rwsim = rwsim.lua");

		return true;
	};
};
}
rw::common::Ptr<rw::common::Extension> LuaPlugin::makeExtension(const std::string& str)
{
    if(str=="RWSimLua"){
        Extension::Ptr extension = rw::common::ownedPtr( new Extension("RWSimLua","rwlibs.swig.LuaState.LuaLibrary",
                this, ownedPtr(new RWSLuaLibrary()) ) );

        // todo: add posible properties to the extension descriptor
        //exts.back().getProperties().set<std::string>(propid, value);
        extension->getProperties().set<std::string>("ID", "rwsim");
        return extension;
    }
    return NULL;
}

