#include "LuaPlugin.hpp"

#include <rwlibs/swig/lua/LuaState.hpp>
#include <rwslibs/swig/ScriptTypes.hpp>

using namespace rwslibs::swig;
using namespace rw::common;

RW_ADD_PLUGIN(LuaPlugin)


LuaPlugin::LuaPlugin():Plugin("RWSLuaPlugin", "RWSLuaPlugin", "0.1")
{
}

LuaPlugin::~LuaPlugin()
{
}

std::vector<rw::common::Extension::Descriptor> LuaPlugin::getExtensionDescriptors()
{
    std::vector<Extension::Descriptor> exts;
    exts.push_back(Extension::Descriptor("RWSLua","rwlibs.swig.LuaState.LuaLibrary"));

    // todo: add posible properties to the extension descriptor
    exts.back().getProperties().set<std::string>("ID", "rws");
    //exts.back().getProperties().set<std::string>("engineID", "ODE");

    return exts;
}
namespace {
struct RWSLuaLibrary: rwlibs::swig::LuaState::LuaLibrary {
	virtual const std::string getId(){ return "RWSLua"; }
	virtual bool initLibrary(rwlibs::swig::LuaState& state){
		rwslibs::swig::openLuaLibRWS( state.get() );
		// initialize variables

		state.runCmd("rws = rws.lua.rwstudio");

		state.runCmd("rwstudio = rws.getRobWorkStudio()");

		return true;
	};
};
}
rw::common::Ptr<rw::common::Extension> LuaPlugin::makeExtension(const std::string& str)
{
    if(str=="RWSLua"){
        Extension::Ptr extension = rw::common::ownedPtr( new Extension("RWSimLua","rwlibs.swig.LuaState.LuaLibrary",
                this, ownedPtr(new RWSLuaLibrary()) ) );

        // todo: add posible properties to the extension descriptor
        //exts.back().getProperties().set<std::string>(propid, value);
        extension->getProperties().set<std::string>("ID", "rwsim");
        return extension;
    }
    return NULL;
}

