#ifndef RWS_LUASTATE_HPP_
#define RWS_LUASTATE_HPP_

#include <sstream>
#include <rw/common/StringUtil.hpp>
#include <rws/RobWorkStudio.hpp>

struct lua_State;

class LuaState {
public:
    LuaState():_rws(NULL){}

    void reset();

    int runCmd(const std::string& str);

    void setRobWorkStudio(rws::RobWorkStudio* rws){
        _rws = rws;
    }

    typedef boost::function<void(lua_State*)> AddLibraryCB;
    void addLibrary(AddLibraryCB cb);

    lua_State* get() { return _lua.get();}

private:
    rw::common::Ptr<struct lua_State> _lua;
    rws::RobWorkStudio* _rws;
    std::vector<AddLibraryCB> _libraryCBs;
};

#endif /* SHAREDLUASTATE_HPP_ */
