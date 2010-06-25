#ifndef RW_ROBWORK_HPP
#define RW_ROBWORK_HPP

#include <RobWorkConfig.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/common/Log.hpp>
#include <rw/plugin/PluginRepository.hpp>

namespace rw {

class RobWork
{
public:
    RobWork(void);
    ~RobWork(void);

    rw::plugin::PluginRepository& getPluginRepository() {
        return _pluginRepository;
    }

    rw::common::Log& getLog() {
        return _log;
    }

    std::string getVersion() const {
        return RW_VERSION;
    }


private:
    rw::plugin::PluginRepository _pluginRepository;

    rw::common::Log _log;
};

typedef rw::common::Ptr<RobWork> RobWorkPtr;

} //end namespace rw


#endif //#ifndef RW_ROBWORK_HPP
