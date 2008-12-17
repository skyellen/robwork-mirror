#ifndef PLUGIN_HPP_
#define PLUGIN_HPP_

#include <rw/kinematics/Frame.hpp>

#include <string>
class Plugin {
public:
    virtual std::string name() = 0;
    virtual rw::kinematics::Frame* getObj() = 0;
    virtual void setObj(rw::kinematics::Frame* obj) = 0;
};


#endif /*PLUGIN_HPP_*/
