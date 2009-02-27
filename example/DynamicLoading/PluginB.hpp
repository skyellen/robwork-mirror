#ifndef PLUGINB_HPP_
#define PLUGINB_HPP_

#include "Plugin.hpp"
#include <iostream>
#include <rwlibs/dll/FactoryMacro.hpp>

#include <rw/kinematics/MovableFrame.hpp>


class PluginB: public Plugin {
public:
    PluginB();

    virtual ~PluginB();

    std::string name();

    rw::kinematics::Frame* getObj();
    void setObj(rw::kinematics::Frame* obj);
private:
	rw::kinematics::MovableFrame _obj;

};


#endif /*PLUGINB_HPP_*/
