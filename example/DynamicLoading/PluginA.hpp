#ifndef PLUGINA_HPP_
#define PLUGINA_HPP_

#include <rwlibs/dll/FactoryMacro.hpp>

#include "Plugin.hpp"
#include <string>
#include <iostream>

#include <rw/kinematics/MovableFrame.hpp>

class PluginA: public Plugin {
public:
    PluginA();

    ~PluginA();

    std::string name();

    rw::kinematics::Frame* getObj();
    void setObj(rw::kinematics::Frame* obj);
private:
	rw::kinematics::MovableFrame _obj;
};


#endif /*PLUGINA_HPP_*/
