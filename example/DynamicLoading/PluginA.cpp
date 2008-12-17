#include "PluginA.hpp"

#include <rw/kinematics/MovableFrame.hpp>
using namespace rw::kinematics;


PluginA::PluginA():_obj("objA") {
    std::cout<<"PluginA Constructor"<<std::endl;
}


PluginA::~PluginA()
{
}

std::string PluginA::name() {
    return "PluginA";
}

Frame* PluginA::getObj(){
	return &_obj;
}

void PluginA::setObj(Frame* obj){
	MovableFrame *vobj = dynamic_cast<MovableFrame*>(obj);
	if( vobj==NULL )
		std::cout << "PluginA: cannot dynamic cast object to MovableFrame!!" << std::endl;
	else
		std::cout << "PluginA: loaded obj!" << std::endl;
}


DLL_FACTORY_METHOD(PluginA);
