#include "PluginB.hpp"

#include <rw/kinematics/MovableFrame.hpp>
using namespace rw::kinematics;

PluginB::PluginB():_obj("objB") {
    std::cout<<"PluginB Constructor"<<std::endl;
}


PluginB::~PluginB()
{
}

std::string PluginB::name() {
    return "PluginB";
}


Frame* PluginB::getObj(){
	return &_obj;
}

void PluginB::setObj(Frame* obj){
	MovableFrame *vobj = dynamic_cast<MovableFrame*>(obj);
	if( vobj==NULL )
		std::cout << "PluginB: cannot dynamic cast object to MovableFrame!!" << std::endl;
	else
		std::cout << "PluginB: loaded obj!" << std::endl;

}


DLL_FACTORY_METHOD(PluginB);

