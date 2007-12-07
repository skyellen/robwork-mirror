#include "Action.hpp"

using namespace rw::task;
using namespace rw::common;
using namespace std;


Action::Action(string name) : _name(name)
{
	
}

Action::~Action()
{
	
}


TaskProperty &Action::Properties()
{
	return _properties;
}
