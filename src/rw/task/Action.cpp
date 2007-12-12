#include "Action.hpp"

using namespace rw::task;
using namespace std;


Action::Action(string name) : _name(name)
{
	
}

Action::~Action()
{
	
}


Property &Action::Properties()
{
	return _properties;
}
