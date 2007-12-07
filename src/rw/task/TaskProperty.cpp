#include "TaskProperty.hpp"

using namespace rw::task;
using namespace rw::common;
using namespace std;

TaskProperty::TaskProperty()
{
	_properties = new PropertyMap();
}

TaskProperty::~TaskProperty()
{
//	delete _properties;
}

void TaskProperty::addProperty(std::string key, double val)
{
	_properties->setValue<double>(key,val);

}

void TaskProperty::addProperty(std::string key, int val)
{
	_properties->setValue<int>(key,val);
}

void TaskProperty::addProperty(std::string key, std::string val)
{
	_properties->setValue<string>(key,val);
}

bool TaskProperty::getProperty(std::string key, double &val)
{
	if(!_properties->has(key))
		return false;
	val = _properties->getValue<double>(key);
	return true;
}

bool TaskProperty::getProperty(std::string key, int &val)
{	
	if(!_properties->has(key))
		return false;
	val = _properties->getValue<int>(key);
	return true;
}

bool TaskProperty::getProperty(std::string key, std::string &val)
{
	if(!_properties->has(key))
		return false;
	val = _properties->getValue<string>(key);
	return true;
}

