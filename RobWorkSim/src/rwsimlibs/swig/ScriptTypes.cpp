
#include "ScriptTypes.hpp"

#include <boost/foreach.hpp>
#include <map>

using namespace rwsim::swig;

rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc_internal;

std::map<std::string, rwsim::simulator::ThreadSimulator::Ptr> sim_instances_internal;

rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> rwsim::swig::getDynamicWorkCell(){
    return dwc_internal;
}

void rwsim::swig::setDynamicWorkCell(rw::common::Ptr<rwsim::dynamics::DynamicWorkCell> dwc){
    dwc_internal = dwc;
}


void rwsim::swig::addSimulatorInstance(rw::common::Ptr<ThreadSimulator> sim, const std::string& id)
{
	sim_instances_internal[id] = sim;
}

rw::common::Ptr<ThreadSimulator> rwsim::swig::getSimulatorInstance(const std::string& id)
{
	return sim_instances_internal[id];
}

void rwsim::swig::removeSimulatorInstance(const std::string& id)
{
	sim_instances_internal[id] = NULL;
}

std::vector<std::string> rwsim::swig::getSimulatorInstances()
{
	typedef std::map<std::string, rw::common::Ptr<ThreadSimulator> >::value_type PairVals;
	std::vector<std::string> result;
	BOOST_FOREACH(PairVals pair, sim_instances_internal){
		if(pair.second!=NULL)
			result.push_back( pair.first );
	}
	return result;
}

rw::common::Ptr<ThreadSimulator> rwsim::swig::getSimulatorInstance(){
	typedef std::map<std::string, rw::common::Ptr<ThreadSimulator> >::value_type PairVals;
	BOOST_FOREACH(PairVals pair, sim_instances_internal){
		if(pair.second!=NULL)
			return  pair.second;
	}
	return NULL;
}
