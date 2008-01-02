#include "ClearanceOptimizer.hpp"

#include <rw/common/Property.hpp>
#include <rw/common/Timer.hpp>
#include <rw/math/Math.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rw/models/Accessor.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
using namespace rwlibs::pathoptimization;

const std::string ClearanceOptimizer::PROP_STEPSIZE = "StepSize";
const std::string ClearanceOptimizer::PROP_LOOPCOUNT = "LoopCount";
const std::string ClearanceOptimizer::PROP_MAXTIME = "MaxTime";



ClearanceOptimizer::ClearanceOptimizer(rw::models::WorkCell* workcell,
                                       rw::models::Device* device,
                                       const rw::kinematics::State& state,
                                       boost::shared_ptr<Metric<double> > metric, 
                                       boost::shared_ptr<ClearanceCalculator> clearanceCalculator):
	_workcell(workcell),
	_device(device),	
	_state(state),
	_metric(metric),
	_clearanceCalculator(clearanceCalculator),
	_stepsize(0.1)	
{
	_dof = device->getDOF();
	_qlower = _device->getBounds().first;
	_qupper = _device->getBounds().second;
	
    _propertymap.addProperty(boost::shared_ptr<PropertyBase>(new Property<double>(PROP_STEPSIZE, "Step Size", 0.1)));
    _propertymap.addProperty(boost::shared_ptr<PropertyBase>(new Property<int>(PROP_LOOPCOUNT, "Maximal Number of Loops", 20)));
    _propertymap.addProperty(boost::shared_ptr<PropertyBase>(new Property<double>(PROP_MAXTIME, "Maximal Time to use (seconds)", 200)));
	
}

ClearanceOptimizer::~ClearanceOptimizer() {
//    ClearanceOptimizer* cl = new ClearanceOptimizer(_workcell, _device, _state, _metric, _clearanceCalculator);
}


bool ClearanceOptimizer::isValid(const Q& q) {
	for (size_t i = 0; i<q.size(); i++) {
		if (q(i) < _qlower(i) || q(i) > _qupper(i))
			return false;
	}
	return true;
}

double ClearanceOptimizer::clearance(const Q& q) {
	_device->setQ(q, _state);
	double d = _clearanceCalculator->clearance(_state);	
	return d;		
}

Path ClearanceOptimizer::optimize(const Path& inputPath) {
    return optimize(inputPath,
                    _propertymap.getValue<double>(PROP_STEPSIZE),
                    _propertymap.getValue<int>(PROP_LOOPCOUNT),
                    _propertymap.getValue<double>(PROP_MAXTIME) );  
}

Path ClearanceOptimizer::optimize(const Path& inputPath, double stepsize, size_t maxcount, double maxtime) {
    _stepsize = stepsize;
	if (inputPath.size() <= 2)
		return inputPath;
	Timer timer;
	timer.reset();
	
	AugmentedPath path;	
	
	subDivideAndAugmentPath(inputPath, path);
	
	
	double newClearance = calcAvgClearance(path);
	double oldClearance = 0;
	size_t cnt = 0;
	while ( (cnt == 0 || cnt < maxcount) && (maxtime == 0 || timer.getTime() < maxtime)) {
	    std::cout<<".";
	    //std::cout<<"AvgClearance = "<<newClearance<<std::endl;
	    oldClearance = newClearance;
		AugmentedPath newPath = path;		
		Q dir = randomDirection();
		for (AugmentedPath::iterator it = ++(newPath.begin()); it != --(newPath.end()); ++it) {
			Q qnew = (*it).first + dir;
			if (isValid(qnew)) {			    
			    double newClearance = clearance(qnew);
			    if ((*it).second < newClearance) {
			        (*it).first = qnew;
			        (*it).second = newClearance;
			    }
			}
		} 
		
		path = validatePath(newPath, path);
		removeBranches(path);
		
		newClearance = calcAvgClearance(path);
		cnt++;
	}  
	
	
	
	Path result;
	for (AugmentedPath::iterator it = path.begin(); it != path.end(); ++it) {
	    result.push_back((*it).first);
	}
	return result;;

}

//Implements the ValidatePath from [1]
ClearanceOptimizer::AugmentedPath ClearanceOptimizer::validatePath(const AugmentedPath& newPath, const AugmentedPath& orgPath) {
    AugmentedPath result;    
        
    AugmentedPath::const_iterator it_new = newPath.begin();
    AugmentedPath::const_iterator it_next = ++newPath.begin();
    AugmentedPath::const_iterator it_org = ++orgPath.begin();
        
    while (it_next != newPath.end()) {    
        //std::cout<<"A"<<(*it_new).first<<" "<<(*it_next).first<<std::endl;
        result.push_back(*it_new);
        if (_metric->distance((*it_new).first, (*it_next).first) > _stepsize) {
            Q qint = interpolate((*it_new).first, (*it_next).first, 0.5);
            double clearanceInt = clearance(qint);
            if (clearanceInt > (*it_org).second) {
                result.push_back(AugmentedQ(qint, clearanceInt));
            } else {
                result.push_back(*it_org);
            }
        }
        it_new++;
        it_next++;
        it_org++;        
    }
    result.push_back(*it_new);   
    
    return result;
}

//Implements the RemoveBranches algorithm from [1]
void ClearanceOptimizer::removeBranches(AugmentedPath& path) {
   AugmentedPath::iterator it1 = path.begin();
   AugmentedPath::iterator it2 = path.begin();
   it2++;
   AugmentedPath::iterator it3 = path.begin();
   it3++;
   it3++;   
   while (it3 != path.end()) {      
       if (_metric->distance((*it1).first, (*it3).first) < _stepsize) {
           it3 = path.erase(it2);
           it2 = it3; it2--;
           if (it2 == path.begin()) {
               it1 = it2;
               it2++; 
               it3++;               
           } else {
               it1 = it2; 
               it1--;
           }
           if (it1 != path.begin()) {
               it1--;
               it2--;
               it3--;
           }               
       } else {
           it1++;
           it2++;
           it3++;
       }
   }
}


double ClearanceOptimizer::calcAvgClearance(const AugmentedPath& path) {
    double sum = 0;
    for (AugmentedPath::const_iterator it = path.begin(); it != path.end(); ++it) {
        sum += (*it).second;
    }
    sum /= (double)path.size();
    return sum;
}

//Interpolates between to nodes. At some point we experiment with other ways of
//interpolating, which is why it is left as a seperate method.
Q ClearanceOptimizer::interpolate(const Q& q1, const Q& q2, double ratio) {
    return q1*ratio+(1-ratio)*q2;
}


void ClearanceOptimizer::subDivideAndAugmentPath(const Path& path, AugmentedPath& result) {
	Path::const_iterator itstart = path.begin();
	Path::const_iterator itnext = path.begin();
	itnext++; 
	//while (it != --(path.end())) {
	for ( ; itnext != path.end(); itstart++, itnext++) {
	 //   std::cout<<".";
		//Q start = *it;
		//Q next = *(++it);
	    Q start = *itstart;
	    Q next = *itnext;
		Q delta = next-start;
		const double fraction = _metric->distance(delta)/_stepsize;
		double divisions = (int)ceil(fraction);
		for (int i = 0; i<(int)divisions; i++) {
		    Q q = start + delta*(double)i/divisions;
		    //result.push_back(AugmentedQ(q, 0.1));
			result.push_back(AugmentedQ(q, clearance(q)));
		}				
	}	
//	std::cout<<"SubDivide Finished "<<result.size()<<std::endl;
	result.push_back(AugmentedQ(path.back(), clearance(path.back())));
}

//Calculates a random direction.
//To get an equal distribution in all directions (at least for a 2-norm based metric) a 
//do-while loop is used, which checks that the direction vector is less than stepsize, 
//before scaling it. Otherwise we might get an uneven distribution.
//Assuming a relationship between the output of the metric and the selected stepsize, the method
//uses stepsize instead of 1 as bounds on the random numbers. 
Q ClearanceOptimizer::randomDirection() {
	Q q(_dof);
	do { 
	    for (size_t i = 0; i<_dof; i++) 
	        q(i) = Math::Ran(-_stepsize, _stepsize);
	} while (_metric->distance(q) > _stepsize);
	
	q *= _stepsize/_metric->distance(q);
	return q;
}


PropertyMap& ClearanceOptimizer::getPropertyMap() {
    return _propertymap;
}
