#ifndef RW_SANDBOX_TRAJECTORY_HPP
#define RW_SANDBOX_TRAJECTORY_HPP

#include <rw/math/Q.hpp>
#include <rw/common/macros.hpp>

#include "Interpolator.hpp"
#include "Blend.hpp"

namespace rw {
namespace sandbox {

template <class T>
class Trajectory
{
public:
	Trajectory() {}
	virtual ~Trajectory() {}
	
	T x(double t) {
	    Segment segment = getSegment(t);
/*	    std::cout<<"Segment "<<segment.startTime<<" "<<segment.endTime<<" "<<segment.endBlend<<std::endl;
	    std::cout<<"endBlend = "<<segment.endBlend<<std::endl;
	    std::cout<<"Interpolatorlength  "<<segment.interpolator->getLength()<<std::endl;
	    std::cout<<"Tau = "<<segment.endBlend->getTau2()<<std::endl;*/
	    
	    
	    if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
	        t = t - segment.t1;
	        return segment.blend1->x(t + segment.blend1->tau1());
	    } else if (segment.blend2 != NULL && 
            segment.t2 - t < segment.blend2->tau1()) {
	        t = t - segment.t2 + segment.blend2->tau1();
	        return segment.blend2->x(t);
	    } else {
	        t = t - segment.t1;
	        return segment.interpolator->x(t);
	    }
	}
	
	
	T dx(double t) {
	    Segment segment = getSegment(t);
        if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
            t = t - segment.t1;
            return segment.blend1->dx(t + segment.blend1->tau1());
        } else if (segment.blend2 != NULL && 
            segment.t2 - t < segment.blend2->tau1()) {
            t = t - segment.t2 + segment.blend2->tau1();
            return segment.blend2->dx(t);
        } else {
            t = t - segment.t1;
            return segment.interpolator->dx(t);
        }
	}
	
	
	T ddx(double t) {
        Segment segment = getSegment(t);
        if (segment.blend1 != NULL && t - segment.t1 < segment.blend1->tau2()) {
            t = t - segment.t1;
            return segment.blend1->ddx(t + segment.blend1->tau1());
        } else if (segment.blend2 != NULL && 
            segment.t2 - t < segment.blend2->tau1()) {
            t = t - segment.t2 + segment.blend2->tau1();
            return segment.blend2->ddx(t);
        } else {
            t = t - segment.t1;
            return segment.interpolator->ddx(t);
        }
	}
	
	void add(rw::sandbox::Interpolator<T>* interpolator) {
	    add(interpolator, NULL);
	}
	
	void add(rw::sandbox::Interpolator<T>* interpolator,
	         rw::sandbox::Blend<T>* blend) {
        Segment segment;
        segment.interpolator = interpolator;
        segment.blend2 = blend;

        if (_segments.size() > 0) {
            segment.blend1 = _segments.back().blend2;
            segment.t1 = _segments.back().t2;
        } else {
            segment.blend1 = NULL;
            segment.t1 = 0;            
        }
       segment.t2 = segment.t1 + interpolator->getLength();
       _segments.push_back(segment);	    
	}
	
	double getLength() {
	    if (_segments.size() == 0)
	        return 0;
	    else
	        return _segments.back().t2;
	}
	
private:
    
    struct Segment {
        Blend<T>* blend1;
        Blend<T>* blend2;
        Interpolator<T>* interpolator;
        double t1;
        double t2;
    };
    
    std::vector<Segment> _segments;
    
    Segment segmentSearch(double t, double index, double delta) {
        Segment& segment = _segments[(int)index];
        if (segment.t1 > t)
            return segmentSearch(t, index - delta/2.0, delta/2.0);
        else if (segment.t2 < t)
            return segmentSearch(t, index + delta/2.0, delta/2.0);
        else 
            return segment;
    }
    
    Segment getSegment(double t) {
        //Perform Binary search for the right segment
        size_t n = _segments.size();
        if (n>0) {
            if (_segments.back().t2 < t || t < 0)
                RW_THROW("The requested time is outside the interval of the Trajectory");
            
            return segmentSearch(t, n/2.0, n/2.0);
        } else {
            RW_THROW("Cannot request values from an empty Trajectory");
        }
    }
	
};


} //end namespace sandbox
} //end namespace rw


#endif //RW_SANDBOX_TRAJECTORY_HPP
