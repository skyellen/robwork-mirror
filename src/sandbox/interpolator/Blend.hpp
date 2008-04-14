#ifndef RW_SANDBOX_BLEND_HPP
#define RW_SANDBOX_BLEND_HPP

#include <rw/math/Q.hpp>

namespace rw {
namespace sandbox {


template <class T>
class Blend
{
public:
	virtual ~Blend() {}
	
	virtual T x(double t) = 0;
	virtual T dx(double t) = 0;
	virtual T ddx(double t) = 0;
	
	virtual double tau1() = 0;
	virtual double tau2() = 0;
	
};

} //end namespace sandbox
} //end namespace rw

#endif //RW_SANDBOX_BLEND_HPP
