#ifndef RW_SANDBOX_INTERPOLATOR_HPP
#define RW_SANDBOX_INTERPOLATOR_HPP


#include <rw/math/Q.hpp>

namespace rw {
namespace sandbox {
    
template <class T>
class Interpolator
{
public:
	virtual ~Interpolator() {}
	
	virtual T x(double t) const = 0;
	virtual T dx(double t) const = 0;
	virtual T ddx(double t) const = 0;
	
	virtual double getLength() const = 0;
};

} //end namespace sandbox
} //end namespace rw

#endif //RW_SANDBOX_INTERPOLATOR_HPP
