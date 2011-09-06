#ifndef NETFTCOMMON_HPP
#define NETFTCOMMON_HPP

// RW
#include <rw/math/Vector3D.hpp>
#include <rw/common/macros.hpp>

namespace rwhw {
   typedef std::pair<rw::math::Vector3D<>, rw::math::Vector3D<> > Wrench3D;
   
   class IIRFilter {
      public:
         // Constructor initializes filter coefficients
         IIRFilter(const std::vector<double>& b,
                   const std::vector<double>& a) : _b(b),
                                                   _a(a) {}
         
         /**
          * Apply filter to an input, also using previous output
          * 
          * @param x input vector, most recent first: {x_i, x_i-1, ...}
          * @param y previous output vector, most recent first: {y_i-1, y_i-2, ...}
          * @return filtered measurement
          */
         inline double operator()(const std::vector<double>& x, const std::vector<double>& y) {
            // Get length and check
            const unsigned int len = x.size();
            if(len == 0 || y.size() != len-1)
               RW_THROW("Error: inconsistent vector sizes!");
            
            // Filter
            double yf = 0.0;
            for(unsigned int i = 0; i < len; ++i) {
               yf += _b[i]*x[i];
               if(i > 0)
                  yf -= _a[i]*y[i-1];
            }
            yf /= _a.front();
            
            return yf;
         }
         
      protected:
         // Private constructor
         IIRFilter() {}
         
         // Filter coefficients
         std::vector<double> _b, _a;
   };
   
   // Fifth order Butterworth filter with cutoff frequency at 0.2 rad/sample
   class IIRFilterB5W20 : public IIRFilter {
      public:
         IIRFilterB5W20() {
            const double barr[6] = {0.001282581078961, 0.006412905394804, 0.012825810789607,
                                    0.012825810789607, 0.006412905394804, 0.001282581078961};
            const double aarr[6] = {1.0, -2.975422109745683, 3.806018119320410,
                                    -2.545252868330465, 0.881130075437836, -0.125430622155356};;
            
            _b.assign(barr, barr+6);
            _a.assign(aarr, aarr+6);
         }
   };
}

#endif
