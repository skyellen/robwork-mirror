#ifndef QPCONTROLLER_H
#define QPCONTROLLER_H

#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <math/VelocityScrew6D.hpp>
#include <vworkcell/Device.hpp>

class QPController {
 public:
  QPController(double h);
  virtual ~QPController();

  boost::numeric::ublas::vector<double> solve(const rw::math::VelocityScrew6D<>& tcp_vel, const rw::vworkcell::Device::Q& q, const rw::vworkcell::Device::Q& dq);

 private:
};


#endif //#ifndef QPCONTROLLER_H
