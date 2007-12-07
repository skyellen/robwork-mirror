#ifndef QPCONTROLLER_H
#define QPCONTROLLER_H

#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <math/VelocityScrew6D.hpp>
#include <math/RPY.hpp>
#include <math/MatrixMath.hpp>
#include <vworkcell/Device.hpp>


class QPController {
 public:
  QPController(double h);
  virtual ~QPController();

    boost::numeric::ublas::vector<double> solve(const rw::math::VelocityScrew6D<>& tcp_vel, const rw::vworkcell::Device::Q& q, const rw::vworkcell::Device::Q& dq, rw::vworkcell::Device* device);

    boost::numeric::ublas::vector<double> simpleSolve(const rw::math::VelocityScrew6D<>& tcp_screw, const rw::vworkcell::Device::Q& q, rw::vworkcell::Device* device);
    // boost::numeric::ublas::vector<double> simpleSolve(const boost::numeric::ublas::vector<double>& tcp_screw, const rw::vworkcell::Device::Q& q, rw::vworkcell::Device* device);

 private:
    
};


#endif //#ifndef QPCONTROLLER_H
