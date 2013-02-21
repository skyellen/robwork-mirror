/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RW_MATH_INTERIORPOINTOPTIMIZER_HPP
#define RW_MATH_INTERIORPOINTOPTIMIZER_HPP

#include <rw/math/Math.hpp>
#include <rw/math/Q.hpp>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace rw {
namespace math {


class InteriorPointOptimizer
{
public:

    typedef boost::function<void(const boost::numeric::ublas::vector<double>& x,
                                 double& f,
                                 boost::numeric::ublas::vector<double>& df,
                                 boost::numeric::ublas::matrix<double>& ddf) > ObjectFunction;

	//typedef void*(rw::math::Q& q) ObjectFunction;

    typedef boost::function<void(const boost::numeric::ublas::vector<double>& x,
                                 size_t no,
                                 boost::numeric::ublas::vector<double>& g,
                                 boost::numeric::ublas::matrix<double>& dg,
                                 boost::numeric::ublas::matrix<double>& ddq) > ConstraintFunction;

    InteriorPointOptimizer(size_t n,
                           size_t m,
                           ObjectFunction objectFunction,
                           ConstraintFunction constraintFunction);

    virtual ~InteriorPointOptimizer();


    boost::numeric::ublas::vector<double> solve(const boost::numeric::ublas::vector<double>& x_init);

    void setAccuracy(double accuracy);
    double getAccuracy();

    void verify_user_defined_objective_and_constraints();


protected:
    InteriorPointOptimizer(size_t n, size_t m);

    void initialize();

    virtual void objectFunction(const boost::numeric::ublas::vector<double>& x,
                                double &f,
                                boost::numeric::ublas::vector<double> &df,
                                boost::numeric::ublas::matrix<double> &ddf);

    virtual void constraintFunction(const boost::numeric::ublas::vector<double>& x,
                                    int i,
                                    boost::numeric::ublas::vector<double> &a,
                                    boost::numeric::ublas::matrix<double> &da,
                                    boost::numeric::ublas::matrix<double> &dda);

private:
    ObjectFunction compute_f_info_EXT;
    ConstraintFunction compute_con_info_i_EXT;


    void choleskySolve(int n_e,
                       int bw,
                       boost::numeric::ublas::matrix<double> &A,
                       boost::numeric::ublas::vector<double> &b,
                       boost::numeric::ublas::vector<double> &x);


    void compute_f_info(boost::numeric::ublas::matrix<double> &A,
                        boost::numeric::ublas::vector<double> &RHS);

    void compute_con_info(boost::numeric::ublas::matrix<double> &A,
                          boost::numeric::ublas::vector<double> &RHS);

    void merit_info(boost::numeric::ublas::vector<double> &x,
                    boost::numeric::ublas::vector<double> &s,
                    double &phi,
                    double &eta);

    void Dmerit_info(boost::numeric::ublas::vector<double> &x,
                     boost::numeric::ublas::vector<double> &s,
                     boost::numeric::ublas::vector<double> &dx,
                     boost::numeric::ublas::vector<double> &ds,
                     double &Dphi,
                     double &eta);

    void update(boost::numeric::ublas::vector<double> &x,
                boost::numeric::ublas::vector<double> &dx,
                boost::numeric::ublas::vector<double> &s,
                boost::numeric::ublas::vector<double> &z);

    const size_t N;
    const size_t M;
    double _accuracy;
    boost::numeric::ublas::vector<double> _x;
    boost::numeric::ublas::vector<double> _s;
    boost::numeric::ublas::vector<double> _z;
    double _mu, _eta;

    // objective and derivatives
    double _f;
    boost::numeric::ublas::vector<double> _df;
    boost::numeric::ublas::matrix<double> _ddf;

    // constraints and derivatives (second derivative only stored for one constraint at a time)
    boost::numeric::ublas::vector<double> _a;
    boost::numeric::ublas::matrix<double> _da;
    boost::numeric::ublas::matrix<double> _dda;


};

} //end namespace math
} //end namespace rw

#endif //end include guard
