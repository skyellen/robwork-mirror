/*
    Copyright 2013 <copyright holder> <email>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
        limitations under the License.
*/


#ifndef MODRUSSEL_NLP_HPP
#define MODRUSSEL_NLP_HPP

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/vector.hpp>


#include "BeamGeometry.hpp"
#include "BeamObstaclePlane.hpp"


#include "IpTNLP.hpp"

namespace rwlibs {
namespace softbody {
class ModRussel_NLP : public Ipopt::TNLP {

public:
    ModRussel_NLP(
        boost::shared_ptr< BeamGeometry > geomPtr,
        boost::shared_ptr< BeamObstaclePlane > obstaclePtr,
        rw::math::Transform3D<> planeTbeam,
        const std::vector<int> & integralIndices
    );
    virtual ~ModRussel_NLP();

    /**@name Overloaded from TNLP */
    //@{
    /** Method to return some info about the nlp */
    virtual bool get_nlp_info ( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style );

    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info ( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                   Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u );

    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point ( Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                      bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                      Ipopt::Index m, bool init_lambda,
                                      Ipopt::Number* lambda );

    /** Method to return the objective value */
    virtual bool eval_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value );

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f );

    /** Method to return the constraint residuals */
    virtual bool eval_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g );
    void eval_g_point ( int pIdx, int gBase, Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g );

    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                              Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                              Ipopt::Number* values );
    
    void eval_jac_g_point ( int pIdx, int gBase,
                            Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                              Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                              Ipopt::Number* values );

    /** Method to return:
     *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
     *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
     */
    virtual bool eval_h ( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                          bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                          Ipopt::Index* jCol, Ipopt::Number* values );


    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution ( Ipopt::SolverReturn status,
                                     Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U,
                                     Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda,
                                     Ipopt::Number obj_value,
                                     const Ipopt::IpoptData* ip_data,
                                     Ipopt::IpoptCalculatedQuantities* ip_cq );

private:
    ModRussel_NLP ( const ModRussel_NLP& );
    ModRussel_NLP& operator= ( const ModRussel_NLP& );
    
public:
    boost::shared_ptr< BeamGeometry > getGeometry(void) const;
    boost::shared_ptr< BeamObstaclePlane > getObstacle(void) const;
    rw::math::Transform3D<> get_planeTbeam(void) const;
    const boost::numeric::ublas::vector<double> &getSolution(void) const;
    
private:
    boost::shared_ptr< BeamGeometry > _geomPtr;
    boost::shared_ptr< BeamObstaclePlane > _obstaclePtr;
    rw::math::Transform3D<> _planeTbeam;
    
    boost::numeric::ublas::vector<double>   _a;
    boost::numeric::ublas::vector<double>   _da;
    
    boost::numeric::ublas::vector<double>   _x;
    
    std::vector<int> _integralIndices;
};
}}

#endif // MODRUSSEL_NLP_HPP
