/*
Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,

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


#ifndef RWLIBS_SOFTBODY_MODRUSSEL_NLP_HPP
#define RWLIBS_SOFTBODY_MODRUSSEL_NLP_HPP

/**
 * @file ModRussel_NLP.hpp
 */

#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/vector.hpp>


#include "BeamGeometry.hpp"
#include "BeamObstaclePlane.hpp"


#include "IpTNLP.hpp"

namespace rwlibs { namespace softbody {
/** @addtogroup softbody */
/*@{*/
    
/**
* @brief Implementation of the Modified Russel beam problem, using the IPOPT TNLP structure
**/
class ModRussel_NLP : public Ipopt::TNLP {

public:
    ModRussel_NLP (
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


    /** Method to return:
     *   1) The structure of the jacobian (if "values" is NULL)
     *   2) The values of the jacobian (if "values" is not NULL)
     */
    virtual bool eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x,
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
    
    //@}

private:
    void eval_g_point ( int pIdx, int gBase, Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g );
    void eval_jac_g_point ( int pIdx, int gBase,
                            Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                            Ipopt::Number* values );

    bool eval_f_elastic ( Ipopt::Index n, const Ipopt::Number* x, Ipopt::Number& obj_value );    

private:
    ModRussel_NLP ( const ModRussel_NLP& );
    ModRussel_NLP& operator= ( const ModRussel_NLP& );

public:
    /**
     * @brief returns the geometry used
     *
     * @return the geometry used
     **/
    boost::shared_ptr< BeamGeometry > getGeometry ( void ) const;
    
    /**
     * @brief returns the obstacle used
     *
     * @return the obstacle used
     **/
    boost::shared_ptr< BeamObstaclePlane > getObstacle ( void ) const;
    
    /**
     * @brief get the plane to beam transform
     *
     * @return the plane to beam transform
     **/
    rw::math::Transform3D<> get_planeTbeam ( void ) const;
    
    
    /**
     * @brief returns the solution vector
     *
     * @return solution vector
     **/
    const boost::numeric::ublas::vector<double> &getSolution ( void ) const;
    
    
    /**
     * @brief returns the total elastic energy for the last solution
     * 
     * returns the total elastic energy for the last solution. The energy returned is in units of [kg mm^ 2 / s^2]
     *
     * @return total elastic energy 
     **/
    double getEnergyElastic ( void ) const;

    /**
     * @brief sets the starting guess for the optimization
     *
     * @param xinituser vector of deformation angles to be used as starting guess
     **/    
    void setStartingGuess ( const boost::numeric::ublas::vector< double >& xinituser );


private:
    boost::shared_ptr< BeamGeometry > _geomPtr;
    boost::shared_ptr< BeamObstaclePlane > _obstaclePtr;
    rw::math::Transform3D<> _planeTbeam;

    boost::numeric::ublas::vector<double>   _a; // vector of M angles
    boost::numeric::ublas::vector<double>   _da; // vector of M angle derivatives

    boost::numeric::ublas::vector<double>   _x; // solution vector, M-1

    std::vector<int> _integralIndices; // vector of indices at which to check for penetration

    boost::numeric::ublas::vector<double>   _xinit; // starting guess

    double _Ee; // total elastic energy for last valid solution
};

    /* @} */
}
}

#endif // MODRUSSEL_NLP_HPP
