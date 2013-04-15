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


#include "ModRussel_NLP.hpp"

using namespace rwlibs::softbody;

ModRussel_NLP::ModRussel_NLP() {

}

ModRussel_NLP::~ModRussel_NLP() {

}



bool ModRussel_NLP::get_nlp_info ( Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, Ipopt::TNLP::IndexStyleEnum& index_style ) {
return true;
}



bool ModRussel_NLP::get_bounds_info ( Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u ) {
return true;
}



bool ModRussel_NLP::get_starting_point ( Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda ) {
return true;
}



bool ModRussel_NLP::eval_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value ) {
return true;
}


bool ModRussel_NLP::eval_grad_f ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f ) {
return true;
}



bool ModRussel_NLP::eval_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g ) {
return true;
}



bool ModRussel_NLP::eval_jac_g ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
return true;
}


bool ModRussel_NLP::eval_h ( Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values ) {
    return true;
}

void ModRussel_NLP::finalize_solution ( Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq ) {

}

