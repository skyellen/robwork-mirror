#include "EKF.hpp"

#include <math/MatrixMath.hpp>

using namespace boost::numeric::ublas;

EKF::EKF(size_t length):
    _n(length),
    _x(length),
    _P(length, length)
{

}


EKF::~EKF() {

}

Vector EKF::getState() {
    return _x;
}

Matrix EKF::getCovariance() {
    return _P;
}

void EKF::initialize(const Vector& x, const Matrix& P) {
    _x = x;
    _P = P;
}

void EKF::predict(double dt) {
    Matrix A = processJacobian(_x);
    Matrix W = processNoiseJacobian(_x);
    Matrix Q = processNoiseCoVariance(_x); 

    _x = process(_x, dt);
          
    _P = prod(Matrix(prod(A,_P)),trans(A))+prod(Matrix(prod(W, Q)),trans(W));
}

void EKF::update(const Vector& z, const Matrix& R) {
    Matrix H = measurementJacobian(_x);
    Matrix V = measurementNoiseJacobian(_x);
    std::cout<<"Got H and V "<<std::endl;
    std::cout<<"H = ("<<H.size1()<<","<<H.size2()<<")"<<std::endl;
    std::cout<<"_P = ("<<_P.size1()<<","<<_P.size2()<<")"<<std::endl;
    std::cout<<"V = ("<<V.size1()<<","<<V.size2()<<")"<<std::endl;
    std::cout<<"R = ("<<R.size1()<<","<<R.size2()<<")"<<std::endl;
    Matrix mat = prod(Matrix(prod(H,_P)),trans(H))+prod(Matrix(prod(V,R)),trans(V));
    std::cout<<"Got mat"<<std::endl;
    Matrix matInv(mat.size1(), mat.size2());
    rw::math::InvertMatrix(mat, matInv);
    std::cout<<"Got mat Inv"<<std::endl;

    Matrix K = prod(Matrix(prod(_P,trans(H))),matInv);
    std::cout<<"Got K"<<std::endl;
    _x = _x+prod(K,(z-measurement(_x)));
    std::cout<<"Got x"<<std::endl;
    
    _P = prod((identity_matrix<double>(_n)-prod(K,H)),_P);
    std::cout<<"Got P"<<std::endl;
}
