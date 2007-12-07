#ifndef EKF_HPP
#define EKF_HPP

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <math/MatrixMath.hpp>

typedef boost::numeric::ublas::vector<double> Vector;
typedef boost::numeric::ublas::matrix<double> Matrix;
typedef boost::numeric::ublas::matrix_expression<double> MatrixExp;


class EKF {
    
public:
    /**
     * @brief Construct EKF with a state vector length \f$length\f$
     * @param length [in] length of state vector
     */
    EKF(size_t length);

    virtual ~EKF();

    /**
     * @brief Initializes state vector and covariance matrix
     * @param x [in] State vector
     * @param P [in] Covariance
     */
    void initialize(const Vector& x, const Matrix& P);

    /**
     * @brief Make a prediction
     * @param dt [in] time to predict ahead
     */
    void predict(double dt);  

    /**
     * @brief Updates states using new measurement and covariance
     * @param z [in] measurement
     * @param R [in] measurement noise covariance
     */
    void update(const Vector& z, const Matrix& R);
	
    Vector getState();

    Matrix getCovariance();


protected:
    virtual Vector process(const Vector& x, double dt) = 0; //f(x,u,w)

    virtual Vector measurement(const Vector& x) = 0; //h(x,v)

    virtual Matrix processJacobian(const Vector& x) = 0; //A
    
    virtual Matrix processNoiseJacobian(const Vector& x) = 0; //W

    virtual Matrix processNoiseCoVariance(const Vector& x) = 0; //Q    
    
    virtual Matrix measurementJacobian(const Vector& x) = 0; //H

    virtual Matrix measurementNoiseJacobian(const Vector& x) = 0; //V

    size_t _n;

private:
    Vector _x;
    Matrix _P;

};

#endif //#ifndef EKF_HPP
