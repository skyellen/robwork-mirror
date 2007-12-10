#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Rotation3D.hpp>
#include <rw/math/EAA.hpp>
#include <rw/math/Constants.hpp>

#include <boost/test/unit_test.hpp>

using namespace boost::numeric::ublas;
using namespace rw::math;

void LinearAlgebraTest(){
  EAA<> eaa(Vector3D<>(1.0, 0.0, 0.0), Pi/4.0);
  Rotation3D<> r = eaa.toRotation3D();

  matrix<double> minv(3, 3);
  LinearAlgebra::InvertMatrix(r.m(), minv);

  BOOST_CHECK(norm_inf(inverse(r).m() - minv) < 1e-10);

  minv = LinearAlgebra::PseudoInverse(r.m());
  BOOST_CHECK(norm_inf(inverse(r).m() - minv) <= 1e-10);


  matrix<double> A = zero_matrix<double>(4);
  A(0,0) = 1;
  A(1,1) = 2;
  A(2,2) = 3;
  A(3,3) = 4;
  A(3,0) = 1;
  A(0,3) = 1;

  std::cout<<"Check Symmetric Matrix EigenValue Decomposition..."<<std::endl;
  std::pair<matrix<double>, vector<double> > val1 = LinearAlgebra::EigenDecompositionSymmetric(A);
  for (size_t i = 0; i<A.size1(); i++) {
      matrix_column<matrix<double> > x(val1.first, i);
      double l = val1.second(i);
      vector<double> r1 = l*x;
      vector<double> r2 = prod(A,x);
      BOOST_CHECK(norm_inf(r1-r2) < 1e-12);

  }


  std::cout<<"Check Matrix EigenValue Decomposition..."<<std::endl;
  A(1,2) = 5; //make it unsymmetric
  std::pair<matrix<double>, vector<std::complex<double> > > val2 = LinearAlgebra::EigenDecomposition(A);
  for (size_t i = 0; i<A.size1(); i++) {
      matrix_column<matrix<double> > x(val2.first, i);
      double l = real(val2.second(i));
      vector<double> r1 = l*x;
      vector<double> r2 = prod(A,x);
      BOOST_CHECK(norm_inf(r1-r2) < 1e-12);

  }




  /*    std::cout<<"mc = "<<mc<<std::endl;
  std::cout<<"mc.l"<<mc*real(val.second(0))<<std::endl;
  std::cout<<"A.mc = "<<prod(A, mc)<<std::endl;

  std::cout<<"First = "<<val.first<<std::endl;
  std::cout<<"Second = "<<val.second<<std::endl;
  */

}
