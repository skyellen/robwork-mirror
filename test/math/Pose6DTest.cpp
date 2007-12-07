#include <rw/math/Rotation3D.hpp>
#include <rw/math/Pose6D.hpp>

#include <boost/test/unit_test.hpp>

#include <iostream>

using namespace rw::math;

void Pose6DTest(){

    Pose6D<double> p(1.1,2.2,3.3,4.4,5.5,6.6);
    
    std::cout<<"p(0) = "<<p(0)<<std::endl;
    std::cout<<"p(0)-1.1"<<(p(0)-1.1)<<std::endl;
    BOOST_CHECK(p(0) == 1.1);
    BOOST_CHECK(p(1) == 2.2);
    BOOST_CHECK(p(2) == 3.3);
    BOOST_CHECK(fabs(p(3) - 4.4)<1e-15);
    BOOST_CHECK(fabs(p(4) - 5.5)<1e-15);
    BOOST_CHECK(fabs(p(5) - 6.6)<1e-15);

    
       
    Pose6D<float> pf = cast<float>(p);
    for (size_t i = 0; i<6; i++)
	BOOST_CHECK(pf(i) == (float)p(i));

}
