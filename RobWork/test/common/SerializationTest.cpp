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

#include "../TestSuiteConfig.hpp"

#include <iostream>
#include <rw/common/Ptr.hpp>

#include <rw/common/INIArchive.hpp>
#include <rw/common/BINArchive.hpp>

#include <rw/common/Serializable.hpp>

#include <rw/math/Math.hpp>
#include <rw/math/Vector2D.hpp>

using namespace rw::common;
using namespace rw::math;


class myOutputArchive {
public:
	OutputArchive &_oa;
	myOutputArchive(OutputArchive &oa):_oa(oa){};

	template<class T>
	inline void doWrite(const T& object, const std::string& id){
		// the data method must have an implementation of load/save and if not then we try the generic write
		// method which could provide a solution by the implementation itself
		writeImpl(object, id);
	}

	/**
	 * this function should only be called if the object inherits from Serializable
	 */
    template<class T>
    inline void writeImpl(T& object, const std::string& id,
    		typename boost::enable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL)
    {
    	object.write(_oa, id);
    }

    /**
     * This function should be called if the object does not inherit from Serializable and if the
     * object is not a pointer
     */
    template<typename T>
    inline void writeImpl(T& object, const std::string& id,
    		typename boost::disable_if_c<boost::is_base_of<Serializable, T>::value, T>::type* def=NULL,
    		typename boost::disable_if_c<boost::is_pointer<T>::value, T>::type* defptr=NULL)
    {
    	//BOOST_MPL_ASSERT_MSG(boost::is_reference<T>::value, "type T cannot be of type reference!" , (T) );

		//if( boost::is_floating_point<T>::value || boost::is_integral<T>::value){
		//	T* val = new T;
		//	write(*val, id);
		//}

		// try and use overloaded method
		serialization::write(object, _oa, id);
    	//serialization::write(sd, _oa, id);
    }

};



/*
namespace rw{ namespace common {
    class OutputArchive; class InputArchive;
namespace serialization {
	template<>
	void write(const rw::math::Vector2D<double>& tmp, rw::common::OutputArchive& oar, const std::string& id){}
	template<>
	void write(const rw::math::Vector2D<float>& tmp, rw::common::OutputArchive& oar, const std::string& id){}
	template<>
	void read(rw::math::Vector2D<double>& tmp, rw::common::InputArchive& iar, const std::string& id){}
	template<>
	void read(rw::math::Vector2D<float>& tmp, rw::common::InputArchive& iar, const std::string& id){}
}}} // end namespaces
*/


struct SerializationData: public rw::common::Serializable {
	SerializationData(){
		data1.push_back(0.3);
		data1.push_back(0.5);
		data1.push_back(0.6);
		data1.push_back(0.7);
		data2 = rw::math::Math::ranQ( Q(3,0,0,0), Q(3,1,1,1) );
		data3 = rw::math::Math::ran();
		data4 = rw::math::Math::ranI(0, 10000);
		data5 = rw::math::Math::ranI(0, 10000);
		data6 = rw::math::Math::ranI(0, 255);
		Eigen::MatrixXd highPrecision(1,3);
		highPrecision << 1.2345678901234, 4.7021e10, 49.1000948272e-19;
		data9 = Eigen::MatrixXd(2,123); // use enough numbers that INI line length is exceeded!
		data9 << highPrecision, Eigen::MatrixXd::Random(1,120),
				Eigen::MatrixXd::Random(1,3), Eigen::MatrixXd::Random(1,120);
	}

	bool operator== (const SerializationData& rhs) const {
	    bool result = true;
	    result &= data1.size() == rhs.data1.size();
	    if(data1.size() == rhs.data1.size()){
	        for(size_t i=0;i<data1.size();i++){
	            result &= data1[i] == rhs.data1[i];
	        }
	    }
	    const double epsilon = 0.00001;
	    result &= (data2-rhs.data2).normInf()<epsilon;
	    result &= fabs(data3-rhs.data3)<epsilon;
	    result &= data4 == rhs.data4;
	    result &= data5 == rhs.data5;
	    result &= data6 == rhs.data6;
	    result &= (data7 - rhs.data7).normInf()<epsilon;
	    result &= data9.rows() == rhs.data9.rows();
	    result &= data9.cols() == rhs.data9.cols();
	    if (result)
	    	result &= (data9-rhs.data9).cwiseAbs().maxCoeff() < epsilon;
	    return result;
	}


	void read(InputArchive& iarchive, const std::string& id_tmp){
		std::string id = "sdata";
		if(!id_tmp.empty())
			id=id_tmp;
        iarchive.read(data1,"data1");
        iarchive.read(data2,"data2");

        iarchive.readEnterScope("primitives");
        iarchive.read(data3,"data3");
        iarchive.read(data4,"data4");
        iarchive.read(data5,"data5");
        iarchive.read(data6,"data6");
        iarchive.read(data7,"data7");
        iarchive.read(data8,"data8");
        iarchive.readLeaveScope("primitives");
        iarchive.read(data9,"data9");
	}

	void write(OutputArchive& oarchive, const std::string& id_tmp) const{
		std::string id = "sdata";
		if(!id_tmp.empty())
			id = id_tmp;

		oarchive.write(data1,"data1");
		oarchive.write(data2,"data2");

		oarchive.writeEnterScope("primitives");
		oarchive.write(data3,"data3");
		oarchive.write(data4,"data4");
		oarchive.write(data5,"data5");
		oarchive.write(data6,"data6");
		oarchive.write(data7,"data7");
		oarchive.write(data8,"data8");
		oarchive.writeLeaveScope("primitives");
		oarchive.write(data9,"data9");
	}


	std::vector<double> data1;
	rw::math::Q data2;

	double data3;
	uint32_t data4;
	uint16_t data5;
	boost::int8_t data6;

	rw::math::Vector3D<double> data7;
	rw::math::Vector2D<double> data8;

	Eigen::MatrixXd data9;
};

// {0.069, 0.204, 0.022, 0, 90, 90}
// {0.075, 0.190, -0.003, 0, 90, 90}


BOOST_AUTO_TEST_CASE( INIArchiveTest )
{
	SerializationData sdata, sdata_in;

	{
        INIArchive iniarchive;
        iniarchive.open("testfile.ini");
        BOOST_CHECK( iniarchive.isOpen() );
        iniarchive.write( sdata, "sdata" );
        iniarchive.close();
	}

	{
	    INIArchive iniarchive;
	    iniarchive.open("testfile.ini");
	    BOOST_CHECK( iniarchive.isOpen() );
	    iniarchive.read( sdata_in, "sdata");
	    iniarchive.close();
	}

	BOOST_CHECK( sdata == sdata_in );

	{
        INIArchive(std::cout).write(sdata,"sdata");
        INIArchive(std::cout).write(sdata_in,"sdata_in");
        INIArchive(std::cout) << sdata;
	}

}

BOOST_AUTO_TEST_CASE( BINArchiveTest )
{
    SerializationData sdata, sdata_in;

    INIArchive(std::cout).write(sdata,"sdata");
    {
        BINArchive iniarchive;
        iniarchive.open("testfile.bin");
        BOOST_CHECK( iniarchive.isOpen() );
        iniarchive.write( sdata, "sdata" );
        iniarchive.close();
    }

    {
        BINArchive iniarchive;
        iniarchive.open("testfile.bin");
        BOOST_CHECK( iniarchive.isOpen() );
        iniarchive.read( sdata_in, "sdata");
        iniarchive.close();
    }

    BOOST_CHECK( sdata == sdata_in );

    {
        BINArchive(std::cout).write(sdata,"sdata");
        BINArchive(std::cout).write(sdata_in,"sdata_in");
        BINArchive(std::cout) << sdata;
    }


}


BOOST_AUTO_TEST_CASE( BoostXMLParser )
{


}

