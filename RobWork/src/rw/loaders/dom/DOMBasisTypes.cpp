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

#include "DOMBasisTypes.hpp"

#include <rw/common/macros.hpp>
#include <rw/common/StringUtil.hpp>
#include <rw/math/LinearAlgebra.hpp>
#include <rw/models/WorkCell.hpp>
#include <sstream>
#include <map>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>

#include <rw/common/DOMElem.hpp>

using namespace rw::math;
using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::loaders;
using namespace rw;

//Definition of Identifiers used in the XML format
const std::string DOMBasisTypes::QId("Q");
const std::string DOMBasisTypes::Vector3DId("Vector3D");
const std::string DOMBasisTypes::Vector2DId("Vector2D");

const std::string DOMBasisTypes::Rotation3DId("Rotation3D");
const std::string DOMBasisTypes::RPYId("RPY");
const std::string DOMBasisTypes::EAAId("EAA");
const std::string DOMBasisTypes::QuaternionId("Quaternion");

const std::string DOMBasisTypes::Rotation2DId("Rotation2D");
const std::string DOMBasisTypes::Rotation2DAngleId("Rotation2DAngle");
const std::string DOMBasisTypes::Transform3DId("Transform3D");
const std::string DOMBasisTypes::Transform2DId("Transform2D");
const std::string DOMBasisTypes::VelocityScrew6DId("VelocityScrew6D");

const std::string DOMBasisTypes::PosId("Pos");
const std::string DOMBasisTypes::MatrixId("Matrix");

const std::string DOMBasisTypes::LinearId("Linear");
const std::string DOMBasisTypes::AngularId("Angular");

const std::string DOMBasisTypes::StateId("State");
const std::string DOMBasisTypes::QStateId("QState");
const std::string DOMBasisTypes::TreeStateId("TreeState");

const std::string DOMBasisTypes::BooleanId("Boolean");
const std::string DOMBasisTypes::DoubleId("Double");
const std::string DOMBasisTypes::FloatId("Float");
const std::string DOMBasisTypes::IntegerId("Integer");
const std::string DOMBasisTypes::StringId("String");
const std::string DOMBasisTypes::StringListId("StringList");
const std::string DOMBasisTypes::StringPairId("StringPair");
const std::string DOMBasisTypes::IntListId("IntList");
const std::string DOMBasisTypes::DoubleListId("DoubleList");

const std::string DOMBasisTypes::UnitAttributeId("unit");

namespace {
struct UnitMap {
public:
    std::map<std::string, double> _map;

    UnitMap(){
        _map["mm"] = 1.0/1000.0;
        _map["cm"] = 1.0/100.0;
        _map["m"] = 1;
        _map["inch"] = 0.0254;

        _map["deg"] = Deg2Rad;
        _map["rad"] = 1;

        _map["m/s"] = 1;
        _map["cm/s"] = 1.0/100.0;
        _map["mm/s"] = 1.0/1000.0;

        _map["m/s^2"] = 1;
        _map["cm/s^2"] = 1.0/100.0;
        _map["mm/s^2"] = 1.0/1000.0;

        _map["deg/s"] = Deg2Rad;
        _map["rad/s"] = 1;

        _map["deg/s^2"] = Deg2Rad;
        _map["rad/s^2"] = 1;
    };
    UnitMap(const std::map<std::string, double>& map):
    _map(map)
    {
    }

    ~UnitMap(){
    }
};
}

//const DOMBasisTypes::UnitMap DOMBasisTypes::_Units;
const UnitMap _Units;



double DOMBasisTypes::getUnit(const std::string key) {
    std::map<std::string, double>::const_iterator it = _Units._map.find(key);
    if (it == _Units._map.end())
        RW_THROW("Invalid Unit Attribute "<<key);
    return (*it).second;

}



namespace {
    double readUnit(DOMElem::Ptr element) {
    	std::string attrval = element->getAttributeValue(DOMBasisTypes::UnitAttributeId, "");
    	if(!attrval.empty())
    		return DOMBasisTypes::getUnit(attrval);
    	return 1;
    }

    void checkHeader(DOMElem::Ptr element, const std::string id) {
        if ( !element->isName(id) )
            RW_THROW("Expected \""<< id <<"\" got "<< element->getName() );
    }

    template <class T>
    inline T readVectorStructure(DOMElem::Ptr element, bool doCheckHeader, const std::string id) {

        if (doCheckHeader)
            checkHeader(element, id);

        double scale = readUnit(element);
        std::vector<double> elements = element->getValueAsDoubleList();

        T result;
        if( elements.size()!=result.size() )
        	RW_THROW("Parse error: in element \"" << element->getName()
        			  << "\" nr of elements must be \""  << result.size() << "\" got \"" << elements.size()<< "\"");

        for (size_t i = 0; i<elements.size(); i++) {
            result(i) = scale*elements[i];
        }
        return result;
    }


} //end internal namespace

Q DOMBasisTypes::readQ(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, QId);

    std::vector<double> values = element->getValueAsDoubleList();
    Q q(values.size());
    for (size_t i = 0; i<values.size(); ++i)
        q(i) = values[i];

    return q;
}

Vector3D<> DOMBasisTypes::readVector3D(DOMElem::Ptr element, bool doCheckHeader) {
    return readVectorStructure<Vector3D<> >(element, doCheckHeader, Vector3DId);
}


Vector2D<> DOMBasisTypes::readVector2D(DOMElem::Ptr element, bool doCheckHeader) {
    return readVectorStructure<Vector2D<> >(element, doCheckHeader, Vector2DId);
}


RPY<> DOMBasisTypes::readRPY(DOMElem::Ptr element, bool doCheckHeader) {
    return readVectorStructure<RPY<> >(element, doCheckHeader, RPYId);
}

EAA<> DOMBasisTypes::readEAA(DOMElem::Ptr element, bool doCheckHeader) {
    return readVectorStructure<EAA<> >(element, doCheckHeader, EAAId);}

Quaternion<> DOMBasisTypes::readQuaternion(DOMElem::Ptr element, bool doCheckHeader) {
    Quaternion<> qua = readVectorStructure<Quaternion<> >(element, doCheckHeader, QuaternionId);
    qua.normalize();
    return qua;
}

Rotation3D<> DOMBasisTypes::readRotation3D(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation3DId);

    std::vector<double> values = element->getValueAsDoubleList();
    // we are switching to quaternions instead... rotation3d are simply to error prone to precision errors

    if (values.size() == 4){
        Quaternion<> quat(values[0], values[1], values[2], values[3]);
        quat.normalize();
        Rotation3D<> rot = quat.toRotation3D();

        using namespace boost::numeric::ublas;
		while(fabs(rot.e().determinant()-1.0)>0.00001  ){
        	Eigen::MatrixXd u, v;
        	Eigen::VectorXd w;

            std::cout.precision(17);
            std::cout << rot << std::endl;
            RW_WARN("Parse of Rotation3D failed. A rotation 3d must be an "
				"orthogonal matrix with determinant of 1! det=" << rot.e().determinant());
            LinearAlgebra::svd(rot.e(), u, w ,v);
            Eigen::MatrixXd res = u * v.transpose();
            rot = Rotation3D<>(res);
            return rot;
        }


        return rot;
    } else if(values.size() == 9){

        Rotation3D<> rot(values[0], values[1], values[2],
                            values[3], values[4], values[5],
                            values[6], values[7], values[8]);

        using namespace boost::numeric::ublas;
		while(fabs(rot.e().determinant()-1.0)>0.00001  ){

        	Eigen::MatrixXd u, v;
        	Eigen::VectorXd w;

            std::cout.precision(17);
            std::cout << rot << std::endl;
            RW_WARN("Parse of Rotation3D failed. A rotation 3d must be an "
				"orthogonal matrix with determinant of 1! det=" << rot.e().determinant());
            LinearAlgebra::svd(rot.e(), u, w ,v);
            Eigen::MatrixXd res = u * v.transpose();
            rot = Rotation3D<>(res);

        }
        return rot;
    }

    RW_THROW("Expected 9 or 4 (quaternion) floating points for Rotation3D. Only "<<values.size()<<" values found");
}

Rotation2D<> DOMBasisTypes::readRotation2D(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Rotation2DId);

    std::vector<double> values = element->getValueAsDoubleList();
    if (values.size() != 4)
        RW_THROW("Expected 4 floating points for Rotation3D. Only "<<values.size()<<" values found");

    return Rotation2D<>(values[0], values[1], values[2], values[3]);
}

Rotation3D<> DOMBasisTypes::readRotation3DStructure(DOMElem::Ptr element) {
    if (element->isName(Rotation3DId))
        return readRotation3D(element, false);
    if (element->isName(RPYId))
        return readRPY(element, false).toRotation3D();
    if (element->isName(EAAId))
        return readEAA(element, false).toRotation3D();
    if (element->isName(QuaternionId))
        return readQuaternion(element, false).toRotation3D();

    RW_THROW("Unable to find match \""<<element->getName()<<"\" with (Rotation3D|RPY|EAA|Quaternion)");
    return Rotation3D<>();
}

Rotation2D<> DOMBasisTypes::readRotation2DStructure(DOMElem::Ptr element) {
    if (element->isName(Rotation2DId))
        return readRotation2D(element, false);
    if (element->isName(Rotation2DAngleId)) {
        double angle = readDouble(element, false);
        return Rotation2D<>(angle);
    }
    
    RW_THROW("Unable to find match \""<<element->getName()<<"\" with (Rotation2D|Angle)");
    return Rotation2D<>();
}


Transform3D<> DOMBasisTypes::readTransform3D(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Transform3DId);

    Vector3D<> position(0,0,0);
    Rotation3D<> rotation(Rotation3D<>::identity());

    {
    	std::vector<double> values = element->getValueAsDoubleList();
		if (values.size() == 12) {
			rotation(0,0) = values[0];
			rotation(0,1) = values[1];
			rotation(0,2) = values[2];
			rotation(1,0) = values[4];
			rotation(1,1) = values[5];
			rotation(1,2) = values[6];
			rotation(2,0) = values[8];
			rotation(2,1) = values[9];
			rotation(2,2) = values[10];

			position(0) = values[3];
			position(1) = values[7];
			position(2) = values[11];
		}
    }

    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
		if (child->isName( MatrixId)) {
			std::vector<double> values = child->getValueAsDoubleList();
			if (values.size() != 12)
				RW_THROW("Expected   <Matrix> with 12 doubles when parsing Transform3D. Found "<<values.size()<<" values");
			rotation(0,0) = values[0];
			rotation(0,1) = values[1];
			rotation(0,2) = values[2];
			rotation(1,0) = values[4];
			rotation(1,1) = values[5];
			rotation(1,2) = values[6];
			rotation(2,0) = values[8];
			rotation(2,1) = values[9];
			rotation(2,2) = values[10];

			position(0) = values[3];
			position(1) = values[7];
			position(2) = values[11];
		} else if (child->isName( PosId)) {
			position = readVector3D(child, false);
		} else {
			rotation = readRotation3DStructure(child);
		}
    }
    //rotation.normalize();
    return Transform3D<>(position, rotation);
}


Transform2D<> DOMBasisTypes::readTransform2D(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, Transform3DId);

    Vector2D<> position(0,0);
    Rotation2D<> rotation(Rotation2D<>::identity());

    {
    	std::vector<double> values = element->getValueAsDoubleList();
		if (values.size() == 6) {
			rotation(0,0) = values[0];
			rotation(0,1) = values[1];
			rotation(1,0) = values[2];
			rotation(1,1) = values[3];			

			position(0) = values[4];
			position(1) = values[5];			
		}
    }

    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
		if (child->isName(MatrixId)) {
			std::vector<double> values = child->getValueAsDoubleList();
			if (values.size() != 6)
				RW_THROW("Expected <Matrix> with 6 doubles when parsing Transform2D. Found "<<values.size()<<" values");
			rotation(0,0) = values[0];
			rotation(0,1) = values[1];
			rotation(1,0) = values[2];
			rotation(1,1) = values[3];

			position(0) = values[4];
			position(1) = values[5];			
		} else if (child->isName( PosId)) {
			position = readVector2D(child, false);
		} else {
			rotation = readRotation2D(child);
		}
    }
    //rotation.normalize();
    return Transform2D<>(position, rotation);
}


VelocityScrew6D<> DOMBasisTypes::readVelocityScrew6D(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, VelocityScrew6DId);

    Vector3D<> linear(0, 0, 0);
    EAA<> angular(0, 0, 0);
    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
		if (child->isName( PosId)) {
			linear = readVector3D(child, false);
		} else if (child->isName( EAAId)) {
			angular = readEAA(child, false);
		} else {
			RW_THROW("Unknown element \""<<child->getName()<<"\" specified in VelocityScrew6D");
		}
    }

    return VelocityScrew6D<>(linear, angular);

}

std::vector<double> DOMBasisTypes::readDoubleList(DOMElem::Ptr element, bool doCheckHeader ){
    return element->getValueAsDoubleList();
}

std::vector<int> DOMBasisTypes::readIntList(DOMElem::Ptr element, bool doCheckHeader ){
    std::vector<double> res2 = element->getValueAsDoubleList();
    std::vector<int> res(res2.size());
    for(size_t i=0;i<res2.size();i++)
        res[i] = (int)res2[i];
    return res;
}

/**
 * @brief Returns rw::kinematics::State<> element read from \b element
 *
 * Read in \b element and returns a rw::kinematics::State corresponding to the content.
 * If \b doCheckHeader = true it checks that the elements tag name matches State.
 * If the name does not an exception is thrown.
 *
 * @param element [in] Element to read
 * @param doCheckHeader [in] True if the header name should be checked
 * @return The element read
 */
rw::kinematics::State DOMBasisTypes::readState(DOMElem::Ptr element, WorkCell::Ptr workcell, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StateId);

    State result = workcell->getDefaultState();

    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){

		if (child->isName( QStateId)) {
			Q q = readQ(child, false);
			if (result.size() != q.size())
				RW_THROW("Length of State loaded does not match workcell");
			for (size_t i = 0; i<q.size(); i++)
				result(i) = q(i);

		} else if (child->isName( TreeStateId)) {
			std::vector<StringPair> dafs = readStringPairs(element);
			for (std::vector<StringPair>::iterator it = dafs.begin(); it != dafs.end(); ++it) {
				Frame* daf = workcell->findFrame((*it).first);
				Frame* parent = workcell->findFrame((*it).second);
				if (daf == NULL)
					RW_THROW("Unable to locate frame named \""<<(*it).first);
				if (parent == NULL)
					RW_THROW("Unable to locate frame named \""<<(*it).second);

				daf->attachTo(parent, result);
			}
		} else {
			RW_THROW("Unknown element \""<<child->getName()<<"\" specified in State");
		}
    }
    return result;
}

std::string DOMBasisTypes::readString(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StringId);

    return element->getValue();
}

std::vector<std::string> DOMBasisTypes::readStringList(DOMElem::Ptr element) {
    return element->getValueAsStringList();
}

//typedef std::pair<std::string,std::string> StringPair;

StringPair DOMBasisTypes::readStringPair(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, StringPairId);

    const std::vector<std::string> result = readStringList(element);

    if (result.size() != 2)
        RW_THROW("Expected 2 string in StringPair but found"<<result.size());

    return std::make_pair(result[0], result[1]);

}

std::vector<StringPair> DOMBasisTypes::readStringPairs(DOMElem::Ptr element) {
    std::vector<StringPair> result;
    BOOST_FOREACH(DOMElem::Ptr child, element->getChildren() ){
    	if (child->isName( StringPairId)) {
			std::string str = readString(child);
			std::vector<std::string> strings = StringUtil::words(str);
			if (strings.size() != 2)
				RW_THROW("Expected two string elements found "<<strings.size()<<" in \""<<str<<"\"");
			result.push_back(std::make_pair(strings[0], strings[1]));
		}
    }
    return result;
}

double DOMBasisTypes::readDouble(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, DoubleId);
    return element->getValueAsDouble();
}

float DOMBasisTypes::readFloat(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, FloatId);
    return (float)element->getValueAsDouble();
}

int DOMBasisTypes::readInt(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, IntegerId);

    return element->getValueAsInt();
}

bool DOMBasisTypes::readBool(DOMElem::Ptr element, bool doCheckHeader) {
    if (doCheckHeader)
        checkHeader(element, BooleanId);

    std::string str = element->getValue();
    bool res = false;
    try {
    	res = boost::lexical_cast<bool>(str);
    } catch (...){
    	if(str=="true"){
    		return true;
    	} else if(str=="false"){
    		return false;
    	} else {
    		RW_THROW("Parse error: Could not parse bool, expected true,false,1 or 0 got \""<< str << "\"");
    	}
    }
    return res;
}


namespace {
    template <class T>
    std::string createStringFromArray(const T& v, size_t n) {
        std::ostringstream str;
        str.unsetf(std::ios::floatfield);            // floatfield not set
        str.precision(16);
        for (size_t i = 0; i<n; ++i) {
            str<<v(i);
            if (i != n-1)
                str<<" ";
        }
        return str.str();
    }

    template <class T>
    std::string createStringFromArray(const T& v) {
        return createStringFromArray<T>(v, v.size());
    }

    template <class T>
    std::string createStringFromVector(const T& v, size_t n) {
        std::ostringstream str;
        str.unsetf(std::ios::floatfield);            // floatfield not set
        str.precision(16);
        for (size_t i = 0; i<n; ++i) {
            str<<v[i];
            if (i != n-1)
                str<<" ";
        }
        return std::string(str.str());
    }

    template <class T>
    std::string createStringFromVector(const T& v) {
        return createStringFromVector<T>(v, v.size());
    }
}

DOMElem::Ptr DOMBasisTypes::write(int val, DOMElem::Ptr elem, bool addHeader) {
	if(addHeader)
		elem->setName(IntegerId);

	std::stringstream sstr;
	sstr<<val;

	elem->setValue( sstr.str() );
	return elem;

}

DOMElem::Ptr DOMBasisTypes::write(double val, DOMElem::Ptr elem, bool addHeader) {
	if(addHeader)
		elem->setName(DoubleId);

	std::stringstream sstr;
	sstr<<val;

	elem->setValue( sstr.str() );
	return elem;

}


DOMElem::Ptr DOMBasisTypes::write(const std::string& str, DOMElem::Ptr elem, bool addHeader) {
	if(addHeader)
		elem->setName(StringId);

	elem->setValue( str );
	return elem;

}

DOMElem::Ptr DOMBasisTypes::write(const Q& val, DOMElem::Ptr elem, bool addHeader){
	if(addHeader)
		elem->setName(QId);

	elem->setValue( createStringFromArray(val) );
	return elem;
}


DOMElem::Ptr DOMBasisTypes::write(const Vector3D<>& val, DOMElem::Ptr elem, bool addHeader) {
	if(addHeader)
		elem->setName(Vector3DId);

	elem->setValue( createStringFromArray(val) );
	return elem;

}

DOMElem::Ptr DOMBasisTypes::write(const Vector2D<>& val, DOMElem::Ptr elem, bool addHeader) {
	if(addHeader)
		elem->setName(Vector2DId);

	elem->setValue( createStringFromArray(val) );
	return elem;

}



DOMElem::Ptr DOMBasisTypes::write(const Transform2D<>& val, DOMElem::Ptr elem, bool addHeader){
	if(addHeader)
		elem->setName(Transform2DId);

	const Rotation2D<> r = val.R();
	const Vector2D<> p = val.P();
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(17);
    str<<r(0,0)<<" "<<r(0,1)<<" "<< p(0) <<" " ;
    str<<r(1,0)<<" "<<r(1,1)<<" "<< p(1) ;

	elem->setValue( str.str() );
	return elem;
}

DOMElem::Ptr DOMBasisTypes::write(const Transform3D<>& val, DOMElem::Ptr elem, bool addHeader){
	if(addHeader)
		elem->setName(Transform3DId);

	const Rotation3D<> r = val.R();
	const Vector3D<> p = val.P();
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(17);

    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" "<< p(0) <<" " ;
    str<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" "<< p(1) <<" " ;
    str<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2)<<" "<< p(2) <<" " ;
	elem->setValue( str.str() );
	return elem;
}

DOMElem::Ptr DOMBasisTypes::write(const Eigen::MatrixXd& val, DOMElem::Ptr elem, bool addHeader){
	if(addHeader)
		elem->setName(MatrixId);

	// we save dimension in the first 2 values
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(17);
    str << val.cols() << " " << val.rows();
    for(int y=0;y<val.rows();y++)
    	for(int x=0;x<val.cols();x++)
    		str<< " " << val(x,y);


	elem->setValue( str.str() );
	return elem;
}


Eigen::MatrixXd DOMBasisTypes::readMatrix( DOMElem::Ptr elem ){
	std::vector<double> res = elem->getValueAsDoubleList();
	int cols = (int)res[0];
	int rows = (int)res[1];
	Eigen::MatrixXd m(cols, rows);
	for(int y=0;y<rows;y++)
		for(int x=0;x<cols;x++)
			m(x,y) = res[2+x+y*cols];
	return m;
}


DOMElem::Ptr DOMBasisTypes::createElement(const std::string& id, const std::string& value, DOMElem::Ptr doc) {
	DOMElem::Ptr element = doc->addChild(id);
	element->setValue( value );
	return element;
 }

DOMElem::Ptr DOMBasisTypes::createQ(const Q& q, DOMElem::Ptr doc) {
    return createElement(QId, createStringFromArray(q), doc);
}

DOMElem::Ptr DOMBasisTypes::createVector3D(const Vector3D<>& v, DOMElem::Ptr doc) {
    return createElement(Vector3DId, createStringFromArray(v), doc);
}

DOMElem::Ptr DOMBasisTypes::createVector2D(const Vector2D<>& v, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(Vector2DId);
    element->setValue(createStringFromArray(v) );
    return element;
}

DOMElem::Ptr DOMBasisTypes::createRPY(const RPY<>& v, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(RPYId);
    element->setValue(createStringFromArray(v, 3));

    return element;
}

DOMElem::Ptr DOMBasisTypes::createEAA(const EAA<>& v, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(EAAId);
    element->setValue(createStringFromArray(v, 3));

    return element;
}

DOMElem::Ptr DOMBasisTypes::createQuaternion(const Quaternion<>& q, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(QuaternionId);
    element->setValue(createStringFromArray(q, 4));

    return element;
}






DOMElem::Ptr DOMBasisTypes::createRotation3D(const Rotation3D<>& r, DOMElem::Ptr doc) {
    //DOMElem::Ptr element = doc->addChild(Rotation3DId);

    // check if rotation is proper orthogonal before saving it
    //RW_ASSERT( fabs(LinearAlgebra::det( target.R().m() ))-1.0 < 0.00000001 );
	double detVal = r.e().determinant();
    if( fabs(detVal-1.0) > 0.0000001 ){
        RW_WARN("A rotation matrix that is being streamed does not have a determinant of 1, det="<<detVal << ", difference: " << fabs(detVal-1.0));
    }

    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(17);
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(0,2)<<" ";
    str<<r(1,0)<<" "<<r(1,1)<<" "<<r(1,2)<<" ";
    str<<r(2,0)<<" "<<r(2,1)<<" "<<r(2,2);
    return createElement(Rotation3DId, std::string(str.str()), doc);
}

DOMElem::Ptr DOMBasisTypes::createRotation2D(const Rotation2D<>& r, DOMElem::Ptr doc) {
    std::ostringstream str;
    str.unsetf(std::ios::floatfield);            // floatfield not set
    str.precision(17);
    str<<r(0,0)<<" "<<r(0,1)<<" "<<r(1,0)<<" "<<r(1,1);
    return createElement(Rotation2DId, std::string(str.str()), doc);
}

DOMElem::Ptr DOMBasisTypes::createTransform3D(const Transform3D<>& t, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(Transform3DId);
    createElement(PosId, createStringFromArray(t.P()), element);
    createRotation3D(t.R(), element);
    return element;
}

DOMElem::Ptr DOMBasisTypes::createTransform2D(const rw::math::Transform2D<>& t, rw::common::DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(Transform2DId);
    createElement(PosId, createStringFromArray(t.P()), element);
    createRotation2D(t.R(), element);
    return element;
}

DOMElem::Ptr DOMBasisTypes::createVelocityScrew6D(const VelocityScrew6D<>& v, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(VelocityScrew6DId);
    createElement(PosId, createStringFromArray(v.linear()), element);
    createEAA(v.angular(), element);
	return element;
	//return createElement(VelocityScrew6DId, createStringFromArray(v, 6), doc);
}

DOMElem::Ptr DOMBasisTypes::createIntList(const std::vector<int>& ints, DOMElem::Ptr doc){
    return createElement(IntListId, createStringFromVector(ints), doc);
}

DOMElem::Ptr DOMBasisTypes::createDoubleList(const std::vector<double>& doubles, DOMElem::Ptr doc){
    return createElement(DoubleListId, createStringFromVector(doubles), doc);
}

DOMElem::Ptr DOMBasisTypes::createQState(const rw::kinematics::State& state, DOMElem::Ptr doc) {
    return createElement(QStateId, createStringFromArray<State>(state, state.size()), doc);
}

DOMElem::Ptr DOMBasisTypes::createBoolean(bool value, DOMElem::Ptr doc) {
    return createElement(BooleanId, boost::lexical_cast<std::string>(value), doc);
}

DOMElem::Ptr DOMBasisTypes::createDouble(double value, DOMElem::Ptr doc) {
    return createElement(DoubleId, boost::lexical_cast<std::string>(value), doc);
}

DOMElem::Ptr DOMBasisTypes::createFloat(float value, DOMElem::Ptr doc) {
    return createElement(FloatId, boost::lexical_cast<std::string>(value), doc);
}


DOMElem::Ptr DOMBasisTypes::createInteger(int value, DOMElem::Ptr doc) {
    return createElement(IntegerId, boost::lexical_cast<std::string>(value), doc);
}



DOMElem::Ptr DOMBasisTypes::createString(const std::string& str, DOMElem::Ptr doc) {
    return createElement(StringId, str, doc);
}

DOMElem::Ptr DOMBasisTypes::createStringList(const std::vector<std::string>& strings, DOMElem::Ptr doc){
    DOMElem::Ptr element = doc->addChild(StringListId);
    std::stringstream sstr;
    sstr << strings[0];
    for(size_t i=1; i<strings.size();i++){
    	sstr << ";" << strings[i];

    }
    element->setValue(sstr.str());
    return element;
}


DOMElem::Ptr DOMBasisTypes::createStringPair(const std::string& first, const std::string& second, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(StringPairId);
    std::stringstream sstr;
    sstr << first << ";" << second;
    element->setValue(sstr.str());
    return element;
}

DOMElem::Ptr DOMBasisTypes::createTreeState(const rw::kinematics::State& state, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(TreeStateId);

    const std::vector<Frame*>& dafs = state.getStateStructure()->getDAFs();

    // Find out what frames are DAFs.
    typedef std::vector<Frame*>::const_iterator I;
    for (I p = dafs.begin(); p != dafs.end(); ++p) {
        Frame* frame = *p;
        createStringPair(frame->getName(), frame->getDafParent(state)->getName(), element);
    }
    return element;
}

DOMElem::Ptr DOMBasisTypes::createState(const rw::kinematics::State& state, DOMElem::Ptr doc) {
    DOMElem::Ptr element = doc->addChild(StateId);
    createQState(state, element);
    createTreeState(state, element);
    return element;
}


