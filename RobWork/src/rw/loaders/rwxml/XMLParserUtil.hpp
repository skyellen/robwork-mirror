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


#ifndef RW_LOADERS_XMLPARSERUTIL_HPP
#define RW_LOADERS_XMLPARSERUTIL_HPP

#include <boost/spirit/include/classic.hpp>
#include <boost/spirit/include/classic_core.hpp>
#include <boost/spirit/include/classic_position_iterator.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/RPY.hpp>

#include <rw/common/Property.hpp>

#include <vector>
#include <map>

template < typename ResultT >
struct result_closure: public boost::spirit::classic::closure<result_closure<ResultT>, ResultT> {
    typedef boost::spirit::classic::closure<result_closure<ResultT>, ResultT> base_t;
    typename base_t::member1 result_;
};


/**********    Useful structures for saving temporary workcell data ********/

typedef enum{Fixed=0,Movable,Prismatic,Revolute,DependRevolute,
             DependtPrismatic} FrameTypeInt;
typedef enum{SerialType,ParallelType,TreeType,MobileType,
             CompositeType,ConveyorType} DeviceType;
typedef enum{AccLimitType,VelLimitType,PosLimitType} LimitType;
typedef enum{PolyType,PlaneType,CubeType,SphereType,ConeType, CylinderType, TubeType, CustomType} GeoType;
typedef enum{ActiveState, PassiveState} FrameState;
typedef enum{MatrixType, LambdaType} InertiaType;

struct DummyNode {
    std::string _name;
    std::string _refframe;
    std::vector<std::string> _scope;
};

struct DummyProperty {
    DummyProperty():_type("string"){};
    std::string _name;
    std::string _refframe;
    std::string _desc;
    std::string _val;
    std::string _type;
};

struct DHParam {
    FrameTypeInt _dhtype;
    double _alpha;
    double _a;
    double _d;
    double _b;
    double _beta;
    double _theta;
    double _offset;
    std::string _hgptype; // parallel, non-parallel
    std::string _type; // craig, schilling, HGP
};

struct DummyLimit {
    std::string _refjoint;
    double _min,_max;
    LimitType _type;
};

struct DummyCollisionSetup {
    boost::spirit::classic::file_position _pos;
    std::string _filename;
    std::vector<std::string> _scope;
};

struct DummyProximitySetup {
    boost::spirit::classic::file_position _pos;
    std::string _filename;
    std::vector<std::string> _scope;
};

struct DummyCalibration {
    boost::spirit::classic::file_position _pos;
    std::string _filename;
    std::vector<std::string> _scope;
};

struct DummyGeometry {
    DummyGeometry():
        _radius(1.0),_x(1.0),_y(1.0),_z(1.0),
        _filename(""), _type(CubeType)
    {}
    double _radius; // sphere, cone
    double _x; // cube
    double _y; // cube
    double _z; // cube, cone
    boost::spirit::classic::file_position _pos;
    std::string _filename;
    std::string _parameters;
    GeoType _type;
};

struct DummyModel {

    DummyModel():
        _refframe(""),_isDrawable(true),_colmodel(true),
        _transform(rw::math::Transform3D<>::identity())
    {}

    std::string _name;
    std::string _refframe;
    bool _isDrawable;
    bool _colmodel;
    rw::math::Transform3D<> _transform;
    std::vector<DummyGeometry> _geo;
    std::vector<std::string> _scope;
};

struct DummyRigidBody{
    DummyRigidBody():
        _transform(rw::math::Transform3D<>::identity()),
        _iType( MatrixType )
        {}

    std::string _refframe;
    rw::math::Transform3D<> _transform;
    double _mass;
    InertiaType _iType;
    double _inertiaM[9];
    double _lambdaX,_lambdaY,_lambdaZ;
};

struct DummyFrame {
    DummyFrame():
        _name(""),
        _refframe(""),
        _type("Fixed"),
        _state(ActiveState),
        _transform(rw::math::Transform3D<>::identity()),
        _isDaf(false),
        _isDepend(false),
        _gain(0),
        _offset(0),
        _hasDHparam(false)
    {}

    std::string getScoped(std::string str){
        std::string tmpstr;
        for(size_t i = 0; i<_scope.size(); i++){
            tmpstr += _scope[i]+".";
        }
        tmpstr += str;
        return tmpstr;
    }

    std::string getName(){
        return getScoped(_name);
    }

    std::string getRefFrame(){
        return _refframe;
    }

    std::string getDependsOn(){
        return getScoped(_dependsOn);
    }

    std::string _name;
    std::string _refframe;
    std::string _type;
    FrameState _state;
    rw::math::Transform3D<> _transform;
    bool _isDaf;

    // For dependent joints
    bool _isDepend;
    double _gain;
    double _offset;
    std::string _dependsOn;

    std::vector< std::string > _scope;
    std::vector<DummyLimit> _limits;
    std::vector<DummyModel> _models;
    std::vector<DummyProperty> _properties;
    DHParam _dhparam;
    bool _hasDHparam;
};

struct QConfig {
public:
    QConfig(){};
    std::string name;
    std::vector<double> q;
};

struct DummyDevice {
public:
    DummyDevice(const std::string& name,
                const DeviceType& type):
        _name(name),_type(type),_axelwidth(0.0)
    {}

    DummyDevice():
        _name(""),_type(SerialType)

    {}

    std::string getScoped(std::string str){
        std::string tmpstr;
        for(size_t i = 0; i<_scope.size(); i++){
            tmpstr += _scope[i]+".";
        }
        tmpstr += str;
        return tmpstr;
    }

    std::string getName(){
        return getScoped(_name);
    }

    std::string getRefFrame(){
        return _refframe;
    }

    std::string _name;
    std::string _refframe;
    DeviceType _type;
    std::vector< DummyFrame > _frames; // base is allways the first frame

    std::vector< DummyCollisionSetup > _colsetups; // collision setups
	std::vector< DummyProximitySetup > _proxsetups; // collision setups

	std::vector<DummyCalibration> _calibration; // device calibration

    std::map<std::string,
        std::vector<boost::shared_ptr<rw::common::Property<std::string> > > > _propMap;

    std::map<std::string, std::vector<DummyModel> > _modelMap;
    std::map<std::string, std::vector<DummyLimit> > _limitMap;
    std::map<std::string, std::vector<DummyProperty> > _propertyMap;

    /* in case of Composite device type */
    std::string _devAName, _devBName;
    /* in case of a mobile device */
    std::string _basename;
    double _axelwidth;
    std::string _leftname, _rightname;
    std::vector<std::string> _scope;
    std::vector<QConfig> _qconfig;
};

struct DummyWorkcell {
public:
    void print( ){
        std::cout << "Name: " << _name << std::endl;
        std::cout << "Nr of Frames: " << _framelist.size() << std::endl;
        for(size_t i=0; i<_framelist.size(); i++)
            std::cout << "\t" << _framelist[i]._name << std::endl;
        std::cout << "Nr of devices: " << _devlist.size() << std::endl;
        for(size_t i=0; i<_devlist.size(); i++)
            std::cout << "\t" << _devlist[i]._name << std::endl;
    }

    void print_const( ) const{
        std::cout << "Nr of Frames: " << _framelist.size() << std::endl;
        std::cout << "Nr of devices: " << _devlist.size() << std::endl;
    }

    std::string _name;
    std::vector< DummyFrame > _framelist;
    std::vector< DummyDevice > _devlist;
    std::vector< DummyModel > _models;
    std::vector< DummyCollisionSetup > _colmodels;
	std::vector< DummyProximitySetup > _proxmodels;
	std::vector< DummyProperty > _properties;
	std::vector<DummyCalibration> _calibration; // workcell calibration
};

/**********    Useful functors for the parsing proces    **********/

struct SetDevScope{
    SetDevScope(std::vector<std::string>& scope, DummyDevice &device):
        _scope(scope),_device(device){}

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        _device._scope = _scope;
    }

    std::vector< std::string > &_scope;
    DummyDevice &_device;
};

struct EnterScope{
    EnterScope(std::vector<std::string>& scope):
        _scope(scope){}

    void operator()(std::string const& next) const {
        _scope.push_back( next );
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        std::string next(first,last);
        _scope.push_back( next );
    }

    std::vector< std::string > &_scope;
};

struct LeaveScope{
    LeaveScope(std::vector<std::string>& scope):
        _scope(scope){}

    void operator()(std::string const& next) const {
        _scope.pop_back( );
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        _scope.pop_back( );
    }

    std::vector< std::string > &_scope;
};

struct AddConfigToDevice {
    AddConfigToDevice(const QConfig& config, DummyDevice &device):
        _config(config),_device(device)

    {
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        _device._qconfig.push_back(_config);
    }

    const QConfig &_config;
    DummyDevice &_device;
};

struct AddFrameToDevice {
    AddFrameToDevice(DummyFrame &frame,
                     DummyDevice &device,
                     std::vector<std::string>& scope):
        _frame(frame), _device(device), _scope(scope){}

    void operator()(DummyFrame const &f) const {
        DummyFrame frame = f;
        std::string absRefPath,absPath;
        frame._scope = _scope;
        for(size_t i=0; i<frame._scope.size(); i++ ){
            absPath += frame._scope[i]+".";
        }

        if( _device._frames.size()==0){
            if( _device._type == MobileType ){
                if( frame._refframe == "")
                    frame._refframe = absPath + _device._basename;
                else
                    frame._refframe = absPath + frame._refframe;
                _device._frames.push_back(frame);
            } else {
                if(frame._refframe != "")
                    std::cout << "*** Warning: base frame of device is referencing to a frame!!"
                              << "***          This will be ignored!" << std::endl;
                frame._refframe = "";
                _device._frames.push_back(frame);
            }
            return;
        }

        DummyFrame lastFrame = _device._frames.back();
        for(size_t i=0; i<lastFrame._scope.size(); i++ ){
            absRefPath += lastFrame._scope[i]+".";
        }

        if( _device._type == SerialType ){
            frame._refframe = absRefPath + lastFrame._name;
            _device._frames.push_back(frame);
        } else if( _device._type == TreeType || _device._type == ParallelType ||
                   _device._type == MobileType )
        {
            if( frame._refframe == "" ){
                frame._refframe = absRefPath + lastFrame._name;
            } else {
                frame._refframe = absPath + frame._refframe;
            }
            _device._frames.push_back(frame);
        }
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        (*this)(_frame);
    }

    DummyFrame &_frame;
    DummyDevice &_device;
    std::vector< std::string > &_scope;
};

struct AddFrameToWorkcell {
    AddFrameToWorkcell( DummyWorkcell &workcell,
                        std::vector<std::string>& scope):
        _workcell(workcell),_scope(scope){}

    void operator()(DummyFrame const &f) const {
        DummyFrame frame = f;
        frame._scope = _scope;
        if( _workcell._framelist.size()==0 || frame._refframe == ""){
            frame._refframe = "WORLD";
        }
        std::string absRefPath;
        for(size_t i=0; i<_scope.size(); i++ )
            absRefPath += _scope[i] + ".";
        frame._refframe = absRefPath + frame._refframe;
        _workcell._framelist.push_back(frame);
    }

    DummyWorkcell &_workcell;
    std::vector< std::string > &_scope;
};

struct AddDeviceToWorkcell {
    AddDeviceToWorkcell( DummyWorkcell &workcell,
                        std::vector<std::string>& scope):
        _workcell(workcell),_scope(scope){}

    void operator()(DummyDevice const &device) const {
        DummyDevice dev = device;
        dev._scope = _scope;
        if( _workcell._framelist.size()!=0 && dev._refframe == "" ){
            dev._refframe = _workcell._framelist.back().getName();
            dev._frames[0]._refframe = dev._refframe;
        } else if( _workcell._framelist.size()==0 && dev._refframe == "" ){
            dev._refframe = "WORLD";
            dev._frames[0]._refframe = dev._refframe;
        }
        _workcell._devlist.push_back(dev);
    }

    DummyWorkcell &_workcell;
    std::vector< std::string > &_scope;
};

struct SetTransform3D {

    SetTransform3D( const double *matrix,
                    rw::math::Transform3D<> &t3d):
        _rpy(NULL),_pos(NULL),_t3d(t3d),_matrix(matrix),_param(NULL){}

    SetTransform3D( const DHParam &param,
                    rw::math::Transform3D<> &t3d ):
        _rpy(NULL),_pos(NULL),_t3d(t3d),_matrix(NULL),
        _param(&param){}

    SetTransform3D( const rw::math::RPY<> &rpy,
                    const rw::math::Vector3D<> &pos,
                    rw::math::Transform3D<> &t3d):
        _rpy(&rpy),_pos(&pos),_t3d(t3d),_matrix(NULL),_param(NULL){}


    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        if( _rpy != NULL ){
            rw::math::Vector3D<> pos = *_pos;
            rw::math::Rotation3D<> rot = (*_rpy).toRotation3D();
            _t3d = rw::math::Transform3D<>(pos,rot);
        } else if( _matrix != NULL ) {
            rw::math::Vector3D<> pos(_matrix[3],_matrix[7],_matrix[11]);
            rw::math::Rotation3D<> rot(_matrix[0],_matrix[1],_matrix[2],
                                       _matrix[4],_matrix[5],_matrix[6],
                                       _matrix[8],_matrix[9],_matrix[10]);
            _t3d = rw::math::Transform3D<>(pos,rot);
        } else {
            if( _param->_dhtype == Revolute ){
            	if(_param->_type == "schilling"){
					_t3d = rw::math::Transform3D<>::DH(
						_param->_alpha,_param->_a,
						_param->_d,_param->_offset);
            	} else if( _param->_type == "HGP" ){
            	    // check if its parallel
            	    if(_param->_hgptype=="parallel"){
                        _t3d = rw::math::Transform3D<>::DHHGP(
                            _param->_alpha,_param->_a,
                            _param->_offset,_param->_b);
            	    } else {
                        _t3d = rw::math::Transform3D<>::DH(
                            _param->_alpha,_param->_a,
                            _param->_d,_param->_offset);
            	    }

            	} else {
					_t3d = rw::math::Transform3D<>::craigDH(
						_param->_alpha,_param->_a,
						_param->_d,_param->_offset);
            	}
            } else if( _param->_dhtype == Prismatic ){
            	if(_param->_type == "schilling"){
					_t3d = rw::math::Transform3D<>::DH(
						_param->_alpha,_param->_a,
						_param->_offset,_param->_theta);
                } else if( _param->_type == "HGP" ){

                    // check if its parallel
                    if(_param->_hgptype=="parallel"){
                        _t3d = rw::math::Transform3D<>::DHHGP(
                            _param->_alpha,_param->_a,
                            _param->_b, _param->_offset);
                    } else {
                        _t3d = rw::math::Transform3D<>::DH(
                            _param->_alpha,_param->_a,
                            _param->_offset, _param->_theta);
                    }

            	} else {
					_t3d = rw::math::Transform3D<>::craigDH(
						_param->_alpha,_param->_a,
						_param->_offset,_param->_theta);
            	}
            } else {
                _t3d = rw::math::Transform3D<>::identity();
            }
        }
    }

    const rw::math::RPY<> *_rpy;
    const rw::math::Vector3D<> *_pos;
    rw::math::Transform3D<> &_t3d;
    const double *_matrix;
    const DHParam *_param;
};

struct SetDHParam {

    SetDHParam( const DHParam &paramFrom,
                    DummyFrame &dframe):
        _paramFrom(paramFrom),_dframe(dframe){}

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
       _dframe._dhparam = _paramFrom;
       _dframe._hasDHparam = true;
    }

    const DHParam &_paramFrom;
    DummyFrame &_dframe;
};

struct InsertModelInMap{
    InsertModelInMap( const DummyModel &model,
                      const std::string &refframe,
                      std::vector<std::string>& scope,
                      std::map<std::string,
                               std::vector<DummyModel> > &map)
        : _model(model), _refframe(refframe), _scope(scope),_map(map){}

    void operator()(DummyModel const& model) const {
        std::string name;
        for(size_t i = 0; i<_scope.size(); i++){
            name += _scope[i]+".";
        }
        if( model._refframe == "" ){
            name += _refframe;
        } else {
            name += model._refframe;
        }
        _map[name].push_back( model );
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        (*this)(_model);
    }

    const DummyModel &_model;
    const std::string &_refframe;
    std::vector<std::string> &_scope;
    std::map<std::string,
             std::vector<DummyModel > > &_map;

};

struct InsertLimitInMap{
    InsertLimitInMap( std::vector<std::string> &scope,
                      std::map<std::string,std::vector<DummyLimit> > &map)
        : _scope(scope),_map(map){}

    void operator()(DummyLimit const& limit) const {

        std::string name;
        for(size_t i = 0; i<_scope.size(); i++){
            name += _scope[i]+".";
        }
        if( limit._refjoint != "" ){
            name += limit._refjoint;
        } else {
            name += limit._refjoint;
        }

        _map[name].push_back( limit );
    }

    std::vector<std::string> &_scope;
    std::map<std::string,
             std::vector<DummyLimit > > &_map;

};

struct InsertPropertyInMap{
    InsertPropertyInMap( std::vector<std::string> scope,
                      std::map<std::string,std::vector<DummyProperty> > &map)
        : _scope(scope),_map(map){}

    void operator()(DummyProperty const& prop) const {
        std::string name;
        for(size_t i = 0; i<_scope.size(); i++){
            name += _scope[i]+".";
        }
        if( prop._refframe != "" ){
            name += prop._refframe;
        } else {
            name += prop._refframe;//! TODO: this should be set to the base frame of the device
        }
        _map[name].push_back( prop );
    }

    std::vector<std::string> _scope;
    std::map<std::string,
             std::vector<DummyProperty > > &_map;
};

struct InsertInMap{
    InsertInMap(std::map<std::string, std::string> &map,
                std::string& id)
        : _map(map),_id(id)
    {}

    void operator()(std::string const& text) const {
        _map[ _id ] = text;
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        //std::string text(first, last);
        std::string str;

        IteratorT tmpIter = first;
        while (tmpIter != last) {
            str += *tmpIter;
            ++tmpIter;
        }

        _map[ _id ] = str;
    }

    std::map<std::string, std::string> &_map;
    std::string& _id;
};

struct AppendToOutput{
    AppendToOutput(std::vector<char> &output)
        : _output(output)
    {}

    void operator()(std::string const& text) const {
        //std::cout << "AppendToOutput " << text << std::endl;
        _output.insert(_output.end(), text.begin(), text.end());
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        //std::string text(first,last);
        //std::cout << "AppendToOutput " << std::endl;
        //_output.insert(_output.end(), first, last);

//        unsigned char tmp = _output[_output.size()-1 ];
//        std::cout << "Last val (output): " << (unsigned int) tmp << std::endl;

        char tmp = 0;
        IteratorT tmpIter = first;
        //while (tmpIter != last) {
        //    tmp = *tmpIter;
        //    std::cout << tmp;
        //    ++tmpIter;
        //}

        //tmpIter = first;
        while (tmpIter != last) {
            tmp = *tmpIter;
            _output.push_back(tmp);
            ++tmpIter;
        }
        //std::cout << "Last val (iterator): " << (unsigned int) (unsigned char) tmp << std::endl;
    }

    std::vector<char> &_output;
};

struct AppendToOutputFromMap{
    AppendToOutputFromMap(std::map<std::string,std::string> &map,
                          std::vector<char> &output)
        : _map(map), _output(output)
    {}

    void operator()(std::string const& text) const {
        std::string& use = _map[ text ];
        //std::cout << "AppendToOutputFromMap " << use << std::endl;
        _output.insert(_output.end(), use.begin(), use.end());
    }

    template < typename IteratorT >
    void operator()(IteratorT const& first, IteratorT const& last) const {
        std::string& use = _map[ std::string(first,last) ];
        //std::cout << "AppendToOutputFromMap " << use << std::endl;
        _output.insert(_output.end(), use.begin(), use.end());
    }

    std::map<std::string, std::string> &_map;
    std::vector<char> &_output;
};

#endif /*XMLPARSERUTIL_HPP_*/
