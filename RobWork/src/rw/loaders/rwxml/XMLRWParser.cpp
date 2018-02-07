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


#include "XMLRWParser.hpp"

using namespace rw::math;

#include "XMLParser.hpp"
#include "XMLErrorHandler.hpp"
#include "XMLRWPreParser.hpp"
#include "MultipleFileIterator.hpp"

#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>

#include <boost/spirit/include/classic_parse_tree.hpp>

using namespace phoenix;
using namespace boost::spirit::classic;
using namespace boost::spirit;
using namespace rw::loaders;

namespace {

    struct FilePosParser
    {
        typedef file_position result_t;
        template <typename ScannerT>
        std::ptrdiff_t
        operator()(ScannerT const& scan, result_t& result) const
        {
            result = scan.first.get_position();
            return 0;
        }
    };

    functor_parser<FilePosParser> filepos_p;

    struct Transform3DParser : public grammar<Transform3DParser, result_closure<Transform3D<> >::context_t>
    {
    public:
        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return transform_r; }

            definition( Transform3DParser const &self ){
                transform_r =
                    (   matrix_transform_r
                      | rpypos_transform_r
                    )
                    ;

                rpy_r =
                    XMLElem_p("RPY",
                        real_p[ var( _rpy(0) ) = arg1*Deg2Rad ] >>
                        real_p[ var( _rpy(1) ) = arg1*Deg2Rad ] >>
                        real_p[ var( _rpy(2) ) = arg1*Deg2Rad ]
                              [ rpy_r.result_ = var( _rpy ) ]
                              )
                        ;

                pos_r =
                    XMLElem_p("Pos",
                        real_p[ var( _pos(0) ) = arg1 ] >>
                        real_p[ var( _pos(1) ) = arg1 ] >>
                        real_p[ var( _pos(2) ) = arg1 ]
                              [ pos_r.result_ = var( _pos ) ]
                    )
                        ;

                rpypos_transform_r =
                    ((  rpy_r[ var(_rpy) = arg1 ] >> pos_r[ var(_pos) = arg1 ] )
                        [ self.result_ = construct_<Transform3D<> >( var(_pos) , var(_rpy) ) ])
                      |
                    ((  pos_r[ var(_pos) = arg1 ] >> rpy_r[ var(_rpy) = arg1 ] )
                        [ self.result_ = construct_<Transform3D<> >( var(_pos) , var(_rpy) ) ])


                     ;

                matrix_transform_r =
                        XMLElem_p("Transform",
                            real_p[ var( _t3d(0,0) ) = arg1 ] >>
                            real_p[ var( _t3d(0,1) ) = arg1 ] >>
                            real_p[ var( _t3d(0,2) ) = arg1 ] >>
                            real_p[ var( _t3d(0,3) ) = arg1 ] >>
                            real_p[ var( _t3d(1,0) ) = arg1 ] >>
                            real_p[ var( _t3d(1,1) ) = arg1 ] >>
                            real_p[ var( _t3d(1,2) ) = arg1 ] >>
                            real_p[ var( _t3d(1,3) ) = arg1 ] >>
                            real_p[ var( _t3d(2,0) ) = arg1 ] >>
                            real_p[ var( _t3d(2,1) ) = arg1 ] >>
                            real_p[ var( _t3d(2,2) ) = arg1 ] >>
                            real_p[ var( _t3d(2,3) ) = arg1 ]
                                  [ self.result_ = var( _t3d ) ]
                        );
            }

        private:
            rule<ScannerT> transform_r, matrix_transform_r, rpypos_transform_r;
            rule<ScannerT, result_closure<RPY<double> >::context_t> rpy_r;
            rule<ScannerT, result_closure<Vector3D<double> >::context_t> pos_r;
            // temp variables
            Transform3D<double> _t3d;
            RPY<double> _rpy;
            Vector3D<double> _pos;
        };
    } transform3d_p;

    struct QuotedStringParser : public grammar<QuotedStringParser, result_closure<std::string>::context_t>
    {
    public:
        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return quotedStr_p; }

            definition( QuotedStringParser const &self ){
                quotedStr_p =
                    token_node_d[ch_p('"') >> lexeme_d[ *(anychar_p - '"') ]
                         [self.result_ = construct_<std::string>(arg1,arg2)] >> ch_p('"')];
            }

        private:
            rule<ScannerT> quotedStr_p;
        };
    } quotedStr_p;

    struct XMLAttrStringParser : public grammar<XMLAttrStringParser, result_closure<std::string>::context_t>
    {
    public:
        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return attrstr_r; }

            definition( XMLAttrStringParser const &self ){
                attrstr_r =
                    (*(anychar_p - '"'))
                         [self.result_ = construct_<std::string>(arg1,arg2)];
            }

        private:
            rule<ScannerT> attrstr_r;
        };
    } attrstr_p;

    struct PropertyParser : public grammar<PropertyParser, result_closure<DummyProperty>::context_t>
    {
    public:

        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return property_r; }

            definition( PropertyParser const &self ){
                property_r =
                    XMLAttElem_p("Property",
                        XMLAtt_p("name", attrstr_p
                            [ var( _property._name ) = arg1 ] ) >>
                        !(XMLAtt_p("type", attrstr_p
                            [ var( _property._type ) = arg1 ] )) >>
                        !(XMLAtt_p("refframe", attrstr_p
                            [ var( _property._refframe ) = arg1 ] )) >>
                        !(XMLAtt_p("desc", attrstr_p
                            [ var( _property._desc ) = arg1 ] )),
                        (*(anychar_p-(str_p("</") >> "Property")))
                            [ var( _property._val ) = construct_<std::string>(arg1,arg2) ]
                            [ self.result_ = var( _property ) ]
                    );
            }

        private:
            rule<ScannerT> property_r;
            DummyProperty _property;
        };
    } property_p;

    struct ModelParser : public grammar<ModelParser, result_closure<DummyModel>::context_t>
    {
    public:
        std::vector<std::string> &_scope;

        ModelParser( std::vector<std::string>& scope ):_scope(scope){}

        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return model_r; }

            definition( ModelParser const &self ){

                model_r = eps_p[ var( _model ) = construct_<DummyModel>() ] >>
                    (drawable_r | colmodel_r)
                        [ var( _model._scope ) = var( self._scope ) ]
                        [ self.result_ = var( _model ) ]
                        ;

                colmodel_r = //eps_p[ var( _model ) = colmodel_r.result_ ] >> // used to initialize _model
                    (XMLAttElem_p("CollisionModel",
                        (XMLAtt_p("name", attrstr_p
                            [ var( _model._name ) = arg1 ] )) >>

                        !(XMLAtt_p("refframe", attrstr_p
                            [ var( _model._refframe ) = arg1 ] ))
                            ,
                        !transform3d_p
                            [ var( _model._transform ) = arg1 ] >>
                        *(geometry_r
                            [ push_back_a( _model._geo ) ])
                    )) [ var( _model._isDrawable ) = false ];

                drawable_r =
                    (XMLAttElem_p("Drawable",
                         (XMLAtt_p("name", attrstr_p
                            [ var( _model._name ) = arg1 ] )) >>
                        !(XMLAtt_p("refframe", attrstr_p
                            [ var( _model._refframe ) = arg1 ] )) >>
                        !(XMLAtt_p("colmodel",
                               (
                                 str_p("Enabled")[var(_model._colmodel) = true] |
                                 str_p("Disabled")[var(_model._colmodel) = false]))
                                )
                            , // used to initialize _model
                        !transform3d_p
                            [ var( _model._transform ) = arg1 ] >>
                        *( geometry_r
                            [ push_back_a( _model._geo ) ])
                    )) [ var( _model._isDrawable ) = true ];

                geometry_r = eps_p[ var( _geo ) = construct_<DummyGeometry>() ] >>
                    ( XMLAttElem_p("Polytope",
                        XMLAtt_p("file",attrstr_p
                           [ var( _geo._filename ) = arg1 ]
                           [ var( _geo._type ) = PolyType ] )>>
                        filepos_p[ var(_geo._pos) = arg1 ] ,
                        eps_p
                     ) // save position of file

					| XMLAttElem_p("Plane",eps_p[ var( _geo._type ) = PlaneType ],eps_p)
                    | XMLAttElem_p("Sphere",
                        XMLAtt_p("radius",real_p
                          [ var( _geo._radius ) = arg1 ]
                          [ var( _geo._type ) = SphereType ]),
                        eps_p)
                    | XMLAttElem_p("Box",
                        XMLAtt_p("x",real_p[ var( _geo._x ) = arg1 ]) >>
                        XMLAtt_p("y",real_p[ var( _geo._y ) = arg1 ]) >>
                        XMLAtt_p("z",real_p[ var( _geo._z ) = arg1 ]
                                           [ var( _geo._type ) = CubeType ] ),
                        eps_p)
                    | XMLAttElem_p("Cone",
                        XMLAtt_p("radius",real_p[ var( _geo._radius ) = arg1 ]) >>
                        XMLAtt_p("z",real_p[ var( _geo._z ) = arg1 ]
                                           [ var( _geo._type ) = ConeType ] ),
                        eps_p)
                    | XMLAttElem_p("Cylinder",
                                   XMLAtt_p("radius",real_p[ var( _geo._radius ) = arg1 ]) >>
                                   XMLAtt_p("z",real_p[ var( _geo._z ) = arg1 ]
                                                      [ var( _geo._type ) = CylinderType ] ),
                                   eps_p)
                    | XMLAttElem_p("Tube",
                                   XMLAtt_p("radius",real_p[ var( _geo._radius ) = arg1 ]) >>
                                   XMLAtt_p("thickness",real_p[ var( _geo._x ) = arg1 ]) >>
                                   XMLAtt_p("z",real_p[ var( _geo._z ) = arg1 ]
                                                      [ var( _geo._type ) = TubeType ] ),
                                   eps_p)
                    | XMLAttElem_p("Custom",
                        XMLAtt_p("type", attrstr_p
                           [ var( _geo._filename ) = arg1 ]
                           [ var( _geo._type ) = CustomType ] ) >>
                        XMLAtt_p("param", attrstr_p
                           [ var( _geo._parameters ) = arg1 ] ),
                        eps_p
                     )
                    )[ geometry_r.result_ = var( _geo ) ];
            }

        private:
            rule<ScannerT> model_r,colmodel_r,drawable_r;
            rule<ScannerT, result_closure<DummyGeometry>::context_t > geometry_r;
            DummyModel _model;
            DummyGeometry _geo;
        };
    };

    struct RigidBodyParser : public grammar<RigidBodyParser, result_closure<DummyRigidBody>::context_t>
    {
    public:
        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return rigidbody_r; }

            definition( RigidBodyParser const &self ){

                rigidbody_r = eps_p[ var( _rigid ) = construct_<DummyRigidBody>() ] >>
                    XMLAttElem_p("RigidBody",
                        !(XMLAtt_p("refframe",attrstr_p
                            [ var(_rigid._refframe) = arg1 ] )),
                        *(
                            transform3d_p
                                [ var(_rigid._transform) = arg1 ]
                          | XMLElem_p("Mass", real_p[ var(_rigid._mass) = arg1 ]  )
                          | inertiamatrix_r
                        )
                    )[ self.result_ = var(_rigid) ];

                inertiamatrix_r =
                      XMLElem_p("InertiaMatrix",
                                real_p >> real_p >> real_p >>
                                real_p >> real_p >> real_p >>
                                real_p >> real_p >> real_p )
                    | ( XMLElem_p("LambdaX", real_p ) >>
                        XMLElem_p("LambdaY", real_p ) >>
                        XMLElem_p("LambdaZ", real_p )
                       );
            }

        private:
            rule<ScannerT> rigidbody_r,inertiamatrix_r;
            DummyRigidBody _rigid;
            ModelParser model_p;
        };
    };

    struct FrameParser : public grammar<FrameParser, result_closure<DummyFrame>::context_t>
    {
    public:
        std::vector<std::string> &_scope;

        FrameParser(std::vector<std::string>& scope):_scope(scope){}

        template <typename ScannerT>
        struct definition
        {
        public:
            rule<ScannerT> const start() const { return frame_r; }

            definition( FrameParser const &self ): model_p(self._scope){

                frame_r = eps_p[ var( _frame ) = construct_<DummyFrame>() ] >>
                    XMLAttElem_p("Frame",
                        XMLAtt_p("name",attrstr_p
                            [ var(_frame._name) = arg1 ]) >>
                        !(XMLAtt_p("refframe",attrstr_p
                            [ var(_frame._refframe) = arg1 ] )) >>
                        !(XMLAtt_p("type",attrstr_p
                            [ var(_frame._type) = arg1 ] )) >>
                        !(XMLAtt_p("daf",
                                   (
                                     str_p("true")[var(_frame._isDaf) = true] |
                                     str_p("false")[var(_frame._isDaf) = false]
                                    ))),
                        // parse transform
                        !(transform3d_p
                            [ var(_frame._transform) = arg1 ]) >>
                        // parse properties
                        *(
                            property_p
                                [ push_back_a( _frame._properties ) ]
                          | model_p
                                [ push_back_a( _frame._models ) ]
                        )
                    )[ self.result_ = var(_frame) ];
            }

        private:
            rule<ScannerT> frame_r;
            DummyFrame _frame;
            ModelParser model_p;
        };
    };

    struct XMLDeviceParser : public grammar<XMLDeviceParser, result_closure<DummyDevice>::context_t>
    {
    public:

        template <typename ScannerT>
        struct definition
        {
        private:
            std::vector< std::string > _scope;
            std::string _currentDir;
            std::string _currentFile;
            DummyProperty _property;
            std::string _refjoint;
            DummyDevice _dev;
            DummyFrame _frame;
            DummyLimit _limit;
            DummyGeometry _geo;
            DummyModel _model;
            DHParam _dhparam;
            DummyCollisionSetup _setup;
            DummyCalibration _calibration;
			DummyProximitySetup _psetup;
            QConfig _config;
            QJunction _junction;
            const QConfig emptyConfig;
            const QJunction emptyJunction;
            Transform3DParser t3d_p;
            rule<ScannerT> device_r, serialdevice_r, paralleldevice_r,
                           treedevice_r, joint_r,dhjoint_r,depend_r,
                           devicebody_r,chainbody_r,serialchain_r,
                           jointstate_r, mobiledevice_r, configuration_r, junction_r;

            //rule<ScannerT, result_closure<DummyConveyorSegment>::context_t> conveyorsegment_r

            rule<ScannerT, result_closure<DummyLimit>::context_t> jointlimit_r;

            rule<ScannerT, result_closure<DummyCollisionSetup>::context_t> colsetup_r;
			rule<ScannerT, result_closure<DummyProximitySetup>::context_t> proxsetup_r;

            ModelParser model_p;
            FrameParser frame_p;
        public:

            /**
             * @brief Gets start rule
             * @return the start rule
             */
            rule<ScannerT> const start() const { return device_r; }


            /**
             * @brief Gets parser definition
             */
            definition( XMLDeviceParser const &self ): model_p(_scope), frame_p(_scope)
            {
                device_r = eps_p[ var( _dev._scope ) = _scope ] >>
                            (   serialdevice_r
                              | paralleldevice_r
                              | treedevice_r
                              | mobiledevice_r
                            )[ self.result_ = var( _dev ) ];

                serialchain_r =
                        XMLAttElem_p("SerialChain",
                            XMLAtt_p("name", attrstr_p[ EnterScope( _scope ) ]),
                            chainbody_r
                        )[ LeaveScope( _scope ) ];

                chainbody_r =
                        *(
                             frame_p
                                [ AddFrameToDevice( _frame, _dev, _scope) ]
                           | joint_r
                                [ AddFrameToDevice( _frame, _dev, _scope) ]
                           | dhjoint_r
                                [ AddFrameToDevice( _frame, _dev, _scope) ]
                           | model_p
                                [ InsertModelInMap( _model , _model._refframe, _scope, _dev._modelMap ) ]
                           | jointlimit_r
                                [ InsertLimitInMap(_scope, _dev._limitMap) ]
                           | property_p
                                [ InsertPropertyInMap(_scope, _dev._propertyMap) ]
                           | serialchain_r
                           | colsetup_r[ push_back_a( _dev._colsetups ) ]
						   | proxsetup_r[ push_back_a( _dev._proxsetups ) ]
                         );

                colsetup_r =
                    XMLAttElem_p("CollisionSetup",
                        XMLAtt_p("file", attrstr_p[ var(_setup._filename) = arg1 ]
                                                  [ var( _setup._scope ) = var( _scope ) ] >>
                        filepos_p[ var(_setup._pos) = arg1 ]
                        ),
                        eps_p
                    )[ colsetup_r.result_ = var(_setup) ];

                /*calibration_r =
                		XMLAttElem_p("Calibration",
                				XMLAtt_p("file", attrstr_p[ var(_calibration._filename) = arg1 ]
                                                            [ var( _calibration._scope ) = var( _scope ) ] >>
                                  filepos_p[ var(_calibration._pos) = arg1 ]
                                  ),
                                  eps_p
                              )[ calibration_r.result_ = var(_calibration) ];

				*/
                proxsetup_r =
                    XMLAttElem_p("ProximitySetup",
                        XMLAtt_p("file", attrstr_p[ var(_psetup._filename) = arg1 ]
                                                  [ var( _psetup._scope ) = var( _scope ) ] >>
                        filepos_p[ var(_psetup._pos) = arg1 ]
                        ),
                        eps_p
                    )[ proxsetup_r.result_ = var(_psetup) ];



                devicebody_r =
                        chainbody_r
                         >>
                        //!frames >> // frames of interest
                        (*configuration_r[ AddConfigToDevice( _config, _dev) ]
                                         [ var( _config ) = var(emptyConfig) ])

                    ;

                configuration_r =
                	XMLAttElem_p("Q",
                		XMLAtt_p("name",attrstr_p[ var( _config.name ) = arg1 ])
                		,
                		*real_p[push_back_a( _config.q )]
                	);

                junction_r =
                	XMLElem_p("Junction",
                		*XMLElem_p("Chains",
                                (*(anychar_p-(str_p("</") >> "Chains")))
                                    [ push_back_a( _junction.chains ) ]

                		)
                	);


                treedevice_r = eps_p[ var( _dev ) = construct_<DummyDevice>() ] >>
                    XMLAttElem_p("TreeDevice",
                        XMLAtt_p("name", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = TreeType ]
                            [ EnterScope(_scope) ]),
                        devicebody_r
                    )[ LeaveScope(_scope) ];

                serialdevice_r = eps_p[ var( _dev ) = construct_<DummyDevice>() ] >>
                    XMLAttElem_p("SerialDevice",
                        XMLAtt_p("name", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = SerialType ]
                            [ EnterScope(_scope) ] ),
                        devicebody_r
                    )[ LeaveScope(_scope) ];

                paralleldevice_r = eps_p[ var( _dev ) = construct_<DummyDevice>() ] >>
                    XMLAttElem_p("ParallelDevice",
                        XMLAtt_p("name", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = ParallelType ]
                            [ EnterScope(_scope) ]),
                        chainbody_r >>
                        (*junction_r[ AddJunctionToDevice( _junction, _dev) ]
                                         [ var( _junction ) = var(emptyJunction) ]) >>
                        (*configuration_r[ AddConfigToDevice( _config, _dev) ]
                                         [ var( _config ) = var(emptyConfig) ])
                    )[ LeaveScope(_scope) ];

                mobiledevice_r = eps_p[ var( _dev ) = construct_<DummyDevice>() ] >>
                    XMLAttElem_p("MobileDevice",
                        XMLAtt_p("name", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = MobileType ]
                            [ EnterScope(_scope) ]) >>
                        XMLAtt_p("basename", attrstr_p
                            [ var( _dev._basename ) = arg1 ]),
                        XMLElem_p("AxelWidth", real_p[ var(_dev._axelwidth) = arg1]) >>
                        XMLAttElem_p("LeftWheel",
                            XMLAtt_p("name", attrstr_p[var(_dev._leftname)= arg1 ]),
                            eps_p) >>
                        XMLAttElem_p("RightWheel",
                            XMLAtt_p("name", attrstr_p[var(_dev._rightname)= arg1 ]),
                            eps_p) >>
                        devicebody_r
                    )[ LeaveScope(_scope) ];

/*
                conveyordevice_r = eps_p[ var( _dev ) = construct_<DummyDevice>() ] >>
                    XMLAttElem_p("ConveyorDevice",
                        XMLAtt_p("name", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = ConveyorType ]
                            [ EnterScope(_scope) ]),
                        *conveyorsegment_r
                            [ push_back_a( _dev._conveyorsegments ) ] >>
                        devicebody_r
                    )[ LeaveScope(_scope) ];

                conveyorsegment_r =
                    XMLAttElem_p("ConveyorSegment",
                        XMLAtt_p("type", attrstr_p
                            [ var( _dev._name ) = arg1 ]
                            [ var( _dev._type ) = ConveyorType ]
                            [ EnterScope(_scope) ]),
                        *conveyorsegment_r
                            [ push_back_a( _dev._conveyorsegments ) ] >>
                        devicebody_r
                    );
                    ;
  */
                jointstate_r =
                       ( str_p("Active")[ var(_frame._state) = ActiveState ]
                       | str_p("Passive")[ var(_frame._state) = PassiveState ]
                       )
                    ;

                joint_r = eps_p[ var( _frame ) = construct_<DummyFrame>() ] >> // init _frame
                    XMLAttElem_p("Joint",
                        XMLAtt_p("name",attrstr_p
                            [ var(_frame._name) = arg1 ]) >>
                        !(XMLAtt_p("refframe", attrstr_p
                            [ var(_frame._refframe) = arg1 ] )) >>
                        XMLAtt_p("type", attrstr_p
                            [ var(_frame._type) = arg1 ]) >>
                        !(XMLAtt_p("state", jointstate_r))
                         ,
                        !(t3d_p
                            [ var(_frame._transform) = arg1 ]) >>
                        *(
                           jointlimit_r
                                [ push_back_a( _frame._limits ) ]
                          | depend_r
                          | property_p
                                [ push_back_a( _frame._properties ) ]
                          | model_p
                                [ push_back_a( _frame._models ) ]
                        )
                    );
                // T alpha, T a,   beta, b,
                // T alpha, T a,   d, theta
                /*

+   if (isParallel)
+       return Transform3D(
+               Vector3D<T>(a * cos(beta), b, - a * sin(beta)),
+               Rotation3D<T>(
+                   cos(beta), sin(alpha) * sin(beta), cos(alpha) * sin(beta),
+                   0, cos(alpha), -sin(alpha),
+                   -sin(beta), sin(alpha) * cos(beta), cos(alpha) * cos(beta)));
+   else
+       return Transform3D<T>::DH(alpha, a, d, theta);

                 */

                dhjoint_r = eps_p[ var( _frame ) = construct_<DummyFrame>() ] >> // init _frame
                    XMLAttElem_p("DHJoint",
                        (XMLAtt_p("name", attrstr_p
                            [ var( _frame._name ) = arg1 ]) >>
                        XMLAtt_p("alpha", real_p
                            [ var( _dhparam._alpha ) = arg1*Deg2Rad ]) >>
                        XMLAtt_p("a", real_p
                            [var( _dhparam._a ) = arg1 ])  >>
                        ( (  XMLAtt_p("d", real_p
                                [ var( _dhparam._dhtype ) = Revolute ]
                                [ var( _frame._type ) = "Revolute" ]
                                [ var( _dhparam._d ) = arg1  ])
                            >> XMLAtt_p("offset", real_p
                                    [ var( _dhparam._offset ) = arg1*Deg2Rad ])
                          )
                          | (XMLAtt_p("theta", real_p
                                [ var( _dhparam._dhtype ) = Prismatic ]
                                [ var( _frame._type ) = "Prismatic" ]
                                [ var( _dhparam._theta ) = arg1*Deg2Rad ])
                            >> (XMLAtt_p("offset", real_p
                                    [ var( _dhparam._offset ) = arg1 ])
                                )
                            )
                          | (XMLAtt_p("beta", real_p
                                      [ var( _dhparam._dhtype ) = Prismatic ]
                                      [ var( _frame._type ) = "Prismatic" ]
                                      [ var( _dhparam._hgptype ) = "parallel" ]
                                      [ var( _dhparam._beta ) = arg1*Deg2Rad ])
                                  >> (XMLAtt_p("offset", real_p
                                          [ var( _dhparam._offset ) = arg1 ])
                                      )
                                  )
                          | (XMLAtt_p("b", real_p
                                      [ var( _dhparam._dhtype ) = Revolute ]
                                      [ var( _frame._type ) = "Revolute" ]
                                      [ var( _dhparam._hgptype ) = "parallel" ]
                                      [ var( _dhparam._b ) = arg1 ])
                                  >> (XMLAtt_p("offset", real_p
                                          [ var( _dhparam._offset ) = arg1*Deg2Rad ])
                                      )
                                  )

                         ) >>
                         !(XMLAtt_p("state", jointstate_r))
                         >>
                         !(XMLAtt_p("type", attrstr_p[var(_dhparam._type)= arg1 ]))
                         )
                             [SetTransform3D( _dhparam, _frame._transform)]
                             [SetDHParam(_dhparam, _frame )],

                        *(
                           jointlimit_r
                            [ push_back_a( _frame._limits ) ]
                         | depend_r
                         | property_p
                            [ push_back_a( _frame._properties ) ]
                         | model_p
                            [ push_back_a( _frame._models ) ]
                        )
                    );

                depend_r =
                    XMLAttElem_p("Depend",
                        XMLAtt_p("on",attrstr_p[ var(_frame._dependsOn) = arg1 ]) >>
                        XMLAtt_p("gain",real_p[ var(_frame._gain) = arg1 ]) >>
                        XMLAtt_p("offset",real_p[ var(_frame._offset) = arg1 ]),
                        eps_p
                    )[ var(_frame._isDepend) = true ];

                jointlimit_r = eps_p[ var( _limit ) = construct_<DummyLimit>() ] >>
                    ( XMLAttElem_p("PosLimit",
                        !(XMLAtt_p("refjoint", attrstr_p
                            [ var(_limit._refjoint) = arg1 ])) >>
                        (XMLAtt_p("min",real_p
                            [ var( _limit._min ) = arg1 ]) >>
                        XMLAtt_p("max",real_p
                            [ var( _limit._max ) = arg1 ]
                            [ var( _limit._type ) = PosLimitType ]))
						|
						(XMLAtt_p("max", real_p
							[var(_limit._max) = arg1]) >>
						XMLAtt_p("min", real_p
							[var(_limit._min) = arg1]
							[var(_limit._type) = PosLimitType]))
						,
                        eps_p
                     )
                    | XMLAttElem_p("VelLimit",
                        !(XMLAtt_p("refjoint", attrstr_p
                            [ var( _limit._refjoint) = arg1 ] )) >>
                        XMLAtt_p("max",real_p
                            [ var( _limit._max) = arg1 ]
                            [ var( _limit._type ) = VelLimitType ]),
                        eps_p
                      )
                    | XMLAttElem_p("AccLimit",
                        !(XMLAtt_p("refjoint", attrstr_p
                            [ var( _limit._refjoint) = arg1 ])) >>
                        XMLAtt_p("max",real_p
                            [ var( _limit._max) = arg1 ]
                            [ var( _limit._type ) = AccLimitType ]),
                        eps_p
                      )
                    ) [ jointlimit_r.result_ = var(_limit) ] ;

            }
        };
    };

    struct XMLWorkcellParser : public grammar<XMLWorkcellParser,
                                              result_closure<DummyWorkcell>::context_t>
    {
    public:
        template <typename ScannerT>
        struct definition
        {
        private:
            std::vector< std::string > _scope;
            DummyWorkcell _wc;
            rule<ScannerT> workcelldev_r, wc_r, camera_r, guardwrapper_r;
            rule<ScannerT, result_closure<DummyCollisionSetup>::context_t> colsetup_r;
			rule<ScannerT, result_closure<DummyProximitySetup>::context_t> proxsetup_r;
			rule<ScannerT, result_closure<DummyCalibration>::context_t> calibration_r;
            XMLDeviceParser device_p;
            ModelParser model_p;
            FrameParser frame_p;
            DummyCollisionSetup _setup;
			DummyProximitySetup _psetup;
			DummyCalibration _calibration;

        public:
            rule<ScannerT> const start() const { return guardwrapper_r; }

            definition( XMLWorkcellParser const &self ): model_p( _scope ),frame_p(_scope) {

                guardwrapper_r =
                    XMLErrorHandler::XMLErrorGuard( workcelldev_r )
                        [XMLErrorHandler()];

                workcelldev_r =
                        (wc_r | device_p[ AddDeviceToWorkcell( _wc , _scope ) ]) >> end_p
                            [ self.result_ = var(_wc) ];

                wc_r =
                    XMLAttElem_p("WorkCell",
                        XMLAtt_p("name",attrstr_p
                            [ var( _wc._name ) = arg1 ]),
                        *(
                             device_p
                                [ AddDeviceToWorkcell( _wc , _scope ) ]
                           | frame_p
                                [ AddFrameToWorkcell( _wc , _scope ) ]
                           | model_p
                           		[ push_back_a( _wc._models ) ]
                           | camera_r
                           | colsetup_r[ push_back_a(_wc._colmodels)]
						   | proxsetup_r[ push_back_a(_wc._proxmodels)]
				           | property_p[push_back_a( _wc._properties ) ]
						   | calibration_r[ push_back_a( _wc._calibration ) ]

                        )
                    );

                colsetup_r =
                    XMLAttElem_p("CollisionSetup",
                        XMLAtt_p("file", attrstr_p[ var(_setup._filename) = arg1 ]
                                                  [ var( _setup._scope ) = var( _scope ) ] >>
                        filepos_p[ var(_setup._pos) = arg1 ]
                        ),
                        eps_p
                    )[ colsetup_r.result_ = var(_setup) ];

				proxsetup_r =
                    XMLAttElem_p("ProximitySetup",
                        XMLAtt_p("file", attrstr_p[ var(_psetup._filename) = arg1 ]
                                                  [ var( _psetup._scope ) = var( _scope ) ] >>
                        filepos_p[ var(_psetup._pos) = arg1 ]
                        ),
                        eps_p
                    )[ proxsetup_r.result_ = var(_psetup) ];

                camera_r =
                    XMLAttElem_p("Camera",
                        XMLAtt_p("name",attrstr_p),
                        !transform3d_p
                    );
				calibration_r =
                		XMLAttElem_p("Calibration",
                				XMLAtt_p("file", attrstr_p[ var(_calibration._filename) = arg1 ]
                                                            [ var( _calibration._scope ) = var( _scope ) ] >>
                                  filepos_p[ var(_calibration._pos) = arg1 ]
                                  ),
                                  eps_p
                              )[ calibration_r.result_ = var(_calibration) ];

            }
        };
    };
} // end of namespace


boost::shared_ptr<DummyWorkcell> XMLRWParser::parseWorkcell( const std::string& filename){

    boost::shared_ptr< std::vector<char> > output( new std::vector<char>());
    boost::shared_ptr< std::vector< std::pair<size_t,file_position> > >
        filemap( new std::vector< std::pair<size_t,file_position> >() );

    if( ! rw::loaders::XMLRWPreParser::parse(filename, *output, *filemap) ){
        RW_THROW( "Pre-parsing of file \"" << filename << "\" failed!" );
    }

    return XMLRWParser::parseWorkcell( output, filemap );
}

boost::shared_ptr<DummyWorkcell> XMLRWParser::parseWorkcell(
        boost::shared_ptr< std::vector<char> > &data,
        boost::shared_ptr<std::vector< std::pair<size_t,file_position> > > &filemap)
{

    typedef MultipleFileIterator iterator_t;
    iterator_t first( data, filemap );
    iterator_t last( first.end() );

    boost::shared_ptr<DummyWorkcell> workcell(new DummyWorkcell);
    XMLWorkcellParser workcell_p;

    parse_info<iterator_t> info =
        parse( first, last, workcell_p[ var( *workcell ) = arg1 ],
            (space_p | "<!--" >> *(anychar_p - "-->") >> "-->")
        );

    if ( !info.hit ) {
        RW_THROW( "Parsing of workcell failed!!" );
    }

    return workcell;
}
