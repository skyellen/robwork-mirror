
#include <iostream>
#include <vector>


#include <xercesc/dom/DOMElement.hpp>

#include <rw/graphics/ProjectionMatrix.hpp>

#ifndef DAE_HPP_
#define DAE_HPP_

class Dae {
public:

    struct Data {
        Data():parent(NULL),initialized(false){}
        Data *parent;
        bool initialized;
        std::list<Data*> children;
    };


    struct SetParam : public Data  {
        std::string ref;
    };

    struct NewParam : public Data  {
        std::string sid;
    };

    struct Float: public Data { double value;};
    struct Int: public Data { int value;};
    struct Bool: public Data { bool value;};
    struct String: public Data {std::string value;};
    struct Extra: public Data {

    };

    //template <class T>
    //struct TechniqueCommon: public Data{ };
    // The technique common depends on parent type, so we use template specialization


    struct Technique: public Data {
        std::string profile, xmlns;
        // TODO: any wellformed xml data is okay here, so lets just save the XML node
        //std::vector<DOMElement*> elements;
    };

    struct Contributor: public Data {
        std::string author, authoring_tool, comments;
    };

    struct Unit: public Data{
        double meter;
        std::string name;
    };

    struct Asset: public Data {
        Contributor contributer;
        std::string created, modified;
        Unit unit;
        std::string up_axis;
    };


    struct TechniqueCommonOptics: public Data {
        rw::math::ProjectionMatrix projection;
    };

    struct Optics: public Data {
        TechniqueCommonOptics techniqueCommon;
        std::vector<Technique> technique;
        std::vector<Extra> extras;
    };


    struct Imager: public Data {
        Technique technique;
        std::vector<Extra> extras;
    };

    struct Camera: public Data {
        int id;
        std::string name;
        Asset asset;
        Optics optics;
        Imager imager;
        std::vector<Extra> extras;
    };




    template <class T>
    struct Library: public Data {
        std::string id, name;
        Asset asset;
        std::vector<T> elements;
        std::vector<Extra> extras;
    };

    /// KINEMATICS

    struct ParamKin: public Data {
        std::string ref;
    };

    struct Param: public Data {
        std::string name,
                    sid,
                    type,
                    semantic;
    };

    struct Bind: public Data {
        std::string symbol;
        // one of the following
        ParamKin param;
        Float floatVal;
        Int intVal;
        Bool boolVal;
        String sidref;
    };

    struct BindFX: public Data {
        std::string semantic, target;
    };

    struct AxisInfoKinematics: public Data {
        std::string sid, name, axis;
        std::vector<NewParam> params;
        bool active;
        bool locked;
        std::vector<int> indexes;
        double minLimit, maxLimit;
        //std::vector<Formula> formula;
        //std::vector<InstanceFormula> instanceFormula;
    };

    struct AxisInfoMotion: public Data {
        std::string sid, name, axis;
        std::vector<Bind> binds;
        std::vector<NewParam> newParams;
        std::vector<SetParam> setParams;
        double speed;
        double acceleration;
        double decceleration;
        double jerk;
    };

    struct TechniqueCommonMotion: public Data {
        std::vector<AxisInfoMotion> axisInfo;
        //std::vector<EffectorInfo> effectorInfo;
    };

    struct InstanceArticulatedSystem: public Data {
        std::string sid, name, url;
        std::vector<Bind> binds;
        std::vector<SetParam> setParams;
        std::vector<NewParam> newParams;
        std::vector<Extra> extras;
    };

    struct Motion: public Data {
        InstanceArticulatedSystem insArticulatedSystem;
        TechniqueCommonMotion techniqueCommon;
        std::vector<Technique> techniques;
        std::vector<Extra> extras;
    };

    template<class T>
    struct Array : public Data {
        unsigned int count;
        std::string id, name;
        rw::common::Ptr<std::vector<T> > data;
    };

    struct Accessor: public Data {
        unsigned int count;
        unsigned int offset;
        std::string source;
        unsigned int stride;

        std::vector<Param> params;
    };

    struct Source {
        std::string id, name;
        Asset asset;

        Array<bool> boolArray;
        Array<double> floatArray;
        Array<int> intArray;
        Array<std::string> idRefArray;
        Array<std::string> nameArray;
        Array<std::string> sidRefArray;
        Array<std::string> tokenArray;
        // technique common
        Accessor accessor;
        // technique
        Technique technique;

    };

    struct InputShared: public Data {
        unsigned int offset;
        std::string semantic;
        std::string source;
        unsigned int set;
    };

    struct Input: public Data {
        std::string semantic;
        std::string source;
    };

    struct Vertices: public Data {
        std::string id, name;
        std::vector<Input> inputs;
        std::vector<Extra> extras;
    };

    struct Triangles: public Data {
        std::string name;
        unsigned int count;
        std::string material;
        std::vector<InputShared> inputs;
        rw::common::Ptr<std::vector<unsigned int> > p;
        std::vector<Extra> extras;
    };

    struct Mesh: public Data {
        std::vector<Source> sources;
        Vertices vertices;
        std::vector<Triangles> tris;
        std::vector<Extra> extras;
    };


    struct Kinematics {

    };

    struct ArticulatedSystem {
        Asset asset;
        Kinematics kinematics;
        Motion motion;
        std::vector<Extra> extras;
    };

    struct Geometry {
        std::string id,name;
        Asset asset;
        //rw::geometry::Geometry::Ptr geom;
        std::vector<Extra> extras;

    };

    struct Transform {
        Transform():matrix(boost::numeric::ublas::zero_matrix<double>(4,4)){}
        std::string sid;
        rw::math::Transform3D<> transform;
        boost::numeric::ublas::bounded_matrix<double, 4, 4> matrix;
    };

    struct Node : public Data {
        std::string id,name, sid;
        std::string type; // NODE or JOINT, default is NODE
        //std::vector<std::string> layers;
        std::string layers;
        Asset asset;
        std::vector<Transform> transforms;
        std::vector<Node> nodes;
        std::vector<Extra> extras;
    };

    struct InstanceMaterial: public Data {
        std::string url;
        // technique override
        std::string ref, pass;

        std::vector<BindFX> binds;
        std::vector<Extra> extras;
    };

    struct Render {
        std::string name, sid;
        std::string cameraNode;

        std::vector<std::string> layers;
        InstanceMaterial iMaterial;
        std::vector<Extra> extras;
    };

    struct EvaluateScene {
        std::string id, name;
        std::string sid;
        bool enable;
        Asset asset;
        std::vector<Render> renders;
        std::vector<Extra> extras;
    };

    struct VisualScene {
        std::string id, name;
        Asset asset;
        std::vector<Node> nodes;
        std::vector<EvaluateScene> evaluateScenes;
        std::vector<Extra> extras;
    };

    struct InstanceEffect{

    };

    struct Material: public Data {
        std::string id, name;
        Asset asset;
        InstanceEffect iEffect;
        std::vector<Extra> extras;
    };

    // COMPLETE COLLADA SCENE
    struct Collada {
        std::string version, xmlns, base;

        Asset asset;

        //std::vector<Library<AnimationClips> > libAnimationClips;
        //std::vector<LibraryAnimation> libAnimations;
        std::vector<Library<ArticulatedSystem> > libArticulatedSystems;
        std::vector<Library<Camera> > libCameras;
        //std::vector<LibraryControllers> libControllers;
        //std::vector<LibraryEffects> libEffects;
        //std::vector<Library<ForceField> > libForceFields;
        //std::vector<Library<Formulas> > libFormulas;
        std::vector<Library<Geometry> > libGeometries;
        //std::vector<Library<Images> > libImages;
        //std::vector<Library<Joints> > libJoints;
        //std::vector<Library<KinematicsModel> > libKinematicsModels;
        //std::vector<Library<KinematicsScenes> > libKinematicsScenes;
        //std::vector<Library<Light> > libLights;
        std::vector<Library<Material> > libMaterials;
        std::vector<Library<Node> > libNodes;
        //std::vector<LibraryPhysicsMaterials> libPhysicsMaterials;
        //std::vector<LibraryPhysicsModels> libPhysicsModels;
        //std::vector<LibraryPhysicsScenes> libPhysicsScenes;
        std::vector<Library<VisualScene> > libVisualScenes;

        //Scene scene;
        std::vector<Extra> extras;
    };


    std::vector<Collada> data;
};

#endif /* DAE_HPP_ */
