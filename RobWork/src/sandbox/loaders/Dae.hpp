
#include <iostream>
#include <vector>


#include <xercesc/dom/DOMElement.hpp>

#include <rw/graphics/ProjectionMatrix.hpp>

#ifndef DAE_HPP_
#define DAE_HPP_

// we use this macro to define some standard interface functione on
#define DAE_DATA_TYPE(dae_type) \
    class dae_type: public Data {\
    public: \
        dae_type(Data* parent):Data(parent){} \
        dae_type(const std::string& id, const std::string& sid, Data* parent):Data(id, sid, parent){}

#define DAE_DATA_TYPE_I(dae_type, INIT_LIST) \
    class dae_type: public Data {\
    public: \
        dae_type(Data* parent):Data(parent),INIT_LIST{} \
        dae_type(const std::string& id, const std::string& sid, Data* parent):Data(id, sid, parent),INIT_LIST{}


class Dae {
public:

    class Asset;

    class Data {
    public:
        Data():parent(NULL),initialized(false),asset(NULL){}
        Data(Data *p):parent(p),initialized(true),asset(NULL){
            p->children.push_back(this);
        }
        Data(const std::string& id_, const std::string& sid_, Data *p):
            parent(p),id(id_),sid(sid_),initialized(true),asset(NULL){
            p->children.push_back(this);
        }
        virtual ~Data(){
            BOOST_FOREACH(Data* d, children){
                delete d;
            }
        };
        Data *parent;
        std::string id;
        std::string sid;
        bool initialized;
        std::list<Data*> children;
        std::list<Data*> scope;
        Asset *asset;
    };

    DAE_DATA_TYPE(SetParam)
        std::string ref;
    };

    DAE_DATA_TYPE(NewParam)

    };

    DAE_DATA_TYPE(Float)
        double value;};
    DAE_DATA_TYPE(Int)
        int value;};
    DAE_DATA_TYPE(Bool)
        bool value;};
    DAE_DATA_TYPE(String)
        std::string value;};
    DAE_DATA_TYPE(Extra)
    };

    //template <class T>
    //struct TechniqueCommon: public Data{ };
    // The technique common depends on parent type, so we use template specialization


    DAE_DATA_TYPE(Technique)
        std::string profile, xmlns;
        // TODO: any wellformed xml data is okay here, so lets just save the XML node
        //std::vector<DOMElement*> elements;
    };

    //DAE_DATA_TYPE(Contributor)
    class Contributor{
    public:
        std::string author, authoring_tool, comments;
    };

    //DAE_DATA_TYPE(Unit)
    class Unit{
    public:
        Unit():meter(1.0),name("meter"){}
        double meter;
        std::string name;
    };

    DAE_DATA_TYPE_I(Asset,up_axis("Z_UP"))
        Contributor contributer;
        std::string created, modified;
        Unit unit;
        std::string up_axis;
    };


    DAE_DATA_TYPE(TechniqueCommonOptics)
        rw::math::ProjectionMatrix projection;
    };

    DAE_DATA_TYPE(Optics)
        TechniqueCommonOptics *techniqueCommon;
        std::vector<Technique*> technique;
        std::vector<Extra*> extras;
    };


    DAE_DATA_TYPE(Imager)
        Technique *technique;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(InstanceCamera)
        std::string name, url;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Camera)
        std::string name;
        //Asset *asset;
        Optics *optics;
        Imager *imager;
        std::vector<Extra*> extras;
    };




    template <class T>
    DAE_DATA_TYPE(Library)
        std::string name;
        //Asset *asset;
        std::vector<T*> elements;
        std::vector<Extra*> extras;
    };

    /// KINEMATICS

    DAE_DATA_TYPE(ParamKin)
        std::string ref;
    };

    DAE_DATA_TYPE(Param)
        std::string name,
                    type,
                    semantic;
    };

    DAE_DATA_TYPE(Bind)
        std::string symbol;
        // one of the following
        ParamKin *param;
        Float *floatVal;
        Int *intVal;
        Bool *boolVal;
        std::string sidref;
    };

    DAE_DATA_TYPE(BindFX)
        std::string semantic, target;
    };

    DAE_DATA_TYPE(AxisInfoKinematics)
        std::string name, axis;
        std::vector<NewParam*> params;
        bool active;
        bool locked;
        std::vector<int> indexes;
        double minLimit, maxLimit;
        //std::vector<Formula> formula;
        //std::vector<InstanceFormula> instanceFormula;
    };

    DAE_DATA_TYPE(AxisInfoMotion)
        std::string  name, axis;
        std::vector<Bind*> binds;
        std::vector<NewParam*> newParams;
        std::vector<SetParam*> setParams;
        double speed;
        double acceleration;
        double decceleration;
        double jerk;
    };

    DAE_DATA_TYPE(TechniqueCommonMotion)
        std::vector<AxisInfoMotion*> axisInfo;
        //std::vector<EffectorInfo> effectorInfo;
    };

    DAE_DATA_TYPE(InstanceArticulatedSystem)
        std::string  name, url;
        std::vector<Bind*> binds;
        std::vector<SetParam*> setParams;
        std::vector<NewParam*> newParams;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Motion)
        InstanceArticulatedSystem *insArticulatedSystem;
        TechniqueCommonMotion *techniqueCommon;
        std::vector<Technique*> techniques;
        std::vector<Extra*> extras;
    };

    template<class T>
    DAE_DATA_TYPE(Array )
        unsigned int count;
        std::string  name;
        rw::common::Ptr<std::vector<T> > data;
    };

    DAE_DATA_TYPE(Accessor)
        unsigned int count;
        unsigned int offset;
        std::string source;
        unsigned int stride;

        std::vector<Param*> params;
    };

    DAE_DATA_TYPE(Source)
        std::string name;
        //Asset *asset;

        Array<bool> *boolArray;
        Array<double> *floatArray;
        Array<int> *intArray;
        Array<std::string> *idRefArray;
        Array<std::string> *nameArray;
        Array<std::string> *sidRefArray;
        Array<std::string> *tokenArray;
        // technique common
        Accessor *accessor;
        // technique
        Technique *technique;

    };

    DAE_DATA_TYPE(InputShared)
        unsigned int offset;
        std::string semantic;
        std::string source;
        unsigned int set;
    };

    DAE_DATA_TYPE(Input)
        std::string semantic;
        std::string source;
    };

    DAE_DATA_TYPE(Vertices)
        std::string  name;
        std::vector<Input*> inputs;
        std::vector<Extra*> extras;

    };

    DAE_DATA_TYPE(Triangles)
        std::string name;
        unsigned int count;
        std::string material;
        std::vector<InputShared*> inputs;
        rw::common::Ptr<std::vector<unsigned int> > p;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Mesh)
        std::vector<Source*> sources;
        Vertices *vertices;
        std::vector<Triangles*> tris;
        std::vector<Extra*> extras;
    };


    DAE_DATA_TYPE(Kinematics)

    };

    DAE_DATA_TYPE(ArticulatedSystem)
        //Asset *asset;
        Kinematics *kinematics;
        Motion *motion;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Geometry)
        std::string name;
        //Asset *asset;
        //rw::geometry::Geometry::Ptr geom;
        //std::vector<Dae::ConvexMesh> cmeshes;
        std::vector<Dae::Mesh*> meshes;
        //std::vector<Dae::Spline> splines;
        //std::vector<Dae::BRep> breps;
        std::vector<Extra*> extras;

    };

    DAE_DATA_TYPE(InstanceGeometry)
        std::string name;
        std::string url;
        //std::vector<BindMaterial*> bindMaterials;
        std::vector<Extra*> extras;
    };


    DAE_DATA_TYPE(Transform)
        Transform():matrix(boost::numeric::ublas::zero_matrix<double>(4,4)){}

        rw::math::Transform3D<> transform;
        boost::numeric::ublas::bounded_matrix<double, 4, 4> matrix;
    };

    DAE_DATA_TYPE(InstanceNode)
        std::string name;
        std::string url;
        std::string proxy;
    };

    DAE_DATA_TYPE(Node)
        std::string name;
        std::string type; // NODE or JOINT, default is NODE
        //std::vector<std::string> layers;
        std::string layers;
        //Asset *asset;
        std::vector<Transform*> transforms;
        std::vector<InstanceCamera*> icameras;
        //std::vector<InstanceController*> icontrollers;
        std::vector<InstanceGeometry*> igeometries;
        //std::vector<InstanceLight*> ilights;
        std::vector<InstanceNode*> inodes;
        std::vector<Node*> nodes;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(InstanceMaterial)
        std::string url;
        // technique override
        std::string ref, pass;

        std::vector<BindFX*> binds;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Render)
        std::string name;
        std::string cameraNode;

        std::vector<std::string> layers;
        InstanceMaterial *iMaterial;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(EvaluateScene)
        std::string  name;

        bool enable;
        //Asset *asset;
        std::vector<Render*> renders;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(VisualScene)
        std::string  name;
        //Asset *asset;
        std::vector<Node*> nodes;
        std::vector<EvaluateScene*> evaluateScenes;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(InstanceVisualScene)
        std::string name;
        std::string url;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(InstanceEffect)
        std::string name;
        std::string url;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Material)
        std::string  name;
        //Asset *asset;
        InstanceEffect* iEffect;
        std::vector<Extra*> extras;
    };

    typedef enum{ConstantShader, LambertShader, PhongShader,BlinnShader} ShaderTypeCommonFX;

    DAE_DATA_TYPE(ShaderElementCommenFX)
        ShaderTypeCommonFX shaderType;
        float emission[4];
        float ambient[4];
        float diffuse[4];
        float specular[4];
        float shininess[4];
        float reflective[4];
        float reflectivity;
        float transparent[4];
        float transparency[4];
        float idxOfRefraction;
    };

    DAE_DATA_TYPE(TechniqueCommenFX)
        std::vector<ShaderElementCommenFX*> shaderElements;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(ProfileCOMMON)
        std::vector<Param*> params;
        TechniqueCommenFX* technique;
        std::vector<Extra*> extras;
    };

    DAE_DATA_TYPE(Effect)
        std::string  name;
        std::vector<Param*> params;

        // profiles
        //std::vector<ProfileBRIDGE*> pBRIDGE;
        //std::vector<ProfileCG*> pCG;
        //std::vector<ProfileGLES*> pGLES;
        //std::vector<ProfileGLES2*> pGLES2;
        //std::vector<ProfileGLSL*> pGLSL;
        std::vector<ProfileCOMMON*> pCommon;

        std::vector<Extra*> extras;
    };


    // COMPLETE COLLADA SCENE
    DAE_DATA_TYPE(Collada)
        Collada():Data(){}
        std::string version, xmlns, base;

        //Asset *asset;

        //std::vector<Library<AnimationClips> > libAnimationClips;
        //std::vector<LibraryAnimation> libAnimations;
        std::vector<Library<ArticulatedSystem>* > libArticulatedSystems;
        std::vector<Library<Camera>* > libCameras;
        //std::vector<LibraryControllers> libControllers;
        std::vector<Library<Effect>* > libEffects;
        //std::vector<Library<ForceField> > libForceFields;
        //std::vector<Library<Formulas> > libFormulas;
        std::vector<Library<Geometry>* > libGeometries;
        //std::vector<Library<Images> > libImages;
        //std::vector<Library<Joints> > libJoints;
        //std::vector<Library<KinematicsModel> > libKinematicsModels;
        //std::vector<Library<KinematicsScenes> > libKinematicsScenes;
        //std::vector<Library<Light> > libLights;
        std::vector<Library<Material>* > libMaterials;
        std::vector<Library<Node>* > libNodes;
        //std::vector<LibraryPhysicsMaterials> libPhysicsMaterials;
        //std::vector<LibraryPhysicsModels> libPhysicsModels;
        //std::vector<LibraryPhysicsScenes> libPhysicsScenes;
        std::vector<Library<VisualScene>* > libVisualScenes;

        //Scene scene;
        std::vector<Extra*> extras;

        /*
        template<class T>
        T* find(std::vector<Library<T> >& libs){
            BOOST_FOREACH(Library<T>& lib, libs){
                lib.
            }
        }

        T* getGeometry(const std::string& url){
            std::vector<Library<T> > &libs = libGeometries;

        }
        */

        Data* getData(const std::string& scope, const std::string& url){
            // first extract element id

            return NULL;
        }

        Data* getData(const std::string& url){
            // look in all libraries
            if(idToData.find(url)==idToData.end())
                RW_THROW("Could Not find data: " << url);
            return idToData[url];
        }

        void addData(Data* data){
            //std::cout << "ADDING DATA: " << data->id << " : " << data->sid << std::endl;
            if(data->id!="")
                idToData[data->id] = data;
        }

        void printAllData(){
            typedef std::pair<std::string, Data*> DataPair;
            std::cout << "\n All data saved so far:" << std::endl;
            BOOST_FOREACH(const DataPair& idAndData, idToData){
                std::cout << idAndData.first << std::endl;
            }
        }


        std::map<std::string, Data*> idToData;

    };


    std::vector<Collada> data;

};

#endif /* DAE_HPP_ */
