%module rw

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/loaders/xml/XMLPropertyLoader.hpp>
#include <rw/loaders/xml/XMLPropertySaver.hpp>
#if defined (SWIGLUA)
#include <rwlibs/swig/lua/Lua.hpp>
#endif

using namespace rwlibs::swig;
using rw::math::Metric;
using rw::trajectory::Interpolator;
using rw::trajectory::Blend;
using rw::trajectory::Path;
using rw::trajectory::Timed;
using rw::trajectory::Trajectory;
using rw::trajectory::InterpolatorTrajectory;
using rw::pathplanning::PathPlanner;
using rwlibs::task::Task;
%}

%pragma(java) jniclassclassmodifiers="class"

%include <std_string.i>
%include <std_vector.i>
%include <shared_ptr.i>

#if !defined(SWIGJAVA)
%include "carrays.i"
%array_class(double, doubleArray);
#else
%include "arrays_java.i";
#endif

#if defined(SWIGJAVA)
	%rename(multiply) operator*;
	%rename(divide) operator/;
	%rename(equals) operator==;
	%rename(negate) operator-() const;
#endif

%include <stl.i>

void writelog(const std::string& msg);

/********************************************
 * General utility functions
 ********************************************/

%inline %{
    void sleep(double t){
        ::rw::common::TimerUtil::sleepMs( (int) (t*1000) );
    }
    double time(){
        return ::rw::common::TimerUtil::currentTime( );
    }
    long long timeMs(){
        return ::rw::common::TimerUtil::currentTimeMs( );
    }
    void info(const std::string& msg){
        ::rw::common::Log::infoLog() << msg;
    }
    void debug(const std::string& msg){
        ::rw::common::Log::debugLog() << msg;
    }
    void warn(const std::string& msg){
        ::rw::common::Log::warningLog() << msg;
    }
    void error(const std::string& msg){
        ::rw::common::Log::errorLog() << msg;
    }
%}



/********************************************
 * Constants
 ********************************************/

%constant double Pi = rw::math::Pi;
%constant double Inch2Meter = rw::math::Inch2Meter;
%constant double Meter2Inch = rw::math::Meter2Inch;
%constant double Deg2Rad = rw::math::Deg2Rad;
%constant double Rad2Deg = rw::math::Rad2Deg;

/********************************************
 * STL vectors (primitive types)
 ********************************************/

namespace std {
	%template(StringVector) std::vector<string>;
	%template(DoubleVector) std::vector<double>;
	%template(IntVector) std::vector<int>;
};

/********************************************
 * COMMON
 ********************************************/

namespace rw { namespace common {
template<class T> class Ptr {
public:
    Ptr();
     
#if defined(SWIGJAVA)
 %typemap (in) T* %{
  jclass objcls = jenv->GetObjectClass(jarg1_);
  const jfieldID memField = jenv->GetFieldID(objcls, "swigCMemOwn", "Z");
  jenv->SetBooleanField(jarg1_, memField, (jboolean)false);
  $1 = *(T **)&jarg1;
 %} 
    Ptr(T* ptr);
 %clear T*;
#else
    Ptr(T* ptr);
#endif

    bool isShared();
    bool isNull();
    //bool operator==(void* p) const;

    template<class A>
    bool operator==(const rw::common::Ptr<A>& p) const;
#if defined(SWIGJAVA)
	%rename(dereference) get;
#endif
    T* get() const;
    T *operator->() const;
};
}}

/** @addtogroup swig */
/* @{ */

//! @copydoc rw::common::PropertyMap
class PropertyMap
{
public: 
	//! @copydoc rw::common::PropertyMap::PropertyMap
	PropertyMap();
	//! @copydoc rw::common::PropertyMap::has
	bool has(const std::string& identifier) const;
    //! @copydoc rw::common::PropertyMap::size
    size_t size() const;
    //! @copydoc rw::common::PropertyMap::empty 
    bool empty() const;
    //! @copydoc rw::common::PropertyMap::erase
    bool erase(const std::string& identifier);
    
	%extend {
		
		std::string& getString(const std::string& id){ return $self->get<std::string>(id); }
		void setString(const std::string& id, std::string val){  $self->set<std::string>(id,val); }
		
		std::vector<std::string>& getStringList(const std::string& id){ return $self->get<std::vector<std::string> >(id); }
		void setStringList(const std::string& id, std::vector<std::string> val){ $self->set<std::vector<std::string> >(id,val); }
		
		Q& getQ(const std::string& id){ return $self->get<Q>(id); }
		void setQ(const std::string& id, Q q){ $self->set<Q>(id, q); }

		Pose6D& getPose(const std::string& id){ return $self->get<Pose6D>(id); }
		void setPose6D(const std::string& id, Pose6D p){  $self->set<Pose6D>(id, p); }
		
		rw::math::Vector3D<double>& getVector3(const std::string& id){ return $self->get<rw::math::Vector3D<double> >(id); }
		void setVector3(const std::string& id, rw::math::Vector3D<double> p){  $self->set<rw::math::Vector3D<double> >(id, p); }

		Transform3D& getTransform3D(const std::string& id){ return $self->get<Transform3D>(id); }
		void setTransform3D(const std::string& id, Transform3D p){  $self->set<Transform3D>(id, p); }

		PropertyMap& getMap(const std::string& id){ return $self->get<PropertyMap>(id); }
		void setMap(const std::string& id, PropertyMap p){  $self->set<PropertyMap>(id, p); }
		
		void load(const std::string& filename){ *($self) = rw::loaders::XMLPropertyLoader::load(filename); }
		void save(const std::string& filename){ rw::loaders::XMLPropertySaver::save( *($self), filename ); }
		
	}    
 
};
%template (PropertyMapPtr) rw::common::Ptr<PropertyMap>;



class ThreadPool { 
public:
    ThreadPool(int threads = -1);
    virtual ~ThreadPool();
    unsigned int getNumberOfThreads() const;
    void stop();
    bool isStopping();
//    typedef boost::function<void(ThreadPool*)> WorkFunction;
//    void addWork(WorkFunction work);
	unsigned int getQueueSize();
	void waitForEmptyQueue();
};

%template (ThreadPoolPtr) rw::common::Ptr<ThreadPool>;

class ThreadTask {
public:
	typedef enum TaskState {
    	INITIALIZATION,
    	IN_QUEUE,
    	EXECUTING,
    	CHILDREN,
    	IDLE,
    	POSTWORK,
    	DONE
    } TaskState;

	ThreadTask(rw::common::Ptr<ThreadTask> parent);
	ThreadTask(rw::common::Ptr<ThreadPool> pool);
	virtual ~ThreadTask();
	bool setThreadPool(rw::common::Ptr<ThreadPool> pool);
	rw::common::Ptr<ThreadPool> getThreadPool();
	//virtual void run();
	//virtual void subTaskDone(ThreadTask* subtask);
	//virtual void idle();
	//virtual void done();
    bool execute();
    TaskState wait(ThreadTask::TaskState previous);
    void waitUntilDone();
    TaskState getState();
    bool addSubTask(rw::common::Ptr<ThreadTask> subtask);
    std::vector<rw::common::Ptr<ThreadTask> > getSubTasks();
    void setKeepAlive(bool keepAlive);
    bool keepAlive();
};

%template (ThreadTaskPtr) rw::common::Ptr<ThreadTask>;
%template (ThreadTaskPtrVector) std::vector<rw::common::Ptr<ThreadTask> >;

class Plugin {
protected:
	 Plugin(const std::string& id, const std::string& name, const std::string& version);
	 
public:
	const std::string& getId();
    const std::string& getName();
    const std::string& getVersion();
};

%template (PluginPtr) rw::common::Ptr<Plugin>;
%template (PluginPtrVector) std::vector<rw::common::Ptr<Plugin> >;

class Extension {
public:
	//struct Descriptor {
	//};
	
	Extension(ExtensionDescriptor desc, Plugin* plugin);
	
	const std::string& getId();
	const std::string& getName();
};

%template (ExtensionPtr) rw::common::Ptr<Extension>;
%template (ExtensionPtrVector) std::vector<rw::common::Ptr<Extension> >;

class ExtensionRegistry {
public:
	ExtensionRegistry();
	static rw::common::Ptr<ExtensionRegistry> getInstance();
	std::vector<rw::common::Ptr<Extension> > getExtensions(const std::string& ext_point_id) const;
	std::vector<rw::common::Ptr<Plugin> > getPlugins() const;
};

%template (ExtensionRegistryPtr) rw::common::Ptr<ExtensionRegistry>;

/********************************************
 * ROBWORK CLASS
 ********************************************/ 
 class RobWork {
 public:
	RobWork();
	
	static rw::common::Ptr<RobWork> getInstance();
	
	std::string getVersion() const;
	void initialize();
 };
 
 %template (RobWorkPtr) rw::common::Ptr<RobWork>;

/********************************************
 * GEOMETRY
 ********************************************/

class GeometryData {
public:
    typedef enum {PlainTriMesh,
                  IdxTriMesh,
                  SpherePrim, BoxPrim, OBBPrim, AABBPrim,
                  LinePrim, PointPrim, PyramidPrim, ConePrim,
                  TrianglePrim, CylinderPrim, PlanePrim, RayPrim,
                  UserType} GeometryType;

    virtual GeometryType getType() const = 0;
    virtual rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true) = 0;
    static std::string toString(GeometryType type);
};

%template (GeometryDataPtr) rw::common::Ptr<GeometryData>;

class TriMesh: public GeometryData {
public:
    virtual Triangle getTriangle(size_t idx) const = 0;
    virtual void getTriangle(size_t idx, Triangle& dst) const = 0;
    virtual void getTriangle(size_t idx, Trianglef& dst) const = 0;
    virtual size_t getSize() const = 0;
    virtual size_t size() const = 0;
    virtual rw::common::Ptr<TriMesh> clone() const = 0;
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    //rw::common::Ptr<const TriMesh> getTriMesh(bool forceCopy=true) const;
};

%template (TriMeshPtr) rw::common::Ptr<TriMesh>;

class Primitive: public GeometryData {
public:
    rw::common::Ptr<TriMesh> getTriMesh(bool forceCopy=true);
    virtual rw::common::Ptr<TriMesh> createMesh(int resolution) const = 0;
    virtual Q getParameters() const = 0;
};

class Sphere: public Primitive {
public:
    //! constructor
    Sphere(const Q& initQ);
    Sphere(double radi):_radius(radi);
    double getRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    Q getParameters() const;
    GeometryData::GeometryType getType() const;
};

class Box: public Primitive {
public:
    Box();
    Box(double x, double y, double z);
    Box(const Q& initQ);
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    Q getParameters() const;
    GeometryType getType() const;
};

class Cone: public Primitive {
public:
    Cone(const Q& initQ);
    Cone(double height, double radiusTop, double radiusBot);
    double getHeight();
    double getTopRadius();
    double getBottomRadius();
    rw::common::Ptr<TriMesh> createMesh(int resolution) const;
    Q getParameters() const;
    GeometryType getType() const;
};

class Plane: public Primitive {
public:
    Plane(const Q& q);
    Plane(const rw::math::Vector3D<double>& n, double d);
    Plane(const rw::math::Vector3D<double>& p1,
          const rw::math::Vector3D<double>& p2,
          const rw::math::Vector3D<double>& p3);

    rw::math::Vector3D<double>& normal();
    //const rw::math::Vector3D<double>& normal() const;
    double& d();
    //double d() const;
    double distance(const rw::math::Vector3D<double>& point);
    double refit( std::vector<rw::math::Vector3D<double> >& data );
    rw::common::Ptr<TriMesh> createMesh(int resolution) const ;
    Q getParameters() const;
    GeometryType getType() const;
};

class Cylinder: public Primitive {
public:
	Cylinder();
	Cylinder(float radius, float height);
	virtual ~Cylinder();
	double getRadius() const;
	double getHeight() const;
	rw::common::Ptr<TriMesh> createMesh(int resolution) const;
	Q getParameters() const;
	GeometryType getType() const;
};

class ConvexHull3D {
public:
    virtual void rebuild(const std::vector<rw::math::Vector3D<double> >& vertices) = 0;
    virtual bool isInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistInside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual double getMinDistOutside(const rw::math::Vector3D<double>& vertex) = 0;
    virtual rw::common::Ptr<PlainTriMeshN1> toTriMesh() = 0;
};


class Geometry {
public:
    Geometry(rw::common::Ptr<GeometryData> data, double scale=1.0);

    Geometry(rw::common::Ptr<GeometryData> data,
             const Transform3D& t3d,
             double scale=1.0);

    double getScale() const;
    void setScale(double scale);
    void setTransform(const Transform3D& t3d);
    const Transform3D& getTransform() const;
    rw::common::Ptr<GeometryData> getGeometryData();
#if !defined(SWIGJAVA)
    const rw::common::Ptr<GeometryData> getGeometryData() const;
#endif
    void setGeometryData(rw::common::Ptr<GeometryData> data);
    const std::string& getName() const;
    const std::string& getId() const;
    void setName(const std::string& name);
    void setId(const std::string& id);
    static rw::common::Ptr<Geometry> makeSphere(double radi);
    static rw::common::Ptr<Geometry> makeBox(double x, double y, double z);
    static rw::common::Ptr<Geometry> makeCone(double height, double radiusTop, double radiusBot);
    static rw::common::Ptr<Geometry> makeCylinder(float radius, float height);
};

%template (GeometryPtr) rw::common::Ptr<Geometry>;

class STLFile {
public:
    static void save(const TriMesh& mesh, const std::string& filename);
    static rw::common::Ptr<PlainTriMeshN1f> load(const std::string& filename);
};

class PlainTriMeshN1
{
};

%template (PlainTriMeshN1Ptr) rw::common::Ptr<PlainTriMeshN1>;

class PlainTriMeshN1f
{
};

%template (PlainTriMeshN1fPtr) rw::common::Ptr<PlainTriMeshN1f>;

%nodefaultctor Triangle;
class Triangle
{
};

%nodefaultctor Trianglef;
class Trianglef
{
};


class PointCloud: public GeometryData {
	public:
        PointCloud();
        PointCloud(int w, int h);

/*
		GeometryType getType() const;
		size_t size() const;
		bool isOrdered();
	    std::vector<rw::math::Vector3D<float> >& getData();
	    const std::vector<rw::math::Vector3D<float> >& getData() const;
        const rw::math::Vector3D<float>& operator()(int x, int y) const;
	    rw::math::Vector3D<float>& operator()(int x, int y);
	    int getWidth() const;
        int getHeight() const;
	    void resize(int w, int h);

		static rw::common::Ptr<PointCloud> loadPCD( const std::string& filename );

        static void savePCD( const PointCloud& cloud,
                                                    const std::string& filename ,
                                                    const rw::math::Transform3D<float>& t3d =
	                                                            rw::math::Transform3D<float>::identity());
*/
	};


/********************************************
 * GRAPHICS
 ********************************************/

%template (WorkCellScenePtr) rw::common::Ptr<WorkCellScene>;
%template (DrawableNodePtr) rw::common::Ptr<DrawableNode>;
%template (DrawableNodePtrVector) std::vector<rw::common::Ptr<DrawableNode> >;
%constant int DNodePhysical = DrawableNode::Physical;
%constant int DNodeVirtual = DrawableNode::Virtual;
%constant int DNodeDrawableObject = DrawableNode::DrawableObject;
%constant int DNodeCollisionObject = DrawableNode::CollisionObject;
%nodefaultctor DrawableNode;
%nodefaultctor WorkCellScene;

class DrawableNode {
public:

    enum DrawType {
        SOLID, //! Render in solid
        WIRE, //! Render in wireframe
        OUTLINE //! Render both solid and wireframe
    };

    virtual void setHighlighted(bool b) = 0;

    virtual bool isHighlighted() const = 0;

    virtual void setDrawType(DrawType drawType) = 0;

    virtual void setTransparency(float alpha) = 0;

    virtual float getTransparency() = 0;

    bool isTransparent();

    virtual void setScale(float scale) = 0;

    virtual float getScale() const = 0;

    virtual void setVisible(bool enable) = 0;

    virtual bool isVisible() = 0;

    virtual const Transform3D& getTransform() const  = 0;

    virtual void setTransform(const Transform3D& t3d) = 0;

    virtual void setMask(unsigned int mask) = 0;
    virtual unsigned int getMask() const = 0;
};

class WorkCellScene {
 public:

     rw::common::Ptr<WorkCell> getWorkCell();

     void setState(const State& state);

     //rw::graphics::GroupNode::Ptr getWorldNode();
     void updateSceneGraph(rw::kinematics::State& state);
     //void clearCache();

     void setVisible(bool visible, Frame* f);

     bool isVisible(Frame* f);

     void setHighlighted( bool highlighted, Frame* f);
     bool isHighlighted( Frame* f);
     void setFrameAxisVisible( bool visible, Frame* f);
     bool isFrameAxisVisible( Frame* f);
     //void setDrawType( DrawableNode::DrawType type, Frame* f);
     //DrawableNode::DrawType getDrawType( Frame* f );

     void setDrawMask( unsigned int mask, Frame* f);
     unsigned int getDrawMask( Frame* f );
     void setTransparency(double alpha, Frame* f);

     //DrawableGeometryNode::Ptr addLines( const std::string& name, const std::vector<rw::geometry::Line >& lines, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
     //DrawableGeometryNode::Ptr addGeometry(const std::string& name, rw::geometry::Geometry::Ptr geom, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
     DrawableNode::Ptr addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
     //DrawableNode::Ptr addModel3D(const std::string& name, Model3D::Ptr model, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);
     //DrawableNode::Ptr addImage(const std::string& name, const rw::sensor::Image& img, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
     //DrawableNode::Ptr addScan(const std::string& name, const rw::sensor::Scan2D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
     //DrawableNode::Ptr addScan(const std::string& name, const rw::sensor::Image25D& scan, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
     //DrawableNode::Ptr addRender(const std::string& name, rw::graphics::Render::Ptr render, rw::kinematics::Frame* frame, int dmask=DrawableNode::Physical);

     rw::common::Ptr<DrawableNode> addDrawable(const std::string& filename, Frame* frame, int dmask);
     void addDrawable(rw::common::Ptr<DrawableNode> drawable, Frame*);

     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables();
     //std::vector<rw::common::Ptr<DrawableNode> > getDrawables(Frame* f);

     //std::vector<DrawableNode::Ptr> getDrawablesRec(rw::kinematics::Frame* f, rw::kinematics::State& state);
     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name);

     rw::common::Ptr<DrawableNode> findDrawable(const std::string& name, Frame* frame);

     std::vector<rw::common::Ptr<DrawableNode> > findDrawables(const std::string& name);

     bool removeDrawables(Frame* f);

     bool removeDrawables(const std::string& name);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable);

     bool removeDrawable(rw::common::Ptr<DrawableNode> drawable, Frame* f);

     bool removeDrawable(const std::string& name);
     bool removeDrawable(const std::string& name, Frame* f);
     Frame* getFrame(rw::common::Ptr<DrawableNode>  d);

     //rw::graphics::GroupNode::Ptr getNode(rw::kinematics::Frame* frame);
 };
 
 %nodefaultctor SceneViewer;
 class SceneViewer
 {
 };
 
 %template (SceneViewerPtr) rw::common::Ptr<SceneViewer>;

/********************************************
 * GRASPPLANNING
 ********************************************/

/********************************************
 * INVKIN
 ********************************************/

class InvKinSolver
{
public:
    virtual std::vector<Q> solve(const Transform3D& baseTend, const State& state) const = 0;
    virtual void setCheckJointLimits(bool check) = 0;
};

%template (InvKinSolverPtr) rw::common::Ptr<InvKinSolver>;

class IterativeIK: public InvKinSolver
{
public:
    virtual void setMaxError(double maxError);

    virtual double getMaxError() const;

    virtual void setMaxIterations(int maxIterations);

    virtual int getMaxIterations() const;

    virtual PropertyMap& getProperties();
#if !defined(SWIGJAVA)
    virtual const PropertyMap& getProperties() const;
#endif
    static rw::common::Ptr<IterativeIK> makeDefault(rw::common::Ptr<Device> device, const State& state);
};

%template (IterativeIKPtr) rw::common::Ptr<IterativeIK>;

class JacobianIKSolver : public IterativeIK
{
public:
    typedef enum{Transpose, SVD, DLS, SDLS} JacobianSolverType;

    JacobianIKSolver(rw::common::Ptr<Device> device, const State& state);

    JacobianIKSolver(rw::common::Ptr<Device> device, Frame *foi, const State& state);

    std::vector<Q> solve(const Transform3D& baseTend, const State& state) const;

    void setInterpolatorStep(double interpolatorStep);

     void setEnableInterpolation(bool enableInterpolation);

     bool solveLocal(const Transform3D &bTed,
                     double maxError,
                     State &state,
                     int maxIter) const;

     void setClampToBounds(bool enableClamping);

     void setSolverType(JacobianSolverType type);

     void setCheckJointLimits(bool check);

};

%template (JacobianIKSolverPtr) rw::common::Ptr<JacobianIKSolver>;

class IKMetaSolver: public IterativeIK
{
public:
    IKMetaSolver(rw::common::Ptr<IterativeIK> iksolver,
        const rw::common::Ptr<Device> device,
        rw::common::Ptr<CollisionDetector> collisionDetector = NULL);

    //IKMetaSolver(rw::common::Ptr<IterativeIK> iksolver,
    //    const rw::common::Ptr<Device> device,
    //    rw::common::Ptr<QConstraint> constraint);

    std::vector<Q> solve(const Transform3D& baseTend, const State& state) const;

    void setMaxAttempts(size_t maxAttempts);

    void setStopAtFirst(bool stopAtFirst);

    void setProximityLimit(double limit);

    void setCheckJointLimits(bool check);

    std::vector<Q> solve(const Transform3D& baseTend, const State& state, size_t cnt, bool stopatfirst) const;

};

%template (IKMetaSolverPtr) rw::common::Ptr<IKMetaSolver>;

class ClosedFormIK: public InvKinSolver
{
public:
    static rw::common::Ptr<ClosedFormIK> make(const Device& device, const State& state);
};


class PieperSolver: public ClosedFormIK {
public:
    PieperSolver(const std::vector<DHParameterSet>& dhparams,
                 const Transform3D& joint6Tend,
                 const Transform3D& baseTdhRef = Transform3D::identity());

    PieperSolver(SerialDevice& dev, const Transform3D& joint6Tend, const State& state);

    virtual std::vector<Q> solve(const Transform3D& baseTend, const State& state) const;

    virtual void setCheckJointLimits(bool check);

};

%template (ClosedFormIKPtr) rw::common::Ptr<ClosedFormIK>;

/********************************************
 * KINEMATICS
 ********************************************/

%nodefaultctor State;
class State
{
public:
	std::size_t size() const;
};
%template (StateVector) std::vector<State>;

class StateData {
protected:
    StateData(int size, const std::string& name);
public:
    const std::string& getName();
    int size() const;
    double* getData(State& state);
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    void setData(State& state, const double* vals) const;
};

class StateStructure {
public:
	StateStructure();
	 
	void addFrame(Frame *frame, Frame *parent=NULL);
	const State& getDefaultState() const;
	const std::vector<Frame*>& getFrames() const;
};
%template (StateStructurePtr) rw::common::Ptr<StateStructure>;

class Frame : public StateData
{
public:

    Transform3D getTransform(const State& state) const;
    PropertyMap& getPropertyMap();
    int getDOF() const ;
    Frame* getParent() ;
    Frame* getParent(const State& state);
    Frame* getDafParent(const State& state);

#if !defined(SWIGJAVA) 
    const PropertyMap& getPropertyMap() const ; 
    const Frame* getParent() const ;
    const Frame* getParent(const State& state) const;
    const Frame* getDafParent(const State& state) const;
#endif
/*
    typedef rw::common::ConcatVectorIterator<Frame> iterator;
    typedef rw::common::ConstConcatVectorIterator<Frame> const_iterator;

    typedef std::pair<iterator, iterator> iterator_pair;
    typedef std::pair<const_iterator, const_iterator> const_iterator_pair;

    const_iterator_pair getChildren() const;
    iterator_pair getChildren();
    const_iterator_pair getChildren(const State& state) const;
    iterator_pair getChildren(const State& state);
    const_iterator_pair getDafChildren(const State& state) const;
    iterator_pair getDafChildren(const State& state);
*/
    %extend {
        Transform3D wTt(const State& state) const{
            return ::rw::kinematics::Kinematics::worldTframe($self, state);
        }

        Transform3D fTf(const Frame* frame, const State& state) const{
            return ::rw::kinematics::Kinematics::frameTframe($self, frame, state);
        }
    }

    void attachTo(Frame* parent, State& state);
    bool isDAF();

private:
    // Frames should not be copied.
    Frame(const Frame&);
    Frame& operator=(const Frame&);
};

%template (FrameVector) std::vector<Frame*>;

class MovableFrame: public Frame{
public:
   explicit MovableFrame(const std::string& name);

   void setTransform(const Transform3D& transform, State& state);
};

class FixedFrame: public Frame {
public:
    FixedFrame(const std::string& name, const Transform3D& transform);
    void setTransform(const Transform3D& transform);

    const Transform3D& getFixedTransform() const;
};

%inline %{
    Transform3D frameTframe(const Frame* from, const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::frameTframe(from, to, state );
    }

    Transform3D worldTframe(const Frame* to, const State& state){
        return ::rw::kinematics::Kinematics::worldTframe( to,  state);
    }

    Frame* worldFrame(Frame* frame, const State& state) {
        return ::rw::kinematics::Kinematics::worldFrame( frame, state );
    }

    void gripFrame(Frame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    void gripFrame(MovableFrame* item, Frame* gripper, State& state){
        return ::rw::kinematics::Kinematics::gripFrame( item, gripper, state);
    }

    bool isDAF(const Frame* frame){
        return ::rw::kinematics::Kinematics::isDAF( frame );
    }
%}

class Joint: public Frame
{
};

%template (JointVector) std::vector<Joint*>;


/********************************************
 * LOADERS
 ********************************************/

class WorkCellFactory{
public:
    static rw::common::Ptr<WorkCell> load(const std::string& filename);
private:
    WorkCellFactory();
};


class ImageFactory{
public:
    static rw::common::Ptr<Image> load(const std::string& filename);
private:
    ImageFactory();
};

class Image {
};

%template (ImagePtr) rw::common::Ptr<Image>;

class XMLTrajectoryLoader
{
public:
    XMLTrajectoryLoader(const std::string& filename, const std::string& schemaFileName = "");
    XMLTrajectoryLoader(std::istream& instream, const std::string& schemaFileName = "");

    enum Type { QType = 0, Vector3DType, Rotation3DType, Transform3DType};
    Type getType();
    rw::common::Ptr<Trajectory<Q> > getQTrajectory();
    rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > > getVector3DTrajectory();
    rw::common::Ptr<Trajectory<Rotation3D> > getRotation3DTrajectory();
    rw::common::Ptr<Trajectory<Transform3D> > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const Trajectory<Q>& trajectory, const std::string& filename);
    static bool save(const Trajectory<rw::math::Vector3D<double> >& trajectory, const std::string& filename);
    static bool save(const Trajectory<Rotation3D>& trajectory, const std::string& filename);
    static bool save(const Trajectory<Transform3D>& trajectory, const std::string& filename);
    static bool write(const Trajectory<Q>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<rw::math::Vector3D<double> >& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<Rotation3D>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<Transform3D>& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

/********************************************
 * MATH
 ********************************************/
%include <rwlibs/swig/rwmath.i>

// Utility function within rw::Math
Rotation3D getRandomRotation3D();
Transform3D getRandomTransform3D(const double translationLength = 1);

namespace rw { namespace math {
    class Math
    {
    public:
        static inline double clamp(double val, double min, double max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const rw::math::Q& min,
                                  const rw::math::Q& max);

        static rw::math::Q clampQ(const rw::math::Q& q,
                                  const std::pair<rw::math::Q, rw::math::Q>& bounds);

        static rw::math::Vector3D<> clamp(const rw::math::Vector3D<>& q,
                                          const rw::math::Vector3D<>& min,
                                          const rw::math::Vector3D<>& max);

        static double ran();

        static void seed(unsigned seed);

        static void seed();

        static double ran(double from, double to);

        static int ranI(int from, int to);

        static double ranNormalDist(double mean, double sigma);

        static rw::math::Q ranQ(const rw::math::Q& from, const rw::math::Q& to);

        static rw::math::Q ranQ(const std::pair<rw::math::Q,rw::math::Q>& bounds);

        static rw::math::Q ranDir(size_t dim, double length = 1);
        
        static rw::math::Q ranWeightedDir(size_t dim, const rw::math::Q& weights, double length = 1);

        static double round(double d);

        static Q sqr(const Q& q);

        static Q sqrt(const Q& q);

        static Q abs(const Q& v);

        static double min(const Q& v);

        static double max(const Q& v);

        static double sign(double s);

        static Q sign(const Q& q);

        static int ceilLog2(int n);
        
        static long long factorial(long long n);

        static bool isNaN(double d);
    };
}} // end namespaces

/********************************************
 * MODELS
 ********************************************/
 

class WorkCell {
public:
    WorkCell(const std::string& name);
    std::string getName() const;
    Frame* getWorldFrame() const;

    void addFrame(Frame* frame, Frame* parent=NULL);
    void addDAF(Frame* frame, Frame* parent=NULL);
    void remove(Frame* frame);
    void addDevice(rw::common::Ptr<Device> device);
    
    Frame* findFrame(const std::string& name) const;

    %extend {
        MovableFrame* findMovableFrame(const std::string& name)
        { return $self->rw::models::WorkCell::findFrame<MovableFrame>(name); }
        FixedFrame* findFixedFrame(const std::string& name)
        { return $self->rw::models::WorkCell::findFrame<FixedFrame>(name); }
        void addObject(rw::common::Ptr<Object> object)
        { $self->rw::models::WorkCell::add(object); }
    };

    std::vector<Frame*> getFrames() const;
    rw::common::Ptr<Device> findDevice(const std::string& name) const;
    %extend {
        rw::common::Ptr<JointDevice> findJointDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<JointDevice>(name); }
        rw::common::Ptr<SerialDevice> findSerialDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<SerialDevice>(name); }
        rw::common::Ptr<TreeDevice> findTreeDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<TreeDevice>(name); }
        rw::common::Ptr<ParallelDevice> findParallelDevice(const std::string& name)
                { return $self->rw::models::WorkCell::findDevice<ParallelDevice>(name); }
        std::vector<rw::common::Ptr<Device> > getDevices() const
                { return $self->rw::models::WorkCell::getDevices(); }
    };
    
    rw::common::Ptr<Object> findObject(const std::string& name) const;
    void add(rw::common::Ptr<Object> object);
    void remove(rw::common::Ptr<Object> object);


    State getDefaultState() const;

    //rw::common::Ptr<StateStructure> getStateStructure();

    PropertyMap& getPropertyMap();
private:
    WorkCell(const WorkCell&);
    WorkCell& operator=(const WorkCell&);
};

%template (WorkCellPtr) rw::common::Ptr<WorkCell>;

class Object
{
public:
    //! destructor
    virtual ~Object();
    const std::string& getName();
    Frame* getBase();
    const std::vector<Frame*>& getFrames();
    void addFrame(Frame* frame);
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry() const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;

    // stuff that should be implemented by deriving classes
     const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    virtual double getMass(State& state) const = 0;
    virtual rw::math::Vector3D<> getCOM(State& state) const = 0;
    virtual rw::math::InertiaMatrix<> getInertia(State& state) const = 0;
};
%template (ObjectPtr) rw::common::Ptr<Object>;

class RigidObject : public Object {
public:
	RigidObject(rw::kinematics::Frame* baseframe);
	RigidObject(rw::kinematics::Frame* baseframe, rw::geometry::Geometry::Ptr geom);
	RigidObject(rw::kinematics::Frame* baseframe, std::vector<rw::geometry::Geometry::Ptr> geom);
	RigidObject(std::vector<rw::kinematics::Frame*> frames);
	RigidObject(std::vector<rw::kinematics::Frame*> frames, rw::geometry::Geometry::Ptr geom);
	RigidObject(std::vector<rw::kinematics::Frame*> frames, std::vector<rw::geometry::Geometry::Ptr> geom);
	void addGeometry(rw::geometry::Geometry::Ptr geom);
	void removeGeometry(rw::geometry::Geometry::Ptr geom);
	void addModel(rw::graphics::Model3D::Ptr model);
	void removeModel(rw::graphics::Model3D::Ptr model);
    double getMass() const;
    void setMass(double mass);
    rw::math::InertiaMatrix<> getInertia() const;
    void setInertia(const rw::math::InertiaMatrix<>& inertia);
    rw::math::Vector3D<> getCOM() const;
    void setCOM(const rw::math::Vector3D<>& com);
    void approximateInertia();
    void approximateInertiaCOM();
    const std::vector<rw::geometry::Geometry::Ptr>& getGeometry() const ;
    const std::vector<rw::graphics::Model3D::Ptr>& getModels() const;
    double getMass(rw::kinematics::State& state) const;
    rw::math::InertiaMatrix<> getInertia(rw::kinematics::State& state) const;
    rw::math::Vector3D<> getCOM(rw::kinematics::State& state) const;
};
%template (RigidObjectPtr) rw::common::Ptr<RigidObject>;



class DeformableObject: public Object
{
public:
     //! constructor
    DeformableObject(Frame* baseframe, int nr_of_nodes);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Model3D> model);

    //DeformableObject(Frame* baseframe, rw::common::Ptr<Geometry> geom);

    //! destructor
    virtual ~DeformableObject();

    rw::math::Vector3D<float>& getNode(int id, State& state) const;
    void setNode(int id, const rw::math::Vector3D<float>& v, State& state);
    
    //const std::vector<rw::geometry::IndexedTriangle<> >& getFaces() const;
    void addFace(unsigned int node1, unsigned int node2, unsigned int node3);
    //rw::geometry::IndexedTriMesh<float>::Ptr getMesh(rw::kinematics::State& cstate);
    const std::vector<rw::common::Ptr<Geometry> >& getGeometry(const State& state) const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels() const;
    const std::vector<rw::common::Ptr<Model3D> >& getModels(const State& state) const;
    
    double getMass(State& state) const;
    rw::math::Vector3D<double> getCOM(rw::kinematics::State& state) const;
    InertiaMatrix getInertia(State& state) const;
    void update(rw::graphics::Model3D::Ptr model, const State& state);
};
%template (DeformableObjectPtr) rw::common::Ptr<DeformableObject>;


class Device
{
public:
    Device(const std::string& name);
    //void registerStateData(rw::kinematics::StateStructure::Ptr sstruct);
    virtual void setQ(const Q& q, State& state) const = 0;
    virtual Q getQ(const State& state) const = 0;
    virtual std::pair<Q,Q> getBounds() const = 0;
    virtual Q getVelocityLimits() const = 0;
    virtual void setVelocityLimits(const Q& vellimits) = 0;
    virtual Q getAccelerationLimits() const = 0;
    virtual void setAccelerationLimits(const Q& acclimits) = 0;
    virtual size_t getDOF() const = 0;
    std::string getName() const;
    void setName(const std::string& name);
    virtual Frame* getBase() = 0;
    virtual Frame* getEnd() = 0;
#if !defined(SWIGJAVA)
    virtual const Frame* getBase() const = 0;
    virtual const Frame* getEnd() const = 0;
#endif
    Transform3D baseTframe(const Frame* f, const State& state) const;
    Transform3D baseTend(const State& state) const;
    Transform3D worldTbase(const State& state) const;
    virtual Jacobian baseJend(const State& state) const = 0;
    virtual Jacobian baseJframe(const Frame* frame,const State& state) const;
    virtual Jacobian baseJframes(const std::vector<Frame*>& frames,const State& state) const;
    //virtual rw::common::Ptr<JacobianCalculator> baseJCend(const kinematics::State& state) const;
    //virtual JacobianCalculatorPtr baseJCframe(const kinematics::Frame* frame, const kinematics::State& state) const;
    //virtual JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames, const kinematics::State& state) const = 0;
private:
    Device(const Device&);
    Device& operator=(const Device&);
};

%template (DevicePtr) rw::common::Ptr<Device>;
%template (DevicePtrVector) std::vector<rw::common::Ptr<Device> >;

class JointDevice: public Device
{
public:
    const std::vector<Joint*>& getJoints() const;
    void setQ(const Q& q, State& state) const;
    Q getQ(const State& state) const;
    size_t getDOF() const;
    std::pair<Q, Q> getBounds() const;
    void setBounds(const std::pair<Q, Q>& bounds);
    Q getVelocityLimits() const;
    void setVelocityLimits(const Q& vellimits);
    Q getAccelerationLimits() const;
    void setAccelerationLimits(const Q& acclimits);
    Jacobian baseJend(const State& state) const;

    //JacobianCalculatorPtr baseJCframes(const std::vector<kinematics::Frame*>& frames,
    //                                   const kinematics::State& state) const;

    Frame* getBase();
    virtual Frame* getEnd();

#if !defined(SWIGJAVA)
    const Frame* getBase() const;
    virtual const Frame* getEnd() const;
#endif
};

%template (JointDevicePtr) rw::common::Ptr<JointDevice>;

class CompositeDevice: public Device
{
public:
    CompositeDevice(
        Frame* base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        Frame* end,
        const std::string& name,
        const State& state);

    CompositeDevice(
        Frame *base,
		const std::vector<rw::common::Ptr<Device> >& devices,
        const std::vector<Frame*>& ends,
        const std::string& name,
        const State& state);

    
};

class SerialDevice: public JointDevice
{
};
%template (SerialDevicePtr) rw::common::Ptr<SerialDevice>;

class ParallelDevice: public JointDevice
{
};
%template (ParallelDevicePtr) rw::common::Ptr<ParallelDevice>;

class TreeDevice: public JointDevice
{
public:
	TreeDevice(
		rw::kinematics::Frame* base,
		const std::vector<rw::kinematics::Frame*>& ends,
		const std::string& name,
		const rw::kinematics::State& state);
};
%template (TreeDevicePtr) rw::common::Ptr<TreeDevice>;

%nodefaultctor DHParameterSet;
class DHParameterSet
{
};

%template (DHParameterSetVector) std::vector<DHParameterSet>;

/********************************************
 * PATHPLANNING
 ********************************************/



/********************************************
 * PLUGIN
 ********************************************/

/********************************************
 * PROXIMITY
 ********************************************/

%nodefaultctor CollisionStrategy;
class CollisionStrategy {
public:
/*
    virtual bool inCollision(
        const kinematics::Frame* a,
        const math::Transform3D<>& wTa,
        const kinematics::Frame *b,
        const math::Transform3D<>& wTb,
        CollisionQueryType type = FirstContact);

    virtual bool inCollision(
        const kinematics::Frame* a,
        const math::Transform3D<>& wTa,
        const kinematics::Frame *b,
        const math::Transform3D<>& wTb,
        ProximityStrategyData& data,
        CollisionQueryType type = FirstContact);

    virtual bool inCollision(
        ProximityModel::Ptr a,
        const math::Transform3D<>& wTa,
        ProximityModel::Ptr b,
        const math::Transform3D<>& wTb,
        ProximityStrategyData& data) = 0;
*/
    /*
    static CollisionStrategy::Ptr make(CollisionToleranceStrategy::Ptr strategy, double tolerance);

    static CollisionStrategy::Ptr make(CollisionToleranceStrategy::Ptr strategy,
                     const rw::kinematics::FrameMap<double>& frameToTolerance,
                     double defaultTolerance);
     */
};

%template (CollisionStrategyPtr) rw::common::Ptr<CollisionStrategy>;

%nodefaultctor CollisionDetector;
class CollisionDetector
{
public:
    %extend {
        static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell){
            return rw::common::ownedPtr( new CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()) );
        }

        static rw::common::Ptr<CollisionDetector> make(rw::common::Ptr<WorkCell> workcell, rw::common::Ptr<CollisionStrategy> strategy){
            return rw::common::ownedPtr( new CollisionDetector(workcell, strategy) );
        }
    }
};

%template (CollisionDetectorPtr) rw::common::Ptr<CollisionDetector>;

/********************************************
 * SENSOR
 ********************************************/

/********************************************
 * TRAJECTORY
 ********************************************/

template <class T>
class Timed
{
public:
    Timed();
    Timed(double time, const T& value);

    double getTime() const;
    T& getValue();

    %extend {
        void setTime(double time){
            $self->rw::trajectory::Timed<T>::getTime() = time;
        }
    };
};

%template (TimedQ) Timed<Q>;
%template (TimedState) Timed<State>;

template <class T>
class Path: public std::vector<T>
{
public:

    Path();
    Path(size_t cnt);
    Path(size_t cnt, const T& value);
    Path(const std::vector<T>& v);

    %extend {
        size_t size(){ return $self->std::vector<T >::size(); }
        T& elem(size_t idx){ return (*$self)[idx]; }
    };
};

%template (TimedQVector) std::vector<Timed<Q> >;
%template (TimedStateVector) std::vector<Timed<State> >;
%template (TimedQVectorPtr) rw::common::Ptr<std::vector<Timed<Q> > >;
%template (TimedStateVectorPtr) rw::common::Ptr<std::vector<Timed<State> > >;

%template (PathSE3) Path<Transform3D>;
%template (PathSE3Ptr) rw::common::Ptr<Path<Transform3D> >;
%template (PathQ) Path<Q>;
%template (PathQPtr) rw::common::Ptr<Path<Q> >;
%template (PathTimedQ) Path<Timed<Q> >;
%template (PathTimedQPtr) rw::common::Ptr<Path<Timed<Q> > >;
%template (PathTimedState) Path<Timed<State> >;
%template (PathTimedStatePtr) rw::common::Ptr<Path<Timed<State> > >;

%extend Path<Q> {
    rw::common::Ptr<Path<Timed<Q> > > toTimedQPath(Q speed){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(speed, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<Q> > > toTimedQPath(rw::common::Ptr<Device> dev){
        rw::trajectory::TimedQPath tpath =
                rw::trajectory::TimedUtil::makeTimedQPath(*dev, *$self);
        return rw::common::ownedPtr( new rw::trajectory::TimedQPath(tpath) );
    }

    rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(rw::common::Ptr<Device> dev,
                                                     const State& state){
        rw::trajectory::TimedStatePath tpath =
                rw::trajectory::TimedUtil::makeTimedStatePath(*dev, *$self, state);
        return rw::common::ownedPtr( new rw::trajectory::TimedStatePath(tpath) );
    }

};

%extend Path<Timed<State> > {
	
	static rw::common::Ptr<Path<Timed<State> > > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
		std::auto_ptr<rw::trajectory::TimedStatePath> spath = 
			rw::loaders::PathLoader::loadTimedStatePath(*wc, filename);
		return rw::common::Ptr<rw::trajectory::TimedStatePath>( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeTimedStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<Timed<State> > > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).back().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate = (*spath)[i]; 
			tstate.getTime() += startTime;
			(*$self).push_back( tstate );
		}
	}
	
};

%extend Path<State > {
	
	static rw::common::Ptr<Path<State> > load(const std::string& filename, rw::common::Ptr<WorkCell> wc){
		std::auto_ptr<rw::trajectory::StatePath> spath = 
			rw::loaders::PathLoader::loadStatePath(*wc, filename);
		return rw::common::ownedPtr( spath );
	}
	
	void save(const std::string& filename, rw::common::Ptr<WorkCell> wc){		 		
		rw::loaders::PathLoader::storeStatePath(*wc,*$self,filename); 
	}
	
	void append(rw::common::Ptr<Path<State> > spath){
		double startTime = 0;
		if($self->size()>0)
			startTime = (*$self).front().getTime(); 
		
		for(size_t i = 0; i<spath->size(); i++){
			(*$self).push_back( (*spath)[i] );
		}
	}
	
	
	rw::common::Ptr<Path<Timed<State> > > toTimedStatePath(double timeStep){
		rw::common::Ptr<TimedStatePath> spath = 
			rw::common::ownedPtr( new rw::trajectory::TimedStatePath() );	
		for(size_t i = 0; i<spath->size(); i++){
			Timed<State> tstate(timeStep*i, (*spath)[i]); 
			spath->push_back( tstate );
		}	
		return spath;
	}
	
};



template <class T>
class Blend
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double tau1() const = 0;
    virtual double tau2() const = 0;
};

%template (BlendR1) Blend<double>;
%template (BlendR2) Blend<Vector2>;
%template (BlendR3) Blend<rw::math::Vector3D<double> >;
%template (BlendSO3) Blend<Rotation3D>;
%template (BlendSE3) Blend<Transform3D>;
%template (BlendQ) Blend<Q>;

%template (BlendR1Ptr) rw::common::Ptr<Blend<double> >;
%template (BlendR2Ptr) rw::common::Ptr<Blend<Vector2> >;
%template (BlendR3Ptr) rw::common::Ptr<Blend<rw::math::Vector3D<double> > >;
%template (BlendSO3Ptr) rw::common::Ptr<Blend<Rotation3D> >;
%template (BlendSE3Ptr) rw::common::Ptr<Blend<Transform3D> >;
%template (BlendQPtr) rw::common::Ptr<Blend<Q> >;

template <class T>
class Interpolator
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
};

%template (InterpolatorR1) Interpolator<double>;
%template (InterpolatorR2) Interpolator<Vector2>;
%template (InterpolatorR3) Interpolator<rw::math::Vector3D<double> >;
%template (InterpolatorSO3) Interpolator<Rotation3D>;
%template (InterpolatorSE3) Interpolator<Transform3D>;
%template (InterpolatorQ) Interpolator<Q>;

%template (InterpolatorR1Ptr) rw::common::Ptr<Interpolator<double> >;
%template (InterpolatorR2Ptr) rw::common::Ptr<Interpolator<Vector2> >;
%template (InterpolatorR3Ptr) rw::common::Ptr<Interpolator<rw::math::Vector3D<double> > >;
%template (InterpolatorSO3Ptr) rw::common::Ptr<Interpolator<Rotation3D> >;
%template (InterpolatorSE3Ptr) rw::common::Ptr<Interpolator<Transform3D> >;
%template (InterpolatorQPtr) rw::common::Ptr<Interpolator<Q> >;

class LinearInterpolator: public Interpolator<double> {
public:
    LinearInterpolator(const double& start,
                          const double& end,
                          double duration);

    virtual ~LinearInterpolator();

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};


class LinearInterpolatorQ: public Interpolator<Q> {
public:
    LinearInterpolatorQ(const Q& start,
                          const Q& end,
                          double duration);

    virtual ~LinearInterpolatorQ();

    Q x(double t) const;
    Q dx(double t) const;
    Q ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorR3: public Interpolator<Rotation3D> {
public:
    LinearInterpolatorR3(const Rotation3D& start,
                          const Rotation3D& end,
                          double duration);

    Rotation3D x(double t) const;
    Rotation3D dx(double t) const;
    Rotation3D ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSO3: public Interpolator<Rotation3D> {
public:
    LinearInterpolatorSO3(const Rotation3D& start,
                          const Rotation3D& end,
                          double duration);

    Rotation3D x(double t) const;
    Rotation3D dx(double t) const;
    Rotation3D ddx(double t) const;
    double duration() const;
};

class LinearInterpolatorSE3: public Interpolator<Transform3D> {
public:
    LinearInterpolatorSE3(const Transform3D& start,
                          const Transform3D& end,
                          double duration);

    Transform3D x(double t) const;
    Transform3D dx(double t) const;
    Transform3D ddx(double t) const;
    double duration() const;
};


//////////// RAMP interpolator


class RampInterpolatorR3: public Interpolator<rw::math::Vector3D<double> > {
public:
    RampInterpolatorR3(const rw::math::Vector3D<double>& start, const rw::math::Vector3D<double>& end,
                       double vellimit,double acclimit);

    rw::math::Vector3D<double> x(double t) const;
    rw::math::Vector3D<double> dx(double t) const;
    rw::math::Vector3D<double> ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSO3: public Interpolator<Rotation3D> {
public:
    RampInterpolatorSO3(const Rotation3D& start,
                          const Rotation3D& end,
                          double vellimit,double acclimit);

    Rotation3D x(double t) const;
    Rotation3D dx(double t) const;
    Rotation3D ddx(double t) const;
    double duration() const;
};

class RampInterpolatorSE3: public Interpolator<Transform3D> {
public:
    RampInterpolatorSE3(const Transform3D& start,
                          const Transform3D& end,
                          double linvellimit,double linacclimit,
                          double angvellimit,double angacclimit);

    Transform3D x(double t) const;
    Transform3D dx(double t) const;
    Transform3D ddx(double t) const;
    double duration() const;
};

class RampInterpolator: public Interpolator<double> {
public:
    RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits);
    //RampInterpolator(const double& start, const double& end, const double& vellimits, const double& acclimits, double duration);

    double x(double t) const;
    double dx(double t) const;
    double ddx(double t) const;
    double duration() const;
};

class RampInterpolatorQ: public Interpolator<Q> {
public:
    RampInterpolatorQ(const Q& start, const Q& end, const Q& vellimits, const Q& acclimits);
    //RampInterpolatorQ(const Q& start, const Q& end, const Q& vellimits, const Q& acclimits, double duration);

    Q x(double t) const;
    Q dx(double t) const;
    Q ddx(double t) const;
    double duration() const;
};



template <class T>
class Trajectory
{
public:
    virtual T x(double t) const = 0;
    virtual T dx(double t) const = 0;
    virtual T ddx(double t) const = 0;
    virtual double duration() const = 0;
    virtual double startTime() const = 0;
    virtual double endTime() const;

    std::vector<T> getPath(double dt, bool uniform = true);
    //virtual typename rw::common::Ptr< TrajectoryIterator<T> > getIterator(double dt = 1) const = 0;

protected:
    /**
     * @brief Construct an empty trajectory
     */
    Trajectory() {};
};

%template (TrajectoryState) Trajectory<State>;
%template (TrajectoryR1) Trajectory<double>;
%template (TrajectoryR2) Trajectory<Vector2>;
%template (TrajectoryR3) Trajectory<rw::math::Vector3D<double> >;
%template (TrajectorySO3) Trajectory<Rotation3D>;
%template (TrajectorySE3) Trajectory<Transform3D>;
%template (TrajectoryQ) Trajectory<Q>;

%template (TrajectoryStatePtr) rw::common::Ptr<Trajectory<State> >;
%template (TrajectoryR1Ptr) rw::common::Ptr<Trajectory<double> >;
%template (TrajectoryR2Ptr) rw::common::Ptr<Trajectory<Vector2> >;
%template (TrajectoryR3Ptr) rw::common::Ptr<Trajectory<rw::math::Vector3D<double> > >;
%template (TrajectorySO3Ptr) rw::common::Ptr<Trajectory<Rotation3D> >;
%template (TrajectorySE3Ptr) rw::common::Ptr<Trajectory<Transform3D> >;
%template (TrajectoryQPtr) rw::common::Ptr<Trajectory<Q> >;

template <class T>
class InterpolatorTrajectory: public Trajectory<T> {
public:
    InterpolatorTrajectory(double startTime = 0);
    void add(rw::common::Ptr<Interpolator<T> > interpolator);
    void add(rw::common::Ptr<Blend<T> > blend,
             rw::common::Ptr<Interpolator<T> > interpolator);
    void add(InterpolatorTrajectory<T>* trajectory);
    size_t getSegmentsCount() const;



    //std::pair<rw::common::Ptr<Blend<T> >, rw::common::Ptr<Interpolator<T> > > getSegment(size_t index) const;
};

%template (InterpolatorTrajectoryR1) InterpolatorTrajectory<double>;
%template (InterpolatorTrajectoryR2) InterpolatorTrajectory<Vector2>;
%template (InterpolatorTrajectoryR3) InterpolatorTrajectory<rw::math::Vector3D<double> >;
%template (InterpolatorTrajectorySO3) InterpolatorTrajectory<Rotation3D>;
%template (InterpolatorTrajectorySE3) InterpolatorTrajectory<Transform3D>;
%template (InterpolatorTrajectoryQ) InterpolatorTrajectory<Q>;


/*
class TrajectoryFactory
{
public:
    static rw::common::Ptr<StateTrajectory> makeFixedTrajectory(const rw::kinematics::State& state, double duration);
    static rw::common::Ptr<QTrajectory> makeFixedTrajectory(const rw::math::Q& q, double duration);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const TimedStatePath& path);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectory(const StatePath& path,
        const models::WorkCell& workcell);
    static rw::common::Ptr<StateTrajectory> makeLinearTrajectoryUnitStep(const StatePath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const TimedQPath& path);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const rw::math::Q& speeds);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, const models::Device& device);
    static rw::common::Ptr<QTrajectory> makeLinearTrajectory(const QPath& path, rw::common::Ptr<QMetric> metric);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const std::vector<double>& times);
    static rw::common::Ptr<Transform3DTrajectory> makeLinearTrajectory(const Transform3DPath& path, const rw::common::Ptr<Transform3DMetric> metric);
    static rw::common::Ptr<StateTrajectory> makeEmptyStateTrajectory();
    static rw::common::Ptr<QTrajectory > makeEmptyQTrajectory();
};

*/

/********************************************
 * RWLIBS ALGORITHMS
 ********************************************/

/********************************************
 * RWLIBS ASSEMBLY
 ********************************************/

class AssemblyControlResponse
{
public:
	AssemblyControlResponse();
	virtual ~AssemblyControlResponse();
	
	/*
	typedef enum Type {
		POSITION,    //!< Position control
		VELOCITY,    //!< Velocity control
		HYBRID_FT_POS//!< Hybrid position and force/torque control
	} Type;

	Type type;
	*/
	
	Transform3D femaleTmaleTarget;
	rw::common::Ptr<Trajectory<Transform3D> > worldTendTrajectory;
	VelocityScrew6D femaleTmaleVelocityTarget;
	Rotation3D offset;
	//VectorND<6,bool> selection;
	Wrench6D force_torque;
	bool done;
	bool success;
};

%template (AssemblyControlResponsePtr) rw::common::Ptr<AssemblyControlResponse>;

class AssemblyControlStrategy
{
public:
	AssemblyControlStrategy();
	virtual ~AssemblyControlStrategy();
	
	/*
	class ControlState {
	public:
		//! @brief smart pointer type to this class
	    typedef rw::common::Ptr<ControlState> Ptr;

		//! @brief Constructor.
		ControlState() {};

		//! @brief Destructor.
		virtual ~ControlState() {};
	};
	virtual rw::common::Ptr<ControlState> createState() const;
	*/
	
	//virtual rw::common::Ptr<AssemblyControlResponse> update(rw::common::Ptr<AssemblyParameterization> parameters, rw::common::Ptr<AssemblyState> real, rw::common::Ptr<AssemblyState> assumed, rw::common::Ptr<ControlState> controlState, State &state, FTSensor* ftSensor, double time) const = 0;
	virtual Transform3D getApproach(rw::common::Ptr<AssemblyParameterization> parameters) = 0;
	virtual std::string getID() = 0;
	virtual std::string getDescription() = 0;
	virtual rw::common::Ptr<AssemblyParameterization> createParameterization(const rw::common::Ptr<PropertyMap> map) = 0;
};

%template (AssemblyControlStrategyPtr) rw::common::Ptr<AssemblyControlStrategy>;

class AssemblyParameterization
{
public:
	AssemblyParameterization();
	AssemblyParameterization(rw::common::Ptr<PropertyMap> pmap);
	virtual ~AssemblyParameterization();
	virtual rw::common::Ptr<PropertyMap> toPropertyMap() const;
	virtual rw::common::Ptr<AssemblyParameterization> clone() const;
};

%template (AssemblyParameterizationPtr) rw::common::Ptr<AssemblyParameterization>;

class AssemblyRegistry
{
public:
	AssemblyRegistry();
	virtual ~AssemblyRegistry();
	void addStrategy(const std::string id, rw::common::Ptr<AssemblyControlStrategy> strategy);
	std::vector<std::string> getStrategies() const;
	bool hasStrategy(const std::string& id) const;
	rw::common::Ptr<AssemblyControlStrategy> getStrategy(const std::string &id) const;
};

%template (AssemblyRegistryPtr) rw::common::Ptr<AssemblyRegistry>;

class AssemblyResult
{
public:
	AssemblyResult();
	AssemblyResult(rw::common::Ptr<Task<Transform3D> > task);
	virtual ~AssemblyResult();
	rw::common::Ptr<AssemblyResult> clone() const;
	rw::common::Ptr<Task<Transform3D> > toCartesianTask();
	static void saveRWResult(rw::common::Ptr<AssemblyResult> result, const std::string& name);
	static void saveRWResult(std::vector<rw::common::Ptr<AssemblyResult> > results, const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyResult> > load(const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyResult> > load(std::istringstream& inputStream);
	
	bool success;
	//Error error;
	Transform3D femaleTmaleEnd;

	std::string taskID;
	std::string resultID;
    
    Path<Timed<AssemblyState> > realState;
	Path<Timed<AssemblyState> > assumedState;
	Transform3D approach;
	std::string errorMessage;
};

%template (AssemblyResultPtr) rw::common::Ptr<AssemblyResult>;
%template (AssemblyResultPtrVector) std::vector<rw::common::Ptr<AssemblyResult> >;

class AssemblyState
{
public:
	AssemblyState();
	//AssemblyState(rw::common::Ptr<Target<Transform3D> > target);
	virtual ~AssemblyState();
	//static rw::common::Ptr<Target<Transform3D> > toCartesianTarget(const AssemblyState &state);

	std::string phase;
	Transform3D femaleOffset;
	Transform3D maleOffset;
	Transform3D femaleTmale;
	Wrench6D ftSensorMale;
	Wrench6D ftSensorFemale;
	bool contact;
	Path<Transform3D> maleflexT;
	Path<Transform3D> femaleflexT;
	Path<Transform3D> contacts;
	rw::math::Vector3D<double> maxContactForce;
};

%template (AssemblyStatePtr) rw::common::Ptr<AssemblyState>;
%template (TimedAssemblyState) Timed<AssemblyState>;
%template (TimedAssemblyStateVector) std::vector<Timed<AssemblyState> >;
%template (PathTimedAssemblyState) Path<Timed<AssemblyState> >;

class AssemblyTask
{
public:
	AssemblyTask();
	AssemblyTask(rw::common::Ptr<Task<Transform3D> > task, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	virtual ~AssemblyTask();
	rw::common::Ptr<Task<Transform3D> > toCartesianTask();
	static void saveRWTask(rw::common::Ptr<AssemblyTask> task, const std::string& name);
	static void saveRWTask(std::vector<rw::common::Ptr<AssemblyTask> > tasks, const std::string& name);
	static std::vector<rw::common::Ptr<AssemblyTask> > load(const std::string& name, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	static std::vector<rw::common::Ptr<AssemblyTask> > load(std::istringstream& inputStream, rw::common::Ptr<AssemblyRegistry> registry = NULL);
	rw::common::Ptr<AssemblyTask> clone() const;

	std::string maleID;
    std::string femaleID;
    Transform3D femaleTmaleTarget;
    rw::common::Ptr<AssemblyControlStrategy> strategy;
    rw::common::Ptr<AssemblyParameterization> parameters;
    
    std::string maleTCP;
    std::string femaleTCP;
    
    std::string taskID;
    std::string workcellName;
    std::string generator;
    std::string date;
    std::string author;
    
    std::string malePoseController;
    std::string femalePoseController;
    std::string maleFTSensor;
    std::string femaleFTSensor;
    std::vector<std::string> maleFlexFrames;
    std::vector<std::string> femaleFlexFrames;
    std::vector<std::string> bodyContactSensors;
};

%template (AssemblyTaskPtr) rw::common::Ptr<AssemblyTask>;
%template (AssemblyTaskPtrVector) std::vector<rw::common::Ptr<AssemblyTask> >;

/********************************************
 * RWLIBS CALIBRATION
 ********************************************/
 
/********************************************
 * RWLIBS CONTROL
 ********************************************/
 
%nodefaultctor Controller;
class Controller {
public:
	const std::string& getName() const;
	void setName(const std::string& name);
};

%nodefaultctor JointController;
class JointController {
public:
	typedef enum {
        POSITION = 1, CNT_POSITION = 2, VELOCITY = 4, FORCE = 8, CURRENT = 16
    } ControlMode;
    
    virtual ~JointController();
    virtual unsigned int getControlModes() = 0;
    virtual void setControlMode(ControlMode mode) = 0;
    virtual void setTargetPos(const Q& vals) = 0;
    virtual void setTargetVel(const Q& vals) = 0;
    virtual void setTargetAcc(const Q& vals) = 0;
    virtual Device& getModel();
    virtual Q getQ() = 0;
    virtual Q getQd() = 0;
};

%template (JointControllerPtr) rw::common::Ptr<JointController>;
 
/********************************************
 * RWLIBS OPENGL
 ********************************************/
 
/********************************************
 * RWLIBS OS
 ********************************************/

/********************************************
 * RWLIBS PATHOPTIMIZATION
 ********************************************/

class PathLengthOptimizer
{
public:

    %extend {

        PathLengthOptimizer(rw::common::Ptr<CollisionDetector> cd,
                            rw::common::Ptr<Device> dev,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, rw::math::MetricFactory::makeEuclidean< Q>());
        }

        PathLengthOptimizer(rw::common::Ptr<CollisionDetector> cd,
                            rw::common::Ptr<Device> dev,
                            rw::common::Ptr<Metric<Q> > metric,
                            const State &state)
        {
            rw::pathplanning::PlannerConstraint constraint =
                    rw::pathplanning::PlannerConstraint::make(cd.get(), dev, state);
            return new PathLengthOptimizer(constraint, metric );
        }

        PathLengthOptimizer(rw::common::Ptr<PlannerConstraint> constraint,
                            rw::common::Ptr<Metric<Q> > metric)
        {
            return new PathLengthOptimizer(*constraint, metric);
        }

        rw::common::Ptr<Path<Q> > pathPruning(rw::common::Ptr<Path<Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::pathPruning(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }
/*
        rw::common::Ptr<Path<Q> > shortCut(rw::common::Ptr<Path<Q> > path,
                                       size_t cnt,
                                       double time,
                                       double subDivideLength);
*/
        rw::common::Ptr<Path<Q> > shortCut(rw::common::Ptr<Path<Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::shortCut(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }

        rw::common::Ptr<Path<Q> > partialShortCut(rw::common::Ptr<Path<Q> > path){
            PathQ res = $self->rwlibs::pathoptimization::PathLengthOptimizer::partialShortCut(*path);
            return rw::common::ownedPtr( new PathQ(res) );
        }
/*
        rw::common::Ptr<Path<Q> > partialShortCut(rw::common::Ptr<Path<Q> > path,
                                              size_t cnt,
                                              double time,
                                              double subDivideLength);
                                              */
    }
    PropertyMap& getPropertyMap();

};

/********************************************
 * RWLIBS PATHPLANNERS
 ********************************************/
%include <rwlibs/swig/rwplanning.i>

/********************************************
 * RWLIBS PROXIMITYSTRATEGIES
 ********************************************/

/********************************************
 * RWLIBS SIMULATION
 ********************************************/

struct UpdateInfo {
	UpdateInfo();
	UpdateInfo(double dt_step);
	
	double dt;
	double dt_prev;
	double time;
	bool rollback;
};

%nestedworkaround Simulator::UpdateInfo;

class Simulator {
public:

	/*
	 * Nested structs not supported
	 * 
   struct UpdateInfo {
	   UpdateInfo();
	   UpdateInfo(double dt_step);

	   double dt;
	   double dt_prev;
	   double time;
	   bool rollback;
   };
   */
   
   virtual ~Simulator();
   virtual void step(double dt, State& state) = 0;
   virtual void reset(State& state) = 0;
   virtual void init(State& state) = 0;
   virtual double getTime() = 0;
   virtual void setEnabled(Frame* frame, bool enabled) = 0;
   virtual PropertyMap& getPropertyMap() = 0;

};

/********************************************
 * RWLIBS SOFTBODY
 ********************************************/

/********************************************
 * RWLIBS SWIG
 ********************************************/

/********************************************
 * RWLIBS TASK
 ********************************************/

template <class T>
class Task
{
};

%template (TaskSE3) Task<Transform3D>;

%template (TaskSE3Ptr) rw::common::Ptr<Task<Transform3D> >;

class GraspTask {
public:
    GraspTask():
    GraspTask(rw::common::Ptr<Task<Transform3D> > task);
    rw::common::Ptr<Task<Transform3D> > toCartesianTask();
    std::string getGripperID();
    std::string getTCPID();
    std::string getGraspControllerID();
    void setGripperID(const std::string& id);
    void setTCPID(const std::string& id);
    void setGraspControllerID(const std::string& id);
    static std::string toString(GraspResult::TestStatus status);
    static void saveUIBK(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveRWTask(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveText(rw::common::Ptr<GraspTask> task, const std::string& name );
    static rw::common::Ptr<GraspTask> load(const std::string& name);
    static rw::common::Ptr<GraspTask> load(std::istringstream& inputStream);
    rw::common::Ptr<GraspTask> clone();
};

%template (GraspTaskPtr) rw::common::Ptr<GraspTask>;

class GraspResult {
public:
	enum TestStatus {
        UnInitialized = 0,
        Success, // 1
        CollisionInitially, // 2
        ObjectMissed, // 3
        ObjectDropped, // 4
        ObjectSlipped, // 5
        TimeOut, // 6
        SimulationFailure, // 7
        InvKinFailure, // 8
        PoseEstimateFailure, // 9
        CollisionFiltered, // 10
        CollisionObjectInitially, // 11
        CollisionEnvironmentInitially, // 12
        CollisionDuringExecution, // 13
        Interference, // 14
        WrenchInsufficient, // 15
        SizeOfStatusArray
     };
};
%template (GraspResultPtr) rw::common::Ptr<GraspResult>;

/* @} */

//TODO add other grasptask related classes

/********************************************
 * RWLIBS TOOLS
 ********************************************/

/********************************************
 * LUA functions
 ********************************************/

#if defined (SWIGLUA)
%luacode {

-- Group: Lua functions
-- Var: print_to_log
local print_to_log = true

-- Var: overrides the global print function
local global_print = print

-- Function: print
--  Forwards the global print functions to the rw.print functions
--  whenever print_to_log is defined.
function print(...)
    if print_to_log then
        for i, v in ipairs{...} do
            if i > 1 then rw.writelog("\t") end
            rw.writelog(tostring(v))
        end
        rw.writelog('\n')
    else
        global_print(...)
    end
end

-- Function:
function reflect( mytableArg )
 local mytable
 if not mytableArg then
  mytable = _G
 else
  mytable = mytableArg
 end
   local a = {} -- all functions
   local b = {} -- all Objects/Tables

 if type(mytable)=="userdata" then
   -- this is a SWIG generated user data, show functions and stuff
   local m = getmetatable( mytable )
   for key,value in pairs( m['.fn'] ) do
      if (key:sub(0, 2)=="__") or (key:sub(0, 1)==".") then
          table.insert(b, key)
      else
          table.insert(a, key)
      end
   end
   table.sort(a)
   table.sort(b)
   print("Object type: \n  " .. m['.type'])

   print("Member Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   for i,n in ipairs(b) do print("  " .. n .. "(...)") end

 else
   local c = {} -- all constants
   for key,value in pairs( mytable ) do
      -- print(type(value))
      if (type(value)=="function") then
          table.insert(a, key)
      elseif (type(value)=="number") then
          table.insert(c, key)
      else
          table.insert(b, key)
      end
   end
   table.sort(a)
   table.sort(b)
   table.sort(c)
   print("Object type: \n  " .. "Table")

   print("Functions:")
   for i,n in ipairs(a) do print("  " .. n .. "(...)") end
   print("Constants:")
   for i,n in ipairs(c) do print("  " .. n) end
   print("Misc:")
   for i,n in ipairs(b) do print("  " .. n) end


--  print("Metatable:")
--  for key,value in pairs( getmetatable(mytable) ) do
--      print(key);
--      print(value);
--  end

 end
 end

function help( mytable )
   reflect( mytable )
end
}
#endif


