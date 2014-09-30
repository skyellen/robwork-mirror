%module rw

%{
#include <rwlibs/swig/ScriptTypes.hpp>
#include <rw/common/Ptr.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#if defined (SWIGLUA)
#include <rwlibs/swig/Lua.hpp>
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
    bool operator==(void* p) const;

    template<class A>
    bool operator==(const rw::common::Ptr<A>& p) const;
#if defined(SWIGJAVA)
	%rename(dereference) get;
#endif
    T* get() const;
    T *operator->() const;
};
}}

class PropertyMap
{
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
    Plane(const Vector3D& n, double d);
    Plane(const Vector3D& p1,
          const Vector3D& p2,
          const Vector3D& p3);

    Vector3D& normal();
    //const Vector3D& normal() const;
    double& d();
    //double d() const;
    double distance(const Vector3D& point);
    double refit( std::vector<Vector3D >& data );
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
    virtual void rebuild(const std::vector<Vector3D>& vertices) = 0;
    virtual bool isInside(const Vector3D& vertex) = 0;
    virtual double getMinDistInside(const Vector3D& vertex) = 0;
    virtual double getMinDistOutside(const Vector3D& vertex) = 0;
    virtual PlainTriMeshN1* toTriMesh() = 0;
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
     //void updateSceneGraph(rw::kinematics::State& state);
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
     //DrawableNode::Ptr addFrameAxis(const std::string& name, double size, rw::kinematics::Frame* frame, int dmask=DrawableNode::Virtual);
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
    rw::common::Ptr<Trajectory<Vector3D> > getVector3DTrajectory();
    rw::common::Ptr<Trajectory<Rotation3D> > getRotation3DTrajectory();
    rw::common::Ptr<Trajectory<Transform3D> > getTransform3DTrajectory();
};

class XMLTrajectorySaver
{
public:
    static bool save(const Trajectory<Q>& trajectory, const std::string& filename);
    static bool save(const Trajectory<Vector3D>& trajectory, const std::string& filename);
    static bool save(const Trajectory<Rotation3D>& trajectory, const std::string& filename);
    static bool save(const Trajectory<Transform3D>& trajectory, const std::string& filename);
    static bool write(const Trajectory<Q>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<Vector3D>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<Rotation3D>& trajectory, std::ostream& outstream);
    static bool write(const Trajectory<Transform3D>& trajectory, std::ostream& outstream);
private:
    XMLTrajectorySaver();
};

/********************************************
 * MATH
 ********************************************/

class Matrix {
public:
	
	Matrix(int dimx, int dimy);
	
#if !defined(SWIGJAVA)
    double& operator()(size_t row, size_t column);
    const double& operator()(size_t row, size_t column) const;

    const Matrix operator+(const Matrix& wrench) const;    
    const Matrix operator-(const Matrix& wrench) const;
    const Matrix operator*(const Matrix& wrench) const;
#endif

#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Matrix&) const;
	%rename(add) operator+(const Matrix&) const;
#endif
	
	%extend {
		Matrix pseudoinverse() {
			 return rw::math::LinearAlgebra::pseudoInverse( (*$self) );
		}
		
		double& elem(int x, int y){
			return (*$self)(x,y);
		}
		
		/* These accesors are neccesary because Python does not allow
		lvalues consisting of access operators calls (cannot assign to a function call).
		Moreover, it's not possible to dereference a pointer obtained from function returning a reference. */
		double get(int x, int y) {
			return (*$self)(x, y);
		}
		
		void set(int x, int y, double value) {
			(*$self)(x, y) = value;
		}

#if (defined(SWIGPYTHON))
		/* This typemap makes it possible to access Matrix class elements using following syntax:
		
		myMatrix[1, 1] = value
		print myMatrix[1, 1]
		
		-- using __getitem__ and __setitem__ methods. */
		%typemap(in) int[2](int temp[2]) {
			int i;
			if (PyTuple_Check($input)) {
				if (!PyArg_ParseTuple($input, "ii", temp, temp+1)) {
					PyErr_SetString(PyExc_TypeError, "tuple must have 2 elements");
					return NULL;
				}
				$1 = &temp[0];
			} else {
				PyErr_SetString(PyExc_TypeError, "expected a tuple.");
				return NULL;
			}
		}
		
        double __getitem__(int i[2])const {return (*$self)(i[0], i[1]); }
        void __setitem__(int i[2], double d){ (*$self)(i[0], i[1]) = d; }
#endif
	}
	
};


class Q
{
public:
    // first we define functions that are native to Q
	Q();
	//%feature("autodoc","1");
	Q(int n, double a0);
    Q(int n, double a0, double a1);
    Q(int n, double a0, double a1, double a2);
    Q(int n, double a0, double a1, double a2, double a3);
    Q(int n, double a0, double a1, double a2, double a3, double a4);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8);
    Q(int n, double a0, double a1, double a2, double a3, double a4, double a5, double a6, double a7, double a8, double a9);

    Q(const std::vector<double>& r);
#if defined(SWIGJAVA)
%apply double[] {double *};
#endif
    Q(int n, const double* values);

    int size() const;

#if !defined(SWIGJAVA)
    %rename(elem) operator[];
    double& operator[](unsigned int i) ;
#endif

#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Q&) const;
	%rename(add) operator+(const Q&) const;
#endif

    const Q operator-() const;
    Q operator-(const Q& b) const;
    Q operator+(const Q& b) const;
    Q operator*(double s) const;
    Q operator/(double s) const;
    double norm2();
    double norm1();
    double normInf();

    %extend {
		
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Q>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Q>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};

%template (QVector) std::vector<Q>;
%template(QPair) std::pair<Q, Q>;

class Vector2D
{
public:
    Vector2D();
    %feature("autodoc","1");
    Vector2D(double x, double y);
    size_t size() const;

    double norm2();
    double norm1();
    double normInf();

    //double& operator[](unsigned int i) ;
    %rename(elem) operator[];

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Vector2D>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Vector2D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };

};

%template (Vector2DVector) std::vector<Vector2D>;

class Vector3D
{
public:
    Vector3D();
    %feature("autodoc","1");
    Vector3D(double x, double y, double z);
    size_t size() const;
    Vector3D operator*(double scale) const;
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Vector3D&) const;
	%rename(add) operator+(const Vector3D&) const;
#endif
    Vector3D operator+(const Vector3D& other) const;
    Vector3D operator-(const Vector3D& other) const;
    bool operator==(const Vector3D& q);

    double norm2();
    double norm1();
    double normInf();

    //double& operator[](unsigned int i) ;
    //%rename(elem) operator[];
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Vector3D>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Vector3D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };
};

%template (Vector3DVector) std::vector<Vector3D>;

class Rotation3D
{
public:
    // Lua methods:
    Rotation3D();
    %feature("autodoc","1");
    Rotation3D(double v0,double v1,double v2,
    			double v3,double v4,double v5,
    			double v6,double v7,double v8);
    			
    explicit Rotation3D(const Rotation3D& R);

    Rotation3D operator*(const Rotation3D& other) const;
    Vector3D operator*(const Vector3D& vec) const;
    //Rotation3D inverse() const;

    static const Rotation3D& identity();
    static Rotation3D skew(const Vector3D& v);
    bool equal(const Rotation3D& rot, double precision) const;

    //EAA operator*(const EAA& other) const;

    bool operator==(const Rotation3D &rhs) const;
    //double& operator[](unsigned int i) ;

    // std::string __tostring() const;
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Rotation3D>(*$self); }
        double __getitem__(int x,int y)const {return (*$self)(x,y); }
        void __setitem__(int x,int y,double d){ (*$self)(x,y) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Rotation3D>(*$self); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
#endif
    };
};

%template (Rotation3DVector) std::vector<Rotation3D>;


//Rotation3D inverse(const Rotation3D& val);

class EAA
{
public:
    // Lua methods:
    EAA();
     %feature("autodoc","1");
    EAA(const EAA& eaa);
     %feature("autodoc","1");
    EAA(const Rotation3D& rot);
     %feature("autodoc","1");
    EAA(const Vector3D& axis, double angle);
     %feature("autodoc","1");
    EAA(double thetakx, double thetaky, double thetakz);
     %feature("autodoc","1");
    EAA(const Vector3D& v1, const Vector3D& v2);

    double angle() const;
    Vector3D axis() const;

    //const double& operator[](unsigned int i) const;
	//double& operator[](unsigned int i);
	//%rename(elem) operator[];

    //EAA operator*(const Rotation3D& other) const;

    Rotation3D toRotation3D() const;

    //bool operator==(const EAA &rhs) const;
    // std::string __tostring() const;
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<EAA>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
    	double& x() { return (*$self)[0]; }
    	double& y() { return (*$self)[1]; }
    	double& z() { return (*$self)[2]; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<EAA>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
    	double x() const { return (*$self)[0]; }
    	double y() const { return (*$self)[1]; }
    	double z() const { return (*$self)[2]; }
#endif
    };
};

class RPY
{
public:
    // Lua methods:
    RPY();
    RPY(const RPY& eaa);
    RPY(const Rotation3D& rot);
    RPY(double roll, double pitch, double yaw);
    Rotation3D toRotation3D() const;
    
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<RPY>(*$self); }
        double __getitem__(int i)const {return (*$self)[i]; }
        void __setitem__(int i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<RPY>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i,double d){ (*$self)[i] = d; }
#endif
    };
};


class Quaternion
{
public:
    // Lua methods:
    Quaternion();
    Quaternion(double qx, double qy, double qz, double qw);
    Quaternion(const Quaternion& eaa);
    Quaternion(const Rotation3D& rot);
    Quaternion operator*(double s);

    void normalize();

    Rotation3D toRotation3D() const;
    Quaternion slerp(const Quaternion& v, const double t) const;

    double getQx() const;
    double getQy() const;
    double getQz() const;
    double getQw() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Quaternion>(*$self); }
        double __getitem__(int i)const {return (*$self)(i); }
        void __setitem__(int i,double d){ (*$self)(i) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Quaternion>(*$self); }
        double get(std::size_t i) const { return (*$self)(i); }
        void set(std::size_t i,double d){ (*$self)(i) = d; }
#endif
    };
};



//Quaternion operator*(double s, const Quaternion& v);

class Transform3D {
public:
	Transform3D();
    Transform3D(const Transform3D& t3d);
    Transform3D(const Vector3D& position,const Rotation3D& rotation);

    Transform3D operator*(const Transform3D& other) const;
    Vector3D operator*(const Vector3D& other) const;

    static Transform3D DH(double alpha, double a, double d, double theta);
    static Transform3D craigDH(double alpha, double a, double d, double theta);
	
    Vector3D& P();
    Rotation3D& R();

    %extend {
       Transform3D inverse(){ return inverse(*$self); }
       //Transform3D inverse(const Transform3D& val);
    };
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Transform3D>(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Transform3D>(*$self); }
#endif
    };
};

%template (Transform3DVector) std::vector<Transform3D>;

/*
%inline %{

    Transform3D inverse(Transform3D t3d){ return rw::math::inverse(t3d); }

    Rotation3D inverse(Rotation3D t3d){
        std::cout << "CALLING LUA INVERSE.... DUNNO WHY" << std::endl;
        return rw::math::inverse(t3d);
    }

%}
*/

class Pose6D {
public:
	Pose6D(const Pose6D& p6d);
    Pose6D(const Vector3D& position,const EAA& rotation);
    Pose6D(const Transform3D& t3d);

    Transform3D toTransform3D();
    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Pose6D>(*$self); }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Pose6D>(*$self); }
#endif
    };
};

class Jacobian
{
public:
    Jacobian(int m, int n);

    int size1() const ;
    int size2() const ;

#if (defined(SWIGLUA) || defined(SWIGPYTHON))
    double& elem(int i, int j);

    %extend {
        char *__str__() { return printCString<Jacobian>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row, column); }
        void __setitem__(std::size_t row, std::size_t column,double d){ (*$self)(row, column) = d; }
    };
#elif defined(SWIGJAVA)
    %extend {
        std::string toString() const { return toString<Jacobian>(*$self); }
        double elem(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row, column); }
        void set(std::size_t row, std::size_t column, double d){ (*$self)(row, column) = d; }
    };
#endif
};



class VelocityScrew6D
{
public:
	VelocityScrew6D();
	VelocityScrew6D(const VelocityScrew6D& p6d);
    VelocityScrew6D(const Vector3D& position,const EAA& rotation);
    VelocityScrew6D(const Transform3D& t3d);

    // lua functions
    VelocityScrew6D operator*(double scale) const;
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const VelocityScrew6D&) const;
	%rename(add) operator+(const VelocityScrew6D&) const;
#endif
    VelocityScrew6D operator+(const VelocityScrew6D& other) const;
    VelocityScrew6D operator-(const VelocityScrew6D& other) const;
    //bool operator==(const VelocityScrew6D& q);

    double norm2();
    double norm1();
    double normInf();

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<VelocityScrew6D>(*$self); }
        double __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<VelocityScrew6D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, double d){ (*$self)[i] = d; }
#endif
    };

    //Transform3D toTransform3D();
    // std::string __tostring() const;
};


class Wrench6D
{
public:		
    Wrench6D(double fx, double fy, double fz, double tx, double ty, double tz);

    // TODO: add constructor on vector

    Wrench6D();

    Wrench6D(const Vector3D& force, const Vector3D& torque);

    const Vector3D force() const;
    const Vector3D torque() const;

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<Wrench6D>(*$self); }
        double __getitem__(std::size_t i)const {return (*$self)[i]; }
        void __setitem__(std::size_t i,double d){ (*$self)[i] = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<Wrench6D>(*$self); }
        double get(std::size_t i) const { return (*$self)[i]; }
        void set(std::size_t i, double d){ (*$self)[i] = d; }
#endif
    };

    const Wrench6D operator*( double s) const;
    /*
    friend const Wrench6D<T> operator*(const Transform3D& aTb,
                                              const Wrench6D& bV);
    
    friend const Wrench6D operator*(const Vector3D& aPb,
                                       const Wrench6D& bV);


    friend const Wrench6D operator*(const Rotation3D& aRb, const Wrench6D& bV);
*/
#if defined(SWIGJAVA)
	%rename(subtract) operator-(const Wrench6D&) const;
	%rename(add) operator+(const Wrench6D&) const;
#endif
    const Wrench6D operator+(const Wrench6D& wrench) const;    
    const Wrench6D operator-(const Wrench6D& wrench) const;

    double norm1() const;
    
    double norm2() const ;
    double normInf() const ;
};


class InertiaMatrix{
public:
    InertiaMatrix(
        double r11, double r12, double r13,
        double r21, double r22, double r23,
        double r31, double r32, double r33);

    InertiaMatrix(
        const Vector3D& i,
        const Vector3D& j,
        const Vector3D& k);

    InertiaMatrix(
        double i = 0.0,
        double j = 0.0,
        double k = 0.0);

#if !defined(SWIGJAVA)
    double& operator()(size_t row, size_t column);
    const double& operator()(size_t row, size_t column) const;
#endif

    %extend {
#if (defined(SWIGLUA) || defined(SWIGPYTHON))
        char *__str__() { return printCString<InertiaMatrix>(*$self); }
        double __getitem__(std::size_t row, std::size_t column)const {return (*$self)(row,column); }
        void __setitem__(std::size_t row, std::size_t column, double d){ (*$self)(row,column) = d; }
#elif defined(SWIGJAVA)
        std::string toString() const { return toString<InertiaMatrix>(*$self); }
        double get(std::size_t row, std::size_t column) const { return (*$self)(row,column); }
        void set(std::size_t row, std::size_t column, double val) { (*$self)(row,column) = val; }
#endif
    };

    //const Base& m() const;
    //Base& m();

    //friend InertiaMatrix operator*(const Rotation3D<T>& aRb, const InertiaMatrix& bRc);
    //friend InertiaMatrix operator*(const InertiaMatrix& aRb, const Rotation3D<T>& bRc);
    //friend InertiaMatrix operator+(const InertiaMatrix& I1, const InertiaMatrix& I2);
    //friend Vector3D<T> operator*(const InertiaMatrix& aRb, const Vector3D<T>& bVc);
    //friend InertiaMatrix inverse(const InertiaMatrix& aRb);
    //friend std::ostream& operator<<(std::ostream &os, const InertiaMatrix& r);

    static InertiaMatrix makeSolidSphereInertia(double mass, double radi);
    static InertiaMatrix makeHollowSphereInertia(double mass, double radi);
    static InertiaMatrix makeCuboidInertia(double mass, double x, double y, double z);
};

%nodefaultctor Metric;
template <class T>
class Metric
{
public:
    double distance(const T& q) const;

    double distance(const T& a, const T& b) const;

    int size() const;

};

%template (MetricQ) Metric<Q>;
%template (MetricQPtr) rw::common::Ptr<Metric<Q> >;
%template (MetricSE3) Metric<Transform3D>;
%template (MetricSE3Ptr) rw::common::Ptr<Metric<Transform3D> >;

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


    State getDefaultState() const;

    //rw::common::Ptr<StateStructure> getStateStructure();

    PropertyMap& getPropertyMap();
private:
    WorkCell(const WorkCell&);
    WorkCell& operator=(const WorkCell&);
};

%template (WorkCellPtr) rw::common::Ptr<WorkCell>;

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

class PlannerConstraint
{
public:
    PlannerConstraint();

    //PlannerConstraint(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

    bool inCollision(const Q& q);

    bool inCollision(const Q& q1, const Q& q2);

    //QConstraint& getQConstraint() const { return *_constraint; }

    //QEdgeConstraint& getQEdgeConstraint() const { return *_edge; }

    //const QConstraint::Ptr& getQConstraintPtr() const { return _constraint; }

    //const QEdgeConstraint::Ptr& getQEdgeConstraintPtr() const { return _edge; }

    //static PlannerConstraint make(QConstraint::Ptr constraint, QEdgeConstraint::Ptr edge);

    static PlannerConstraint make(rw::common::Ptr<CollisionDetector> detector,
                                  rw::common::Ptr<Device> device,
                                  const State& state);

    static PlannerConstraint make(rw::common::Ptr<CollisionStrategy> strategy,
                                  rw::common::Ptr<WorkCell> workcell,
                                  rw::common::Ptr<Device> device,
                                  const State& state);

    /*
    static PlannerConstraint make(rw::proximity::CollisionStrategy::Ptr strategy,
        const rw::proximity::CollisionSetup& setup,
        rw::common::Ptr<WorkCell> workcell,
        rw::common::Ptr<Device> device,
        const State& state);
     */
};

%template (PlannerConstraintPtr) rw::common::Ptr<PlannerConstraint>;

%nodefaultctor StopCriteria;
class StopCriteria
 {
#if defined(SWIGJAVA)
%apply bool[] {bool *};
#endif
 
 public:
     bool stop() const;
     rw::common::Ptr<StopCriteria> instance() const;
     virtual ~StopCriteria();
     static rw::common::Ptr<StopCriteria> stopAfter(double time);
     static rw::common::Ptr<StopCriteria> stopNever();
     static rw::common::Ptr<StopCriteria> stopNow();
     static rw::common::Ptr<StopCriteria> stopByFlag(bool* stop);
     //static rw::common::Ptr<StopCriteria> stopByFun(boost::function<bool ()> fun);
     static rw::common::Ptr<StopCriteria> stopCnt(int cnt);
     static rw::common::Ptr<StopCriteria> stopEither(
         const std::vector<rw::common::Ptr<StopCriteria> >& criteria);

     static rw::common::Ptr<StopCriteria> stopEither(
         const rw::common::Ptr<StopCriteria>& a,
         const rw::common::Ptr<StopCriteria>& b);
};

%template (StopCriteriaPtr) rw::common::Ptr<StopCriteria>;
%template (StopCriteriaPtrVector) std::vector<rw::common::Ptr<StopCriteria> >;

/*
%nodefaultctor PathPlanner;
template <class From, class To, class PATH = Path<From> >
class PathPlanner
{
public:

    bool query(const From& from, To& to, PATH& path, const StopCriteria& stop);

    bool query(const From& from, To& to, PATH& path, double time);

    bool query(const From& from, To& to, PATH& path);

    PropertyMap& getProperties();

};
%nodefaultctor QToQPathPlanner;
%template (QToQPathPlanner) PathPlanner<Q,const Q,QPath>;
*/
%nodefaultctor QToQPlanner;
class QToQPlanner {
public:

    %extend {

        rw::common::Ptr<Path<Q> > query(Q from, Q to, rw::common::Ptr<StopCriteria> stop){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Q>::query(from,to,*path,*stop);
            return path;
        }

        rw::common::Ptr<Path<Q> > query(Q from, Q to, double time){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Q>::query(from,to,*path,time);
            return path;
        }

        rw::common::Ptr<Path<Q> > query(Q from, Q to){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Q>::query(from,to,*path);
            return path;
        }

        PropertyMap& getProperties(){
            return $self->rw::pathplanning::PathPlanner<Q,const Q>::getProperties();
        }

        static rw::common::Ptr<QToQPlanner> makeRRT(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev, const State& state){
            const rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(
                cdect.get(), dev, state);
            return rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, dev);
        }

        static rw::common::Ptr<QToQPlanner> makeSBL(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev, const State& state){
            rw::pathplanning::QConstraint::Ptr qconstraint = rw::pathplanning::QConstraint::make(cdect.get(), dev, state);
            return rwlibs::pathplanners::SBLPlanner::makeQToQPlanner(rwlibs::pathplanners::SBLSetup::make(qconstraint, rw::pathplanning::QEdgeConstraintIncremental::makeDefault(qconstraint, dev), dev));
        }
    };
};

%template (QToQPlannerPtr) rw::common::Ptr<QToQPlanner>;


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
%template (BlendR2) Blend<Vector2D>;
%template (BlendR3) Blend<Vector3D>;
%template (BlendSO3) Blend<Rotation3D>;
%template (BlendSE3) Blend<Transform3D>;
%template (BlendQ) Blend<Q>;

%template (BlendR1Ptr) rw::common::Ptr<Blend<double> >;
%template (BlendR2Ptr) rw::common::Ptr<Blend<Vector2D> >;
%template (BlendR3Ptr) rw::common::Ptr<Blend<Vector3D> >;
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
%template (InterpolatorR2) Interpolator<Vector2D>;
%template (InterpolatorR3) Interpolator<Vector3D>;
%template (InterpolatorSO3) Interpolator<Rotation3D>;
%template (InterpolatorSE3) Interpolator<Transform3D>;
%template (InterpolatorQ) Interpolator<Q>;

%template (InterpolatorR1Ptr) rw::common::Ptr<Interpolator<double> >;
%template (InterpolatorR2Ptr) rw::common::Ptr<Interpolator<Vector2D> >;
%template (InterpolatorR3Ptr) rw::common::Ptr<Interpolator<Vector3D> >;
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


class RampInterpolatorR3: public Interpolator<Vector3D> {
public:
    RampInterpolatorR3(const Vector3D& start, const Vector3D& end,
                       double vellimit,double acclimit);

    Vector3D x(double t) const;
    Vector3D dx(double t) const;
    Vector3D ddx(double t) const;
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
%template (TrajectoryR2) Trajectory<Vector2D>;
%template (TrajectoryR3) Trajectory<Vector3D>;
%template (TrajectorySO3) Trajectory<Rotation3D>;
%template (TrajectorySE3) Trajectory<Transform3D>;
%template (TrajectoryQ) Trajectory<Q>;

%template (TrajectoryStatePtr) rw::common::Ptr<Trajectory<State> >;
%template (TrajectoryR1Ptr) rw::common::Ptr<Trajectory<double> >;
%template (TrajectoryR2Ptr) rw::common::Ptr<Trajectory<Vector2D> >;
%template (TrajectoryR3Ptr) rw::common::Ptr<Trajectory<Vector3D> >;
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
%template (InterpolatorTrajectoryR2) InterpolatorTrajectory<Vector2D>;
%template (InterpolatorTrajectoryR3) InterpolatorTrajectory<Vector3D>;
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
	Vector3D maxContactForce;
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
    typedef enum Status {
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
     } TestStatus;

    GraspTask():
    GraspTask(rw::common::Ptr<Task<Transform3D> > task);
    rw::common::Ptr<Task<Transform3D> > toCartesianTask();
    std::string getGripperID();
    std::string getTCPID();
    std::string getGraspControllerID();
    void setGripperID(const std::string& id);
    void setTCPID(const std::string& id);
    void setGraspControllerID(const std::string& id);
    //void addSubTask( class GraspSubTask& stask);
    //std::vector<class GraspSubTask>& getSubTasks(){ return _subtasks;};
    //void filterTasks( std::vector<GraspTask::TestStatus> &includeMask);
    //std::vector<std::pair<class GraspSubTask*,class GraspTarget*> > getAllTargets();
    static std::string toString(TestStatus status);
    static void saveUIBK(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveRWTask(rw::common::Ptr<GraspTask> task, const std::string& name );
    static void saveText(rw::common::Ptr<GraspTask> task, const std::string& name );
    static rw::common::Ptr<GraspTask> load(const std::string& name);
    static rw::common::Ptr<GraspTask> load(std::istringstream& inputStream);
    rw::common::Ptr<GraspTask> clone();
};

%template (GraspTaskPtr) rw::common::Ptr<GraspTask>;

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


