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
%nodefaultctor QToTPlanner;

class QToTPlanner 
{
public:
	//! @brief smart pointer type to this class
	typedef rw::common::Ptr<QToTPlanner> Ptr;

    %extend {

        rw::common::Ptr<Path<Q> > query(Q from, Transform3D to, rw::common::Ptr<StopCriteria> stop){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Transform3D>::query(from,to,*path,*stop);
            return path;
        }

        rw::common::Ptr<Path<Q> > query(Q from, Transform3D to, double time){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Transform3D>::query(from,to,*path,time);
            return path;
        }

        rw::common::Ptr<Path<Q> > query(Q from, Transform3D to){
            rw::common::Ptr<Path<Q> > path = rw::common::ownedPtr(new PathQ());
            $self->rw::pathplanning::PathPlanner<Q,const Transform3D>::query(from,to,*path);
            return path;
        }

        PropertyMap& getProperties(){
            return $self->rw::pathplanning::PathPlanner<Q,const Transform3D>::getProperties();
        }
	}	
};

%template (QToTPlannerPtr) rw::common::Ptr<QToTPlanner>;

%inline %{
/*
	rw::common::Ptr<QToTPlanner> makeQToTBidirectionalRRT(
		QToQSamplerPlanner::Ptr planner,
		QIKSampler::Ptr ikSampler)
	{ return NULL; }
  */  
	rw::common::Ptr<QToTPlanner> makeToNearestRRT(rw::common::Ptr<CollisionDetector> cdect, rw::common::Ptr<Device> dev)
    { 
		rw::kinematics::State state = dev->getStateStructure()->getDefaultState();
      
        const rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(
            cdect.get(), dev, state);
      
        rw::common::Ptr<QToQPlanner> planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, dev);

		// creaty ikmeta solver for the planer
        rw::invkin::JacobianIKSolver::Ptr iksolver = rw::common::ownedPtr( new rw::invkin::JacobianIKSolver(dev,state) );
       	rw::invkin::IKMetaSolver::Ptr metasolver = rw::common::ownedPtr( new rw::invkin::IKMetaSolver(iksolver, dev, cdect) );
		rw::common::Ptr<rw::pathplanning::QIKSampler> sampler = rw::pathplanning::QIKSampler::make(dev,state,metasolver);
        
		rw::math::QMetric::Ptr metric = rw::pathplanning::PlannerUtil::timeMetric(*dev);

		return rw::pathplanning::QToTPlanner::makeToNearest(planner, sampler, metric, 10); 
    }
/*
	rw::common::Ptr<QToTPlanner> makeToNearest(
		rw::common::Ptr<QToQPlanner> planner,
		rw::common::Ptr<QIKSampler> sampler,
		rw::common::Ptr<QMetric> metric,
        int cnt)
    { return NULL; }
  */  
%}


