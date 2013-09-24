
#include "SDHInvKinSolver.hpp"

#include <rw/models/CompositeJointDevice.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

SDHInvKinSolver::SDHInvKinSolver()
{
    _wc = rw::loaders::WorkCellLoader::Factory::load("SDHDeviceInvKin.wc.xml");
    RW_ASSERT(_wc);
    State state = _wc->getDefaultState();

    _sdh = _wc->findDevice<TreeDevice>("SchunkHand");
    RW_ASSERT(_sdh);
    Frame *tcpframe = _wc->findFrame("SchunkHand.TCP");

    _sdhbase = dynamic_cast<MovableFrame*>(_sdh->getBase());
    RW_ASSERT(_sdhbase);

    _baseTtcp = Kinematics::frameTframe( _sdhbase, tcpframe, state);

    //SE3Device *se3dev = new SE3Device("SE3Dev", _sdhbase->getParent(state), _sdhbase);
    _se3dev = _wc->findDevice("SE3Dev");

    std::vector<Device::Ptr> devices;
    std::vector<rw::kinematics::Frame*> ends = _sdh->getEnds();
    devices.push_back( _se3dev );
    devices.push_back( _sdh );

    CompositeJointDevice *cdev = new CompositeJointDevice(_se3dev->getBase(), devices, ends, "composite", state);
    _jiksolver =  new JacobianIKSolverM(cdev, ends, _wc->getDefaultState());
    _jiksolver->setReturnBestFit(true);
    _jiksolver->setClampToBounds(true);
    _jiksolver->setEnableInterpolation(true);
    _jiksolver->setMaxIterations(6);
    _jiksolver->setSolverType(JacobianIKSolverM::SVD);

    std::vector<rw::kinematics::Frame*> ends2f;
    ends2f.push_back(_sdh->getEnds()[1] );
    ends2f.push_back(_sdh->getEnds()[2] );
    CompositeJointDevice *cdev2f = new CompositeJointDevice(_se3dev->getBase(), devices, ends2f, "composite2f", state);
    _jiksolver2f = new JacobianIKSolverM(cdev2f, ends2f, _wc->getDefaultState());
    _jiksolver2f->setReturnBestFit(true);
    _jiksolver2f->setClampToBounds(true);
    _jiksolver2f->setEnableInterpolation(true);
    _jiksolver2f->setMaxIterations(6);
    _jiksolver2f->setSolverType(JacobianIKSolverM::SVD);


    _iksolver =  new IKSoftCSolver(_sdh.get(), _sdh->getEnds(), _wc->getDefaultState() );
    _iksolver->setReturnBestFit(true);


    _defState = _wc->getDefaultState();


    _nrReducedTargets = 0;
}


std::vector<boost::tuple<rw::math::Transform3D<>, rw::math::Q, bool> > SDHInvKinSolver::solve(
                                   const std::vector<rw::math::Transform3D<> >& targetsDef,
                                   const rw::math::Vector3D<>& approach) const
{
    if(targetsDef.size()!=3)
        RW_WARN("target size wrong! " << targetsDef.size());

    if(_path.size()%100>90)
        PathLoader::storeTimedStatePath(*_wc,_path,"MyInvKinStatePath.traj.xml");

    // test if any targets are too close too consider a 3-finger grasp
    std::vector<Transform3D<> > reducedTargets(1,targetsDef[0]);
    for(std::size_t i=1;i<targetsDef.size();i++){
        bool found=false;
        for(std::size_t j=0;j<reducedTargets.size();j++){
            if( MetricUtil::dist2(targetsDef[i].P(), reducedTargets[j].P())<0.02
                    && angle( targetsDef[i].R()*Vector3D<>::z(),  reducedTargets[j].R()*Vector3D<>::z())<45*Deg2Rad
                ){
                reducedTargets[j].P() = (reducedTargets[j].P()+targetsDef[i].P())/2.0;
                found = true;
                break;
            }
        }
        if(!found)
            reducedTargets.push_back( targetsDef[i] );
    }
    if(reducedTargets.size()<3)
        _nrReducedTargets++;
    //std::cout << "  [" << _nrReducedTargets << "]  ";


    std::vector<boost::tuple<Transform3D<>, Q, bool> > result;
    State stateInit = _defState;
    std::vector<Transform3D<> > targets = targetsDef;
    for(std::size_t i=0;i<targets.size(); i++){
        Vector3D<> zaxis = targetsDef[i].R()*Vector3D<>::z();
        targets[i].P() -= zaxis*0.01;
    }

    _wc->findFrame<MovableFrame>("target1")->setTransform( targets[0], stateInit );
    _wc->findFrame<MovableFrame>("target2")->setTransform( targets[1], stateInit );
    _wc->findFrame<MovableFrame>("target3")->setTransform( targets[2], stateInit );

    Vector3D<> center = (targets[0].P() + targets[1].P() + targets[2].P())/3.0;

    Transform3D<> t3dtarget4(center, EAA<>(Vector3D<>::z(), approach).toRotation3D());
    _wc->findFrame<MovableFrame>("target4")->setTransform( t3dtarget4, stateInit );


    if(reducedTargets.size()==2){
        std::vector<rw::math::Transform3D<> > bTeds = reducedTargets;
        State state = stateInit;
        // place the hand in the default pinch grasp state
        _sdh->setQ( Q(7,-1.571,-1.571,1.571,0.0,0.0,0.0,0.0), state);

        Vector3D<> centroid = (bTeds[0].P() + bTeds[1].P())/2.0;
        // remember now the hand can be placed on either side of this plane.

        Vector3D<> basePos = centroid-(approach*0.23);
        Rotation3D<> baseRot = EAA<>(Vector3D<>(0,0,1), approach).toRotation3D();
        Transform3D<> baseTarget( basePos , baseRot );

        // next we rotate the hand around the z-axis such that the position of the finger is close to the
        // position of the first target point

        Vector3D<> f1Point = baseTarget * Kinematics::frameTframe(_sdh->getBase(), _sdh->getEnds()[1], state).P();
        double ang = angle(f1Point-centroid, bTeds[0].P()-centroid, approach);

        EAA<> r1( approach, ang );
        baseTarget.R() = r1.toRotation3D()*baseTarget.R();
        Vector3D<> v = baseTarget.P();
        RPY<> r(baseTarget.R());

        //_sdhbase->setTransform(baseTarget, state);
        _se3dev->setQ(Q(6, v[0], v[1], v[2],  r(0), r(1), r(2) ), state);
        std::vector<Q> res = _jiksolver2f->solve(bTeds, state);
        if( res.size() > 0 ){
            //Transform3D<> t3d = _iksolver->getBaseT3D();
            //Q q = res[0];
            Q qp = res[0].getSubPart(0,6);
            Q q = res[0].getSubPart(6,_sdh->getDOF());
            //Transform3D<> t3d( Vector3D<>(qp[0],qp[1],qp[2]) , RPY<>(qp[3],qp[4],qp[5])) ;

            _sdh->setQ( q, state );
            //_sdhbase->setTransform( t3d, state );
            _se3dev->setQ(qp, state);
            Transform3D<> t3d = Kinematics::frameTframe(_se3dev->getBase(),_sdhbase, state);

            bool badInvKin = false;
            for(size_t i=0;i<bTeds.size();i++){
                const Transform3D<>& bTed = bTeds[i];
                const Transform3D<>& bTe  = Kinematics::worldTframe( _sdh->getEnds()[i+1] , state);
                if( MetricUtil::dist2(bTe.P(),bTed.P())>0.01 ){
                    badInvKin=true;
                }
            }


            if(!badInvKin){

                bool wrongApproach = dot(t3d.R()*Vector3D<>::z(), approach)<0;
                if(!wrongApproach){
                    _sdh->setQ( q, state );
                    //_sdhbase->setTransform( t3d, state );
                    _se3dev->setQ(qp, state);
                    _path.push_back( TimedState(_path.size()+0.3, state) );
                    result.push_back( boost::make_tuple(t3d*_baseTtcp, q, true));
                    return result;
                }
            }
        }

    }

    boost::tuple<double, Transform3D<>, Q, Q> tmpResult( boost::make_tuple(100.0, Transform3D<>::identity(),Q::zero(7), Q::zero(7)) );
    for(int k=0;k<3;k++){
        State state = stateInit;
        std::vector<rw::math::Transform3D<> > bTeds = targets;
        if(k==1){
            bTeds[0] = targets[1];
            bTeds[1] = targets[2];
            bTeds[2] = targets[0];
        } else if( k==2){
            bTeds[0] = targets[2];
            bTeds[1] = targets[0];
            bTeds[2] = targets[1];
        }

        // first we need to try and place the hand perpendicular to the plane of the contacts
        //plane normal
        Vector3D<> normal = cross(bTeds[1].P()-bTeds[0].P(), bTeds[2].P()-bTeds[0].P());
        normal = -normalize(normal);
        // make sure the normal points in the approach direction
        if(dot(normal,approach)<0 )
            normal = -normal;

        //normal = approach;

        Vector3D<> centroid = (bTeds[0].P() + bTeds[1].P() + bTeds[2].P())/3.0;
        // remember now the hand can be placed on either side of this plane.

        Vector3D<> basePos = centroid-(normal*0.23);
        Rotation3D<> baseRot = EAA<>(Vector3D<>(0,0,1), normal).toRotation3D();
        Transform3D<> baseTarget( basePos , baseRot );

        // next we rotate the hand around the z-axis such that the position of the finger is close to the
        // position of the first target point

        Vector3D<> f1Point = baseTarget * Kinematics::frameTframe(_sdh->getBase(), _sdh->getEnds()[0], state).P();
        double ang = angle(f1Point-centroid, bTeds[0].P()-centroid, normal);

        EAA<> r1( normal, ang );
        baseTarget.R() = r1.toRotation3D()*baseTarget.R();
        Vector3D<> v = baseTarget.P();
        RPY<> r(baseTarget.R());

        //_sdhbase->setTransform(baseTarget, state);
        _se3dev->setQ(Q(6, v[0], v[1], v[2],  r(0), r(1), r(2) ), state);

        // we sort the targets such that they fit
        f1Point = baseTarget * Kinematics::frameTframe(_sdh->getBase(), _sdh->getEnds()[0], state).P();
        double ang1 = angle(f1Point-centroid, bTeds[1].P()-centroid, normal);
        double ang2 = angle(f1Point-centroid, bTeds[2].P()-centroid, normal);
        if(ang1<ang2){
            std::swap(bTeds[1],bTeds[2]);
        }


        //_path.push_back( TimedState(_path.size(), state) );

        //std::cout << i <<";"<< k << ";" << j << std::endl;
        //std::vector<Q> res = _iksolver->solve(bTeds, state);
        std::vector<Q> res = _jiksolver->solve(bTeds, state);

        bool badInvKin=false;
        if( res.size() > 0 ){
            //Transform3D<> t3d = _iksolver->getBaseT3D();
            //Q q = res[0];
            Q qp = res[0].getSubPart(0,6);
            Q q = res[0].getSubPart(6,_sdh->getDOF());


            _sdh->setQ( q, state );
            //_sdhbase->setTransform( t3d, state );
            _se3dev->setQ(qp, state);
            Transform3D<> t3d = Kinematics::frameTframe(_se3dev->getBase(),_sdhbase, state);
            //( Vector3D<>(qp[0],qp[1],qp[2]) , RPY<>(qp[3],qp[4],qp[5])) ;

            double maxAngError = 0, maxDistError = 0;
            for(size_t i=0;i<bTeds.size();i++){
                const Transform3D<>& bTed = bTeds[i];
                const Transform3D<>& bTe  = Kinematics::worldTframe( _sdh->getEnds()[i] , state);
                maxDistError = std::max(maxDistError, MetricUtil::dist2(bTe.P(),bTed.P()));
                maxAngError = std::max(maxAngError, angle( bTe.R()*Vector3D<>::z(),  bTed.R()*Vector3D<>::z()));
                if( maxDistError>0.01  || maxAngError>30*Deg2Rad ){
                    badInvKin=true;
                }
            }

            if(!badInvKin){

                bool wrongApproach = dot(t3d.R()*Vector3D<>::z(), approach)<0;
                if(!wrongApproach){
                    _sdh->setQ( q, state );
                    //_sdhbase->setTransform( t3d, state );
                    _se3dev->setQ(qp, state);
                    _path.push_back( TimedState(_path.size()+0.3, state) );
                    result.push_back( boost::make_tuple( t3d*_baseTtcp, q, true));
                    return result;
                }
            } else {
                if(tmpResult.get<0>()>maxDistError){
                    tmpResult.get<0>() = maxDistError;
                    tmpResult.get<1>() = t3d;
                    tmpResult.get<2>() = q;
                    tmpResult.get<3>() = qp;
                }
            }
        } else {
            // else
            //_sdhbase->setTransform( Transform3D<>::identity(), state );
            _se3dev->setQ(Q::zero(6), state);
            _path.push_back( TimedState(_path.size()+0.3, state) );
        }
    }
    // choose the best of the failed grasps and return that
    _sdh->setQ( tmpResult.get<2>(), stateInit );
    _se3dev->setQ(tmpResult.get<3>(), stateInit);
    _path.push_back( TimedState(_path.size()+0.3, stateInit) );
    result.push_back( boost::make_tuple(tmpResult.get<1>()*_baseTtcp, tmpResult.get<2>(), false));
    return result;
}
