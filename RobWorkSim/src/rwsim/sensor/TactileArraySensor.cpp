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

#include "TactileArraySensor.hpp"

#include <rw/math/Vector2D.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/MetricUtil.hpp>
#include <rw/math/Math.hpp>

#include <rw/sensor/Contact3D.hpp>

#include <rw/geometry.hpp>
#include <rw/common.hpp>
//#include <rw/proximity/Proximity.hpp>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rw::sensor;
using namespace rw::kinematics;
using namespace rw::proximity;
using namespace rw::geometry;
using namespace rw::common;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rwsim;

#define MIN_CONTACT_FORCE 0.1

namespace {

    /**
     * @brief gets the average value of a 3x3 vertex matrix and the associated
     * tactile values around the coordinate (i,j).
     * @return the max tactile value and the calculatedcontact point
     */
    std::pair<double,Vector3D<> > getWeightAverage(
                          size_t i, size_t j,
                          const TactileArraySensor::ValueMatrix& tMatrix,
                          const TactileArraySensor::VertexMatrix& vMatrix)
    {
    	size_t i_low = i-1,i_upp=i+1;
        if(i_low<0) i_low = 0;
        if(i_upp>=(size_t)tMatrix.rows())
        	i_upp = tMatrix.rows()-1;

        size_t j_low = j-1,j_upp=j+1;
        if(j_low<0) j_low = 0;
        if(j_upp>=(size_t)tMatrix.cols())
        	j_upp = tMatrix.cols()-1;

        Vector3D<> wp(0,0,0);
        double valSum = 0, maxVal=0;
        for(size_t k=i_low;k<=i_upp;k++)
            for(size_t l=j_low;l<=j_upp;l++){
                wp += vMatrix[k][l]*tMatrix(k,l);
                valSum += tMatrix(k,l);
                maxVal = std::max(tMatrix(k,l), (float)maxVal);
            }
        return std::make_pair(maxVal, wp/valSum);
    }

    /**
     * @brief the penetration is calculated using a simple linearized model
     * of the sensor. The penetration depth is calculated as linear with the
     * force and in the unit mm
     */
    double calcPenetration(
        size_t i, size_t j,
        const Eigen::MatrixXf& tMatrix)
    {
        // the pen depth depends on force like this pendepth = 0.00025 f - 0.0625
        return 0.00025*tMatrix(i,j) - 0.0625;
    }
    typedef boost::multi_array<double, 2> array_type;

    boost::array<array_type::index, 2> getShape(const TactileArraySensor::ValueMatrix& tMatrix,int x, int y){
        boost::array<array_type::index, 2> shape = {{ tMatrix.rows() +x, tMatrix.cols()+y }};
        return shape;
    }

    class TactileArrayWrapper: public rw::sensor::TactileArray {
    private:
        TactileArraySensor* _sensor;
        rwlibs::simulation::Simulator::Ptr _sim;
    public:

        TactileArrayWrapper(TactileArraySensor* sensor, rwlibs::simulation::Simulator::Ptr sim):
            TactileArray(sensor->getName()),
            _sensor(sensor),
    		_sim(sim)
        {
            this->setSensorModel(sensor->getSensorModel());
        }

        rw::math::Vector2D<> getTexelSize(int x, int y) const{ return _sensor->getTexelSize(x,y); }
        std::pair<double,double> getPressureLimit() const{ return _sensor->getPressureLimit(); }
        const TactileArray::VertexMatrix& getVertexGrid() const{ return _sensor->getVertexGrid();}
        const rw::math::Transform3D<>& getTransform() const{ return _sensor->getTransform(); }
        const TactileArray::VertexMatrix& getCenters() const{ return _sensor->getCenters(); }
        const TactileArray::VertexMatrix& getNormals()  const{ return _sensor->getNormals(); }
        int getWidth() const{ return _sensor->getWidth(); }
        int getHeight() const{ return _sensor->getHeight(); }

        void acquire(){ /*_sensor->acquire(_sim->getState());*/ }
        const rw::sensor::TactileArrayModel::ValueMatrix& getTexelData( ) const{ return _sensor->getTexelData(_sim->getState()); };

    };

}


const std::vector<rw::sensor::Contact3D>& TactileArraySensor::getActualContacts(const rw::kinematics::State& state){
	return _sdata.getStateCache<ClassState>(state)->getActualContacts();
}

rw::sensor::TactileArray::Ptr TactileArraySensor::getTactileArraySensor(rwlibs::simulation::Simulator::Ptr sim){
	// check if handle has already been added to simulator
	TactileArray::Ptr sensor;
	if(!sim->hasHandle(this)){
		sensor = rw::common::ownedPtr( new TactileArrayWrapper(this, sim) );
		sim->addHandle(this, sensor);
	} else {
		sensor = sim->getSensorHandle(this).cast<TactileArray>();
	}
	return sensor;
}
rw::sensor::Sensor::Ptr TactileArraySensor::getSensor(rwlibs::simulation::Simulator::Ptr sim){
    return getTactileArraySensor(sim);
}

TactileArraySensor::~TactileArraySensor(){

}

TactileArraySensor::TactileArraySensor(const std::string& name,
    dynamics::Body::Ptr obj,
    const rw::math::Transform3D<>& fThmap,
    const ValueMatrix& heightMap,
    const rw::math::Vector2D<>& texelSize):
        SimulatedTactileSensor( ownedPtr( new rw::sensor::TactileArrayModel(name, _body->getBodyFrame(), fThmap, heightMap, texelSize[0], texelSize[1])  ) ),
        _contactMatrix(getShape(heightMap,-1,-1)),
        _distCenterMatrix(getShape(heightMap,-1,-1)),
        _distDefMatrix(Eigen::MatrixXf::Zero(heightMap.rows()-1,heightMap.cols()-1)),
        _texelSize(texelSize),
        _texelArea(texelSize(0)*texelSize(1)),
        _fThmap(fThmap),
        _hmapTf(inverse(fThmap)),
        _dmask(5*3,5*3),
        _narrowStrategy(new rwlibs::proximitystrategies::ProximityStrategyPQP()),
        _maxPenetration(0.0015),
        _elasticity(700),// KPa ~ 0.0008 GPa
        _tau(0.1),
        _body(obj)
{

	_tmodel = getSensorModel().cast<TactileArrayModel>();
	_tmodel->setPressureLimit(0,250); // in kPa


	int w = getWidth();
	int h = getHeight();

    // calculate the distribution matrix, let it span 4x4 texels
    _maskWidth = 3*_texelSize(0);
    _maskHeight = 3*_texelSize(1);

    // we use the distribution function 1/(1+x^2+y^2) where x and y is described
    // relative to the center of the mask
    // the resolution of the mask is 5 points per texel which yields a mask of 20x20
    _dmask = Eigen::MatrixXf(5*3,5*3);

    double dmaskSum = 0;
    double cx = 1.0/2.0;
    double cy = 1.0/2.0;

    double tSize1 = 1.0/_dmask.rows();
    double tSize2 = 1.0/_dmask.cols();

    for(Eigen::DenseIndex i=0;i<_dmask.rows();i++){
        for(Eigen::DenseIndex j=0;j<_dmask.cols();j++){
            double x = i*tSize1-cx;
            double y = j*tSize2-cy;
            double r = 4.0*sqrt(x*x+y*y);
            double val = -0.3476635514018692+1.224299065420561/(1.0+r*r);
            _dmask(i,j) = (float)std::max(val,0.0);
            dmaskSum += std::max(val,0.0);
            std::cout << r << ",";
        }
    }

    _dmask = _dmask/((float)dmaskSum);

    // create a geometry of the normals

    const TactileArray::VertexMatrix& normals = _tmodel->getNormals();
    const TactileArray::VertexMatrix& centers = _tmodel->getCenters();

    Vector3D<> offsetDir = (centers[1][0]-centers[0][0])/5;
    //std::cout << "offesetDir: " << offsetDir << std::endl;

    PlainTriMesh<Triangle<> > *trimesh = new PlainTriMesh<Triangle<> >(h*w);
    for(int x=0;x<w;x++){
    	for(int y=0;y<h;y++){
    		int i=x*h+y;
    		(*trimesh)[i][0] = centers[x][y];
    		(*trimesh)[i][1] = centers[x][y]+offsetDir;
    		(*trimesh)[i][2] = centers[x][y]+normals[x][y]*0.008;
        }
    }

    _ntrimesh = ownedPtr(trimesh);
    _ngeom = ownedPtr( new Geometry( _ntrimesh ));
    _nmodel = _narrowStrategy->createModel();

    _narrowStrategy->addGeometry(_nmodel.get(), *_ngeom );
	_frameToGeoms[_body->getBodyFrame()] = _body->getGeometry();
	std::vector<Geometry::Ptr> &geoms = _frameToGeoms[_body->getBodyFrame()];
    _narrowStrategy->setFirstContact(false);
    //std::cout << "DMask: " << _dmask << std::endl;
    //std::cout << "Finger pad dimensions: (" << _texelSize(0)*(_w+1) << "," << (_h+1)*_texelSize(1) <<")" <<  std::endl;

    /// now build the map of contacts between normals and surface of finger

	Transform3D<> wTb = Transform3D<>::identity();
	//ProximityModelPtr modelA = _narrowStrategy->getModel(tframe);
	ProximityModel::Ptr model = _narrowStrategy->getModel(_body->getBodyFrame());
	ProximityStrategyData pdata;
	_narrowStrategy->inCollision(_nmodel, _fThmap, model, wTb, pdata);
	CollisionResult &data = pdata.getCollisionData();
	if(data._collisionPairs.size()>0){
		int bodyGeomId = data._collisionPairs[0].geoIdxB;
		//std::cout << "BodyGeomId: " << std::endl;
		TriMesh *mesh = dynamic_cast<TriMesh*> (geoms[bodyGeomId]->getGeometryData().get());
		if(mesh){
		    int startIdx = data._collisionPairs[0].startIdx;
		    int size = data._collisionPairs[0].size;
			for(int i = startIdx; i<startIdx+size;i++){
				std::pair<int,int> &pids = data._geomPrimIds[i];
				//std::cout << "Colliding pairs: " << pids.first << " <---> " << pids.second << std::endl;
				RW_ASSERT(0<=pids.first);
				RW_ASSERT(pids.first<(int)_ntrimesh->getSize());

				// for each colliding pair we find the closest intersection
				// get the triangle
				Triangle<> triA = _ntrimesh->getTriangle(pids.first);
				Triangle<> tri = mesh->getTriangle(pids.second);
/*
				std::cout << "POINTS1:"
						  << "\n  " <<  triA[0]
						  << "\n  " <<  triA[1]
						  << "\n  " <<  triA[2] << std::endl;

				std::cout << "POINTS2:"
						  << "\n  " <<  data._aTb*tri[0]
						  << "\n  " <<  data._aTb*tri[1]
						  << "\n  " <<  data._aTb*tri[2] << std::endl;
*/
				Vector3D<> point;
				if( !IntersectUtil::intersetPtRayPlane(triA[0], triA[2], pdata.aTb()*tri[0], pdata.aTb()*tri[1], pdata.aTb()*tri[2], point) )
					continue;

				// now we have the point of intersection, now save it in the contact array
				// if its closer than the existing point

				double ndist = MetricUtil::dist2( triA[0], point);
				//std::cout << "Intersect: " << point << std::endl;
				//std::cout << "NDist: " << ndist << std::endl;
				if( ndist < (&_distDefMatrix(0,0))[pids.first] )
					continue;
				(&_distDefMatrix(0,0))[pids.first] = (float)ndist;
				(&_contactMatrix[0][0])[pids.first] = point;
			}
		}
	}
	//std::cout << "Dist" << _distDefMatrix << std::endl;

    // create state object and add it to
	_sdata = StatelessData<int>(1, rw::common::ownedPtr( new ClassState(this, getWidth(), getHeight())).cast<rw::kinematics::StateCache>()),
    add( _sdata );
}

TactileArraySensor::ClassState::ClassState(TactileArraySensor* tsensor, size_t dim_x, size_t dim_y):
    _tsensor(tsensor),
    _distMatrix(Eigen::MatrixXf::Zero(dim_x,dim_y)),
    _accForces(Eigen::MatrixXf::Zero(dim_x,dim_y)),
    _pressure(Eigen::MatrixXf::Zero(dim_x,dim_y)),
    _accTime(0),
    _stime(0.005)
{

};

void TactileArraySensor::ClassState::reset(const rw::kinematics::State& state){

}

TactileArraySensor::ClassState::Ptr TactileArraySensor::getClassState(rw::kinematics::State& state) const {
	if( _sdata.getStateCache<ClassState>(state) == NULL)
		return NULL;
	return _sdata.getStateCache<ClassState>(state);
}

TactileArraySensor::ClassState::Ptr TactileArraySensor::getClassState(rw::kinematics::State& state) {
	if( _sdata.getStateCache<ClassState>(state) == NULL)
		_sdata.getStateData()->setCache( rw::common::ownedPtr( new ClassState(this, getWidth(), getHeight())) , state);
	return _sdata.getStateCache<ClassState>(state);
}

namespace {

    double getValueOfTexel(double tx, double ty, // texel position data
						   double tw, double th, // texel width
                           double cx, double cy,
                           Eigen::MatrixXf& distM, double mwidth, double mheight)
    {
        // all values in distM that is inside the bounds of the texel need to be summed
        float texelVal = 0;

        // the distM matrix is centered around the coordinate (cx,cy)
        //double mwidth = tw*distM.size1();
        //double mheight = th*distM.size2();
        // transforming texel coordinates to
        double x = mwidth/2+(tx-cx);
        double y = mheight/2+(ty-cy);
        //std::cout << mwidth/2 << "+("<<tx<<"-"<<cx<<")" << std::endl;
        //std::cout << mheight/2 << "+("<<ty<<"-"<<cy<<")" << std::endl;
        //std::cout << "TexCoords: " << x << " , "  << y << std::endl;

        // now calculate the start and end index
        // the point must lie inside the texel so we round up
        int xStartIdx = (int)ceil( x/mwidth*distM.rows() );
        int yStartIdx = (int)ceil( y/mheight*distM.cols() );

        int xEndIdx = (int)ceil( (x+tw)/mwidth*distM.rows() );
        int yEndIdx = (int)ceil( (y+tw)/mheight*distM.cols() );

        // now make sure we don't exeed the matrix bounds
        xStartIdx = std::max(xStartIdx,0);
        yStartIdx = std::max(yStartIdx,0);
        xEndIdx = std::min(xEndIdx,(int)distM.rows());
        yEndIdx = std::min(yEndIdx,(int)distM.cols());


        for(int i=xStartIdx; i<xEndIdx; i++){
            for(int j=yStartIdx; j<yEndIdx; j++){
                texelVal += distM(i,j);
            }
        }
        return texelVal;
    }
    using namespace boost;
    double determinePenetration(double c, double initPen, std::vector<TactileArraySensor::DistPoint>& points, double force){
        // this should be full filled
        // force == SUM[ points[i] ]*c

        //std::cout << "init penetration: " << initPen << std::endl;
        double penetration = initPen;
        for(int i=0;i<10;i++){
            double pensum = 0;
            int nrOfPoints = 0;
            BOOST_FOREACH(TactileArraySensor::DistPoint& dp, points){
                if(dp.dist<0)
                    continue;
                pensum += dp.dist;
                nrOfPoints++;
            }
            //std::cout << "NrOfPoints: " << nrOfPoints << "\n";
            //std::cout << "pensum: " << pensum << "\n";

            double nforce = pensum*c;
            double diff = force-nforce;
            double diffStep = Math::clamp(diff/(nrOfPoints*2), -initPen/10, initPen/10);
            penetration += diffStep;
            //std::cout << "DIFF: " << diff << " " << diffStep << std::endl;
            BOOST_FOREACH(TactileArraySensor::DistPoint& dp, points){
                dp.dist = dp.dist + diffStep;
                //std::cout << "Dist: " << dp.dist << std::endl;
            }
        }
        return penetration;
    }

}


void TactileArraySensor::ClassState::acquire(){

}

Eigen::MatrixXf TactileArraySensor::ClassState::getTexelData()  const {
    return _pressure;
}

void TactileArraySensor::ClassState::addForceW(const Vector3D<>& point,
                                    const Vector3D<>& force,
                                    const Vector3D<>& snormal,
                                    dynamics::Body::Ptr body)
{
    addForce(_fTw*point, _fTw.R()*force, _fTw.R()*snormal, body);
}


void TactileArraySensor::ClassState::addForce(const Vector3D<>& point,
                                   const Vector3D<>& force,
                                   const Vector3D<>& snormal,
                                   dynamics::Body::Ptr body)
{
    //std::cout << "ADDING FORCE.... " << snormal << force << std::endl;
    //std::cout << "ADDING FORCE dot.... " << dot(snormal,force) << std::endl;
    //if( dot(force,snormal)<0 ){
    //    return;
    //}
    //std::cout << "1";
    Contact3D c3d(_wTf*point,_wTf.R()*snormal,_wTf.R()*force);
    _allAccForces.push_back( c3d );

    //std::cout << "3";
    // remember to test if the force direction is in the negative z-direction of each texel
    Vector3D<> f = _tsensor->_hmapTf.R() * force;


    // test if point lie in (+x,+y,+z) relative to the sensor base transform
    // though we give a little slack to allow forces on the boundary in
    Vector3D<> p = _tsensor->_hmapTf * point;
    if( /*p(0)<-_texelSize(0) || p(1)<-_texelSize(1) ||*/ p(2)<-0.002){
        //std::cout << "1pos wrong: " << p << std::endl;
        return;
    }

    //std::cout << "2";
    // and also if it lie inside the width and height of the grid
    //if(p(0)>(_w+1)*_texelSize(0) || p(1)>(_h+1)*_texelSize(1)){
        ////std::cout << "1pos wrong: " << p << std::endl;
    //    return;
    //}



    // oki so we know that the force acts on the grid somewhere
    // now locate all at most 15 texels points within
    //Vector2D<> texelSize = _tsensor->getTexelSize(0,0);
    int xIdx = (int)( floor(p(0)/_tsensor->_texelSize(0)));
    int yIdx = (int)( floor(p(1)/_tsensor->_texelSize(1)));

    // if the index is boundary then pick the closest texel normal
    int xIdxTmp = (int)Math::clamp(xIdx, 0, (int)_tsensor->getWidth()-1);
    int yIdxTmp = (int)Math::clamp(yIdx, 0, (int)_tsensor->getHeight()-1);

    //std::cout << "4";
    Vector3D<> normal = _tsensor->getNormals()[xIdxTmp][yIdxTmp];
    if( dot(f, normal)>=0 ){
        //std::cout << "SAME DIRECTION" << std::endl;
        return;
    }

    //std::cout << "5";
    // so heres the point force...
    //double forceVal = fabs( dot(f,normal) );
    //double scaleSum = 0;


    // we save all forces until the update is called
    //,_wTf.R()*snormal,_wTf.R()*force
    Contact3D con(p, normal, f);
    _forces[body].push_back(con);
}

// contact normal in a's coordinates. describe the contact normal from a to b

std::vector<TactileArraySensor::DistPoint> TactileArraySensor::ClassState::generateContacts(dynamics::Body *body, const Vector3D<>& cnormal, const State& state){
    Frame *tframe = _tsensor->getSensorFrame();
    Frame *bframe = body->getBodyFrame();
    RW_ASSERT(tframe);
    RW_ASSERT(bframe);

    Transform3D<> wTa = Kinematics::worldTframe(tframe, state);
    Transform3D<> wTb = Kinematics::worldTframe(bframe, state);
    //std::cout << "getting models" << std::endl;
    //std::cout << "Frame name: " << tframe->getName();
    ProximityModel::Ptr modelA = _tsensor->_narrowStrategy->getModel(tframe);
    //std::cout << "getting models" << std::endl;
    ProximityModel::Ptr modelB = _tsensor->_narrowStrategy->getModel(bframe);

    //double stepSize = _maxPenetration;
    Vector3D<> step = _tsensor->_maxPenetration * (wTa.R()*cnormal);
    //std::cout << "collides" << std::endl;
    // we first need to make sure that the boddies are not penetrating
    // so we move the bodies in the opposite direction of the contact normal
    // until they are not colliding
    bool colliding = _tsensor->_narrowStrategy->inCollision(modelA, wTa , modelB, wTb, _pdata);
    int loopcount =0;
    if( colliding ){
        while(colliding){
            wTb.P() += step;
            colliding = _tsensor->_narrowStrategy->inCollision(modelA, wTa, modelB, wTb, _pdata);
            //std::cout << "Step: " << wTa.P() << " " << wTb.P() << std::endl;
            loopcount++;
            if(loopcount>20){
            	RW_WARN("PENETRATING: Body contact normal is incorrect!!!");
            	return std::vector<TactileArraySensor::DistPoint>();
            }
            RW_ASSERT(loopcount<100);
        }
    }
    // if they where not colliding then we make sure that they at least is
    // very close
    else {
        while(!colliding){
            wTb.P() -= step;
            colliding = _tsensor->_narrowStrategy->inCollision(modelA, wTa, modelB, wTb, _pdata);
            loopcount++;
            if(loopcount>10){
            	RW_WARN("Body contact normal is incorrect!!!");
            	return std::vector<TactileArraySensor::DistPoint>();
            }
            if(loopcount>90){
            	//std::cout << "step: " << step << std::endl;
                //std::cout << "Step: " << wTa.P() << " " << wTb.P() << std::endl;
            }
            RW_ASSERT(loopcount<100);
        }
        wTb.P() += step;
    }

    // okay, so now we have the bodies that is within one mm of
    // each other. Assuming the colliding body is rigid and that
    // we know the maximum possible penetration (because of elasticity)
    // we find the worst case area that is in contact by looking for contacts
    // within a distance of 2 mm
    //std::cout << "calculating distances" << std::endl;
    _tsensor->_narrowStrategy->distances(modelA,wTa,modelB,wTb,_tsensor->_maxPenetration*2,_pdata);
    MultiDistanceResult &result = _pdata.getMultiDistanceData();

    //std::cout << " distance result: " << result.distances.size() << std::endl;
    // the shortest distance between the models
    double sdistance = result.distance;
    //std::cout << "sdistance: " << sdistance << std::endl;
    // now all contacts that is further away than sdistance+0.001 cannot
    // be in collision so we remove these.
    std::vector<DistPoint> validResult;
    for(size_t i=0;i<result.distances.size();i++){
        //std::cout << "d: " << result.distances[i] << std::endl;
        if(result.distances[i] < sdistance+_tsensor->_maxPenetration){
            DistPoint dp;
            dp.p1 = result.p1s[i];
            dp.p2 = result.p2s[i];
            dp.dist = result.distances[i]-sdistance;
            validResult.push_back(dp);
        }
    }
    //std::cout << "ValidResults: " << validResult.size() << std::endl;
    return validResult;
}

namespace {

/*	getContactPoints(){

	}
	*/


}


void TactileArraySensor::ClassState::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
	// make sure not to sample more often than absolutely necessary
	_accTime+=info.dt;
	if(_accTime<_stime){
	    _wTf = Kinematics::worldTframe( _tsensor->getSensorFrame(), state);
	    _fTw = inverse(_wTf);
	    _forces.clear();
	    _allAccForces.clear();
		return;
	}
	double rdt = _accTime;
	_accTime -= _stime;

	//std::cout << "update!" << std::endl;
    // we have collected all forces that affect the sensor.
    // Now we need to extrapolate the area of the force since the
    // surface is elastic. We do this by checking for collision between
	// the touching object and the normal trimesh

	bool hasCollision = false;
	double totalNormalForce = 0;
    typedef std::map<dynamics::Body::Ptr, std::vector<Contact3D> > BodyForceMap;
    BodyForceMap::iterator iter = _forces.begin();
    for(;iter!=_forces.end();++iter){
    	//std::cout << "BODY" << std::endl;
    	Body::Ptr body = (*iter).first;


		Frame *tframe = _tsensor->getSensorFrame();
		Frame *bframe = body->getBodyFrame();
		RW_ASSERT(tframe);
		RW_ASSERT(bframe);

		Transform3D<> wTa = Kinematics::worldTframe(tframe, state)*_tsensor->_fThmap;
		Transform3D<> wTb = Kinematics::worldTframe(bframe, state);
		//ProximityModelPtr modelA = _narrowStrategy->getModel(tframe);
		ProximityModel::Ptr modelB = _tsensor->_narrowStrategy->getModel(bframe);

		bool collides = _tsensor->_narrowStrategy->inCollision(_tsensor->_nmodel, wTa, modelB, wTb, _pdata); //wTa*_fThmap
		CollisionResult &data = _pdata.getCollisionData();
		if( !collides )
			continue;
		if( !hasCollision ){
			// initialize variables
			_distMatrix =  ValueMatrix::Constant(_distMatrix.rows(),_distMatrix.cols(),100);
		}
		hasCollision = true;
		//std::cout << "Yes it really collides!" << bframe->getName() << std::endl;

		if(_tsensor->_frameToGeoms.find(bframe)==_tsensor->_frameToGeoms.end())
		    _tsensor->_frameToGeoms[bframe] = body->getGeometry();

		std::vector<Geometry::Ptr> &geoms = _tsensor->_frameToGeoms[bframe];
		// now we try to get the contact information
		if(data._collisionPairs.size()>0){
			int bodyGeomId = data._collisionPairs[0].geoIdxB;
			//std::cout << "BodyGeomId: " << std::endl;
			TriMesh *mesh = dynamic_cast<TriMesh*> (geoms[bodyGeomId]->getGeometryData().get());
			if(mesh){
	            int startIdx = data._collisionPairs[0].startIdx;
	            int size = data._collisionPairs[0].size;

				for(int i = startIdx; i<startIdx+size;i++){
					std::pair<int,int> &pids = data._geomPrimIds[i];
					//std::cout << "Colliding pairs: " << pids.first << " <---> " << pids.second << std::endl;
					RW_ASSERT(0<=pids.first);
					RW_ASSERT(pids.first<(int)_tsensor->_ntrimesh->getSize());

					// for each colliding pair we find the closest intersection
					// get the triangle
					Triangle<> tri = mesh->getTriangle(pids.second);
					Triangle<> triA = _tsensor->_ntrimesh->getTriangle(pids.first);

					Vector3D<> point;
					if( !IntersectUtil::intersetPtRayPlane(triA[0], triA[2], _pdata.aTb()*tri[0], _pdata.aTb()*tri[1], _pdata.aTb()*tri[2], point) )
						continue;

					// now we have the point of intersection, now save it in the contact array
					// if its closer than the existing point

					double ndist = MetricUtil::dist2( triA[0], point);
					//std::cout << "Intersect: " << point << std::endl;
					//std::cout << "NDist: " << ndist << std::endl;
					if( ndist > (&_distMatrix(0,0))[pids.first] )
						continue;
					(&_distMatrix(0,0))[pids.first] = (float)ndist;
					//(&_contactMatrix[0][0])[pids.first] = point;
				}
			}
	    	std::vector<rw::sensor::Contact3D> &bforce = (*iter).second;
	    	// add to total force
	    	BOOST_FOREACH(rw::sensor::Contact3D& c, bforce){
	    		//std::cout << "TotalNormalForce += " << (-c.normalForce) << std::endl;
	    		totalNormalForce += -c.normalForce;
	    	}

		}
    }
    _forces.clear();

    double closest = 100;
    if( hasCollision ){
		for(Eigen::DenseIndex y=0; y<_distMatrix.cols(); y++){
			for(Eigen::DenseIndex x=0; x<_distMatrix.rows(); x++){
				_distMatrix(x,y) = _distMatrix(x,y) - _tsensor->_distDefMatrix(x,y);
				if(_distMatrix(x,y)<closest)
					closest = _distMatrix(x,y);
			}
		}
		//std::cout << "\n\n" << _distMatrix << "\n\n"<< std::endl;
    }
    // now we need to convert the _distMatrix to pressure values
    ValueMatrix pressure =  Eigen::MatrixXf::Zero(_pressure.rows(),_pressure.cols());
    //
    // we only have the total normal force, we expect the position between finger and
    // object to be error prone so we iteratively determine the area of contact such
    // that area*Ftotal = deformed_volume * defToStress
    //
    if( hasCollision ){
		double offset = 0;
		double bestOffset = 0;
		double bestScore = 100000;
		for(int i=0;i<20;i++){
			//std::cout << i << " offset: " << offset << std::endl;
			// 1. we aallready have total force
			// 2. calculate area and "volume"
			double area = 0;
			double volume = 0;
			double totalVolume = 0;
			for(Eigen::DenseIndex y=0; y<_distMatrix.cols(); y++){
				for(Eigen::DenseIndex x=0; x<_distMatrix.rows(); x++){
					//std::cout << _distMatrix(x,y) << "\n";
					double val = _distMatrix(x,y)-closest+offset;
					if( val<0 ){
						area += _tsensor->_texelArea;
						volume += -val*_tsensor->_texelArea;
						totalVolume += 0.002*_tsensor->_texelArea;
					}
				}
			}
			//std::cout << "area: " << area << std::endl;
			if(area>0){
				// (area*Ftotal) < (deformed_volume * defToStress)
				// increase offset
				// and if
				// (area*Ftotal) < (deformed_volume * defToStress)
				//   <   GPa * unitless = stress

				double score = totalNormalForce/(area*1000) - (volume * _tsensor->_elasticity)/totalVolume;

				//std::cout << "Score: " << score << std::endl;
				if(fabs(score)>bestScore)
					continue;
				bestScore = fabs(score);
				bestOffset = offset;

				//std::cout << offset << ":" << score << " = " << totalNormalForce/(area*1000)
				//			<< "kPa -" << ((volume*_elasticity)/totalVolume) << "kPa" << std::endl;
				//std::cout << offset << ":" << score << " = " << totalNormalForce<< "/" << (area*1000)
				//		<< "-" << "(" << volume << "*" <<_elasticity << ")"
				//		<< "/" << totalVolume << std::endl;

			}
			offset -= 0.001/20;
		}
		double totalDist = 0;
		for(Eigen::DenseIndex y=0; y<_distMatrix.cols(); y++){
			for(Eigen::DenseIndex x=0; x<_distMatrix.rows(); x++){
				double dist = _distMatrix(x,y)-closest+bestOffset;
				if( dist>0 )
					continue;
				totalDist +=  dist;
			}
		}
		//std::cout << "TOTAL DIST: " << totalDist << std::endl;
		double distToForce = totalNormalForce/totalDist;
		if(distToForce>10000){
			std::cout << "Dist force: " << distToForce << std::endl;
			std::cout << "totalNormalForce: " << totalNormalForce << std::endl;
			std::cout << "totalDist: " << totalDist << std::endl;

		} else {
			//std::cout << "BestScore: " << bestScore << std::endl;
			// now we now the actual penetration/position we can apply point force function
			// to calculate the pressure on the sensor surface
			for(Eigen::DenseIndex y=0; y<_distMatrix.cols(); y++){
				for(Eigen::DenseIndex x=0; x<_distMatrix.rows(); x++){

					//std::cout << _distMatrix(x,y) << "\n";
					double dist = _distMatrix(x,y)-closest+bestOffset;
					if( dist>0 )
						continue;

					double force = dist*distToForce;

					//Vector3D<> p = _hmapTf * dp.p1;
					Vector3D<> p = _tsensor->getCenters()[x][y];//_contactMatrix[x][y];


					//int xIdx = (int)( floor(p(0)/_texelSize(0)));
					//int yIdx = (int)( floor(p(1)/_texelSize(1)));
					int xIdx = (int)x;
					int yIdx = (int)y;
					for(int xi=std::max(0,xIdx-1); xi<std::min(xIdx+2,_tsensor->getWidth()); xi++){
						for(int yi=std::max(0,yIdx-1); yi<std::min(yIdx+2,_tsensor->getHeight()); yi++){
							double texelScale = getValueOfTexel(xi*_tsensor->_texelSize(0),yi*_tsensor->_texelSize(1),
							                                    _tsensor->_texelSize(0),_tsensor->_texelSize(1),
																p(0),p(1),
																_tsensor->_dmask, _tsensor->_maskWidth, _tsensor->_maskHeight);
							RW_ASSERT(texelScale<100000);
							_accForces(xi,yi) += (float)(force*texelScale);
							//std::cout << "accForce +="<< force << "*" << texelScale << std::endl;
						}
					}
				}
			}
		}

	    // copy and convert accumulated forces into pressure values
	    //double texelArea = _texelSize(0)*_texelSize(1);
		double totalPressure = 0, totalArea = 0;
	    for(Eigen::DenseIndex x=0; x<_accForces.rows(); x++){
	        for(Eigen::DenseIndex y=0; y<_accForces.cols(); y++){
	            // clamp to max pressure
	            //if( _accForces(x,y)/texelArea>1.0 )
	            //    std::cout << "Pressure: ("<<x<<","<<y<<") " << _accForces(x,y)/texelArea << " N/m^2 " << std::endl;
	        	if(_accForces(x,y)<=0){
	        		continue;
	        	}

	            pressure(x,y) = (float)std::min( _accForces(x,y)/(_tsensor->_texelArea*1000), _tsensor->getPressureLimit().second );
	            totalArea += _tsensor->_texelArea;
	            totalPressure += pressure(x,y);
	        }
	    }
	    //std::cout << "totalPressure: "
	    //		<< totalPressure/(totalNormalForce/(totalArea*1000)) << " --> "
	    //		<< totalPressure << "kPa == "
	    //		<< (totalNormalForce/(totalArea*1000))
	    //		<< "kPa == "<< totalNormalForce << "/" << totalArea << std::endl;

	    // the actual pressure should not be more than totForce/(totalArea*1000)
	    // so we scale it down to fit-....
	    double presScale = (totalNormalForce/(totalArea*1000))/totalPressure;
	    double ntotpressure = 0;
	    for(Eigen::DenseIndex x=0; x<pressure.rows(); x++){
	        for(Eigen::DenseIndex y=0; y<pressure.cols(); y++){
	        	if(pressure(x,y)>0){
	        		// and convert it to pascal
	        		pressure(x,y) *= (float)(presScale*1000);
	        		ntotpressure += pressure(x,y)/1000;
	        	}
	        }
	    }

	    //std::cout << "New total pressure: " << ntotpressure << std::endl;

	    _accForces = ValueMatrix::Zero(_accForces.rows(), _accForces.cols());
	    _allForces = _allAccForces;
	    _allAccForces.clear();
    }


    // Low pass filtering is done in the end
    // for i from 1 to n
    //    y[i] := y[i-1] + alpha * (x[i] - y[i-1])
    ValueMatrix &ym = _pressure;
    ValueMatrix &xm = pressure;
    const double alpha = rdt/(_tsensor->_tau+rdt);
    for(Eigen::DenseIndex y=0; y<ym.rows(); y++){
    	for(Eigen::DenseIndex x=0; x<ym.cols(); x++){
        	ym(x,y) += (float)(alpha * (xm(x,y)-ym(x,y)));
        }
    }

    _pressure = pressure;
    //std::cout << _pressure << std::endl;

    // update aux variables
    _wTf = Kinematics::worldTframe( _tsensor->getSensorFrame(), state);
    _fTw = inverse(_wTf);
}

void TactileArraySensor::setDeformationMask(const ValueMatrix& dmask, double width, double height){
	_dmask = dmask;
	_maskWidth = width;
	_maskHeight = height;
}

void TactileArraySensor::reset(const rw::kinematics::State& state){
	_sdata.getStateCache<ClassState>(state)->reset(state);
}

void TactileArraySensor::addForceW(const rw::math::Vector3D<>& point,
               const rw::math::Vector3D<>& force,
               const rw::math::Vector3D<>& snormal,
               rw::kinematics::State& state,
               dynamics::Body::Ptr body)
{
	_sdata.getStateCache<ClassState>(state)->addForceW(point,force,snormal,body);
}

void TactileArraySensor::addForce(const rw::math::Vector3D<>& point,
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& snormal,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{
	_sdata.getStateCache<ClassState>(state)->addForce(point,force,snormal,body);
}

void TactileArraySensor::addWrenchToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{

}

void TactileArraySensor::addWrenchWToCOM(
              const rw::math::Vector3D<>& force,
              const rw::math::Vector3D<>& torque,
              rw::kinematics::State& state,
              dynamics::Body::Ptr body)
{

}

//! @copydoc rwlibs::simulation::SimulatedSensor::update
void TactileArraySensor::update(const rwlibs::simulation::Simulator::UpdateInfo& info, rw::kinematics::State& state){
	_sdata.getStateCache<ClassState>(state)->update(info,state);
}



#ifdef OLD_STUFF
void TactileArraySensor::update(double dt, rw::kinematics::State& state){
    // we have collected all forces that affect the sensor.
    // Now we need to extrapolate the area of the force since the
    // surface is elastic. We do this by finding all point to point distances
    // between the two bodies.
    //std::cout << "Update tactile sensor array" << std::endl;
	ValueMatrix lastpres = _pressure;
    for(size_t x=0; x<_pressure.size1(); x++){
        for(size_t y=0; y<_pressure.size2(); y++){
        	_pressure(x,y) = lastpres(x,y)*0.8;
        }
    }

    // We first calulate the common contact normal
    Vector3D<> cnormal(0,0,0);
    double totalForce = 0;
    dynamics::Body* body = NULL;
    typedef std::map<dynamics::Body*, std::vector<Contact3D> > BodyForceMap;
    BodyForceMap::iterator iter = _forces.begin();
    for(;iter!=_forces.end();++iter){
        //std::cout << "ITER" << std::endl;
        BOOST_FOREACH(const Contact3D& c, (*iter).second){
            //std::cout << "N: " << c.n << "\n";
            //std::cout << "F: " << c.f << "\n";
            if(MetricUtil::norm2(c.f)<0.0000001)
            	continue;
            cnormal += normalize(c.n);
            //cnormal += normalize(c.f);
            totalForce += MetricUtil::norm2(c.f);
        }
        body = (*iter).first;
    }
    //std::cout << "TOTAL FORCE: " << totalForce << "N" << std::endl;

    //std::cout << "Clear forces" << std::endl;
    _forces.clear();
    if(body!=NULL && totalForce>0.0000001 ){
        cnormal = normalize(cnormal);
        //std::cout << "Generate contacts: " << cnormal << std::endl;
        RW_ASSERT(MetricUtil::norm2(cnormal)>0.1);
        std::vector<DistPoint> res = generateContacts(body, -cnormal, state);

        boost::numeric::ublas::matrix<DistPoint> contacts(_accForces.size1(),_accForces.size2());
        double weightedArea = 0;
        // now filter away the distances that are double in cells
        BOOST_FOREACH(DistPoint& dp, res){
            Vector3D<> p = _hmapTf * dp.p1;

            int xIdx = (int)( floor(p(0)/_texelSize(0)));
            int yIdx = (int)( floor(p(1)/_texelSize(1)));

            if( xIdx<0 || yIdx<0 ){
                //std::cout << xIdx << "<" << 0 << " && " << yIdx << "<" << 0<< std::endl;
            }
            if( !(xIdx<contacts.size1() && yIdx<contacts.size2()) ){
                //std::cout << xIdx << "<" << contacts.size1() << " && " << yIdx << "<" << contacts.size2()<< std::endl;
            }

            if( xIdx<0 || yIdx<0 )
                continue;
            if( contacts.size1()<=xIdx || contacts.size2()<=yIdx )
                continue;
            RW_ASSERT(0<=xIdx && 0<=yIdx);
            RW_ASSERT(xIdx<contacts.size1() && yIdx<contacts.size2());
        	if( contacts(xIdx,yIdx).dist > dp.dist ){
        	    //std::cout << "ADD DIST!" << dp.dist << std::endl;
        		contacts(xIdx,yIdx) = dp;
        	}
        }
        double totalDist = 0;
        int nrOfActiveTexels = 0;
        res.clear();
        for(int x=0;x<contacts.size1();x++){
            for(int y=0;y<contacts.size2();y++){
            	if(contacts(x,y).dist>1){
            	    // check if any of the neighbors are high
            	    if( 0<x && x<contacts.size1()-1 && contacts(x-1,y).dist<1 && contacts(x+1,y).dist<1){
            	        contacts(x,y).p1 = (contacts(x+1,y).p1-contacts(x-1,y).p1)/2+contacts(x-1,y).p1;
                        contacts(x,y).dist = (contacts(x+1,y).dist-contacts(x-1,y).dist)/2+contacts(x-1,y).dist;
            	    } else if(0<y && y<contacts.size2()-1 && contacts(x,y-1).dist<1 && contacts(x,y+1).dist<1){
            	        contacts(x,y).p1 = (contacts(x,y+1).p1-contacts(x,y-1).p1)/2+contacts(x,y-1).p1;
                        contacts(x,y).dist = (contacts(x,y+1).dist-contacts(x,y-1).dist)/2+contacts(x,y-1).dist;
                    } else if(0<x && x<contacts.size1()-1 && 0<y && y<contacts.size2()-1){
                        if( contacts(x-1,y-1).dist<1 &&  contacts(x+1,y+1).dist<1 ){
                            contacts(x,y).p1 = (contacts(x-1,y-1).p1-contacts(x+1,y+1).p1)/2+contacts(x+1,y+1).p1;
                            contacts(x,y).dist = (contacts(x-1,y-1).dist-contacts(x+1,y+1).dist)/2+contacts(x+1,y+1).dist;
                        } else if (contacts(x-1,y+1).dist<1 &&  contacts(x+1,y-1).dist<1){
                            contacts(x,y).p1 = (contacts(x-1,y+1).p1-contacts(x+1,y-1).p1)/2+contacts(x+1,y-1).p1;
                            contacts(x,y).dist = (contacts(x-1,y+1).dist-contacts(x+1,y-1).dist)/2+contacts(x+1,y-1).dist;
                        } else {
                            continue;
                        }
                    } else {
            	        continue;
            	    }

            	}

                /*if(contacts(x,y).dist>1){
                    continue;
                }*/
                //std::cout << "ADD CONTACT: " <<  x << " " << y << " " << contacts(x,y).dist << std::endl;

                contacts(x,y).dist = _maxPenetration-contacts(x,y).dist;
                res.push_back(contacts(x,y));
            	totalDist += contacts(x,y).dist;
            	nrOfActiveTexels++;
            	weightedArea += (contacts(x,y).dist*1.0/_maxPenetration);
            }
        }
/*
        BOOST_FOREACH(DistPoint& dp, res){
            totalDist += 0.001-dp.dist;
        }*/
        //std::cout << "TOTAL DIST: " << totalDist << std::endl;
        //RW_ASSERT(totalDist==0);
        double maxArea = nrOfActiveTexels*_texelSize(0)*_texelSize(1);
        //std::cout << "Worst case area: " << maxArea << std::endl;
        //std::cout << "Estimated area : " << (weightedArea*_texelSize(0)*_texelSize(1)) << std::endl;

        // area should be used to determine the actual penetration depth
        // area*force == pressure | pressure*k = penetration

        // realPenetration = (realarea*force)/k

        // realArea = perhaps using parabol fitted to points.

        double penetration = determinePenetration(_elasticity, _maxPenetration, res, totalForce);
        totalDist = 0;
        BOOST_FOREACH(DistPoint& dp, res){
            double dist = dp.dist;
            if(dist<=0)
                continue;
            totalDist += dist;
        }
        //double penetration = _maxPenetration*weightedArea/maxArea;

        if( totalDist>0 ){
            double scale = totalForce/totalDist;
            //std::cout << "scale: " << scale << std::endl;
            BOOST_FOREACH(DistPoint& dp, res){
                double dist = dp.dist;
                if(dist<=0)
                    continue;
                //if(dist>_maxPenetration)
                //    dist = _maxPenetration;

                double force = dist*scale;

                Vector3D<> p = _hmapTf * dp.p1;

                int xIdx = (int)( floor(p(0)/_texelSize(0)));
                int yIdx = (int)( floor(p(1)/_texelSize(1)));

                //std::cout << xIdx << " " << yIdx << " " << force <<  std::endl;

                for(int x=std::max(0,xIdx-1); x<std::min(xIdx+2,_w); x++){
                    for(int y=std::max(0,yIdx-1); y<std::min(yIdx+2,_h); y++){
                        double texelScale = getValueOfTexel(x*_texelSize(0),y*_texelSize(1),
                                                            _texelSize(0),_texelSize(1),
                                                            p(0),p(1),
                                                            _dmask, _maskWidth, _maskHeight);
                        _accForces(x,y) += force*texelScale;
                        //std::cout << force << "*" << texelScale << std::endl;
                    }
                }
            }


            // copy and convert accumulated forces into pressure values
            double texelArea = _texelSize(0)*_texelSize(1);
            for(size_t x=0; x<_accForces.size1(); x++){
                for(size_t y=0; y<_accForces.size2(); y++){
                    // clamp to max pressure
                    //if( _accForces(x,y)/texelArea>1.0 )
                    //    std::cout << "Pressure: ("<<x<<","<<y<<") " << _accForces(x,y)/texelArea << " N/m^2 " << std::endl;
                    _pressure(x,y) = std::min( _accForces(x,y)/texelArea, _maxPressure );
                }
            }
            //std::cout << "_pressure" << std::endl;
            //std::cout << _pressure << std::endl;

            _accForces = ublas::zero_matrix<double>(_accForces.size1(),_accForces.size2());
            _allForces = _allAccForces;
            _allAccForces.clear();
        }
    }

    // update aux variables
    _wTf = Kinematics::worldTframe( getFrame(), state);
    _fTw = inverse(_wTf);
}
#endif

void TactileArraySensor::ClassState::setTexelData(const Eigen::MatrixXf& data){
	if(data.rows()!=_pressure.rows()){
		RW_WARN("dimension mismatch: " << data.rows() <<"!=" <<_pressure.rows());
		return;
	}
	if(data.cols()!=_pressure.cols()){
		RW_WARN("dimension mismatch: " << data.cols() <<"!=" <<_pressure.cols());
		return;
	}

	_pressure = data;
};



#ifdef OLD_ADDFORCE

void TactileArraySensor::addForce(const Vector3D<>& point,
                                   const Vector3D<>& force,
                                   const Vector3D<>& snormal,
                                   dynamics::Body *body)
{
    if( dot(force,snormal)<0 ){
        return;
    }

    Contact3D c3d(_wTf*point,_wTf.R()*force,_wTf.R()*snormal);
    _allAccForces.push_back( c3d );


    // test if point lie in (+x,+y,+z) relative to the sensor base transform
    // though we give a little slack to allow forces on the boundary in
    Vector3D<> p = _hmapTf * point;
    if(p(0)<-_texelSize(0) || p(1)<-_texelSize(1) || p(2)<-0.002){
        //std::cout << "1pos wrong: " << p << std::endl;
        return;
    }

    // and also if it lie inside the width and height of the grid
    if(p(0)>(_w+1)*_texelSize(0) || p(1)>(_h+1)*_texelSize(1)){
        //std::cout << "1pos wrong: " << p << std::endl;
        return;
    }

    // remember to test if the force direction is in the negative z-direction of each texel
    Vector3D<> f = _hmapTf.R() * force;

    // oki so we know that the force acts on the grid somewhere
    // now locate all at most 15 texels points within
    int xIdx = (int)( floor(p(0)/_texelSize(0)));
    int yIdx = (int)( floor(p(1)/_texelSize(1)));

    // if the index is boundary then pick the closest texel normal
    int xIdxTmp = std::min( std::max(xIdx,0), _w-1);
    int yIdxTmp = std::min( std::max(yIdx,0), _h-1);

    Vector3D<> normal = _normalMatrix[xIdxTmp][yIdxTmp];
    if( dot(f, normal)>0 ){
        //std::cout << "SAME DIRECTION" << std::endl;
        return;
    }

    // so heres the point force...
    double forceVal = fabs( dot(f,normal) );
    double scaleSum = 0;
    //std::cout << "ADDING FORCE to: (" << p(0) << "," << p(1)<< ")" << std::endl;
    // and now to distribute it on the different texels
    //int x=std::min(std::max(0,xIdx),_w-1);
    //int y=std::min(std::max(0,yIdx),_h-1);
    //_accForces(x,y) = 10000;//forceVal*texelScale;

    for(int x=std::max(0,xIdx-1); x<std::min(xIdx+2,_w); x++){
        for(int y=std::max(0,yIdx-1); y<std::min(yIdx+2,_h); y++){
            double texelScale = getValueOfTexel(x*_texelSize(0),y*_texelSize(1),
                                                _texelSize(0),_texelSize(1),
                                                p(0),p(1),
                                                _dmask, _maskWidth, _maskHeight);
            _accForces(x,y) += forceVal*texelScale;
            scaleSum += texelScale;

        }
    }

    if( scaleSum>1.01 )
        std::cout << "SCALE SUM: " << scaleSum << std::endl;

}

void TactileArraySensor::update(double dt, rw::kinematics::State& state){
    // all forces accumulated

    // copy and convert accumulated forces into pressure values
    double texelArea = _texelSize(0)*_texelSize(1);
    for(size_t x=0; x<_accForces.size1(); x++){
        for(size_t y=0; y<_accForces.size2(); y++){
            // clamp to max pressure
            //if( _accForces(x,y)/texelArea>1.0 )
            //    std::cout << "Pressure: ("<<x<<","<<y<<") " << _accForces(x,y)/texelArea << " N/m^2 " << std::endl;
            _pressure(x,y) = std::min( _accForces(x,y)/texelArea, _maxPressure );
        }
    }
    //std::cout << _pressure << std::endl;
    _accForces = ublas::zero_matrix<double>(_accForces.size1(),_accForces.size2());

    _allForces = _allAccForces;
    _allAccForces.clear();

    // update aux variables
    _wTf = Kinematics::worldTframe( getFrame(), state);
    _fTw = inverse(_wTf);

    //std::cout << _pressure << std::endl;
}
#endif
