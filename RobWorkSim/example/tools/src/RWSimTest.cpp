
//#include "OBB.hpp"
#include <stdio.h>
// ifstream::is_open
#include <fstream>
#include <rwsim/dynamics/OBRManifold.hpp>
#include <vector>

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwsim/dynamics/ContactPoint.hpp>
#include <rwsim/dynamics/ContactCluster.hpp>

#include <boost/foreach.hpp>

using namespace rw::math;
using namespace rwsim::dynamics;


ContactPoint makeContact(double x,double y,double z, double pen, Vector3D<> normal){
    normal = normalize(normal);
    ContactPoint point;
    Transform3D<> wTa(Transform3D<>::identity());
    Transform3D<> wTb(Transform3D<>::identity());
    point.p = Vector3D<>(x,y,z);
    point.penetration = pen;
    point.n = normal;
    point.pA = inverse(wTa) * (point.p - point.n*0.005);
    point.pB = inverse(wTb) * (point.p + point.n*0.005);
    return point;
}

ContactPoint makeContact(double x,double y,double z, double nx,double ny,double nz, double pen){
    return makeContact(x,y,z,pen,Vector3D<>(nx,ny,nz));
}

/**
 * @brief Contact manifold based on Oriented Bounding Boxes
 */
class OBBManifold {
public:
    /**
     * @brief
     * @param thres
     * @param sepThres
     * @return
     */
    OBBManifold(double thres = 0.03, double sepThres = 0.01):
        _nrOfContacts(0),
        _deepestIdx(0),
        _threshold(thres),
        _sepThreshold(sepThres)
    {};

    virtual ~OBBManifold(){};

    /**
     * @brief adds and updates the manifold with a new point
     * @param p
     */
    void addPoint(ContactPoint& p){
        if( !inManifold(p) )
            RW_THROW("Not in manifold");

        // handle if we have less than 5 points
        if(_nrOfContacts==5){
            // if the point is inside the obb then we only need to check if its
            // deeper than the deepest point and if it s replace it
            if( isInsideOBB(p.p) ){
                if(_points[_deepestIdx].penetration<p.penetration)
                    _points[_deepestIdx] = p;
                return;
            }
            // if its outside then we need to remove another point
            fit(p);
            // find the point closest to center of obb
            int minIdx = -1;
            double minDist = MetricUtil::dist1(p.p,_t3d.P());
            for(int i=0;i<5;i++){
                double dist = MetricUtil::dist1(_points[i].p,_t3d.P());
                if( dist<minDist && _deepestIdx!=i ){
                    minIdx = i;
                    minDist = dist;
                }
            }
            if(minIdx<0)
                return;
            _points[minIdx] = p;
            if( _points[_deepestIdx].penetration>p.penetration )
                _deepestIdx = minIdx;
        } else if(_nrOfContacts>1) {
            fit(p);
            _normal = p.n;
            _points[_nrOfContacts] = p;
            _nrOfContacts++;
            _deepestIdx = 0;
        } else if(_nrOfContacts==1) {
            _normal = (p.n + _points[0].n)/2;
            _points[_nrOfContacts] = p;
            _t3d.P() = (p.p + _points[0].p)/2;
            if( _points[_deepestIdx].penetration>p.penetration )
                _deepestIdx = _nrOfContacts;
            _nrOfContacts++;
        } else {
            _normal = p.n;
            _points[0] = p;
            _t3d.P() = p.p;
            _deepestIdx = 0;
            _nrOfContacts++;
        }
    }

    void update(const Transform3D<> &aT, const Transform3D<> &bT){
        // update the position of the contact pointshttp://www.cplusplus.com/reference/iostream/ifstream/
        for(int i=0; i<_nrOfContacts; i++){
            const Vector3D<>& wPa = aT * _points[i].pA;
            const Vector3D<>& wPb = bT * _points[i].pB;
            if( MetricUtil::dist2(wPa,wPb)> _sepThreshold ){
                //remove point
                _nrOfContacts--;
                _points[i] = _points[_nrOfContacts];
                i--;
            } else {
                _points[i].p = (wPa+wPb)/2;
            }
        }
    }

    int getNrOfContacts(){ return _nrOfContacts; };

    bool inManifold(ContactPoint& p){

        if( _nrOfContacts==0 ){
            return true;
        } else if( _nrOfContacts == 1){
            // check distance to the deepest point
            const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
            std::cout << "1: Dist too point: " << dist << std::endl;
            if( dist <_threshold ) return true;
            else return false;
        } else if( _nrOfContacts == 2 ){
            // check distance to the line
            const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
            std::cout << "1: Dist too point: " << dist << std::endl;
            if( dist <_threshold ) return true;
            else return false;
/*            const Vector3D<> &p1 = p.p;
            const Vector3D<> &x1 = _points[0].p;
            const Vector3D<> &x2 = _points[1].p;
            double dist = MetricUtil::norm2(cross((x2-x1),(x1-p1)))/
                          MetricUtil::norm2((x2-x1));
            if( dist <_threshold ) return true;
            else return false;*/
        } else {
            const double dist = MetricUtil::dist2(p.p,_points[_deepestIdx].p);
            if( dist <_threshold ) return true;
            // check if the point is inside the manifold box
            // project it onto the plane
            return isInsideOBB(p.p);
        }
    }

    /**
     * @brief fits a new manifold to the list of contact points
     * @param p
     */
    void fit(ContactPoint& p){
        // re-fit the bounding box
        Eigen::MatrixXd covar( Eigen::MatrixXd::Zero(3, 3) );
        Vector3D<> c = p.p+_points[0].p+_points[1].p;
        covar(0,0) += _points[0].p(0)*_points[0].p(0) +
                      _points[1].p(0)*_points[1].p(0) +
                      p.p(0)+p.p(0);
        covar(1,1) += _points[0].p(1)*_points[0].p(1) +
                      _points[1].p(1)*_points[1].p(1) +
                      p.p(1)+p.p(1);
        covar(2,2) += _points[0].p(2)*_points[0].p(2) +
                      _points[1].p(2)*_points[1].p(2) +
                      p.p(2)+p.p(2);
        covar(0,1) += _points[0].p(0)*_points[0].p(1) +
                      _points[1].p(0)*_points[1].p(1) +
                      p.p(0)+p.p(1);
        covar(0,2) += _points[0].p(0)*_points[0].p(2) +
                      _points[1].p(0)*_points[1].p(2) +
                      p.p(0)+p.p(2);
        covar(1,2) += _points[0].p(1)*_points[0].p(2) +
                      _points[1].p(1)*_points[1].p(2) +
                      p.p(1)+p.p(2);

        for(int i=2; i<_nrOfContacts;i++){
            const Vector3D<> &point = _points[i].p;
            c += point;
            covar(0,0) += point(0)*point(0);
            covar(1,1) += point(1)*point(1);
            covar(2,2) += point(2)*point(2);
            covar(0,1) += point(0)*point(1);
            covar(0,2) += point(0)*point(2);
            covar(1,2) += point(1)*point(2);
        }
        const int n = _nrOfContacts+1;
        covar(0,0) = covar(0,0)-c(0)*c(0)/n;
        covar(1,1) = covar(1,1)-c(1)*c(1)/n;
        covar(2,2) = covar(2,2)-c(2)*c(2)/n;
        covar(0,1) = covar(0,1)-c(0)*c(1)/n;
        covar(0,2) = covar(0,2)-c(0)*c(2)/n;
        covar(1,2) = covar(1,2)-c(1)*c(2)/n;
        covar(1,0) = covar(0,1);
        covar(2,0) = covar(0,2);
        covar(2,1) = covar(1,2);

        typedef std::pair<Eigen::MatrixXd,Eigen::VectorXd> ResultType;
        //std::cout << "COVAR: " << covar << std::endl;
        ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

        // 4.1 create the rotationmatrix from the normalized eigenvectors
        // find max and the second maximal eigenvalue
        size_t maxEigIdx=2, midEigIdx=1, minEigIdx=0;
        double maxEigVal = res.second(maxEigIdx);
        double midEigVal = res.second(midEigIdx);
        double minEigVal = res.second(minEigIdx);
        if( maxEigVal < midEigVal ){
            std::swap(midEigVal,maxEigVal);
            std::swap(midEigIdx,maxEigIdx);
        }
        if( minEigVal>midEigVal ){
            std::swap(midEigVal,minEigVal);
            std::swap(midEigIdx,minEigIdx);
            if( midEigVal>maxEigVal ){
                std::swap(midEigVal,maxEigVal);
                std::swap(midEigIdx,maxEigIdx);
            }
        }
        // specify x and y axis, x will be the axis with largest spred
        Vector3D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx), res.first(2,maxEigIdx) );
        Vector3D<> midAxis( res.first(0,midEigIdx), res.first(1,midEigIdx), res.first(2,midEigIdx) );

        // make sure to turn the z-axis in the direction of the normal
        Vector3D<> crossAxis = cross(maxAxis,midAxis);
        if( dot(crossAxis,_points[_deepestIdx].n)<0 ){
            std::swap(maxAxis,midAxis);
            crossAxis = -crossAxis;
        }

        _normal = normalize(crossAxis);

        const Rotation3D<> rot(normalize(maxAxis),normalize(midAxis), _normal);
        const Rotation3D<> rotInv = inverse( rot );

        Vector3D<> max = rotInv * p.p;
        Vector3D<> min = max;
        for(int i = 0; i<_nrOfContacts; i++ ){
            const Vector3D<> prot = rotInv * _points[i].p;
            if( prot(0)>max(0) ) max(0) = prot(0);
            else if( prot(0)<min(0) ) min(0) = prot(0);
            if( prot(1)>max(1) ) max(1) = prot(1);
            else if( prot(1)<min(1) ) min(1) = prot(1);
            if( prot(2)>max(2) ) max(2) = prot(2);
            else if( prot(2)<min(2) ) min(2) = prot(2);
        }

        // compute halflength of box and its midpoint
        _t3d.P() = rot*( 0.5*(max+min));
        _t3d.R() = rot;
        _h = 0.5*(max-min);
    }

    Vector3D<> getNormal(){
        return _normal;
    }

    Transform3D<> getTransform(){ return _t3d;};

    Vector3D<> getHalfLengths(){ return _h; };

private:
    bool isInsideOBB(Vector3D<>& p){

        Vector3D<> pproj = inverse(_t3d) * p;
        if( fabs(pproj(0))<_h(0) && fabs(pproj(1))<_h(1) ){
            return true;
        }
        return false;
    }

    /**
     * The normal is an average of the normals of the contact normals until 3
     * or more contacts are available. Then the normal becomes the z axis in
     * the OBB that the contacts span
     */
    Vector3D<> _normal; //
    Transform3D<> _t3d; // transform of the obb
    Vector3D<> _h; //halflengths off the obb
    // only used when 3 or more points are available
    ContactPoint _points[5];
    //Frame *_objA,*_objB;
    int _nrOfContacts,_deepestIdx;
    double _threshold,_sepThreshold;
};

std::pair<Transform3D<>, Vector3D<> > fit(std::vector<ContactPoint>& points){
    // re-fit the bounding box
    Eigen::MatrixXd covar( Eigen::MatrixXd::Zero(3, 3) );
    int nrContacts = points.size();
    Vector3D<> c;
    BOOST_FOREACH(ContactPoint &p, points){
        c += p.p;
        covar(0,0) += p.p(0)+p.p(0);
        covar(1,1) += p.p(1)+p.p(1);
        covar(2,2) += p.p(2)+p.p(2);
        covar(0,1) += p.p(0)+p.p(1);
        covar(0,2) += p.p(0)+p.p(2);
        covar(1,2) += p.p(1)+p.p(2);
    }

    const int n = nrContacts;
    covar(0,0) = covar(0,0)-c(0)*c(0)/n;
    covar(1,1) = covar(1,1)-c(1)*c(1)/n;
    covar(2,2) = covar(2,2)-c(2)*c(2)/n;
    covar(0,1) = covar(0,1)-c(0)*c(1)/n;
    covar(0,2) = covar(0,2)-c(0)*c(2)/n;
    covar(1,2) = covar(1,2)-c(1)*c(2)/n;
    covar(1,0) = covar(0,1);
    covar(2,0) = covar(0,2);
    covar(2,1) = covar(1,2);

    typedef std::pair<Eigen::MatrixXd,Eigen::VectorXd> ResultType;
    //std::cout << "COVAR: " << covar << std::endl;
    ResultType res = LinearAlgebra::eigenDecompositionSymmetric( covar );

    // 4.1 create the rotationmatrix from the normalized eigenvectors
    // find max and the second maximal eigenvalue
    size_t maxEigIdx=2, midEigIdx=1, minEigIdx=0;
    double maxEigVal = res.second(maxEigIdx);
    double midEigVal = res.second(midEigIdx);
    double minEigVal = res.second(minEigIdx);
    if( maxEigVal < midEigVal ){
        std::swap(midEigVal,maxEigVal);
        std::swap(midEigIdx,maxEigIdx);
    }
    if( minEigVal>midEigVal ){
        std::swap(midEigVal,minEigVal);
        std::swap(midEigIdx,minEigIdx);
        if( midEigVal>maxEigVal ){
            std::swap(midEigVal,maxEigVal);
            std::swap(midEigIdx,maxEigIdx);
        }
    }
    // specify x and y axis, x will be the axis with largest spred
    Vector3D<> maxAxis( res.first(0,maxEigIdx), res.first(1,maxEigIdx), res.first(2,maxEigIdx) );
    Vector3D<> midAxis( res.first(0,midEigIdx), res.first(1,midEigIdx), res.first(2,midEigIdx) );

    // make sure to turn the z-axis in the direction of the normal
    Vector3D<> crossAxis = cross(maxAxis,midAxis);

    Vector3D<> normal = normalize(crossAxis);

    const Rotation3D<> rot(normalize(maxAxis),normalize(midAxis), normal);
    const Rotation3D<> rotInv = inverse( rot );

    Vector3D<> max = rotInv * points[0].p;
    Vector3D<> min = max;
    for(int i = 0; i<nrContacts; i++ ){
        const Vector3D<> prot = rotInv * points[i].p;
        if( prot(0)>max(0) ) max(0) = prot(0);
        else if( prot(0)<min(0) ) min(0) = prot(0);
        if( prot(1)>max(1) ) max(1) = prot(1);
        else if( prot(1)<min(1) ) min(1) = prot(1);
        if( prot(2)>max(2) ) max(2) = prot(2);
        else if( prot(2)<min(2) ) min(2) = prot(2);
    }

    // compute halflength of box and its midpoint
    Transform3D<> t3d;
    t3d.P() = rot*( 0.5*(max+min));
    t3d.R() = rot;
    Vector3D<> h = 0.5*(max-min);
    return std::make_pair(t3d,h);
}



std::vector<ContactPoint> loadContacts(std::string filename){
    std::vector<ContactPoint> contacts;
    std::ifstream in;
    in.open(filename.c_str());
    char line[200];
    while( !in.eof() ){
        if(in.eof())
           break;

        in.getline(line,200);
        std::cout << ".";
        if(line[0]==';')
            continue;
        if(line[0]==0)
            continue;
        if(line[0]=='\n')
            continue;
        float x,y,z,nx,ny,nz,pen;

        sscanf(line, "%f %f %f %f %f %f %f", &x, &y, &z,&nx, &ny, &nz,&pen);

        contacts.push_back( makeContact(x,y,z,nx,ny,nz,pen) );

    }
    return contacts;
}

int main(int argc, char** argv)
{

	if( argc < 2 ){
	    std::cout << "Arg 1 is input Filename!" << std::endl;
		std::cout << "Arg 2 is cluster threshold distance!" << std::endl;
		std::cout << "Arg 3 is manifold threshold!" << std::endl;
		return 0;
	}

	std::string filename(argv[1]);

	double n = 10*Deg2Rad;
	if(argc>2)
	    n = (double) std::atof(argv[2]);

    /*Unused:
     float mThres = 1.0;
     if(argc>3)
		mThres = std::atof(argv[3]);
     */

	// test contact cluster
	std::vector<ContactPoint> src = loadContacts(filename);

	std::vector<ContactPoint> dst(src.size());
	std::vector<int> srcIdx(src.size());
	std::vector<int> dstIdx(src.size());


	//int num = ContactCluster::thresClustering(&src[0],10,&srcIdx[0],&dstIdx[0],&dst[0],(double)n);
	int num = ContactCluster::normalThresClustering(&src[0],src.size(),
	                                                &srcIdx[0],&dstIdx[0],
	                                                &dst[0],n);
	std::cout << "Number of contacts: " << num << std::endl;
	//for(int i=0;i<num;i++)
	//    std::cout << "i:" << i << " -> " << dst[i].penetration << " " << dst[i].p<< " " << dst[i].n << std::endl;

	//std::cout << "Dst idx array: " << std::endl;
    //for(size_t i=0;i<dstIdx.size();i++)
    //    std::cout << "i:" << i << " -> " << dstIdx[i] << " " << src[dstIdx[i] ].p<< " " << src[dstIdx[i] ].n << std::endl;

    //std::cout << "Src idx array: " << std::endl;
    //for(size_t i=0;i<srcIdx.size();i++)
    //    std::cout << "i:" << i << " -> " << srcIdx[i] << std::endl;

    // test manifold functionality

    /*
    std::vector< OBBManifold > manifolds;
    std::vector< std::vector<ContactPoint> > inManifolds;
    // for each cluster we fit a manifold
    for(int i=0;i<num;i++){
        int idxFrom = srcIdx[i];
        const int idxTo = srcIdx[i+1];
        // locate the manifold that idxFrom is located in
        ContactPoint &deepestP = src[dstIdx[idxFrom]];

        OBBManifold manifold(mThres,0.1);
        std::vector<ContactPoint> pointsinmanifold;
        std::cout << "Adding clustered points to manifold!" << std::endl;
        for(;idxFrom<idxTo; idxFrom++){
            ContactPoint &point = src[dstIdx[idxFrom]];
            std::cout << point.p << std::endl;
            if( manifold.inManifold(point) ){
                std::cout << "The point is in manifold, add it!" << std::endl;
                manifold.addPoint(point);
                pointsinmanifold.push_back(point);
            } else {
                std::cout << "The point is outside manifold, discard it!" << std::endl;
            }
        }
        manifolds.push_back(manifold);
        inManifolds.push_back(pointsinmanifold);
    }
    */


    std::vector< OBRManifold > manifolds;
    std::vector< std::vector<ContactPoint> > inManifolds;
    // for each cluster we fit a manifold
    for(int i=0;i<num;i++){
        int idxFrom = srcIdx[i];
        const int idxTo = srcIdx[i+1];
        // locate the manifold that idxFrom is located in
        //Unused: ContactPoint &deepestP = src[dstIdx[idxFrom]];

        OBRManifold manifold(13*Deg2Rad,0.2);

        std::vector<ContactPoint> pointsinmanifold;
        std::cout << "Adding clustered points to manifold!" << std::endl;
        for(;idxFrom<idxTo; idxFrom++){
            //if(idxFrom>10)
            //    break;
            ContactPoint &point = src[dstIdx[idxFrom]];
            std::cout << point.p << std::endl;
            if( manifold.addPoint(point) ){
                //std::cout << "The point is in manifold, add it!" << std::endl;
                //manifold.addPoint(point);
                pointsinmanifold.push_back(point);
            } else {
                std::cout << "The point is outside manifold, discard it!" << std::endl;
            }
        }
        manifolds.push_back(manifold);
        inManifolds.push_back(pointsinmanifold);
    }


/*
	//OBBManifold manifold(mThres,0.1);
    for(int i=0;i<num;i++){
        int idxFrom = srcIdx[i];
        const int idxTo = srcIdx[i+1];
        // locate the manifold that idxFrom is located in
        ContactPoint &deepestP = src[dstIdx[idxFrom]];
        int manifoldIdx = -1;
        std::cout << "Looking for manifold: ";
        for(int j=0;j<manifolds.size(); j++){
            if( manifolds[j].inManifold(deepestP) ){
                manifoldIdx = j;
                break;
            }
        }
        if(manifoldIdx <0){
            std::cout << "not found!" << std::endl;
            manifoldIdx = manifolds.size();
            manifolds.push_back(OBBManifold(mThres,0.1));
        } else {
            std::cout << "found!" << std::endl;
        }
        OBBManifold &manifold = manifolds[manifoldIdx];
        std::cout << "Adding clustered points to manifold!" << std::endl;
        for(;idxFrom<idxTo; idxFrom++){
            ContactPoint &point = src[dstIdx[idxFrom]];
            std::cout << point.p << std::endl;
            if( manifold.inManifold(point) ){
                std::cout << "The point is in manifold, add it!" << std::endl;
                manifold.addPoint(point);
            } else {
                std::cout << "The point is outside manifold, discard it!" << std::endl;
            }
        }

    }
    */

    std::cout << "\n\nTOTAL Nr contacts in input: " << src.size() << std::endl;
    for(size_t i=0;i<manifolds.size();i++){
        std::cout << i+1 << "'th Manifold" << std::endl;
        //OBBManifold &manifold = manifolds[i];
        OBRManifold &manifold = manifolds[i];
        std::cout << "-- Nr of points  : " << manifold.getNrOfContacts() << std::endl;
        std::cout << "-- HalfLengths   : " << manifold.getHalfLengths() << std::endl;
        std::cout << "-- center        : " << manifold.getTransform().P() << std::endl;
        std::cout << "-- generatedfrom : " << inManifolds[i].size() << std::endl;
        std::pair<Transform3D<>,Vector3D<> > res = fit(inManifolds[i]);
        std::cout << "-- HalfLengths   : " << res.second << std::endl;
        std::cout << "-- center        : " << res.first.P() << std::endl;

    }

	return 1;
}


