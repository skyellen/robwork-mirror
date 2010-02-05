#ifndef FIXEDBODY_HPP_
#define FIXEDBODY_HPP_

#include "Body.hpp"

namespace dynamics {


    class FixedBody : public Body
    {
    public:
    	FixedBody(
    	    const BodyInfo& info,
    	    rw::kinematics::Frame *bodyframe,
            const std::vector<rw::geometry::sandbox::Geometry*>& geoms):
    	    Body(info, bodyframe, geoms)
    	{

    	}

    	virtual ~FixedBody(){}

    public: // inheritet from Body interface

        virtual void saveState(double h, rw::kinematics::State& state){};

        virtual void rollBack(rw::kinematics::State& state){};

        virtual void updateVelocity(double h, rw::kinematics::State& state){};

        virtual void updatePosition(double h, rw::kinematics::State& state){};

        virtual void updateImpulse(){};

        virtual rw::math::InertiaMatrix<> getEffectiveMassW(const rw::math::Vector3D<>& wPc){
        	// TODO: The EffectiveMass of a fixed object should be infinite...
        	return rw::math::InertiaMatrix<>(1000000,1000000,1000000);
        };

        virtual rw::math::Vector3D<> getPointVelW(const rw::math::Vector3D<>& p){
        	return rw::math::Vector3D<>(0,0,0);
        };

        virtual void reset(){};

    	const std::string& getMaterial(){
    	    return this->getInfo().material;
    	}

    	 void resetState(rw::kinematics::State &state){}

    	 void calcAuxVarialbles(rw::kinematics::State& state){}

    	 double calcEnergy(){return 0;};

    };

}

#endif /*FIXEDBODY_HPP_*/
