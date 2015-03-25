#include "SuctionCupController.hpp"

using namespace rwsim::control;
using namespace rwlibs::control;
using namespace rwlibs::simulation;
using namespace rwsim::dynamics;
using namespace rwsim::sensor;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::geometry;
using namespace rw::math;
using namespace rw::common;


SuctionCupController::SuctionCupController(const std::string& name, rwsim::dynamics::SuctionCup::Ptr dev):
		SimulatedController(rw::common::ownedPtr(new rw::models::ControllerModel(name,dev->getBaseBody()->getBodyFrame()))),
        _dev(dev),_name(name)
{
    // initialize springs between the body parts of the device
    double length = _dev->getHeight();

    // 50N is the vacum force, k*y_max = -50, where y_max = -height
    // k=50N/height

    // create vertical springs to each mass
    /*double k_vertical = (50/length)/_dev->getBodyParts().size();
    for(size_t i=0;i<_dev->getBodyParts().size();i++ ){
        _springs.push_back( Spring(length, k_vertical, 0.001) ); // base to body 1
        _bodyPairs.push_back( make_pair(_dev->getBodyPart(), _dev->getBodyParts()[i]) );
    }

    // create springs to neighboring particels with fairly stiff parameters
    for(size_t i=0;i<_dev->getBodyParts().size();i++ ){
        _springs.push_back( Spring(length, k_vertical, 0.001) ); // base to body 1
    }
    */

    // TODO: for now only 4 body parts is supported
    _bodyTransforms.resize( _dev->getBodyParts().size());
    RW_ASSERT(_bodyTransforms.size()==4);
    _bodyTransforms[0] = Transform3D<>( Vector3D<>(0,-_dev->getRadius(),0) );
    _bodyTransforms[1] = Transform3D<>( Vector3D<>(-_dev->getRadius(),0,0) );
    _bodyTransforms[2] = Transform3D<>( Vector3D<>(0,_dev->getRadius(),0) );
    _bodyTransforms[3] = Transform3D<>( Vector3D<>(_dev->getRadius(),0,0) );


}

SuctionCupController::~SuctionCupController(){

}

void SuctionCupController::update(double dt, rw::kinematics::State& state){
    // check the contact state of the suction cup mouth
    ;
    std::vector<BodyContactSensor::Ptr> &sensors = _dev->getBodySensors();
    bool inContact=true;
    Body* body = NULL;
    BOOST_FOREACH(BodyContactSensor::Ptr sensor, sensors){
        const std::vector<rw::sensor::Contact3D>& contacts = sensor->getContacts();
        if(contacts.size()==0){
            inContact = false;
            break;
        }
        BOOST_FOREACH(Body *b, sensor->getBodies() ){
            if(b!=NULL){
                body = b;
            }

        }
    }



    // TODO: we have to update forces on all bodies to uphold the integrity of the "internal" bodies
    if( body==NULL ){
/*
        std::cout << "NO CONTACT" << std::endl;
        // no contacts, apply forces such that the default configuration will be reached.
        // calculate contact center
        Vector3D<> contactCenter(0,0,0);
        std::vector<Transform3D<> > t3ds( _dev->getFrameParts().size() );
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){
            t3ds[i] = Kinematics::worldTframe(_dev->getFrameParts()[i], state);
            contactCenter += t3ds[i].P();
        }

        Vector3D<> xaxis_tmp = t3ds[1].P() - t3ds[0].P();
        Vector3D<> yaxis = normalize( t3ds[2].P() - t3ds[0].P() );
        Vector3D<> normal = normalize( cross( xaxis_tmp, yaxis ));
        Vector3D<> xaxis = cross(yaxis, normal);
        Rotation3D<> rot(xaxis, yaxis, normal);
        // we also need the orientation, for now we assume that p[0] and p[2] is on the y axis
        Transform3D<> centerFrame(contactCenter, rot);
        ;
        // calculate force direction
        Transform3D<> wTb = Kinematics::worldTframe(_dev->getBodyFrame(), state);




        const double N = _dev->getBodyParts().size();
        const double mainstrainF_elasticity = 50/_dev->getHeight(); // max force on 50N
        const double rotaionstrain_elasticity = (10*_dev->getRadius())/(90*Deg2Rad); // max force in point 10N and max angular displacement 90Deg, therefore t_max = (r*F_max)/90deg
        //const double torsionalstrain_elasticity = ;//
        ;
        // first we look at the main strain, which is a spring connecting the center point to the bodyframe
        Vector3D<> mainstrainF = contactCenter-wTb.P();
        Vector3D<> mainstrainF_dir = normalize(mainstrainF);
        double mainstrainF_length = mainstrainF.norm2();
        double mainstrainF_springforce = mainstrainF_elasticity * _dev->getHeight()-mainstrainF_length;
        ;
        // add the mainstrain force to the bodies
        BOOST_FOREACH(Body* bodyPart, _dev->getBodyParts()){
            bodyPart->addForceW( mainstrainF_dir*(mainstrainF_springforce/N), state  );
            std::cout << "mainStrain: " << mainstrainF_dir*(mainstrainF_springforce/N) << std::endl;
        }
        _dev->getBodyPart()->addForceW( -mainstrainF_dir*mainstrainF_springforce, state );

        ;
        // now add forces related to the rotational stress
        Vector3D<> rotationVector = cross(wTb*Vector3D<>::z(), mainstrainF_dir);
        if(rotationVector.norm2()>0.00001 ){
            rotationVector = normalize( rotationVector );
            double ang = angle( wTb*Vector3D<>::z(), mainstrainF_dir, rotationVector );
            double torque = -ang*rotaionstrain_elasticity/N;

            // apply the torque onto all bodies
            for(size_t i=0; i<_dev->getFrameParts().size(); i++){
                Body* bodyPart = _dev->getBodyParts()[i];
                Vector3D<> toPoint = t3ds[i].P()-(rotationVector*dot( t3ds[i].P(), rotationVector));

                // we calculate the force on the bodies as f = t/r
                double force = 0;
                if(toPoint.norm2()>0.00001)
                    force = torque/toPoint.norm2();
                bodyPart->addForceW(  force*-normal, state );

            }
            //_dev->getBodyPart()->addForceW( -mainstrainF_dir*mainstrainF_springforce );
        }
*/


        ;
        // next we look at the torsional stress
        // first we calculate the angular displacement of the y-axis around the
        /*
        double angNormal = angle(yaxis, wTb.R()*Vector3D<>::y(), normal );
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){


        }
        */
/*
        // lastly secure all points relative to the fixed positions of the centerFrame
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){
            Body* body = _dev->getBodyParts()[i];
            Vector3D<> displacement = (centerFrame*_bodyTransforms[i]).P() - t3ds[i].P();
            Vector3D<> force = normalize(displacement)*displacement.norm2()*mainstrainF_elasticity*10;
            body->addForceW(force, state);
        }
*/





    }

    if(!inContact && body!=NULL){
        std::cout << "SuctionCupController: All bodies are not in contact!" << std::endl;
        return;
    }

    if(inContact){
        // if any of the mouth parts are in contact then check how close the whole mouth are to
        // the external geometry and calculate the possible suction force.
        // TODO


        // if the mouth is close enough then apply the suction force to the ext object
        double suctionForce = 30; // TODO: suctionForce should be calulated from area and pressure of suction cup

        // calculate contact center
        Vector3D<> contactCenter(0,0,0);
        std::vector<Transform3D<> > t3ds( _dev->getFrameParts().size() );
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){
            t3ds[i] = Kinematics::worldTframe(_dev->getFrameParts()[i], state);
            contactCenter += t3ds[i].P();
        }

        Vector3D<> xaxis_tmp = t3ds[1].P() - t3ds[0].P();
        Vector3D<> yaxis = normalize( t3ds[2].P() - t3ds[0].P() );
        Vector3D<> normal = normalize( cross( xaxis_tmp, yaxis ));
        Vector3D<> xaxis = cross(yaxis, normal);
        Rotation3D<> rot(xaxis, yaxis, normal);
        // we also need the orientation, for now we assume that p[0] and p[2] is on the y axis
        Transform3D<> centerFrame(contactCenter, rot);



        // calculate force direction
        Transform3D<> wTb = Kinematics::worldTframe(_dev->getBodyFrame(), state);
        //Vector3D<> vacumForce = normalize(wTb.P()-contactCenter)*suctionForce;

        body->addForceWToPosW( normal*suctionForce , contactCenter, state );


        // -------------------------------------------------------------------------------
        // next update the contact force of the contacting mouth parts, such that the elasticity of the
        // mouth piece is taken into account


        const double N = _dev->getBodyParts().size();
        const double mainstrainF_elasticity = 50/_dev->getHeight(); // max force on 50N
        const double rotaionstrain_elasticity = (10*_dev->getRadius())/(90*Deg2Rad); // max force in point 10N and max angular displacement 90Deg, therefore t_max = (r*F_max)/90deg
        //const double torsionalstrain_elasticity = ;//

        // first we look at the main strain, which is a spring connecting the center point to the bodyframe
        Vector3D<> mainstrainF = contactCenter-wTb.P();
        Vector3D<> mainstrainF_dir = normalize(mainstrainF);
        double mainstrainF_length = mainstrainF.norm2();
        double mainstrainF_springforce = mainstrainF_elasticity * _dev->getHeight()-mainstrainF_length;

        // add the mainstrain force to the bodies
        BOOST_FOREACH(Body* body, _dev->getBodyParts()){
            body->addForceW( mainstrainF_dir*(mainstrainF_springforce/N), state  );
        }
        _dev->getBodyPart()->addForceW( -mainstrainF_dir*mainstrainF_springforce, state );


        // now add forces related to the rotational stress
        Vector3D<> rotationVector = cross(wTb*Vector3D<>::z(), mainstrainF_dir);
        if(rotationVector.norm2()>0.00001 ){
            rotationVector = normalize( rotationVector );
            double ang = angle( wTb*Vector3D<>::z(), mainstrainF_dir, rotationVector );
            double torque = -ang*rotaionstrain_elasticity/N;

            // apply the torque onto all bodies
            for(size_t i=0; i<_dev->getFrameParts().size(); i++){
                Body* body = _dev->getBodyParts()[i];
                Vector3D<> toPoint = t3ds[i].P()-(rotationVector*dot( t3ds[i].P(), rotationVector));

                // we calculate the force on the bodies as f = t/r
                double force = 0;
                if(toPoint.norm2()>0.00001)
                    force = torque/toPoint.norm2();
                body->addForceW(  force*-normal, state );

            }
            //_dev->getBodyPart()->addForceW( -mainstrainF_dir*mainstrainF_springforce );
        }

        // next we look at the torsional stress
        // first we calculate the angular displacement of the y-axis around the
        /*
        double angNormal = angle(yaxis, wTb.R()*Vector3D<>::y(), normal );
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){


        }
        */

        // lastly secure all points relative to the fixed positions of the centerFrame
        for(size_t i=0; i<_dev->getFrameParts().size(); i++){
            Body* body = _dev->getBodyParts()[i];
            Vector3D<> displacement = (centerFrame*_bodyTransforms[i]).P() - t3ds[i].P();
            Vector3D<> force = normalize(displacement)*displacement.norm2()*mainstrainF_elasticity*10;
            body->addForceW(force, state);
        }

    }
}

void SuctionCupController::reset(const rw::kinematics::State& state){
    // reset the suction cup to the current state

    for(size_t i=0; i<_dev->getFrameParts().size(); i++){
        RigidBody* body = _dev->getBodyParts()[i];
        _bodyTransforms[i] = body->getMovableFrame()->getTransform(state);

    }


}

Controller* SuctionCupController::getController(){

    return NULL;
}

