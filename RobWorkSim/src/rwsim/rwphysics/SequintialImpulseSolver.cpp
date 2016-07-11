/*
 * SequintialImpulseSolver.cpp
 *
 *  Created on: 21-10-2008
 *      Author: jimali
 */

#include "SequintialImpulseSolver.hpp"
#include "ConstraintEdge.hpp"

#include "ContactModel.hpp"

#include <rw/math/Math.hpp>

using namespace rw::kinematics;
using namespace rw::math;
using namespace rwsim::simulator;


//#define RW_DEBUG( str ) std::cout << str  << std::endl;

bool SequintialImpulseSolver::solveGroup( CEdgeGroup& group,
                                          SolverInfo& info,
                                          rw::kinematics::State& state){

    double dtInv = 1.0/info.dt;

    RW_DEBUG("* Filter out all actual physical contacts for each group!");
    RW_DEBUG("* Total nr of constraints in group: " << group.size());

    unsigned int nrOfContacts = 0;
    std::vector<Contact*> realContacts;
    std::vector<ConstraintEdge*>& contacts = group;

    for(size_t i=0; i<contacts.size(); i++){
        RW_DEBUG("1");
        if( contacts[i]->getType() != ConstraintEdge::Physical){// TODO: this should be unneasesary
            RW_DEBUG("Contact is not physical");
            continue;
        }
        RW_DEBUG("2");
        if( !contacts[i]->isTouching() ){// TODO: this should be unnessesary
            RW_DEBUG("Contact is not touching");
            continue;
        }
        RW_DEBUG("3");
        // The contact information is only valid in physical constraint edges
        Contact *contact = contacts[i]->_contact;
        if( contact == NULL ){
            RW_DEBUG("Contact is null");
            continue;
        }
        RW_DEBUG("4");
        realContacts.push_back(contact);

        // initialize aux variables
        nrOfContacts += (int)contact->contactPoints.size();
        ContactPointList::iterator point = contact->contactPoints.begin();
        for(;point!=contact->contactPoints.end();++point){
            std::cout << "PreImpulseCalc:" << std::endl;
            contact->getModel().preImpulseCalc( *contact, *point, dtInv );
            // hot start the sequential impulse solver by adding cahced impulses

            //std::cout << "addImpulse:" << std::endl;
            //contact->getModel().addImpulse(*contact,*point,point->nImpulse,point->tImpulse);
            // and remember to update the impulse velocities of the bodies
            //std::cout << "update Velocity:" << std::endl;
            //contact->getModel().updateVelocity(*contact);
        }
    }
    if( nrOfContacts == 0 ){
        RW_DEBUG("No contacts found!");
        return true;
    }

    // now comes the actual impulse determination using sequential impulses
    RW_DEBUG("* Calculate and accumulate impulses on individual contacts");
    boost::numeric::ublas::vector<double> error(nrOfContacts*2);
    boost::numeric::ublas::vector<double> jnLast(nrOfContacts);
    boost::numeric::ublas::vector<double> jtLast(nrOfContacts);
    size_t MAX_ITERATIONS = 20; // TODO: use another measure than fixed number
    double avgErrChg = 0.0, lastAvgErrChg=0.0, err;
    size_t j;
    for(j=0; j<MAX_ITERATIONS; j++){
        unsigned int index = 0;
        for(size_t i=0;i<realContacts.size();i++){
            RW_DEBUG("Get contact and model");
            Contact &contact = *realContacts[i];
            ContactModel &model = contact.getModel();

            RW_DEBUG("Calc impulse through iterations");
            ContactPointList::iterator point = contact.contactPoints.begin();
            for(double jn=0,jt=0;point!=contact.contactPoints.end();++point,jn=0,jt=0,index++){
                // first calculate the normal and tanget impulse
                model.calcCollisionImpulse(contact,*point, jn, jt, (int)j);

                // then clamp normal impulse
                double tmp = point->nImpulse;
                point->nImpulse = std::max(tmp+jn, 0.0);
                jn = point->nImpulse - tmp;

                //error( index*2 ) = jn - jnLast( index );
                avgErrChg += fabs(jn - jnLast( index ));
                jnLast( index ) = jn;

                // and tangential impulse
                double maxPt = contact.staticFriction * point->nImpulse;
                tmp = point->tImpulse;
                point->tImpulse = Math::clamp(tmp+jt, -maxPt, maxPt );
                jt = point->tImpulse - tmp;

                //error( index*2 + 1 ) = jt - jtLast( index );
                avgErrChg += fabs(jt - jtLast( index ));
                jtLast( index ) = jt;

                // and last add impulse to body and update velocity
                model.addImpulse(contact,*point,jn,jt);
                model.updateVelocity(contact);
            }
        }
        //double err = norm_2( error );
        err = (lastAvgErrChg - avgErrChg)/(double)error.size();
        lastAvgErrChg = avgErrChg;
        avgErrChg = 0;

        RW_DEBUG("Error: " << fabs(err) );
        if( fabs(err)<0.000001 && j>6 ){
            //std::cout << "ERROR: " << err <<  "Iteration: " << j <<  std::endl;
            break;
        }
    }
    RW_DEBUG("* ERROR: " << err <<  "Iterations: " << j);
    /*for(size_t i=0;i<realContacts.size();i++){
        Contact &contact = *realContacts[i];
        ContactPointList::iterator point = contact.contactPoints.begin();
       // for(;point!=contact.contactPoints.end();++point){
       //   std::cout << "Impulses: " << point->nImpulse << " " << point->tImpulse << " " << point->dist << std::endl;
        //}
    }*/

    return true;
}
