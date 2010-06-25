#ifndef CONTACTMODEL_HPP_
#define CONTACTMODEL_HPP_

#include <rw/models/Device.hpp>

#include <rw/kinematics/Kinematics.hpp>

#include <rw/math/Transform3D.hpp>
#include <rw/math/Vector3D.hpp>

#include <dynamics/RigidBody.hpp>
#include "Contact.hpp"
#include <dynamics/ContactPoint.hpp>
#include <dynamics/FixedLink.hpp>

#include <boost/numeric/ublas/matrix.hpp>

#include "ConstraintNode.hpp"

//#define DYNAMIC_TEST 1

namespace rwsim {
namespace simulator {


	class ContactModelFactory;

    /**
     * @brief The ContactModel describe how impulses between two bodies
     * are calculated and added to the bodies. The contact model is supposed
     * to be used for iterative solving impulses between two bodies.
     *
     */

    class ContactModel
    {
    private:
        typedef enum {FixedRigid,RigidFixed,RigidRigid,
                      FixedLink,LinkFixed,LinkLink,
                      RigidLink,LinkRigid,Unknown} ContactType;

    public:

        /**
         * @brief default constructor
         */
    	ContactModel(ConstraintNode &bodyA, ConstraintNode &bodyB, ContactModelFactory *factory);

    	/**
    	 * @brief default destructor
    	 */
    	virtual ~ContactModel(){};

        /**
         * @brief this call will be made prior to any other calls in a timestep.
         * It is meant to be a function that caches aux variables that can be used
         * when calling add*Impulses multiple times.
         */
        virtual void preImpulseCalc(Contact& contact,
                                    ContactPoint& point,
                                    double dtInv);

        /**
         * @brief calculates collision impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcCollisionImpulse(Contact& contact,
                                        ContactPoint& point,
                                        double& nimpulse,
                                        double& timpulse,
                                        int iter);

        /**
         * @brief calculates contact impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcContactImpulse(Contact& contact,
                                        ContactPoint& point,
                                        double& nimpulse,
                                        double& timpulse);

        /**
         * @brief adds the impulse given by nimpulse and timpulse to contactpoint
         * point
         */
        virtual void addImpulse(Contact& contact,
                                ContactPoint& point,
                                double nimpulse,
                                double timpulse);

        virtual void updateVelocity(Contact& contact);


        /**
         * @brief calculates contact impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcContactForce(Contact& contact,
                                        ContactPoint& point,
                                        double& nforce,
                                        double& tforce);

        /**
         * @brief adds the impulse given by nimpulse and timpulse to contactpoint
         * point
         */
        virtual void addForce(Contact& contact,
                                ContactPoint& point,
                                double nforce,
                                double tforce);


        inline RWBody * toRWBody(dynamics::Body* body){

        }



    private:
        ContactType _type;
        ContactModelFactory *_factory;

    };

} // namespace dynamics
}

#endif /*CONTACTMODEL_HPP_*/
