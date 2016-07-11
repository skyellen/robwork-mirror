#ifndef RWSIM_SIMULATOR_CONTACTMODEL_HPP_
#define RWSIM_SIMULATOR_CONTACTMODEL_HPP_

#include "RWBody.hpp"

namespace rwsim { namespace dynamics { class ContactPoint; } }

namespace rwsim {
namespace simulator {
	class Contact;
	class ContactModelFactory;
	class ConstraintNode;

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
        						rwsim::dynamics::ContactPoint& point,
                                    double dtInv);

        /**
         * @brief calculates collision impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcCollisionImpulse(Contact& contact,
                                        rwsim::dynamics::ContactPoint& point,
                                        double& nimpulse,
                                        double& timpulse,
                                        int iter);

        /**
         * @brief calculates contact impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcContactImpulse(Contact& contact,
        		rwsim::dynamics::ContactPoint& point,
                                        double& nimpulse,
                                        double& timpulse);

        /**
         * @brief adds the impulse given by nimpulse and timpulse to contactpoint
         * point
         */
        virtual void addImpulse(Contact& contact,
        		rwsim::dynamics::ContactPoint& point,
                                double nimpulse,
                                double timpulse);

        virtual void updateVelocity(Contact& contact);


        /**
         * @brief calculates contact impulse for contactpoint point and
         * sets the result in nimpulse, timpulse.
         */
        virtual void calcContactForce(Contact& contact,
        		rwsim::dynamics::ContactPoint& point,
                                        double& nforce,
                                        double& tforce);

        /**
         * @brief adds the impulse given by nimpulse and timpulse to contactpoint
         * point
         */
        virtual void addForce(Contact& contact,
        		rwsim::dynamics::ContactPoint& point,
                                double nforce,
                                double tforce);


        inline RWBody* toRWBody(dynamics::Body* body){
        	RW_ASSERT(0);
        	return NULL;
        }



    private:
        ContactType _type;
        ContactModelFactory *_factory;

    };

} // namespace dynamics
}

#endif /*CONTACTMODEL_HPP_*/
