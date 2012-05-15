#ifndef GUENDELCONTACTMODEL_HPP_
#define GUENDELCONTACTMODEL_HPP_


namespace rwsim {
namespace simulator {

class GuendelContactModel : public dynamics::ContactModel
{
public:
    
    /**
     * @brief constructor
     * @param colRestCoeff [in] the restitution coefficient of collisions
     * @param colRestCoeff [in] the restitution coefficient of contacts
     */
	GuendelContactModel(double colRestCoeff, double conRestCoeff):
	    _colRestCoeff(colRestCoeff), 
	    _contactRestCoeff(conRestCoeff)
	{};
	
	/**
	 * @brief destructor
	 */
	virtual ~GuendelContactModel(){};
	
	/**
	 * @copydoc dynamics::ContactModel::preImpulseCalc 
	 */
    virtual void preImpulseCalc(Contact& contact,
                                ContactPoint& point);
    
    /**
     * @copydoc dynamics::ContactModel::preImpulseCalc 
     */
    virtual void calcCollisionImpulse(Contact& contact, 
                                    ContactPoint& point,
                                    double& nimpulse,
                                    double& timpulse);

    /**
     * @copydoc dynamics::ContactModel::preImpulseCalc 
     */
    virtual void calcContactImpulse(Contact& contact, 
                                    ContactPoint& point,
                                    double& nimpulse,
                                    double& timpulse);
    
    /**
     * @copydoc dynamics::ContactModel::preImpulseCalc 
     */
    virtual void addImpulse(Contact& contact,
                            ContactPoint& point,
                            double nimpulse,
                            double timpulse);
    
private:
    
    GuendelContactModel(){};
    
private:
    double _colRestCoeff;
    double _contactRestCoeff;
};
}
}
#endif /*GUENDELCONTACTMODEL_HPP_*/
