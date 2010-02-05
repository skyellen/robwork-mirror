#ifndef DRAWABLECONTACT_HPP_
#define DRAWABLECONTACT_HPP_

#include <rwlibs/drawable/Drawable.hpp>

class DrawableContact: public rwlibs::drawable::Drawable
{
public:
	DrawableContact();
	
	virtual ~DrawableContact();

	/**
	 * @brief The point of contact
	 */
	void setPosition(rw::math::Vector3D<> pos);
	
	/**
	 * @brief the Force acting on the contact point
	 */
	void setForce(rw::math::Vector3D<> force);
	
    /**
     * @brief draws the object.
     */
    virtual void draw() const;
	
private:
    /**
     * @copydoc Drawable::update
     */
    void update(UpdateType type);
	
};

#endif /*DRAWABLECONTACT_HPP_*/
