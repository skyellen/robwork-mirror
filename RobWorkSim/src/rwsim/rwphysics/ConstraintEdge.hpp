#ifndef DYNAMICS_CONSTRAINTEDGE_HPP_
#define DYNAMICS_CONSTRAINTEDGE_HPP_

#include "ConstraintNode.hpp"

#include "Contact.hpp"

namespace rwsim {
namespace simulator {


    class ConstraintEdge
    {
    private:
        ConstraintEdge();

    public:

        //! @brief edge states
        typedef enum {NewTouch, PersistentTouch, VanishingTouch,
                      NewProximity, PersistentProximity,
                      VanishingProximity, Static } EdgeState;

        //! @brief Types of edges
        typedef enum {Logical, //!
        			  Structural,
        			  Geometric,
        			  Physical
        			  } EdgeType;

        /**
         * @brief Default constructor
         */
        ConstraintEdge(CNodePair n,
                    EdgeType type,
                    double touchDist,
                    double penDist,
                    double sepDist);

        ConstraintEdge(CNodePair n,
                    EdgeType type, int id);

        /**
         * @brief destructor
         */
        virtual ~ConstraintEdge(){};

        inline void setType(EdgeType type) {
            _type = type;
        }
        inline EdgeType getType() const {
            return _type;
        }

        inline void setColor(unsigned char color){
            _color = color;
        }

        inline unsigned char getColor() const {
            return _color;
        }

        inline const CNodePair& getNodes() const {
            return _nodes;
        }

        inline double getLastDistance()  const {
            return _shortestDist[1];
        }

        inline double getDistance()  const {
            return _shortestDist[0];
        }

        inline void setDistance(double dist) {
            _shortestDist[0] = dist;
        }

        bool isTouching() const {
            return  _state[0]==ConstraintEdge::NewTouch ||
                    _state[0]==ConstraintEdge::PersistentTouch;
        }

        bool wasTouching() const {
            return _state[1]==ConstraintEdge::NewTouch ||
                   _state[1]==ConstraintEdge::PersistentTouch;
        }

        bool isObsolete() const {
            return _state[1]==ConstraintEdge::VanishingProximity &&
                   _state[0]==ConstraintEdge::VanishingProximity;
        }
/*
        inline ContactModel *getContactModel(){
            return _model;
        }
*/
        inline EdgeState getState() const {
            return _state[0];
        }

        inline void setState( EdgeState state ){
            _state[0] = state;
        }

        inline EdgeState getLastState() const {
            return _state[1];
        }

        bool isPenetrating() const {
        	if( _shortestDist[0] < _penDist ){
        		std::cout << "PENETRATING: " << _shortestDist[0] << " < " << _penDist << std::endl;
        		std::cout << _nodes.first->getFrame()->getName() << " " << _nodes.second->getFrame()->getName() << std::endl;
        	}
            return _shortestDist[0] < _penDist;
        }

        bool wasSeperating() const {
            return _shortestDist[1] > _sepDist;
        }

        bool isSeperating() const {
            return _shortestDist[0]> _sepDist;
        }

        void rollBack();

        void saveState();

        void setResting(bool r) {
            _resting = r;
        }

        bool isResting() const {
            return _resting;
        }

        void print();

        void printState(EdgeState state);

        Contact& getContact(){
            return *_contact;
        }

        bool isDeleted(){ return _deleted; };

        void setDeleted(bool del){ _deleted = del; }

        int getID(){ return _id; };

        void setID( int id ){ _id=id; };

        void setThresholds(double touch, double pen, double sep){
            _touchDist = touch;
            _penDist = pen;
            _sepDist = sep;
        };

    private: // private variables
        // color of this edge
        unsigned char _color;

        // The constraint works between these two nodes
        CNodePair _nodes;

        // edge type
        EdgeType _type;

        bool _deleted;

        //
        double _shortestDist[3];
        // states that should be able to roll back
        EdgeState _state[3];

        bool _resting;
        double _touchDist,_penDist,_sepDist;

        int _id;

    public:
        // TODO: these should be template variables
        /*rw::proximity::DistanceResult _narrowCache;
        rw::proximity::MultiDistanceResult _contactSeed;

        ContactModel *_model;
        */
        Contact *_contact;

        void *data;

    };
} // namespace dynamics
}
#endif /*ConstraintEdge_HPP_*/
