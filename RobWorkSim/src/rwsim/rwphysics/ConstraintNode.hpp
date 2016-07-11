#ifndef CONTACTNODE_HPP_
#define CONTACTNODE_HPP_

#include <list>

#include "RWBody.hpp"

namespace rw { namespace kinematics { class Frame; } }

namespace rwsim {
namespace simulator {


    class ConstraintEdge;

    /**
     * @brief The ConstraintNode describes a specific object that can be constrained in some way.
     * It is an interface that enables inheriting classes to be used by the ConstraintGraph class.
     * The specific constraints between ConstraintNode's are described by the ConstraintEdge.
     *
     *  A ConstraintNode will allways be related to a frame. Multiple ConstraintNodes cannot point to
     * the same frame. Which means that there is a one-to-one relationship between frame and node.
     */
    class ConstraintNode
    {
    public:
        //! @brief types of constraintnodes
        typedef enum{Rigid, Fixed, Link, Scripted, Part, // Physical
                     CompositeBody, MultiBody,    // Container
                     Rule, Trigger, Timer         // Logical
                     } NodeType;

    public:

        ConstraintNode(NodeType type, int id);

        ConstraintNode(ConstraintNode* parent, NodeType type, int id);

        virtual ~ConstraintNode(){};

        void setType(NodeType type);

        /**
         * @brief hmm?
         */
        inline void setDeleted(bool del) {
            _deleted = del;
        }

        /**
         * @brief hmm?
         */
        inline bool isDeleted() const {
            return _deleted;
        }

        /**
         * @brief hmm?
         */
        inline bool isPassive() const {
            return false;
        }

        /**
         * @brief returns true if this node has physical
         */
        inline bool isPhysical() const {
            return _isPhysical;
        }

        /**
         * @brief returns true is this node is of the type trigger
         */
        inline bool isTrigger() const {
            return _type==Trigger;
        }

        /**
         * @brief ad a constraint to this node
         */
        inline void addEdge(ConstraintEdge& edge){
            _edges.push_back(&edge);
        }

        /**
         * @brief returns the list of constraints that works on this node
         */
        inline std::list<ConstraintEdge*>& getEdges(){
            return _edges;
        }

        /**
         * @removes a constraint (Edge) from this node
         */
        inline void removeEdge(ConstraintEdge *edge){
            _edges.remove(edge);
        }

        /**
         * @brief return the type of this node
         */
        const NodeType getNodeType() const {
            return _type;
        }

        ConstraintNode* getParentNode(){
            return _parentNode;
        }

        void setFrame(rw::kinematics::Frame* frame){
            _frame = frame;
        }

        /**
         * @brief return the frames which this node is associated with
         */
        rw::kinematics::Frame* getFrame() {
            return _frame;
        }

        inline void setID(int id) {
            _id = id;
        }

        inline int getID() const {
            return _id;
        }

        // user data type
        void *data;

        RWBody* getBody(){ return _body; };

        void setBody(RWBody* body){
            _body = body;
            setFrame( body->getBodyFrame() );
        };

    private:
        // the type of this constraint node
        NodeType _type;

        ConstraintNode *_parentNode;

        // variable indicating if this node is physical
        bool _isPhysical;

        // list of constraints that are working on this node
        std::list<ConstraintEdge*> _edges;

        // the frame that this node is associated with
        rw::kinematics::Frame* _frame;

        RWBody* _body;

        int _id;

        bool _deleted;
    public:
        int _idx;

    };


    struct CNodePair {
    public:
        CNodePair(ConstraintNode* n1, ConstraintNode* n2){
            if( n1->getID() > n2->getID() ){
                first = n2;
                second = n1;
            } else {
                first = n1;
                second = n2;
            }
        }
        ConstraintNode* first;
        ConstraintNode* second;
    };

}
}

#endif /*ConstraintNode_HPP_*/
