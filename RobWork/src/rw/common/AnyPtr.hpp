
#ifndef RW_COMMON_ANY_HPP
#define RW_COMMON_ANY_HPP

#include <typeinfo>

#include <rw/common/Ptr.hpp>

#include <boost/shared_ptr.hpp>

namespace rw {
namespace common {

/**
 * @brief
 */
class AnyPtr {
    public: // structors
		//typedef rw::common::Ptr<AnyPtr> Ptr;

        AnyPtr()
          : content(0)
        {
        }

        /**
         * @brief constructor - ownership of pointers are allways taken
         * @param value
         */
        template<typename ValueType>
        AnyPtr(ValueType* value)
          : content(new holder<ValueType>(value))
        {
        }

        /**
         * @brief constructor - ownership of pointers are allways taken
         * @param value
         */
        template<typename ValueType>
        AnyPtr(const boost::shared_ptr<ValueType> & value)
          : content(new holder<ValueType>(value))
        {
        }

        template<typename ValueType>
        AnyPtr(const rw::common::Ptr<ValueType> & value)
          : content(new holder<ValueType>(value))
        {
        }


        AnyPtr(const AnyPtr & other)
          : content(other.content ? other.content->clone() : 0)
        {
        }

        ~AnyPtr()
        {
            delete content;
        }


		template <class S>
        Ptr<S> cast() const {

			S* data = dynamic_cast<S*>((S*)content->getVoidPtr());
			if(data==NULL){
				return Ptr<S>();
			}

			return ((holder<S>*)content)->_ptr.template cast<S>();
        }


        /**
           @brief The pointer stored in the object.
        */
		template<class S>
        S* get() {
			return  dynamic_cast<S*>((S*)content->getVoidPtr());
		}


        /**
           @brief Support for implicit conversion to bool.
        */
        operator void* () const { return content->getVoidPtr(); }

        /**
         * @brief equallity operator, this only tests if the pointers to the referenced objects are the same
         * and NOT if the smart pointers are the same.
         * @param p [in] smart pointer to compare with
         * @return true if the referenced objects are the same
         */
        template<class A>
        bool operator==(const Ptr<A>& p) const { return content->getVoidPtr()==p.get(); }

		/**
		 * @brief Tests if the smart pointer points to the same instance as \b p
		 */
        bool operator==(void* p) const { return content->getVoidPtr()==p; }


		/**
		 * @brief Returns true is the smart pointer is null
		 */
		bool isNull() {
			return content->getVoidPtr() == NULL;
		}

    private: // types
        class placeholder
        {
        public: // structors

            virtual ~placeholder()
            {
            }

        public: // queries

            virtual const std::type_info & type() const = 0;

            virtual placeholder * clone() const = 0;

            virtual void * getVoidPtr() const = 0;
        };

        template<typename ValueType>
        class holder : public placeholder
        {
        public: // structors

            holder(ValueType* value): _ptr(value){}
            holder(boost::shared_ptr<ValueType> value):  _ptr(value){}
            holder(Ptr<ValueType> value):  _ptr(value){}

        public: // queries

            virtual const std::type_info & type() const
            {
                return typeid(ValueType);
            }

            virtual placeholder * clone() const
            {
                return dynamic_cast<placeholder*>(new holder(*this));
            }

            virtual void * getVoidPtr() const {
            	return (void*)_ptr.get();
            }

        private: // intentionally left unimplemented
            holder & operator=(const holder &);

        public:
            Ptr<ValueType> _ptr;
        };

    private: // representation


        template<typename ValueType>
        friend ValueType* cast(AnyPtr *);

        placeholder * content;
        //void * _vptr;
};

/*
template<typename ValueType>
Ptr<ValueType> cast(AnyPtr& operand) {
	dynamic_cast<ValueType*>(operand->content)
}

template<typename ValueType>
inline const Ptr<ValueType>  cast(const AnyPtr& operand)
{
	return cast<ValueType>(const_cast<AnyPtr *>(operand));
}

template<typename ValueType>
ValueType& cast(AnyPtr & operand)
{
	typedef typename boost::remove_reference<ValueType>::type nonref;

	nonref * result = cast<nonref>(&operand);
	if(!result)
		throw std::bad_cast();
	return *result;
}

template<typename ValueType>
const ValueType& cast(const AnyPtr & operand)
{
	typedef typename boost::remove_reference<ValueType>::type nonref;

	return cast<const nonref &>(const_cast<AnyPtr &>(operand));
}

//template <class T>
//Ptr<AnyPtr> ownedAnyPtr(T* ptr) { return Ptr<Any>(boost::shared_ptr<Any>( new Any(ptr) )); }

*/
}
}


#endif
