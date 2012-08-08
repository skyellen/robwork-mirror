#ifndef RW_COMMON_ARCHIVE_HPP
#define RW_COMMON_ARCHIVE_HPP

#include <cstdlib>
#include <cmath>
#include <string>

#include <boost/any.hpp>
#include <cstdio>
#include <fstream>
#include <rw/common/macros.hpp>
#include <boost/any.hpp>


class InputArchive;
class OutputArchive;


namespace serialization {
    template<class T>
    T* load(T *data, class InputArchive& iarchive, const std::string& id){
        std::cout << "No overloaded method to load class: " << typeid(T).name() << std::endl;
        return NULL;
    }

    template<class T>
    bool save(const T& data, class OutputArchive& iarchive, const std::string& id){
        std::cout << "No overloaded method to save class: " << typeid(T).name() << std::endl;
        return false;
    }

}


/**
 * @brief archive for loading and saving serializable classes.
 */
class Archive {
public:

    virtual void open(const std::string& filename) = 0;
    virtual bool isOpen() = 0;
    virtual void close() = 0;



    class Access {
    public:
/*
        // SFINAE test
        template <typename T>
        class
        {
            typedef char one;
            typedef long two;

            template <typename C> static one test( char[sizeof(&C::load)] ) ;
            template <typename C> static two test(...);


        public:
            enum { value = sizeof(test<T>(0)) == sizeof(char) };
        };
  */
        template<typename T>
        class has_load_func
        {
            struct Fallback { int load; }; // add member name "X"
            struct Derived : T, Fallback { };

            template<typename U, U> struct Check;

            typedef char ArrayOfOne[1];  // typedef for an array of size one.
            typedef char ArrayOfTwo[2];  // typedef for an array of size two.

            template<typename U>
            static ArrayOfOne & func(Check<int Fallback::*, &U::load> *);

            template<typename U>
            static ArrayOfTwo & func(...);

          public:
            typedef has_load_func type;
            enum { value = sizeof(func<Derived>(0)) == 2 };
        };


        template<typename T>
        class has_save_func
        {
            struct Fallback { int save; }; // add member name "X"
            struct Derived : T, Fallback { };

            template<typename U, U> struct Check;

            typedef char ArrayOfOne[1];  // typedef for an array of size one.
            typedef char ArrayOfTwo[2];  // typedef for an array of size two.

            template<typename U>
            static ArrayOfOne & func(Check<int Fallback::*, &U::save> *);

            template<typename U>
            static ArrayOfTwo & func(...);

          public:
            typedef has_save_func type;
            enum { value = sizeof(func<Derived>(0)) == 2 };
        };


        template <class A>
        static A* load(class InputArchive& iarchive, const std::string& id, typename boost::enable_if_c<has_load_func<A>::value, A>::type* def=NULL){
            // todo check if A actually is friend with Access class, else return *NULL
            //typedef int(*A_type)(int,double) ;

            //CREATE_MEMBER_FUNC_SIG_CHECK(x, void (T::*)(), void__x);
            //bool has_func_sig_void__x = has_member_func_void__x<class_to_check_for_x>::value;
            std::cout << "has_load_func: " << has_load_func<A>::value << std::endl;

            A* result = A::load(iarchive);
            return result;
        }

        template <class A>
        static A* load(class InputArchive& iarchive, const std::string& id, typename boost::disable_if_c<has_load_func<A>::value, A>::type* def=NULL){
            // the class does not define a load function so we search for a util function in
            // serialization namespace
            std::cout << "has_load_func: " << has_load_func<A>::value << std::endl;

            return serialization::load<A>( (A*)NULL, iarchive, id);
        }


        template <class A>
        static bool save(const A& data, class OutputArchive& oarchive, const std::string& id, typename boost::enable_if_c<has_save_func<A>::value, A>::type* def=NULL){
            // todo check if A actually is friend with Access class, else return false
            A::save(data, oarchive);
            return true;
        }

        template <class A>
        static bool save(const A& data, class OutputArchive& oarchive, const std::string& id, typename boost::disable_if_c<has_save_func<A>::value, A>::type* def=NULL){
            // todo check if A actually is friend with Access class, else return false
            return serialization::save<A>( data, oarchive, id);
        }

    };
};

#endif
