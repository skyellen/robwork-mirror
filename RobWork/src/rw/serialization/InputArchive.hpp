#ifndef RW_COMMON_INPUTARCHIVE_HPP
#define RW_COMMON_INPUTARCHIVE_HPP

#include "Archive.hpp"

#include <boost/cstdint.hpp>

/**
 * @brief archive for loading and saving serializable classes.
 */
class InputArchive: public Archive {
public:
    InputArchive(){};

    virtual void open(std::istream& ifs) = 0;

    // utils to handle arrays
    virtual void readEnterScope(const std::string& id) = 0;
    virtual void readLeaveScope(const std::string& id) = 0;
    virtual void readEnterArray(const std::string& id) = 0;
    virtual void readLeaveArray(const std::string& id) = 0;


    // writing primitives to archive
    virtual void read(bool& val, const std::string& id) = 0;
    virtual void read(int& val, const std::string& id) = 0;
    virtual void read(boost::uint64_t &val, const std::string& id) = 0;
    virtual void read(double& val, const std::string& id) = 0;
    virtual void read(std::string&  val, const std::string& id) = 0;

    bool readBool(const std::string& id){ bool b; read(b,id); return b;};
    int readInt(const std::string& id){ int b; read(b,id); return b;};;
    boost::uint64_t readUInt64(const std::string& id){ boost::uint64_t b; read(b,id); return b;};;
    double readDouble(const std::string& id) { double b; read(b,id); return b;};
    std::string readString(const std::string& id) { std::string b; read(b,id); return b;};;


/*
    std::string readString(const std::string& id){
        std::string result;
        read(result, id);
        return result;
    }
*/
    //
    template<class T>
    T* read(const std::string& id){
        readEnterScope(id);
        T* data = Archive::Access::load<T>(*this, id);

        if(data==NULL){
            // try read with generic call
            boost::any anyval = read(id);
            data = boost::any_cast<T>(&anyval);
            if(data == NULL ){
                RW_THROW("No leader for data! id:"<< id);
            }
        }
        readLeaveScope(id);
        return data;
    }


protected:

    /**
     * @brief this is the fallback call in case a class does not implement load/save funtionality
     * then the implementation on the OutputArchive might be able to save the class.
     *
     * @note unfortunately this require the implementing InputArchive to use downcast methods...
     */
    virtual boost::any read(const std::string& id){ RW_THROW("No handler for this type of data!");};
};
#endif
