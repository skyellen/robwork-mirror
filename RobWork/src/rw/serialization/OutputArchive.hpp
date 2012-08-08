#ifndef RW_COMMON_OUTPUTARCHIVE_HPP
#define RW_COMMON_OUTPUTARCHIVE_HPP

#include "Archive.hpp"
#include <boost/cstdint.hpp>

class OutputArchive : public Archive {
public:
    virtual ~OutputArchive(){};

    virtual void open(std::ostream& ofs) = 0;

    // utils to handle arrays
    virtual void writeEnterScope(const std::string& id) = 0;
    virtual void writeLeaveScope(const std::string& id) = 0;
    virtual void writeEnterArray(const std::string& id) = 0;
    virtual void writeLeaveArray(const std::string& id) = 0;

    // writing primitives to archive
    virtual void write(bool val, const std::string& id) = 0;
    virtual void write(int val, const std::string& id) = 0;
    virtual void write(boost::uint64_t val, const std::string& id) = 0;
    virtual void write(double val, const std::string& id) = 0;
    virtual void write(const std::string&  val, const std::string& id) = 0;

    // a pointer value

/*
    // what about list/ types?
    template<class T>
    void write(std::vector<T>& data, const std::string& id){
        enterArray(id);

        leaveArray(id);
    }

    template<class T>
    void write(std::list<T>& data, const std::string& id){

    }

    template<class T>
    void write(T* array, int nrElems, const std::string& id){

    }
*/
    // now for the complex types, these needs to implement save/load functionality
    template<class T>
    void write(const T& data, const std::string& id){
        // the data method must have an implementation of load/save and if not then we try the generic write
        // method which could provide a solution by the implementation itself
        writeEnterScope(id);
        try {
            Archive::Access::save<T>(data, *this, id);
        } catch (...){
            // we fall back to this call
            boost::any adata(data);
            write(adata, id);
        }
        writeLeaveScope(id);
    }

    /**
     * @brief this is the fallback call in case a class does not implement load/save funtionality
     * then the implementation on the OutputArchive might be able to save the class.
     *
     * @note unfortunately this require the implementing InputArchive to use downcast methods...
     */
    virtual void write(boost::any& anydata, const std::string& id){ RW_THROW("No handler for this type of data!");};
};
#endif
