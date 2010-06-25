#include <rw/common/Ptr.hpp>

using namespace rw::common;

class T {};
typedef Ptr<T> TPtr;

TPtr makeT() { return ownedPtr(new T); }
