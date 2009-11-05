#include <rw/common/Ptr.hpp>
#include <rw/use_robwork_namespace.hpp>
using namespace robwork;

class T {};
typedef Ptr<T> TPtr;

TPtr makeT() { return ownedPtr(new T); }
